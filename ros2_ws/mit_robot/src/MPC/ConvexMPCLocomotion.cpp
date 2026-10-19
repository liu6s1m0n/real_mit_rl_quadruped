/**
 * @file ConvexMPCLocomotion.cpp
 * @brief 步态、速度命令、预测轨迹和接触力求解器之间的 MPC 运行封装。
 */

#include "MPC/ConvexMPCLocomotion.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace mpc
{

/**
 * @brief 创建 STAND/TROT 步态和接触力求解器。
 * @throws std::invalid_argument 当控制周期、求解间隔或预测窗非法时抛出。
 */
template<typename T>
ConvexMPCLocomotion<T>::ConvexMPCLocomotion(
  const Quadruped<T> & quadruped, T control_time_step,
  std::size_t iterations_between_mpc, const SolverSettings<T> & settings)
: control_time_step_(control_time_step),
  iterations_between_mpc_(iterations_between_mpc),
  solver_(quadruped, settings)
{
  // 对角小跑中 LF+RH 与 RF+LH 分成两组，相位相差半个预测时域。
  if (!std::isfinite(static_cast<double>(control_time_step_)) ||
    control_time_step_ <= T(0) || iterations_between_mpc_ == 0 ||
    settings.horizon < 2 ||
    settings.horizon % 2 != 0)
  {
    throw std::invalid_argument("invalid convex MPC locomotion timing");
  }
  normal_force_bounds_.minimum.fill(settings.minimum_normal_force);
  normal_force_bounds_.maximum.fill(settings.maximum_normal_force);
}

/** @brief 清零内部计数、滤波命令和上一周期的反力缓存。 */
template<typename T>
void ConvexMPCLocomotion<T>::initialize() noexcept
{
  // 清零高速控制周期计数和命令初始化状态。
  control_iteration_ = 0;
  command_initialized_ = false;
  // 清零经过加速度限幅后的速度和偏航命令。
  commanded_velocity_body_.setZero();
  commanded_yaw_rate_ = T(0);
  command_yaw_ = T(0);
  rpy_integral_roll_ = T(0);
  rpy_integral_pitch_ = T(0);
  position_offset_world_.setZero();
  // 清除上一周期的反力、接触状态和求解有效性缓存。
  cached_reaction_forces_world_ = {};
  cached_contact_state_ = {};
  cached_mpc_table_.clear();
  cached_solution_valid_ = false;
  cached_solution_converged_ = false;
}

/** @brief 将前向速度包装成平面速度命令。 */
template<typename T>
void ConvexMPCLocomotion<T>::setForwardVelocity(T velocity)
{
  setVelocityCommand(velocity, T(0), T(0));
}

/**
 * @brief 校验并保存机身坐标系速度命令。
 * @throws std::invalid_argument 当平面速度或偏航速度超出限制时抛出。
 */
template<typename T>
void ConvexMPCLocomotion<T>::setVelocityCommand(
  T forward_velocity, T lateral_velocity, T yaw_rate)
{
  constexpr T maximum_linear_velocity = T(1);
  constexpr T maximum_yaw_rate = T(2);
  const Vec2<T> linear_velocity(forward_velocity, lateral_velocity);
  if (!linear_velocity.allFinite() ||
    linear_velocity.norm() > maximum_linear_velocity)
  {
    throw std::invalid_argument(
            "locomotion planar velocity magnitude must not exceed 1 m/s");
  }
  if (!std::isfinite(static_cast<double>(yaw_rate)) ||
    std::abs(yaw_rate) > maximum_yaw_rate)
  {
    throw std::invalid_argument(
            "locomotion yaw rate must be within [-2, 2] rad/s");
  }
  velocity_body_ = linear_velocity;
  yaw_rate_ = yaw_rate;
}

template<typename T>
void ConvexMPCLocomotion<T>::setPositionOffsetWorld(const Vec2<T> & offset)
{
  if (!offset.allFinite() || offset.norm() > T(0.05)) {
    throw std::invalid_argument("position offset must be finite and within 5 cm");
  }
  position_offset_world_ = offset;
}

/**
 * @brief 将外部期望转换成 MPC 内部命令。
 *
 * 包括 MIT 速度滤波、速度相关姿态补偿、偏航角连续化、机身坐标系速度到
 * 世界系速度的转换，以及基于当前测量位置的短时预测起点设置。
 */
template<typename T>
DesiredState<T> ConvexMPCLocomotion<T>::setupCommand(
  const StateEstimate<T> & estimate, const DesiredState<T> & desired)
{
  DesiredState<T> command = desired;
  if (!command_initialized_) {
    command_yaw_ = desired.body_rpy.z();
    command_position_world_ = estimate.position_world;
    command_initialized_ = true;
  }

  // MIT ConvexMPCLocomotion uses a first-order command filter.
  commanded_velocity_body_ += velocity_filter_ *
    (velocity_body_ - commanded_velocity_body_);
  commanded_yaw_rate_ = yaw_rate_;

  // MIT's velocity-dependent attitude compensation keeps the body level while
  // the robot accelerates.  The integral is bounded to the same range as MIT.
  const Vec3<T> velocity_body =
    estimate.rotation_world_from_body.transpose() * estimate.velocity_world;
  if (std::abs(velocity_body.x()) > T(0.2)) {
    rpy_integral_pitch_ += control_time_step_ *
      (T(0) - estimate.rpy.y()) / velocity_body.x();
  }
  if (std::abs(velocity_body.y()) > T(0.1)) {
    rpy_integral_roll_ += control_time_step_ *
      (T(0) - estimate.rpy.x()) / velocity_body.y();
  }
  rpy_integral_roll_ = std::clamp(rpy_integral_roll_, T(-0.25), T(0.25));
  rpy_integral_pitch_ = std::clamp(rpy_integral_pitch_, T(-0.25), T(0.25));

  // 将连续偏航目标重表达到实测偏航附近，跨越 +/-pi 时避免产生 2*pi 跳变。
  const T relative_yaw = command_yaw_ - estimate.rpy.z();
  command_yaw_ = estimate.rpy.z() +
    std::atan2(std::sin(relative_yaw), std::cos(relative_yaw));
  // MIT 做法：静止时偏航目标跟随当前估计，不锁存绝对航向。DM IMU 航向
  // 会缓慢漂移，锁存会让 WBC 把真实机身拧回漂移前的角度，支撑脚被扭转。
  if (commanded_velocity_body_.squaredNorm() <= T(1e-8) &&
    std::abs(commanded_yaw_rate_) <= T(1e-4))
  {
    command_yaw_ = estimate.rpy.z();
  }
  command_yaw_ += commanded_yaw_rate_ * control_time_step_;
  command.body_rpy.x() = velocity_body.y() * rpy_integral_roll_;
  command.body_rpy.y() = velocity_body.x() * rpy_integral_pitch_;
  command.body_rpy.z() = command_yaw_;

  const T cosine_yaw = std::cos(command_yaw_);
  const T sine_yaw = std::sin(command_yaw_);
  command.body_velocity_world <<
    cosine_yaw * commanded_velocity_body_.x() -
    sine_yaw * commanded_velocity_body_.y(),
    sine_yaw * commanded_velocity_body_.x() +
    cosine_yaw * commanded_velocity_body_.y(),
    T(0);

  command.body_acceleration_world.setZero();
  command.body_angular_velocity << T(0), T(0), commanded_yaw_rate_;
  // MIT 对期望速度积分得到世界系水平位置目标，并把目标限制在当前
  // 估计位置附近，避免遥控目标无限漂移。
  if (commanded_velocity_body_.squaredNorm() > T(1e-8) ||
    std::abs(commanded_yaw_rate_) > T(1e-4))
  {
    command_position_world_.template head<2>() +=
      control_time_step_ * command.body_velocity_world.template head<2>();
    const Vec2<T> position_error =
      command_position_world_.template head<2>() -
      estimate.position_world.template head<2>();
    command_position_world_.x() = estimate.position_world.x() +
      std::clamp(position_error.x(), T(-0.1), T(0.1));
    command_position_world_.y() = estimate.position_world.y() +
      std::clamp(position_error.y(), T(-0.1), T(0.1));
  }
  command_position_world_.z() = desired.body_position_world.z();
  command.body_position_world = command_position_world_;
  command.body_position_world.template head<2>() += position_offset_world_;

  command.valid = desired.valid && command.body_position_world.allFinite() &&
    command.body_velocity_world.allFinite() && command.body_rpy.allFinite() &&
    command.body_angular_velocity.allFinite();
  return command;
}

template<typename T>
void ConvexMPCLocomotion<T>::setNormalForceBounds(
  const NormalForceBounds<T> & bounds)
{
  const auto & settings = solver_.settings();
  bool changed = false;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    if (!std::isfinite(static_cast<double>(bounds.minimum[leg])) ||
      !std::isfinite(static_cast<double>(bounds.maximum[leg])) ||
      bounds.minimum[leg] < settings.minimum_normal_force ||
      bounds.maximum[leg] > settings.maximum_normal_force ||
      bounds.minimum[leg] > bounds.maximum[leg])
    {
      throw std::invalid_argument("invalid per-leg normal force bounds");
    }
    changed = changed || bounds.minimum[leg] != normal_force_bounds_.minimum[leg] ||
      bounds.maximum[leg] != normal_force_bounds_.maximum[leg];
  }
  if (changed) {
    normal_force_bounds_ = bounds;
    cached_solution_valid_ = false;
  }
}

/**
 * @brief 执行一个高速控制周期的 MPC 封装流程。
 *
 * 每个周期读取统一步态快照；达到求解周期或接触状态变化时，构造预测
 * 轨迹并调用 SolverMPC。未触发求解时复用最近一次有效的第一步反力。
 */
template<typename T>
LocomotionResult<T> ConvexMPCLocomotion<T>::run(
  const StateEstimate<T> & estimate, const DesiredState<T> & desired,
  const std::array<Vec3<T>, kNumLegs> & foot_positions_world,
  const GaitFrame<T> & gait_frame)
{
  LocomotionResult<T> result;
  if (!estimate.valid || !desired.valid) {return result;}
  if (!gait_frame.valid(solver_.settings().horizon)) {return result;}
  result.gait_frame = gait_frame;
  result.command = setupCommand(estimate, desired);
  if (!result.command.valid) {return result;}
  const auto & contact_table = gait_frame.mpc_table;

  // 500 Hz 控制循环通常每 iterations_between_mpc_ 帧执行一次昂贵的优化。
  // 其余帧仍更新命令、接触相位和摆腿轨迹，并把最近的有效反力交给 WBC；
  // 接触组合发生变化时例外，必须立即刷新反力。
  // 求解周期和步态分段解耦后，接触切换不一定落在固定的 25 Hz 刷新点。
  // 接触组合变化时立即补算一次，避免新支撑腿短时间沿用摆动期的零反力。
  const bool contact_state_changed = cached_solution_valid_ &&
    gait_frame.contact_state != cached_contact_state_;
  const bool mpc_table_changed = cached_solution_valid_ &&
    gait_frame.mpc_table != cached_mpc_table_;
  const bool solve_due = !cached_solution_valid_ || contact_state_changed ||
    mpc_table_changed ||
    control_iteration_ % iterations_between_mpc_ == 0;

  if (solve_due) {
    std::vector<DesiredState<T>> trajectory(
      solver_.settings().horizon, result.command);
    // 用期望速度外推未来位置和偏航角，形成 MPC 需要的整段参考轨迹。
    for (std::size_t step = 0; step < trajectory.size(); ++step) {
      // MIT 的预测窗第 0 段就是当前命令，后续段再按 dtMPC 外推。
      const T lookahead = solver_.settings().time_step * static_cast<T>(step);

      trajectory[step].body_position_world =
        result.command.body_position_world +
        lookahead * result.command.body_velocity_world;
      const Vec3<T> desired_rpy_rate =
        RobotState<T>::rpyRateFromBodyAngularVelocity(
        result.command.body_rpy, result.command.body_angular_velocity);
      trajectory[step].body_rpy =
        result.command.body_rpy + lookahead * desired_rpy_rate;
      // DesiredState 明确定义为机身系角速度；SolverMPC 在构造目标向量时负责
      // 按目标姿态转换为 RPY 导数，此处不得提前改写成世界系。
      trajectory[step].body_angular_velocity = result.command.body_angular_velocity;
    }

    const RobotState<T> state =
      RobotState<T>::fromEstimate(estimate, foot_positions_world);
    /*关键函数*/
    const SolverResult<T> solution = solver_.solve(
      state, trajectory, contact_table, &normal_force_bounds_);
    if (solution.valid) {
      cached_reaction_forces_world_ = solution.reaction_forces_world;
      cached_contact_state_ = gait_frame.contact_state;
      cached_mpc_table_ = gait_frame.mpc_table;
      cached_solution_valid_ = true;
      cached_solution_converged_ = solution.converged;
      result.mpc_updated = true;
    } else {
      // 刷新点通常也是接触预测进入下一段的边界。新解失败时不能继续把
      // 上一段接触组合的反力施加到当前支撑腿；标记无效并在下一帧重试。
      cached_solution_valid_ = false;
      cached_solution_converged_ = false;
    }
  }

  if (cached_solution_valid_) {
    result.reaction_forces_world = cached_reaction_forces_world_;
    result.mpc_converged = cached_solution_converged_;
    result.valid = true;
  }
  ++control_iteration_;
  return result;
}

template struct LocomotionResult<float>;
template struct LocomotionResult<double>;
template class ConvexMPCLocomotion<float>;
template class ConvexMPCLocomotion<double>;

}  // namespace mpc
