/**
 * @file ConvexMPCLocomotion.h
 * @brief 将步态相位、状态预测和凸 MPC 求解器组织成一次行走规划。
 * 它最终输出的是：期望机身状态；当前支撑状态；足端接触/摆动相位；
 * 期望接触反力。它不直接输出电机力矩。
 *
 * 输出仍是世界坐标系下的足端反作用力和相位，不直接输出电机力矩。
 */
#ifndef MYMIT_ROBOT_MPC_CONVEX_MPC_LOCOMOTION_H_
#define MYMIT_ROBOT_MPC_CONVEX_MPC_LOCOMOTION_H_

#include <array>
#include <cstddef>

#include "MPC/Gait.h"
#include "MPC/SolverMPC.h"
#include "controller/GaitScheduler.hpp"

namespace mpc
{

template<typename T>
struct LocomotionResult
{
  /** @brief Eigen 固定尺寸成员使用的对齐分配支持。 */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief 经过限幅、坐标转换和连续化处理后的 MPC 内部目标。 */
  DesiredState<T> command{};
  /** @brief 四条腿当前控制时刻的世界坐标系地面反力。 */
  std::array<Vec3<T>, kNumLegs> reaction_forces_world{};
  /** @brief 四条腿当前支撑阶段的归一化相位。 */
  std::array<T, kNumLegs> contact_phase{};
  /** @brief 四条腿当前摆动阶段的归一化相位。 */
  std::array<T, kNumLegs> swing_phase{};
  /** @brief 四条腿的支撑持续时间，单位 s。 */
  std::array<T, kNumLegs> stance_time{};
  /** @brief 四条腿的摆动持续时间，单位 s。 */
  std::array<T, kNumLegs> swing_time{};
  /** @brief 当前预测表第一行给出的接触状态，true 表示支撑。 */
  std::array<bool, kNumLegs> contact_state{};
  /** @brief 本控制周期是否实际执行并刷新了 MPC 优化解。 */
  bool mpc_updated = false;
  /** @brief 最近一次 MPC 优化是否在迭代上限前收敛。 */
  bool mpc_converged = false;
  /** @brief 本次输出是否可以交给下游 WBC 或控制器使用。 */
  bool valid = false;
};

template<typename T>
class ConvexMPCLocomotion
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief 创建步态和接触力 MPC 封装。
   * @param quadruped 四足机器人模型。
   * @param control_time_step 外部高速控制周期，单位 s。
   * @param iterations_between_mpc 两次 MPC 求解之间的控制周期数。
   * @param settings 接触力求解器参数。
   * @param iterations_per_gait_segment 每个步态段对应的控制周期数；为 0 时沿用求解间隔。
   */
  ConvexMPCLocomotion(
    const Quadruped<T> & quadruped, T control_time_step,
    std::size_t iterations_between_mpc = 15,
    const SolverSettings<T> & settings = SolverSettings<T>{},
    std::size_t iterations_per_gait_segment = 0);

  /** @brief 清零周期、命令斜坡和缓存的 MPC 输出。 */
  void initialize() noexcept;
  /** @brief 设置当前步态；支持 STAND、TROT、TROT_WALK 和 STATIC_WALK。 */
  void setGait(GaitType gait);
  /** @brief 设置机身前向速度，单位 m/s。 */
  void setForwardVelocity(T velocity);
  /**
   * @brief 设置机身坐标系平面速度和偏航角速度命令。
   *
   * x 轴表示前向，y 轴表示左向，yaw 为绕 z 轴的逆时针角速度。
   * 速度和角速度会在函数内部进行范围检查，并在运行时进行加速度限幅。
   *
   * @param forward_velocity 机身坐标系前向速度，单位 m/s。
   * @param lateral_velocity 机身坐标系横向速度，单位 m/s。
   * @param yaw_rate 期望偏航角速度，单位 rad/s。
   */
  void setVelocityCommand(T forward_velocity, T lateral_velocity, T yaw_rate);
  /**
   * @brief 推进一步运动 MPC。
   * @param estimate 当前状态估计。
   * @param desired 外部期望机身状态。
   * @param foot_positions_world 当前四条腿足端世界系位置。
   * @return 步态相位、当前接触状态和最近一次有效反力。
   */
  LocomotionResult<T> run(
    const StateEstimate<T> & estimate, const DesiredState<T> & desired,
    const std::array<Vec3<T>, kNumLegs> & foot_positions_world);

private:
  /** @brief 返回当前 STAND、TROT、TROT_WALK 或 STATIC_WALK 步态对象。 */
  OffsetDurationGait & activeGait() noexcept;
  /**
   * @brief 将外部目标和速度命令转换为 MPC 内部目标状态。
   * @param estimate 当前状态估计，用于偏航连续化和位置锚定。
   * @param desired 外部期望机身状态。
   * @return 经过限幅、坐标转换和预测起点处理后的目标状态。
   */
  DesiredState<T> setupCommand(
    const StateEstimate<T> & estimate, const DesiredState<T> & desired);

  /** @brief 外部高速控制周期，单位 s。 */
  T control_time_step_;
  /** @brief 两次 MPC 求解之间的高速控制周期数。 */
  std::size_t iterations_between_mpc_;
  /** @brief 每个离散步态段对应的高速控制周期数。 */
  std::size_t iterations_per_gait_segment_;
  /** @brief 当前高速控制周期编号。 */
  std::size_t iteration_ = 0;
  /** @brief 当前使用的步态类型。 */
  GaitType gait_type_ = GaitType::STAND;
  /** @brief 用户输入的机身坐标系平面速度。 */
  Vec2<T> velocity_body_ = Vec2<T>::Zero();
  /** @brief 经加速度限幅后真正进入 MPC 的机身坐标系速度。 */
  Vec2<T> commanded_velocity_body_ = Vec2<T>::Zero();
  /** @brief 用户输入的偏航角速度，单位 rad/s。 */
  T yaw_rate_ = T(0);
  /** @brief 经加速度限幅后真正使用的偏航角速度，单位 rad/s。 */
  T commanded_yaw_rate_ = T(0);
  /** @brief 连续化后的目标偏航角，单位 rad。 */
  T command_yaw_ = T(0);

  /** @brief 平面速度命令的最大变化率，单位 m/s^2。 */
  T maximum_linear_acceleration_ = T(0.75);
  /** @brief 偏航角速度命令的最大变化率，单位 rad/s^2。 */
  T maximum_yaw_acceleration_ = T(0.8);
  /** @brief 速度遥控模式下用于 MPC 的世界坐标位置锚点。 */
  Vec3<T> command_position_world_ = Vec3<T>::Zero();
  /** @brief 是否已经用初始目标初始化过目标偏航角。 */
  bool command_initialized_ = false;
  /** @brief 最近一次有效求解得到的四腿世界系反力。 */
  std::array<Vec3<T>, kNumLegs> cached_reaction_forces_world_{};
  /** @brief 上一次有效反力对应的接触状态。 */
  std::array<bool, kNumLegs> cached_contact_state_{};
  /** @brief 反力缓存是否有效。 */
  bool cached_solution_valid_ = false;
  /** @brief 最近一次缓存求解是否收敛。 */
  bool cached_solution_converged_ = false;
  /** @brief 底层有限时域接触力 MPC 求解器。 */
  SolverMPC<T> solver_;
  /** @brief 始终四脚支撑的站立步态。 */
  OffsetDurationGait stand_;
  /** @brief 50% 支撑率的对角小跑，用于按键 2 原地踏步。 */
  OffsetDurationGait trot_;
  /** @brief 60% 支撑率的对角行走，用于普通方向运动。 */
  OffsetDurationGait trot_walk_;
  /** @brief 单腿依次摆动、始终至少三腿支撑的静态步态。 */
  OffsetDurationGait static_walk_;
};

extern template struct LocomotionResult<float>;
extern template struct LocomotionResult<double>;
extern template class ConvexMPCLocomotion<float>;
extern template class ConvexMPCLocomotion<double>;

}  // namespace mpc

#endif  // MYMIT_ROBOT_MPC_CONVEX_MPC_LOCOMOTION_H_
