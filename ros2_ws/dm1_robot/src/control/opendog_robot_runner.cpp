/*! @file opendog_robot_runner.cpp
 *  @brief OpenDogRobotRunner 实现：操作员命令到期望状态的转换和关节插值流程。
 */
#include "control/opendog_robot_runner.hpp"

#include <algorithm>
#include <cmath>

#include "common/console_log.hpp"
#include "common/control_config.h"

namespace
{
// DM1 真机启动时关节插值到趴卧姿态的时长，单位 s。
constexpr float kJointInitializationDuration = 1.2F;
// P 趴下：从站立姿态平滑折叠到趴卧姿态的时长和位置增益。
// 2026-11-16：原 3 s、kp 4，kp 4 撑不住体重，站立前馈一卸载机身就往下掉（实机最大 22 cm/s 砸到肚子）；
// 仿真（关节摩擦 2 Nm）5 s、kp 30 时最大下降速度 21.6→8.9 cm/s、触地冲击 6.6→2.5 m/s²。
// 趴下从实测关节角起步，切换时位置误差为 0，提高 kp 不会跳变；完成后回到 JointPd 的常规 kp。
constexpr float kProneDownDuration = 5.0F;
constexpr float kProneDownKp = 30.0F;

// 零点标定动作：四脚不动，机身逐项做一个来回。每段 5 s，偏移 = 幅值 × 平滑的一正一负波形，
// 段首段尾速度为 0；前后各留 1 s 静止。轴：0/1/2 = 世界系 x/y/z（x/y 按起始航向旋转），3/4/5 = 横滚/俯仰/偏航。
// 参考轨迹在这期间按四脚不动反解关节目标（DesiredState::zero_calibration），否则默认站姿的关节权重
// 会抵消平移和升降（2026-11-13 实机命令前后 ±4、左右 ±3 cm，只动了 0.2/0 cm）。
struct CalibrationSegment
{
  int axis;
  float amplitude;  ///< m 或 rad
  const char * name;
};
constexpr float kCalibrationSettle = 1.0F;
constexpr float kCalibrationSegmentDuration = 5.0F;
constexpr std::array<CalibrationSegment, 6> kCalibrationSegments{{
  {2, 0.04F, "升降 ±4 cm"},
  {4, 0.175F, "俯仰 ±10°"},
  {3, 0.14F, "横滚 ±8°"},
  {0, 0.04F, "前后 ±4 cm"},
  {1, 0.03F, "左右 ±3 cm"},
  {5, 0.21F, "偏航 ±12°"}}};
constexpr float kCalibrationDuration =
  2.0F * kCalibrationSettle + kCalibrationSegmentDuration * kCalibrationSegments.size();

// 一正一负的平滑波形，起止点的值和速度都为 0，峰值为 1。
constexpr float kTwoPi = 6.28318530718F;
constexpr float kPeakScale = 1.0F / 0.6495F;  // sin(x)(1-cos x)/2 的最大值约 0.6495
float calibrationWave(float phase)
{
  return kPeakScale * std::sin(kTwoPi * phase) * 0.5F * (1.0F - std::cos(kTwoPi * phase));
}

// calibrationWave 对 phase 的导数。
float calibrationWaveRate(float phase)
{
  const float c = std::cos(kTwoPi * phase);
  const float s = std::sin(kTwoPi * phase);
  return kPeakScale * 0.5F * kTwoPi * (c * (1.0F - c) + s * s);
}

// 启动回零/趴下的关节目标：趴卧姿态。电机零位是 OpenDog 机械零位（小腿伸直），
// 不能作为着地目标。
Vec3<float> proneJointTarget()
{
  return Vec3<float>(
    static_cast<float>(control_config::pose::prone_hip),
    static_cast<float>(control_config::pose::prone_thigh),
    static_cast<float>(control_config::pose::prone_calf));
}

// Eigen 的 Vec3 默认构造不会清零，`= {}` 得到的是未初始化内存；必须显式 setZero。
void zeroFeedforward(std::array<Vec3<float>, kNumLegs> & feedforward)
{
  for (auto & value : feedforward) {value.setZero();}
}
}

OpenDogRobotRunner::OpenDogRobotRunner(const std::string & config_directory, float period)
: algorithm_(config_directory, period)
{
  zeroFeedforward(prone_feedforward_);
}

/// 配置算法并把期望高度设为站立高度。
bool OpenDogRobotRunner::configure()
{
  if (!algorithm_.configure()) {return false;}
  desired_ = {};
  desired_.body_position_world.z() = algorithm_.commandSettings().standing_height;
  desired_.valid = true;
  return true;
}

/// 复位所有命令状态，下一周期从实测关节角重新插值到趴卧姿态。
void OpenDogRobotRunner::reset() noexcept
{
  algorithm_.reset();
  desired_ = {};
  desired_.body_position_world.z() = algorithm_.commandSettings().standing_height;
  desired_.mode = ControlMode::JointPd;
  desired_.valid = true;
  estimate_ = {};
  motion_sequence_ = 0;
  output_sequence_ = 0;
  initialization_active_ = true;
  initialization_started_ = false;
  initialization_complete_ = false;
  stand_up_pending_ = false;
  zeroFeedforward(prone_feedforward_);
  initialization_duration_ = kJointInitializationDuration;
  prone_down_active_ = false;
  zero_calibration_active_ = false;
}

bool OpenDogRobotRunner::startZeroCalibration()
{
  const float standing_height = algorithm_.commandSettings().standing_height;
  const bool standing = !initialization_active_ &&
    (desired_.mode == ControlMode::StandUp || desired_.mode == ControlMode::BalanceStand) &&
    estimate_.position_world.z() > 0.85F * standing_height;
  if (!standing) {
    imu_log::print(
      imu_log::Level::Warning,
      "[OpenDog][ZERO_CAL] 只能在站立静止时开始：先按 1 站起并站稳（踏步中先按空格停下）。\n");
    return false;
  }
  holdCurrentPose(ControlMode::BalanceStand);
  zero_calibration_origin_ = Vec3<float>(
    estimate_.position_world.x(), estimate_.position_world.y(), standing_height);
  zero_calibration_yaw_ = estimate_.rpy.z();
  zero_calibration_start_ = -1.0F;
  zero_calibration_active_ = true;
  imu_log::print(
    imu_log::Level::Info,
    "[OpenDog][ZERO_CAL] 开始零点标定动作，约 %.0f s：四脚保持不动，机身依次升降、俯仰、横滚、前后、左右、偏航。"
    "空格可随时中止。\n",
    static_cast<double>(kCalibrationDuration));
  return true;
}

void OpenDogRobotRunner::updateZeroCalibration(float timestamp)
{
  if (zero_calibration_start_ < 0.0F) {
    zero_calibration_start_ = timestamp;
    zero_calibration_segment_ = -1;
  }
  const float elapsed = timestamp - zero_calibration_start_;
  std::array<float, 6> offset{};
  std::array<float, 6> rate{};
  const float active = elapsed - kCalibrationSettle;
  if (active >= 0.0F) {
    const auto segment = static_cast<std::size_t>(active / kCalibrationSegmentDuration);
    if (segment < kCalibrationSegments.size()) {
      const float phase = (active - segment * kCalibrationSegmentDuration) / kCalibrationSegmentDuration;
      const auto axis = static_cast<std::size_t>(kCalibrationSegments[segment].axis);
      offset[axis] = kCalibrationSegments[segment].amplitude * calibrationWave(phase);
      rate[axis] = kCalibrationSegments[segment].amplitude * calibrationWaveRate(phase) /
        kCalibrationSegmentDuration;
      if (static_cast<int>(segment) != zero_calibration_segment_) {
        zero_calibration_segment_ = static_cast<int>(segment);
        imu_log::print(
          imu_log::Level::Info, "[OpenDog][ZERO_CAL] %zu/%zu %s\n",
          segment + 1, kCalibrationSegments.size(), kCalibrationSegments[segment].name);
      }
    }
  }
  const float c = std::cos(zero_calibration_yaw_);
  const float s = std::sin(zero_calibration_yaw_);
  desired_.mode = ControlMode::BalanceStand;
  // 速度只给参考轨迹算关节参考速度（摩擦补偿方向），站立模式下不进 MPC 目标。
  desired_.body_velocity_world = Vec3<float>(c * rate[0] - s * rate[1], s * rate[0] + c * rate[1], rate[2]);
  desired_.body_angular_velocity = Vec3<float>(rate[3], rate[4], rate[5]);
  desired_.body_position_world = zero_calibration_origin_ +
    Vec3<float>(c * offset[0] - s * offset[1], s * offset[0] + c * offset[1], offset[2]);
  desired_.body_rpy = Vec3<float>(offset[3], offset[4], zero_calibration_yaw_ + offset[5]);
  if (elapsed >= kCalibrationDuration) {
    zero_calibration_active_ = false;
    desired_.body_velocity_world.setZero();
    desired_.body_angular_velocity.setZero();
    desired_.body_rpy = Vec3<float>(0.0F, 0.0F, zero_calibration_yaw_);
    desired_.body_position_world = zero_calibration_origin_;
    imu_log::print(
      imu_log::Level::Warning,
      "\n================================================================\n"
      "[OpenDog][ZERO_CAL] 零点标定动作完成，已回到起始站姿。\n"
      "  用 scripts/zero_calibration.py 解算本次 CSV。\n"
      "================================================================\n\n");
  }
}

void OpenDogRobotRunner::startProneDown()
{
  // 复用回零插值：从当前实测关节角平滑回到趴卧姿态；前馈从最后一帧 WBC 输出线性卸载，
  // 开始时仍承重，结束时为 0。完成后复位算法，再按 1 可重新站立。
  initialization_active_ = true;
  initialization_started_ = false;
  initialization_complete_ = false;
  stand_up_pending_ = false;
  prone_down_active_ = true;
  zero_calibration_active_ = false;
  initialization_duration_ = kProneDownDuration;
  desired_.mode = ControlMode::ProneDown;
  imu_log::print(
    imu_log::Level::Info,
    "[OpenDog][PRONE] 开始趴下：%.1f s 内从当前姿态平滑折叠到趴卧姿态，前馈力矩逐步卸载。\n",
    static_cast<double>(kProneDownDuration));
}

/// 锁定当前水平位置和航向，避免切换模式时机身被拉回旧目标；同时结束标定动作。
void OpenDogRobotRunner::holdCurrentPose(ControlMode mode)
{
  if (zero_calibration_active_) {
    zero_calibration_active_ = false;
    desired_.body_rpy.x() = 0.0F;
    desired_.body_rpy.y() = 0.0F;
    desired_.body_position_world.z() = algorithm_.commandSettings().standing_height;
    imu_log::print(imu_log::Level::Info, "[OpenDog][ZERO_CAL] 标定动作已中止。\n");
  }
  desired_.mode = mode;
  desired_.body_velocity_world.setZero();
  desired_.body_angular_velocity.setZero();
  desired_.body_position_world.head<2>() = estimate_.position_world.head<2>();
  desired_.body_rpy.z() = estimate_.rpy.z();
}

/// 初始化未完成时挂起站立请求，完成后在 computeJointInitialization() 中补执行。
bool OpenDogRobotRunner::requestStandUp()
{
  if (initialization_active_ && !initialization_complete_) {
    stand_up_pending_ = true;
    return false;
  }
  initialization_active_ = false;
  stand_up_pending_ = false;
  holdCurrentPose(ControlMode::StandUp);
  return true;
}

/// 同一运动按键重复按下时递增序号，使单次踏步动作可以再次触发。
void OpenDogRobotRunner::acceptCommands(
  const teleop::OperatorCommandArbiter & arbiter,
  const std::vector<teleop::OperatorCommand> & commands)
{
  for (const auto & command : commands) {
    if (command.type == teleop::CommandType::Motion && arbiter.motionActive() &&
      command.motion == arbiter.motion()) {++motion_sequence_;}
  }
}

/// 按停止、站立、趴下、运动的优先顺序处理仲裁器中的请求。
bool OpenDogRobotRunner::handleCommands(
  teleop::OperatorCommandArbiter & arbiter, const ImuData<float> & imu, bool active)
{
  if (arbiter.takeStopRequest() && !initialization_active_) {
    holdCurrentPose(ControlMode::BalanceStand);
  }
  const bool stand_up = arbiter.takeStandUpRequest();
  if (stand_up) {
    // 与原 legged_keyboard.cpp 一致：按 1 时先清除 trot 请求并发布零速度，
    // 再发送 /set_stance。这里保持相同的命令顺序。
    arbiter.stopMotion();
    arbiter.takeStopRequest();
    requestStandUp();
  }
  if (arbiter.takeZeroCalibrationRequest() && !stand_up) {
    if (arbiter.motionActive()) {
      imu_log::print(imu_log::Level::Warning, "[OpenDog][ZERO_CAL] 踏步中不能开始标定：先按空格停下。\n");
    } else if (!zero_calibration_active_) {
      startZeroCalibration();
    }
  }
  if (arbiter.takeProneDownRequest()) {
    if (initialization_active_) {
      imu_log::print(
        imu_log::Level::Info, "OpenDog 正在回零或趴下，忽略本次 P。\n");
    } else {
      arbiter.stopMotion();
      arbiter.takeStopRequest();
      startProneDown();
    }
  }
  if (!stand_up && arbiter.motionActive()) {
    if (initialization_active_) {
      arbiter.stopMotion();
      arbiter.takeStopRequest();
      imu_log::print(imu_log::Level::Warning, "OpenDog 运动请求已取消：先完成回零并按 1 站立。\n");
      return active;
    }
    if (zero_calibration_active_) {holdCurrentPose(ControlMode::BalanceStand);}
    // 速度档位在机身系定义，按当前 IMU 姿态旋转到世界系后交给参考轨迹。
    algorithm_.setMotionRequest(arbiter.motion(), motion_sequence_);
    const auto velocity = teleop::velocityForMotion(
      arbiter.motion(), algorithm_.commandSettings().velocity);
    desired_.body_velocity_world = imu.orientation_world_from_body *
      Vec3<float>(velocity.forward, velocity.lateral, 0.0F);
    desired_.body_angular_velocity.z() = velocity.yaw;
    desired_.mode = ControlMode::Locomotion;
  }
  return active;
}

/// 初始化/趴下期间走关节插值；其余模式交给算法，并记录前馈供趴下时逐步卸载。
bool OpenDogRobotRunner::compute(const ControlInputSnapshot & input, ControlOutput & output)
{
  if (initialization_active_) {return computeJointInitialization(input, output);}
  if (zero_calibration_active_) {updateZeroCalibration(input.timestamp);}
  desired_.zero_calibration = zero_calibration_active_;
  auto snapshot = input;
  desired_.timestamp = input.timestamp;
  snapshot.desired = desired_;
  if (!algorithm_.compute(snapshot, output)) {return false;}
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    output.commands[leg].sequence = output_sequence_ + 1;
    prone_feedforward_[leg] = output.commands[leg].torque_feedforward;
  }
  ++output_sequence_;
  estimate_ = output.estimate;
  return true;
}

bool OpenDogRobotRunner::computeJointInitialization(
  const ControlInputSnapshot & input, ControlOutput & output)
{
  // 记录实测关节角作为起点，用三次 smoothstep 插值到趴卧姿态。
  // 这条路径不运行优化器。
  if (!initialization_started_) {
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      initial_joint_positions_[leg] = input.joints[leg].position;
    }
    initialization_start_time_ = input.timestamp;
    initialization_started_ = true;
  }
  const float phase = std::clamp(
    (input.timestamp - initialization_start_time_) / initialization_duration_, 0.0F, 1.0F);
  // blend = 3p^2 - 2p^3，blend_rate 为其对时间的导数，用作期望关节速度。
  const float blend = phase * phase * (3.0F - 2.0F * phase);
  const float blend_rate = 6.0F * phase * (1.0F - phase) / initialization_duration_;
  output = {};
  output.valid = true;
  output.estimate.orientation_world_from_body = input.imu.orientation_world_from_body;
  output.estimate.rotation_world_from_body =
    input.imu.orientation_world_from_body.toRotationMatrix();
  output.estimate.timestamp = input.timestamp;
  const auto & settings = algorithm_.commandSettings();
  ++output_sequence_;
  const Vec3<float> prone = proneJointTarget();
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    auto & command = output.commands[leg];
    command.leg = static_cast<LegId>(leg);
    const Vec3<float> travel = prone - initial_joint_positions_[leg];
    command.position_desired = initial_joint_positions_[leg] + blend * travel;
    command.velocity_desired = blend_rate * travel;
    // 启动回零时 prone_feedforward_ 为 0，行为不变；趴下时从站立前馈卸载到 0。
    // 前馈按 blend 的 smoothstep 卸载，起止都平缓。
    command.torque_feedforward = (1.0F - blend * blend * (3.0F - 2.0F * blend)) * prone_feedforward_[leg];
    command.kp.setConstant(prone_down_active_ ? kProneDownKp : settings.joint_kp);
    command.kd.setConstant(settings.joint_kd);
    command.timestamp = input.timestamp;
    command.sequence = output_sequence_;
    command.enabled = true;
  }
  if (phase >= 1.0F && !initialization_complete_) {
    initialization_complete_ = true;
    if (prone_down_active_) {
      // 趴下完成：复位估计器/MPC/WBC，下次按 1 从趴地状态干净地重新站立。
      algorithm_.reset();
      prone_down_active_ = false;
      zeroFeedforward(prone_feedforward_);
      initialization_duration_ = kJointInitializationDuration;
      desired_.mode = ControlMode::JointPd;
      imu_log::print(
        imu_log::Level::Info, "[OpenDog][PRONE] 趴下完成，已回到趴卧姿态；可按 1 重新站立。\n");
    } else {
      imu_log::print(
        imu_log::Level::Info,
        "[OpenDog][INIT] 趴卧初始化完成：期望位置为趴卧姿态 (0,-0.203,-2.22)；实际位置仍以反馈为准。\n");
    }
    if (stand_up_pending_) {requestStandUp();}
  }
  return true;
}

/// 返回当前模式对应的日志标签。
const char * OpenDogRobotRunner::stateLabel() const noexcept
{
  switch (desired_.mode) {
    case ControlMode::JointPd: return "OPENDOG_JOINT_PD";
    case ControlMode::StandUp: return "OPENDOG_STAND_UP";
    case ControlMode::BalanceStand: return zero_calibration_active_ ? "OPENDOG_ZERO_CAL" : "OPENDOG_STANCE";
    case ControlMode::Locomotion: return "OPENDOG_LOCOMOTION";
    case ControlMode::ProneDown: return "OPENDOG_PRONE_DOWN";
    default: return "OPENDOG_PASSIVE";
  }
}

/// 打印下发时刻的数据新鲜度；初始化期间没有 MPC 策略，不打印。
void OpenDogRobotRunner::reportPublishTiming(
  double snapshot_age_ms, double motor_age_ms, bool ok) const
{
  if (initialization_active_) {return;}
  static robot_log::Throttle freshness_log(1000);
  if (!freshness_log.ready()) {return;}
  const auto & timing = algorithm_.timing();
  imu_log::print(
    imu_log::Level::Info,
    "[OpenDog][FRESHNESS] snapshot_age_ms=%.3f motor_feedback_max_age_ms=%.3f "
    "policy_age_at_publish_ms=%.3f horizon_at_publish_ms=%.3f "
    "publish_ok=%s monitor_only=true\n",
    snapshot_age_ms, motor_age_ms, timing.policy_age_ms + snapshot_age_ms,
    timing.horizon_remaining_ms - snapshot_age_ms, ok ? "true" : "false");
}
