#include "hardware/opendog_robot_runner.hpp"

#include <algorithm>

#include "common/console_log.hpp"

namespace
{
// DM1 hardware startup duration from RobotControlParameters; copied here so
// OpenDog startup does not depend on the MIT controller or its configuration.
constexpr float kJointInitializationDuration = 1.2F;
}

OpenDogRobotRunner::OpenDogRobotRunner(const std::string & config_directory, float period)
: algorithm_(config_directory, period)
{}

bool OpenDogRobotRunner::configure()
{
  if (!algorithm_.configure()) {return false;}
  desired_ = {};
  desired_.body_position_world.z() = algorithm_.commandSettings().standing_height;
  desired_.valid = true;
  return true;
}

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
}

void OpenDogRobotRunner::holdCurrentPose(ControlMode mode)
{
  desired_.mode = mode;
  desired_.body_velocity_world.setZero();
  desired_.body_angular_velocity.setZero();
  desired_.body_position_world.head<2>() = estimate_.position_world.head<2>();
  desired_.body_rpy.z() = estimate_.rpy.z();
}

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

void OpenDogRobotRunner::acceptCommands(
  const teleop::OperatorCommandArbiter & arbiter,
  const std::vector<teleop::OperatorCommand> & commands)
{
  for (const auto & command : commands) {
    if (command.type == teleop::CommandType::Motion && arbiter.motionActive() &&
      command.motion == arbiter.motion()) {++motion_sequence_;}
  }
}

bool OpenDogRobotRunner::handleCommands(
  teleop::OperatorCommandArbiter & arbiter, const ImuData<float> & imu, bool active)
{
  if (arbiter.takeStopRequest() && !initialization_active_) {
    holdCurrentPose(ControlMode::BalanceStand);
  }
  const bool stand_up = arbiter.takeStandUpRequest();
  if (stand_up) {
    // legged_keyboard.cpp: key '1' clears trot_requested_ and publishes zero
    // velocity together with /set_stance. Preserve that command ordering here.
    arbiter.stopMotion();
    arbiter.takeStopRequest();
    requestStandUp();
  }
  if (arbiter.takeProneDownRequest()) {
    imu_log::print(imu_log::Level::Info, "OpenDog 原目标轨迹未定义趴下命令。\n");
  }
  if (!stand_up && arbiter.motionActive()) {
    if (initialization_active_) {
      arbiter.stopMotion();
      arbiter.takeStopRequest();
      imu_log::print(imu_log::Level::Warning, "OpenDog 运动请求已取消：先完成回零并按 1 站立。\n");
      return active;
    }
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

bool OpenDogRobotRunner::compute(const ControlInputSnapshot & input, ControlOutput & output)
{
  if (initialization_active_) {return computeJointInitialization(input, output);}
  auto snapshot = input;
  desired_.timestamp = input.timestamp;
  snapshot.desired = desired_;
  if (!algorithm_.compute(snapshot, output)) {return false;}
  for (auto & command : output.commands) {
    command.sequence = output_sequence_ + 1;
  }
  ++output_sequence_;
  estimate_ = output.estimate;
  return true;
}

bool OpenDogRobotRunner::computeJointInitialization(
  const ControlInputSnapshot & input, ControlOutput & output)
{
  // Ported from RobotRunner::prepareJointInitialization(): capture measured q,
  // cubic smoothstep to motor zero, zero feedforward. No optimizer in this path.
  if (!initialization_started_) {
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      initial_joint_positions_[leg] = input.joints[leg].position;
    }
    initialization_start_time_ = input.timestamp;
    initialization_started_ = true;
  }
  const float phase = std::clamp(
    (input.timestamp - initialization_start_time_) / kJointInitializationDuration, 0.0F, 1.0F);
  const float blend = phase * phase * (3.0F - 2.0F * phase);
  const float blend_rate = 6.0F * phase * (1.0F - phase) / kJointInitializationDuration;
  output = {};
  output.backend = AlgorithmBackendId::OpenDog;
  output.valid = true;
  output.estimate.orientation_world_from_body = input.imu.orientation_world_from_body;
  output.estimate.rotation_world_from_body =
    input.imu.orientation_world_from_body.toRotationMatrix();
  output.estimate.timestamp = input.timestamp;
  const auto & settings = algorithm_.commandSettings();
  ++output_sequence_;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    auto & command = output.commands[leg];
    command.leg = static_cast<LegId>(leg);
    command.position_desired = (1.0F - blend) * initial_joint_positions_[leg];
    command.velocity_desired = -blend_rate * initial_joint_positions_[leg];
    command.kp.setConstant(settings.joint_kp);
    command.kd.setConstant(settings.joint_kd);
    command.timestamp = input.timestamp;
    command.sequence = output_sequence_;
    command.enabled = true;
  }
  if (phase >= 1.0F && !initialization_complete_) {
    initialization_complete_ = true;
    imu_log::print(
      imu_log::Level::Info,
      "[OpenDog][INIT] 回零轨迹完成：12 路期望位置/速度为零；实际位置仍以反馈为准。\n");
    if (stand_up_pending_) {requestStandUp();}
  }
  return true;
}

const char * OpenDogRobotRunner::stateLabel() const noexcept
{
  switch (desired_.mode) {
    case ControlMode::JointPd: return "OPENDOG_JOINT_PD";
    case ControlMode::StandUp: return "OPENDOG_STAND_UP";
    case ControlMode::BalanceStand: return "OPENDOG_STANCE";
    case ControlMode::Locomotion: return "OPENDOG_LOCOMOTION";
    default: return "OPENDOG_PASSIVE";
  }
}

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
