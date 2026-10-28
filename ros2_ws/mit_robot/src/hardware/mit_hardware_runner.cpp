#include "hardware/mit_hardware_runner.hpp"

#include "common/console_log.hpp"

namespace
{
const char * fsmStateLabel(FSM_StateName state) noexcept
{
  switch (state) {
    case FSM_StateName::PASSIVE: return "PASSIVE";
    case FSM_StateName::JOINT_PD: return "JOINT_PD";
    case FSM_StateName::STAND_UP: return "STAND_UP";
    case FSM_StateName::RECOVERY_STAND: return "RECOVERY_STAND";
    case FSM_StateName::BALANCE_STAND: return "BALANCE_STAND";
    case FSM_StateName::LOCOMOTION: return "LOCOMOTION";
    case FSM_StateName::LIE_DOWN: return "LIE_DOWN";
    default: return "INVALID";
  }
}

}

void MitHardwareRunner::reset() noexcept
{
  backend_.reset();
  stand_up_pending_ = false;
}

bool MitHardwareRunner::handleCommands(
  teleop::OperatorCommandArbiter & arbiter,
  const ImuData<float> &, bool active)
{
  if (arbiter.takeStopRequest()) {
    if (active) {
      runner_.setLocomotionVelocityCommand(0.0F, 0.0F, 0.0F);
      if (runner_.standingReady()) {runner_.setControlMode(ControlMode::BalanceStand);}
    }
  }
  if (arbiter.takeStandUpRequest()) {
    if (!active) {
      runner_.prepareForMotionControl();
      active = true;
    }
    stand_up_pending_ = true;
  }
  if (arbiter.takeProneDownRequest()) {
    if (!active || !runner_.requestProneDown()) {
      imu_log::print(
        imu_log::Level::Warning,
        "趴下请求被拒绝：请按 1 并等待进入 BalanceStand。\n");
    } else {
      arbiter.stopMotion();
      runner_.setLocomotionVelocityCommand(0.0F, 0.0F, 0.0F);
      stand_up_pending_ = false;
      imu_log::print(
        imu_log::Level::Info,
        "趴下请求已接受：MPC/WBC 下降控制已激活。\n");
    }
  }
  if (arbiter.motionActive()) {
    const auto motion = arbiter.motion();
    const bool two_leg_stand_experiment =
      motion == teleop::Motion::TwoLegStand ||
      motion == teleop::Motion::TwoLegStandReverse;
    if (!active || (!runner_.standingReady() && !two_leg_stand_experiment)) {
      imu_log::print(
        imu_log::Level::Warning,
        "运动请求被拒绝：请先按 1 并等待进入 BalanceStand。\n");
      arbiter.stopMotion();
      if (active) {
        runner_.setLocomotionVelocityCommand(0.0F, 0.0F, 0.0F);
      }
    } else {
      const auto velocity = teleop::velocityForMotion(motion);
      runner_.setLocomotionVelocityCommand(
        velocity.forward, velocity.lateral, velocity.yaw);
      runner_.setLocomotionGait(
        two_leg_stand_experiment ?
        (motion == teleop::Motion::TwoLegStandReverse ?
        GaitType::TWO_LEG_STAND_REVERSE : GaitType::TWO_LEG_STAND) :
        GaitType::TROT);
      runner_.setControlMode(ControlMode::Locomotion);
    }
  }
  return active;
}

bool MitHardwareRunner::compute(const ControlInputSnapshot & input, ControlOutput & output)
{
  state_before_control_ = runner_.currentStateName();
  return backend_.compute(input, output);
}

void MitHardwareRunner::afterCompute(teleop::OperatorCommandArbiter & arbiter)
{
  if (arbiter.motionActive() && state_before_control_ == FSM_StateName::LOCOMOTION &&
    runner_.currentStateName() == FSM_StateName::BALANCE_STAND)
  {
    arbiter.stopMotion();
    runner_.setLocomotionVelocityCommand(0.0F, 0.0F, 0.0F);
    imu_log::print(
      imu_log::Level::Warning,
      "Locomotion 安全回退已取消当前运动请求；保持 BalanceStand，需重新按运动键。\n");
  }
  if (stand_up_pending_ && runner_.requestStandUp()) {
    stand_up_pending_ = false;
    imu_log::print(imu_log::Level::Info, "站立请求已接受。\n");
  }
}

const char * MitHardwareRunner::stateLabel() const noexcept
{
  return fsmStateLabel(runner_.currentStateName());
}
