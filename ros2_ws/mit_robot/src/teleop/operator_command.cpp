#include "teleop/operator_command.hpp"

namespace teleop
{

std::optional<OperatorCommand> decodeKey(int key)
{
  OperatorCommand command;
  command.received_at = std::chrono::steady_clock::now();
  // 保持这个分支为显式决策树。这样既便于审查终端按键映射，
  // 也可以避免在现有枚举值之间插入 CommandType 成员时，
  // 产生依赖位置的跳转表 ABI 风险。
  if (key == 'u' || key == 'U') {
    command.type = CommandType::EnableMotors;
  } else if (key == '0') {
    command.type = CommandType::DisableMotors;
  } else if (key == '1') {
    command.type = CommandType::StandUp;
  } else if (key == '2') {
    command.type = CommandType::Motion;
    command.motion = Motion::MarchInPlace;
  } else if (key == 'p' || key == 'P') {
    command.type = CommandType::ProneDown;
  } else if (key == 'w' || key == 'W') {
    command.type = CommandType::Motion;
    command.motion = Motion::Forward;
  } else if (key == 's' || key == 'S') {
    command.type = CommandType::Motion;
    command.motion = Motion::Backward;
  } else if (key == 'a' || key == 'A') {
    command.type = CommandType::Motion;
    command.motion = Motion::Left;
  } else if (key == 'd' || key == 'D') {
    command.type = CommandType::Motion;
    command.motion = Motion::Right;
  } else if (key == 'q' || key == 'Q') {
    command.type = CommandType::Motion;
    command.motion = Motion::RotateCounterClockwise;
  } else if (key == 'e' || key == 'E') {
    command.type = CommandType::Motion;
    command.motion = Motion::RotateClockwise;
  } else if (key == ' ') {
    command.type = CommandType::Stop;
  } else if (key == 27) {
    command.type = CommandType::Quit;
  } else if (key == 'h' || key == 'H') {
    command.type = CommandType::Help;
  } else {
    return std::nullopt;
  }
  return command;
}

VelocityCommand velocityForMotion(Motion motion) noexcept
{
  return velocityForMotion(motion, VelocityProfile{});
}

VelocityCommand velocityForMotion(
  Motion motion, const VelocityProfile & profile) noexcept
{
  switch (motion) {
    case Motion::Forward: return {profile.forward, 0.0F, 0.0F};
    case Motion::ForwardFast: return {profile.forward_fast, 0.0F, 0.0F};
    case Motion::Backward: return {-profile.backward, 0.0F, 0.0F};
    case Motion::Left: return {0.0F, profile.lateral, 0.0F};
    case Motion::Right: return {0.0F, -profile.lateral, 0.0F};
    case Motion::RotateCounterClockwise: return {0.0F, 0.0F, profile.yaw};
    case Motion::RotateClockwise: return {0.0F, 0.0F, -profile.yaw};
    case Motion::MarchInPlace: return {0.0F, 0.0F, 0.0F};
  }
  return {};
}

void OperatorCommandArbiter::apply(const OperatorCommand & command) noexcept
{
  switch (command.type) {
    case CommandType::DisableMotors:
      lock();
      return;
    case CommandType::Quit:
      quit_requested_ = true;
      lock();
      return;
    case CommandType::KeyboardFailure:
      // 这是输入源故障，不是电机或控制器故障。它会让仲裁器回到可恢复的锁定状态；
      // 拥有该仲裁器的模块仍可以继续接受 GUI 命令，或在输入源修复后重新使能。
      // 物理/硬件故障仍然使用 markFault() 处理。
      lock();
      return;
    case CommandType::EnableMotors:
      if (state_ == MotorOutputState::Locked) {
        state_ = MotorOutputState::Enabling;
        enable_requested_ = true;
      }
      return;
    case CommandType::StandUp:
      if (state_ == MotorOutputState::Enabled) {stand_up_requested_ = true;}
      return;
    case CommandType::ProneDown:
      if (state_ == MotorOutputState::Enabled) {prone_down_requested_ = true;}
      return;
    case CommandType::Motion:
      if (state_ == MotorOutputState::Enabled) {
        motion_ = command.motion;
        if (command.motion == Motion::MarchInPlace) {
          selected_gait_motion_ = command.motion;
        }
        last_motion_at_ = command.received_at;
        motion_watchdog_enabled_ = command.watchdog;
      }
      return;
    case CommandType::Stop:
      if (state_ == MotorOutputState::Enabled) {
        motion_.reset();
        motion_watchdog_enabled_ = true;
        stop_requested_ = true;
      }
      return;
    case CommandType::Help:
      help_requested_ = true;
      return;
  }
}

void OperatorCommandArbiter::applyBatch(
  const std::vector<OperatorCommand> & commands) noexcept
{
  bool disable_requested = false;
  bool quit_requested = false;
  bool keyboard_failure = false;
  for (const auto & command : commands) {
    disable_requested = disable_requested || command.type == CommandType::DisableMotors;
    quit_requested = quit_requested || command.type == CommandType::Quit;
    keyboard_failure = keyboard_failure || command.type == CommandType::KeyboardFailure;
  }

  if (quit_requested) {
    this->quit_requested_ = true;
    lock();
    return;
  }
  if (keyboard_failure) {
    lock();
    return;
  }
  if (disable_requested) {
    lock();
    return;
  }
  for (const auto & command : commands) {
    apply(command);
  }
}

void OperatorCommandArbiter::lock() noexcept
{
  state_ = MotorOutputState::Locked;
  motion_.reset();
  motion_watchdog_enabled_ = true;
  enable_requested_ = false;
  stand_up_requested_ = false;
  prone_down_requested_ = false;
  stop_requested_ = true;
}

void OperatorCommandArbiter::markEnabled() noexcept
{
  if (state_ == MotorOutputState::Enabling) {state_ = MotorOutputState::Enabled;}
}

void OperatorCommandArbiter::markFault() noexcept
{
  state_ = MotorOutputState::Fault;
  motion_.reset();
  motion_watchdog_enabled_ = true;
  enable_requested_ = false;
  stand_up_requested_ = false;
  prone_down_requested_ = false;
  stop_requested_ = true;
}

void OperatorCommandArbiter::expireMotion(
  std::chrono::steady_clock::time_point now) noexcept
{
  if (motion_.has_value() && motion_watchdog_enabled_ &&
    now - last_motion_at_ > kMotionWatchdog)
  {
    motion_.reset();
    stop_requested_ = true;
  }
}

}  // namespace teleop
