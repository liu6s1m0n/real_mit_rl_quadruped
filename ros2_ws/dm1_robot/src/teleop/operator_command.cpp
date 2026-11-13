/*! @file operator_command.cpp
 *  @brief 按键解码、运动速度映射和操作员命令仲裁实现。
 */
#include "teleop/operator_command.hpp"

namespace teleop
{

/**
 * 按键映射：U 使能，0 失能，1 站立，2 原地踏步，3 零点标定动作，4 对角单步，P 趴下，
 * W/S 前进/后退，A/D 左移/右移，Q/E 左转/右转，空格停止，Esc 退出，H 帮助。
 */
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
  } else if (key == '3') {
    command.type = CommandType::ZeroCalibration;
  } else if (key == '4') {
    command.type = CommandType::Motion;
    command.motion = Motion::TwoLegStandReverse;
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

/// 使用默认速度档位。
VelocityCommand velocityForMotion(Motion motion) noexcept
{
  return velocityForMotion(motion, VelocityProfile{});
}

/// 原地踏步和对角单步只改变步态，不产生机身速度。
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
    case Motion::TwoLegStand: return {0.0F, 0.0F, 0.0F};
    case Motion::TwoLegStandReverse: return {0.0F, 0.0F, 0.0F};
  }
  return {};
}

/// 失能/退出/键盘故障立即锁定；站立、趴下、运动和停止只在电机已使能时生效。
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
    case CommandType::ZeroCalibration:
      if (state_ == MotorOutputState::Enabled) {zero_calibration_requested_ = true;}
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

  // 安全类命令优先：只要批次中出现，就锁定并丢弃同批次其他命令。
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

/// 回到锁定状态并清除所有待处理请求，同时产生一次停止请求。
void OperatorCommandArbiter::lock() noexcept
{
  state_ = MotorOutputState::Locked;
  motion_.reset();
  motion_watchdog_enabled_ = true;
  enable_requested_ = false;
  stand_up_requested_ = false;
  prone_down_requested_ = false;
  zero_calibration_requested_ = false;
  stop_requested_ = true;
}

/// 只有处于 Enabling 时才进入 Enabled，避免锁定后被迟到的使能确认覆盖。
void OperatorCommandArbiter::markEnabled() noexcept
{
  if (state_ == MotorOutputState::Enabling) {state_ = MotorOutputState::Enabled;}
}

/// 只对开启 watchdog 的运动命令生效；锁存式命令不会超时。
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
