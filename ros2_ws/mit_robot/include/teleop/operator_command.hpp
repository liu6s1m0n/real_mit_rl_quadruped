/**
 * @file operator_command.hpp
 * @brief Shared operator commands and the motor-output safety arbiter.
 */
#ifndef MYMIT_ROBOT_TELEOP_OPERATOR_COMMAND_HPP_
#define MYMIT_ROBOT_TELEOP_OPERATOR_COMMAND_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

namespace teleop
{

enum class MotorOutputState : std::uint8_t
{
  Locked,
  Enabling,
  Enabled,
  Fault
};

enum class CommandType : std::uint8_t
{
  EnableMotors,
  DisableMotors,
  StandUp,
  ProneDown,
  Motion,
  Stop,
  Quit,
  Help,
  KeyboardFailure
};

enum class Motion : std::uint8_t
{
  Forward,
  ForwardFast,
  Backward,
  Left,
  Right,
  RotateCounterClockwise,
  RotateClockwise
};

struct VelocityCommand
{
  float forward = 0.0F;
  float lateral = 0.0F;
  float yaw = 0.0F;
};

constexpr float kForwardVelocity = 0.18F;
constexpr float kFastForwardVelocity = 0.36F;
constexpr float kBackwardVelocity = -0.30F;
constexpr float kLateralVelocity = 0.20F;
constexpr float kYawRate = 0.30F;
// A terminal key is a latched selection.  Space is the explicit stop command;
// terminal auto-repeat is not a safety heartbeat.
constexpr std::chrono::milliseconds kMotionWatchdog{500};

struct VelocityProfile
{
  float forward = kForwardVelocity;
  float forward_fast = kFastForwardVelocity;
  float backward = 0.30F;
  float lateral = kLateralVelocity;
  float yaw = kYawRate;
};

struct OperatorCommand
{
  CommandType type = CommandType::Help;
  Motion motion = Motion::Forward;
  std::chrono::steady_clock::time_point received_at{};
  std::uint64_t sequence = 0;
  // Motion commands are latched selections.  The field remains available for
  // a future dead-man producer, but neither terminal input nor GUI input uses
  // the operating system's key-repeat cadence as a watchdog.
  bool watchdog = false;
};

/** Decode one raw terminal byte. Unknown bytes intentionally produce no command. */
std::optional<OperatorCommand> decodeKey(int key);

/** Convert a shared motion command to the controller's body-frame velocity. */
VelocityCommand velocityForMotion(Motion motion) noexcept;
VelocityCommand velocityForMotion(
  Motion motion, const VelocityProfile & profile) noexcept;

/**
 * @brief Pure command/state boundary shared by hardware and simulation.
 *
 * It never touches CAN, MuJoCo, or RobotRunner. The owning bridge performs the
 * actual side effect after checking this arbiter's state.
 */
class OperatorCommandArbiter
{
public:
  MotorOutputState state() const noexcept {return state_;}
  bool quitRequested() const noexcept {return quit_requested_;}
  bool motionActive() const noexcept {return motion_.has_value();}
  Motion motion() const noexcept {return motion_.value_or(Motion::Forward);}

  bool takeEnableRequest() noexcept
  {
    const bool requested = enable_requested_;
    enable_requested_ = false;
    return requested;
  }

  bool takeStandUpRequest() noexcept
  {
    const bool requested = stand_up_requested_;
    stand_up_requested_ = false;
    return requested;
  }

  bool takeProneDownRequest() noexcept
  {
    const bool requested = prone_down_requested_;
    prone_down_requested_ = false;
    return requested;
  }

  bool takeStopRequest() noexcept
  {
    const bool requested = stop_requested_;
    stop_requested_ = false;
    return requested;
  }

  bool takeHelpRequest() noexcept
  {
    const bool requested = help_requested_;
    help_requested_ = false;
    return requested;
  }

  void stopMotion() noexcept
  {
    motion_.reset();
    motion_watchdog_enabled_ = true;
    stop_requested_ = true;
  }

  /** Apply one command. */
  void apply(const OperatorCommand & command) noexcept;

  /**
   * Apply one control-cycle batch. Any disable, quit, or keyboard-failure
   * command wins over every enable/motion command in the same batch.
   */
  void applyBatch(const std::vector<OperatorCommand> & commands) noexcept;

  /** Lock on a dead/disconnected keyboard or another asynchronous fault. */
  void lock() noexcept;
  void markEnabled() noexcept;
  void markFault() noexcept;

  /** Expire held-motion input without changing the motor output state. */
  void expireMotion(std::chrono::steady_clock::time_point now) noexcept;

private:
  MotorOutputState state_ = MotorOutputState::Locked;
  std::optional<Motion> motion_;
  std::chrono::steady_clock::time_point last_motion_at_{};
  bool motion_watchdog_enabled_ = true;
  bool enable_requested_ = false;
  bool stand_up_requested_ = false;
  bool prone_down_requested_ = false;
  bool stop_requested_ = false;
  bool help_requested_ = false;
  bool quit_requested_ = false;
};

}  // namespace teleop

#endif  // MYMIT_ROBOT_TELEOP_OPERATOR_COMMAND_HPP_
