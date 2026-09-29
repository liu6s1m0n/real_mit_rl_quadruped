/**
 * @file operator_command.hpp
 * @brief 共享的操作员命令和电机输出安全仲裁器。
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
  RotateClockwise,
  /// 原地对角小跑：周期 0.5 s、支撑占比 50%，FR+RL 与 FL+RR 交替抬起。
  MarchInPlace,
  /// 原地静态行走：周期 0.5 s、支撑占比 80%，按侧向序列 RR->FR->RL->FL
  /// 四腿依次抬起，始终至少三腿支撑。
  StaticWalkInPlace
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
// 终端按键是锁存式选择。空格是显式停止命令；
// 终端自动重复并不是安全心跳信号。
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
  // 运动命令是锁存式选择。该字段为未来的死手控制输入源保留，
  // 但终端输入和 GUI 输入都不会使用操作系统的按键重复频率作为 watchdog。
  bool watchdog = false;
};

/** 解码一个原始终端字节。未知字节不会生成命令。 */
std::optional<OperatorCommand> decodeKey(int key);

/** 将共享运动命令转换为控制器机体坐标系下的速度。 */
VelocityCommand velocityForMotion(Motion motion) noexcept;
VelocityCommand velocityForMotion(
  Motion motion, const VelocityProfile & profile) noexcept;

/**
 * @brief 供硬件和仿真共享的纯命令/状态边界。
 *
 * 它不会接触 CAN、MuJoCo 或 RobotRunner。拥有它的桥接层检查仲裁器状态后，
 * 才执行实际的副作用操作。
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

  /** 应用一条命令。 */
  void apply(const OperatorCommand & command) noexcept;

  /**
   * 应用一个控制周期内的命令批次。同一批次中的禁用、退出或键盘故障命令，
   * 优先级高于所有使能或运动命令。
   */
  void applyBatch(const std::vector<OperatorCommand> & commands) noexcept;

  /** 键盘线程死亡、断开或发生其他异步故障时，将状态锁定。 */
  void lock() noexcept;
  void markEnabled() noexcept;
  void markFault() noexcept;

  /** 让保持中的运动输入超时，但不改变电机输出状态。 */
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
