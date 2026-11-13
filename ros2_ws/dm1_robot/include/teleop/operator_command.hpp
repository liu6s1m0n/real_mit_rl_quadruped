/**
 * @file operator_command.hpp
 * @brief 共享的操作员命令和电机输出安全仲裁器。
 */
#ifndef DM1_ROBOT_TELEOP_OPERATOR_COMMAND_HPP_
#define DM1_ROBOT_TELEOP_OPERATOR_COMMAND_HPP_

#include "common/control_config.h"
#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

namespace teleop
{

/// 电机输出状态机：Locked → Enabling → Enabled，任何禁用/故障回到 Locked。
enum class MotorOutputState : std::uint8_t
{
  Locked,    ///< 电机输出锁定，不下发命令。
  Enabling,  ///< 已收到使能请求，等待桥接层完成使能。
  Enabled    ///< 电机已使能，允许下发控制命令。
};

/// 操作员命令类型。
enum class CommandType : std::uint8_t
{
  EnableMotors,    ///< 使能电机。
  DisableMotors,   ///< 失能电机。
  StandUp,         ///< 站立。
  ProneDown,       ///< 趴下。
  Motion,          ///< 运动选择，具体动作见 Motion。
  Stop,            ///< 停止运动，保持当前姿态。
  Quit,            ///< 退出程序。
  Help,            ///< 打印帮助。
  KeyboardFailure,  ///< 键盘输入线程故障。
  ZeroCalibration   ///< 零点标定动作（站立后有效）：四脚不动，机身依次升降、俯仰、横滚、平移、偏航。
};

/// 运动命令。
enum class Motion : std::uint8_t
{
  Forward,                 ///< 慢速前进。
  ForwardFast,             ///< 快速前进。
  Backward,                ///< 后退。
  Left,                    ///< 左移。
  Right,                   ///< 右移。
  RotateCounterClockwise,  ///< 逆时针（向左）转向。
  RotateClockwise,         ///< 顺时针（向右）转向。
  /// 原地对角小跑，使用 OpenDog trot 步态。
  MarchInPlace,
  /// 一次 LF/RH 对角单步，使用完整 MPC/WBC 链。
  TwoLegStand,
  /// 一次 RF/LH 对角单步，仅交换支撑腿组合。
  TwoLegStandReverse,
};

/// 机身坐标系下的速度命令。
struct VelocityCommand
{
  float forward = 0.0F;  ///< 前向速度，m/s。
  float lateral = 0.0F;  ///< 横向速度（左为正），m/s。
  float yaw = 0.0F;      ///< 偏航角速度（逆时针为正），rad/s。
};

constexpr float kForwardVelocity = control_config::motion::forward;          ///< 默认慢速前进速度。
constexpr float kFastForwardVelocity = control_config::motion::forward_fast;  ///< 默认快速前进速度。
constexpr float kLateralVelocity = control_config::motion::lateral;          ///< 默认横移速度。
constexpr float kYawRate = control_config::motion::yaw_rate;                 ///< 默认转向角速度。
/// 终端按键是锁存式选择。空格是显式停止命令；
/// 终端自动重复并不是安全心跳信号。
constexpr std::chrono::milliseconds kMotionWatchdog{control_config::motion::watchdog_ms};

/// 各运动档位的速度大小（均为正数，方向由 Motion 决定）。
struct VelocityProfile
{
  float forward = kForwardVelocity;                   ///< 慢速前进，m/s。
  float forward_fast = kFastForwardVelocity;          ///< 快速前进，m/s。
  float backward = control_config::motion::backward;  ///< 后退，m/s。
  float lateral = kLateralVelocity;                   ///< 横移，m/s。
  float yaw = kYawRate;                               ///< 转向，rad/s。
};

/// 一条操作员命令，来自终端键盘、GUI 窗口或测试输入。
struct OperatorCommand
{
  CommandType type = CommandType::Help;                 ///< 命令类型。
  Motion motion = Motion::Forward;                      ///< type 为 Motion 时的具体动作。
  std::chrono::steady_clock::time_point received_at{};  ///< 接收时间。
  std::uint64_t sequence = 0;                           ///< 输入源内的递增序号。
  /// 运动命令是锁存式选择。该字段为未来的死手控制输入源保留，
  /// 但终端输入和 GUI 输入都不会使用操作系统的按键重复频率作为 watchdog。
  bool watchdog = false;
};

/** 解码一个原始终端字节。未知字节不会生成命令。 */
std::optional<OperatorCommand> decodeKey(int key);

/** 将共享运动命令转换为控制器机体坐标系下的速度。 */
VelocityCommand velocityForMotion(Motion motion) noexcept;
/** 同上，使用指定的速度档位。 */
VelocityCommand velocityForMotion(
  Motion motion, const VelocityProfile & profile) noexcept;

/**
 * @brief 供硬件和仿真共享的纯命令/状态边界。
 *
 * 它不会接触 CAN、MuJoCo 或控制算法。拥有它的桥接层检查仲裁器状态后，
 * 才执行实际的副作用操作。
 */
class OperatorCommandArbiter
{
public:
  MotorOutputState state() const noexcept {return state_;}             ///< 当前电机输出状态。
  bool quitRequested() const noexcept {return quit_requested_;}         ///< 是否已请求退出。
  bool motionActive() const noexcept {return motion_.has_value();}      ///< 是否有锁存的运动命令。
  Motion motion() const noexcept {return motion_.value_or(Motion::Forward);}  ///< 当前锁存的运动。

  /// take* 系列：读取并清除对应的一次性请求标志。
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

  bool takeZeroCalibrationRequest() noexcept
  {
    const bool requested = zero_calibration_requested_;
    zero_calibration_requested_ = false;
    return requested;
  }

  bool takeHelpRequest() noexcept
  {
    const bool requested = help_requested_;
    help_requested_ = false;
    return requested;
  }

  /// 清除锁存的运动，并产生一次停止请求。
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
  /** 桥接层完成电机使能后调用，状态从 Enabling 进入 Enabled。 */
  void markEnabled() noexcept;

  /** 让保持中的运动输入超时，但不改变电机输出状态。 */
  void expireMotion(std::chrono::steady_clock::time_point now) noexcept;

private:
  MotorOutputState state_ = MotorOutputState::Locked;     ///< 电机输出状态。
  std::optional<Motion> motion_;                          ///< 锁存的运动命令。
  std::chrono::steady_clock::time_point last_motion_at_{};  ///< 最近一次运动命令时间。
  bool motion_watchdog_enabled_ = true;                   ///< 当前运动是否受 watchdog 超时约束。
  bool enable_requested_ = false;                         ///< 待处理的使能请求。
  bool stand_up_requested_ = false;                       ///< 待处理的站立请求。
  bool prone_down_requested_ = false;                     ///< 待处理的趴下请求。
  bool stop_requested_ = false;                           ///< 待处理的停止请求。
  bool help_requested_ = false;                           ///< 待处理的帮助请求。
  bool zero_calibration_requested_ = false;               ///< 待处理的零点标定请求。
  bool quit_requested_ = false;                           ///< 已请求退出。
};

}  // namespace teleop

#endif  // DM1_ROBOT_TELEOP_OPERATOR_COMMAND_HPP_
