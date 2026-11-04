/*! @file opendog_algorithm.hpp
 *  @brief OpenDog 控制算法封装：卡尔曼状态估计 + SQP-MPC + 加权 WBC。
 *
 *  算法只通过 ControlInputSnapshot / ControlOutput 与外部交换数据，
 *  不直接访问硬件或仿真器。所有 OCS2 / Pinocchio 依赖都隐藏在 Impl 中。
 */

#ifndef DM1_ROBOT_CONTROL_OPENDOG_ALGORITHM_HPP_
#define DM1_ROBOT_CONTROL_OPENDOG_ALGORITHM_HPP_

#include <memory>
#include <string>

#include "common/control_config.h"
#include "control/control_types.hpp"
#include "teleop/operator_command.hpp"

/**
 * @brief OpenDog 原始控制链路的独立实现。
 *
 * 首次 compute() 时同步求出初始策略，随后启动后台线程按 mpcDesiredFrequency
 * 持续求解 MPC；控制线程每周期只做状态估计、策略插值和 WBC。
 */
class OpenDogAlgorithm final
{
public:
  /**
   * @param config_directory 存放 opendog.yaml、task/reference/gait 文件和 URDF 的目录。
   * @param control_period 首帧状态估计使用的控制周期，单位 s。
   */
  explicit OpenDogAlgorithm(
    std::string config_directory,
    double control_period = control_config::control_time_step);
  /// 析构时停止 MPC 线程。
  ~OpenDogAlgorithm();

  /// 转发 OpenDog 专用的动作请求（如对角踏步）；锁存的请求保持原序号，避免重复触发。
  void setMotionRequest(teleop::Motion motion, std::uint64_t sequence);

  /// 最近一个控制周期的耗时统计，仅用于监测，不参与控制。
  struct Timing
  {
    double compute_ms = 0.0;            ///< compute() 总耗时。
    double wbc_ms = 0.0;                ///< WBC 求解耗时。
    double mpc_ms = 0.0;                ///< 后台线程最近一次 MPC 求解耗时。
    double policy_age_ms = 0.0;         ///< 当前使用的 MPC 策略相对当前时刻的滞后。
    double horizon_remaining_ms = 0.0;  ///< 策略剩余可用的预测时长。
    std::size_t active_mode = 15;       ///< 当前接触模式，15 为四足支撑。
    std::size_t planned_mode = 15;      ///< 策略插值得到的计划接触模式。
    bool initial_policy = false;        ///< 本周期是否在等待首个策略。
  };
  /// 返回最近一个周期的耗时统计。
  const Timing & timing() const noexcept;

  /// 从 OpenDog reference/task 配置文件读取的命令参数，与 MIT 预设无关。
  struct CommandSettings
  {
    float standing_height = 0.0F;  ///< 站立机身高度（comHeight），单位 m。
    float joint_kp = 0.0F;         ///< 关节位置增益，N*m/rad。
    float joint_kd = 0.0F;         ///< 关节阻尼增益，N*m/(rad/s)。
    /// 键盘运动对应的速度档位，可被 task 文件 hardware_command 覆盖。
    teleop::VelocityProfile velocity{0.18F, 0.36F, 0.30F, 0.20F, 0.30F};
  };
  /// 返回 configure() 之后的命令参数。
  const CommandSettings & commandSettings() const noexcept;

  /// 加载配置并构建估计器、MPC、WBC；配置文件缺失或 ROS 未初始化时返回 false。
  bool configure();
  /// 执行一个控制周期；未配置或 IMU 无效时 output.valid 置 false 并返回 false。
  bool compute(const ControlInputSnapshot & input, ControlOutput & output);
  /// 停止 MPC 线程并复位求解器、参考轨迹和步态，下一次 compute() 重新求初始策略。
  void stop() noexcept;
  /// 在 stop() 基础上额外复位状态估计器和 WBC，用于重新站立。
  void reset() noexcept;

private:
  struct Impl;                  ///< OCS2 / Pinocchio 相关的私有实现。
  std::unique_ptr<Impl> impl_;
};

#endif  // DM1_ROBOT_CONTROL_OPENDOG_ALGORITHM_HPP_
