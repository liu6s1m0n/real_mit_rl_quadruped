/*! @file control_types.hpp
 *  @brief 硬件桥/仿真桥与 OpenDog 控制算法之间交换的一帧输入和输出。
 *
 *  输入快照由接入层在每个控制周期组装，输出由算法填写后再经限幅和超时保护
 *  下发到电机。两者都只使用 robot_types.hpp 中与中间件无关的类型。
 */

#ifndef DM1_ROBOT_CONTROL_CONTROL_TYPES_HPP_
#define DM1_ROBOT_CONTROL_CONTROL_TYPES_HPP_

#include <array>
#include <cstdint>

#include "common/robot_types.hpp"

/**
 * @brief 一个控制周期的算法输入快照。
 *
 * joints 按 FR/FL/RR/RL 顺序存放四条腿的反馈。desired 由 runner 根据操作员命令
 * 写入，接入层无需填写。
 */
struct ControlInputSnapshot
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuData<float> imu{};                              ///< 最新 IMU 采样。
  std::array<JointState<float>, kNumLegs> joints{};  ///< 四条腿关节反馈。
  DesiredState<float> desired{};                     ///< 期望机身状态。
  float timestamp = 0.0F;                            ///< 快照时间，单调时钟，单位 s。
};

/**
 * @brief 一个控制周期的算法输出。
 *
 * valid 为 false 时 commands 不可下发，接入层应保持上一帧安全命令或进入保护。
 */
struct ControlOutput
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief OpenDog 求解器诊断信息，仅用于日志和测试。
   *
   * 模式号沿用 OCS2 legged_robot 的接触模式编码，15 表示四足全部支撑（STANCE）。
   * 足端力按 FR/FL/RR/RL 存放，每条腿为世界坐标系 (x, y, z)，单位 N。
   */
  struct OpenDogDiagnostics
  {
    bool valid = false;                                      ///< 本帧诊断是否已填写。
    std::size_t active_mode = 15;                            ///< 当前时刻生效的接触模式。
    std::size_t planned_mode = 15;                           ///< MPC 计划中的接触模式。
    std::array<bool, kNumLegs> contact{};                    ///< 各腿当前是否处于支撑相。
    std::array<bool, kNumLegs> contact_detected{};           ///< 触地检测结果（检测关闭时等于计划）。
    std::array<std::array<float, 3>, kNumLegs> force_des{};  ///< MPC 给出的期望足端力。
    std::array<std::array<float, 3>, kNumLegs> force_wbc{};  ///< WBC 求解后的足端力。
  };

  std::array<JointCommand<float>, kNumLegs> commands{};  ///< 四条腿关节命令。
  StateEstimate<float> estimate{};                       ///< 本周期状态估计结果。
  bool valid = false;                                    ///< false 时命令不可下发。
  OpenDogDiagnostics opendog{};                          ///< 求解器诊断。
};

#endif  // DM1_ROBOT_CONTROL_CONTROL_TYPES_HPP_
