/**
 * @file SimulationActuatorWriter.hpp
 * @brief GUI 和测试共用的 MuJoCo 执行器写入路径。
 */
#ifndef DM1_ROBOT_SIMULATION_SIMULATION_ACTUATOR_WRITER_HPP_
#define DM1_ROBOT_SIMULATION_SIMULATION_ACTUATOR_WRITER_HPP_

#include <array>
#include <cstddef>

#include <mujoco/mujoco.h>

#include "common/robot_types.hpp"
#include "control/control_types.hpp"

/** 以统一的执行器语义把控制器关节命令写入 MuJoCo。 */
class SimulationActuatorWriter
{
public:
  /// 按 DM1 关节名和执行器名查找地址；映射不完整时抛出异常。
  explicit SimulationActuatorWriter(const mjModel * model);

  /**
   * @brief 把一帧控制输出写入 MuJoCo。
   *
   * XML 中的 motor 控制量保持为 0，完整的前馈+PD 力矩通过 qfrc_applied 施加，
   * 只保留一条执行器路径。motors_enabled 为 false 时不施加任何力矩。
   */
  void write(
    const ControlOutput & output, mjData * data,
    bool motors_enabled = true) const;

private:
  /// 单个关节在 MuJoCo 中的地址。
  struct JointAddress
  {
    int qpos = -1;      ///< mjData::qpos 下标。
    int dof = -1;       ///< 自由度下标（qvel/qfrc_applied）。
    int actuator = -1;  ///< 执行器下标（ctrl）。
  };

  /// 按 [腿][关节] 存放的地址表。
  using JointAddresses =
    std::array<std::array<JointAddress, kJointsPerLeg>, kNumLegs>;

  const mjModel * model_;       ///< MuJoCo 模型。
  JointAddresses addresses_{};  ///< 12 个关节的地址。
};

#endif  // DM1_ROBOT_SIMULATION_SIMULATION_ACTUATOR_WRITER_HPP_
