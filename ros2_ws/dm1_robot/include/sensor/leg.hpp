/*! @file leg.hpp
 *  @brief 从 MuJoCo 仿真读取单腿关节反馈。
 */

#ifndef DM1_ROBOT_SENSOR_LEG_HPP_
#define DM1_ROBOT_SENSOR_LEG_HPP_

#include <array>

#include <mujoco/mujoco.h>

#include "common/robot_types.hpp"

/**
 * @brief 从 MuJoCo 的 qpos、qvel 和 qfrc_actuator 读取一条腿的关节反馈。
 *
 * 构造时按 DM1 关节名（如 FR_hip_joint）查找三个铰链关节并缓存地址。
 * torque_estimate 表示 MuJoCo 当前计算出的关节执行器广义力。
 */
class SimLeg
{
public:
  /// 按 leg_id 查找该腿三个关节；关节不存在或不是铰链关节时抛出异常。
  SimLeg(const mjModel * model, const mjData * data, LegId leg_id);

  /** @brief 读取三个关节（Hip、thigh、calf）的位置、速度、力矩和仿真时间。 */
  JointState<float> read() const;

private:
  /// 单个关节在 MuJoCo 状态数组中的地址。
  struct JointAddress
  {
    int position = -1;  ///< 关节位置在 mjData::qpos 中的地址。
    int velocity = -1;  ///< 关节速度和广义力在自由度数组中的地址。
  };

  const mjData * data_;                                        ///< 绑定的仿真数据。
  LegId leg_id_;                                               ///< 本腿编号。
  std::array<JointAddress, kJointsPerLeg> joint_addresses_{};  ///< Hip/thigh/calf 的地址。
};

#endif  // DM1_ROBOT_SENSOR_LEG_HPP_
