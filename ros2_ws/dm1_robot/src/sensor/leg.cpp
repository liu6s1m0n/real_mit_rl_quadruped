/*! @file leg.cpp
 *  @brief SimLeg 实现：从 MuJoCo 读取单腿关节反馈。
 */
#include "sensor/leg.hpp"

#include <stdexcept>
#include <string>

/// 按 "<腿>_<关节>_joint" 命名规则查找三个铰链关节。
SimLeg::SimLeg(const mjModel * model, const mjData * data, LegId leg_id)
: data_(data), leg_id_(leg_id)
{
  static constexpr std::array<const char *, kNumLegs> kPrefixes{"FR", "FL", "RR", "RL"};
  static constexpr std::array<const char *, kJointsPerLeg> kJoints{"hip", "thigh", "calf"};
  for (std::size_t index = 0; index < kJointsPerLeg; ++index) {
    const std::string name = std::string(kPrefixes[static_cast<std::size_t>(leg_id)]) +
      "_" + kJoints[index] + "_joint";
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, name.c_str());
    if (joint_id < 0 || model->jnt_type[joint_id] != mjJNT_HINGE) {
      throw std::invalid_argument("MuJoCo hinge joint not found: " + name);
    }
    joint_addresses_[index] = {model->jnt_qposadr[joint_id], model->jnt_dofadr[joint_id]};
  }
}

/// 读取当前仿真时刻的关节位置、速度和执行器力，时间戳取 mjData::time。
JointState<float> SimLeg::read() const
{
  JointState<float> result;
  result.leg = leg_id_;
  for (std::size_t index = 0; index < kJointsPerLeg; ++index) {
    const auto & address = joint_addresses_[index];
    const auto i = static_cast<Eigen::Index>(index);
    result.position[i] = static_cast<float>(data_->qpos[address.position]);
    result.velocity[i] = static_cast<float>(data_->qvel[address.velocity]);
    // qfrc_actuator 与 qvel 共用自由度索引。
    result.torque_estimate[i] = static_cast<float>(data_->qfrc_actuator[address.velocity]);
  }
  result.timestamp = static_cast<float>(data_->time);
  result.valid = true;
  return result;
}
