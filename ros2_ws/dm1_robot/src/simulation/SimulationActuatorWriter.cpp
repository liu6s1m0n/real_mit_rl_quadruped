/*! @file SimulationActuatorWriter.cpp
 *  @brief SimulationActuatorWriter 实现：MIT PD+前馈力矩计算和电机特性限幅。
 */
#include "common/control_config.h"
#include "simulation/SimulationActuatorWriter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace
{
constexpr std::array<std::array<const char *, kJointsPerLeg>, kNumLegs>
kJointNames{{
  {{"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"}},
  {{"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"}},
  {{"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"}},
  {{"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"}}
}};
constexpr std::array<std::array<const char *, kJointsPerLeg>, kNumLegs>
kActuatorNames{{
  {{"FR_hip", "FR_thigh", "FR_calf"}},
  {{"FL_hip", "FL_thigh", "FL_calf"}},
  {{"RR_hip", "RR_thigh", "RR_calf"}},
  {{"RL_hip", "RL_thigh", "RL_calf"}}
}};
constexpr double kCommandTorqueLimitNm = control_config::hardware::torque_limit;  ///< 指令力矩上限，N*m。
constexpr double kContinuousTorqueNm = 30.0;                  ///< 额定（连续）力矩，N*m。
constexpr double kPi = 3.14159265358979323846;
constexpr double kRatedSpeedRadS = 40.0 * 2.0 * kPi / 60.0;   ///< 额定转速 40 rpm。
constexpr double kNoLoadSpeedRadS = 60.0 * 2.0 * kPi / 60.0;  ///< 空载转速 60 rpm。

/// 驱动方向的转速-力矩降额曲线：额定转速内从指令上限线性降到额定力矩，之后降到空载为 0。
double motoringTorqueLimit(double speed)
{
  speed = std::abs(speed);
  if (speed <= kRatedSpeedRadS) {
    const double blend = speed / kRatedSpeedRadS;
    return kCommandTorqueLimitNm + blend *
      (kContinuousTorqueNm - kCommandTorqueLimitNm);
  }
  const double blend = std::clamp(
    (speed - kRatedSpeedRadS) / (kNoLoadSpeedRadS - kRatedSpeedRadS), 0.0, 1.0);
  return kContinuousTorqueNm * (1.0 - blend);
}

/// 按指令上限、XML forcerange 和转速降额对关节力矩限幅。
double clampActuatorForce(
  const mjModel * model, int actuator, double force, double velocity)
{
  double symmetric_limit = kCommandTorqueLimitNm;
  if (model->actuator_forcelimited[actuator] != 0) {
    const double minimum = model->actuator_forcerange[2 * actuator];
    const double maximum = model->actuator_forcerange[2 * actuator + 1];
    symmetric_limit = std::min(
      symmetric_limit, std::max(std::abs(minimum), std::abs(maximum)));
  }
  // 只对输出机械功率的驱动方向应用转速降额；反向制动仍受默认 30 Nm 指令上限约束。
  if (force * velocity > 0.0) {
    symmetric_limit = std::min(symmetric_limit, motoringTorqueLimit(velocity));
  }
  return std::clamp(force, -symmetric_limit, symmetric_limit);
}
}  // namespace

SimulationActuatorWriter::SimulationActuatorWriter(const mjModel * model)
: model_(model)
{
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const int joint_id = mj_name2id(model_, mjOBJ_JOINT, kJointNames[leg][joint]);
      const int actuator_id =
        mj_name2id(model_, mjOBJ_ACTUATOR, kActuatorNames[leg][joint]);
      if (joint_id < 0 || actuator_id < 0) {
        throw std::runtime_error("MuJoCo joint or actuator mapping is incomplete");
      }
      addresses_[leg][joint] = JointAddress{
        model_->jnt_qposadr[joint_id], model_->jnt_dofadr[joint_id], actuator_id};
    }
  }
}

void SimulationActuatorWriter::write(
  const ControlOutput & output, mjData * data,
  bool motors_enabled) const
{
  mju_zero(data->qfrc_applied, model_->nv);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & command = output.commands[leg];
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const auto & address = addresses_[leg][joint];
      // XML 中的 motor 有意保持零控制输入。控制器完整的 MIT-PD 力矩经
      // qfrc_applied 写入，避免仿真中出现第二条位置伺服路径。
      data->ctrl[address.actuator] = 0.0;
      const Eigen::Index index = static_cast<Eigen::Index>(joint);
      if (!motors_enabled || !command.enabled) {continue;}
      const double torque = command.torque_feedforward[index] +
        command.kp[index] * (command.position_desired[index] - data->qpos[address.qpos]) +
        command.kd[index] * (command.velocity_desired[index] - data->qvel[address.dof]);
      data->qfrc_applied[address.dof] = clampActuatorForce(
        model_, address.actuator, torque, data->qvel[address.dof]);
    }
  }
}
