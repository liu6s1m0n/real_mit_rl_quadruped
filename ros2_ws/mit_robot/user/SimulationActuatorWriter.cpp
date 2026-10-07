#include "SimulationActuatorWriter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "RobotRunner.hpp"

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
constexpr std::size_t kMotorFrameDivider = 4;
constexpr double kCommandTorqueLimitNm = 88.0;
constexpr double kContinuousTorqueNm = 30.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRatedSpeedRadS = 40.0 * 2.0 * kPi / 60.0;
constexpr double kNoLoadSpeedRadS = 60.0 * 2.0 * kPi / 60.0;

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
  // 只对输出机械功率的驱动方向应用转速降额；反向制动仍受 88 Nm 指令上限约束。
  if (force * velocity > 0.0) {
    symmetric_limit = std::min(symmetric_limit, motoringTorqueLimit(velocity));
  }
  return std::clamp(force, -symmetric_limit, symmetric_limit);
}
}  // namespace

SimulationActuatorWriter::SimulationActuatorWriter(const mjModel * model)
: model_(model)
{
  if (model_ == nullptr) {
    throw std::invalid_argument("SimulationActuatorWriter requires a MuJoCo model");
  }
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
  const RobotRunner & runner, mjData * data,
  bool motors_enabled) const
{
  if (data == nullptr) {throw std::invalid_argument("MuJoCo data is null");}
  mju_zero(data->qfrc_applied, model_->nv);
  const auto & commands = runner.jointCommands();
  if (!motors_enabled) {
    held_commands_ = {};
    write_count_ = 0;
  }
  const bool latch_commands = motors_enabled && write_count_ % kMotorFrameDivider == 0;
  if (motors_enabled) {++write_count_;}
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & command = commands[leg];
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const auto & address = addresses_[leg][joint];
      // The XML motor is intentionally driven with zero control input. The
      // controller's complete MIT-PD torque is written through qfrc_applied;
      // this keeps the simulation from adding a second position-servo path.
      data->ctrl[address.actuator] = 0.0;
      const Eigen::Index index = static_cast<Eigen::Index>(joint);
      auto & held = held_commands_[leg][joint];
      if (latch_commands) {
        held = HeldCommand{
          command.position_desired[index], command.velocity_desired[index],
          command.kp[index], command.kd[index], command.torque_feedforward[index],
          command.enabled};
      }
      if (!motors_enabled || !held.enabled) {continue;}
      const double torque = held.torque_feedforward +
        held.kp * (held.position - data->qpos[address.qpos]) +
        held.kd * (held.velocity - data->qvel[address.dof]);
      data->qfrc_applied[address.dof] = clampActuatorForce(
        model_, address.actuator, torque, data->qvel[address.dof]);
    }
  }
}
