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

double clampActuatorForce(
  const mjModel * model, const mjData * data, int actuator, int dof, double force,
  bool * saturated)
{
  (void)data;
  (void)dof;
  if (saturated != nullptr) {*saturated = false;}
  if (model->actuator_forcelimited[actuator] == 0) {return force;}
  const double minimum = model->actuator_forcerange[2 * actuator];
  const double maximum = model->actuator_forcerange[2 * actuator + 1];
  const double symmetric_limit = std::max(std::abs(minimum), std::abs(maximum));
  // MuJoCo's actuator forcerange is already the configured motor limit.  A
  // second linear speed derating here could drive the available torque to zero
  // during a normal swing, which presents as one step followed by a pause and
  // is not part of the controller/training contract.
  if (saturated != nullptr && std::abs(force) > symmetric_limit + 1.0e-6) {
    *saturated = true;
  }
  return std::clamp(force, -symmetric_limit, symmetric_limit);
}
}  // namespace

std::size_t countSimulationTargetJointLimitHits(const RobotRunner & runner)
{
  constexpr float kLimitTolerance = 1.0e-4F;
  std::size_t count = 0;
  const auto & commands = runner.jointCommands();
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & joints = runner.quadruped().leg(static_cast<LegId>(leg)).joints;
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const auto index = static_cast<Eigen::Index>(joint);
      const float target = commands[leg].position_desired(index);
      if (std::abs(target - static_cast<float>(joints.lower_limit(index))) <=
        kLimitTolerance ||
        std::abs(target - static_cast<float>(joints.upper_limit(index))) <=
        kLimitTolerance)
      {
        ++count;
      }
    }
  }
  return count;
}

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
  std::array<std::size_t, kNumLegs> & torque_speed_saturation_by_leg,
  bool motors_enabled) const
{
  if (data == nullptr) {throw std::invalid_argument("MuJoCo data is null");}
  mju_zero(data->qfrc_applied, model_->nv);
  torque_speed_saturation_by_leg.fill(0);
  const auto & commands = runner.jointCommands();
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & command = commands[leg];
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const auto & address = addresses_[leg][joint];
      // The XML motor is intentionally driven with zero control input. The
      // controller's complete MIT-PD torque is written through qfrc_applied;
      // this keeps the simulation from adding a second position-servo path.
      data->ctrl[address.actuator] = 0.0;
      if (!motors_enabled || !command.enabled) {continue;}
      const Eigen::Index index = static_cast<Eigen::Index>(joint);
      const double torque = command.torque_feedforward[index] +
        command.kp[index] * (command.position_desired[index] - data->qpos[address.qpos]) +
        command.kd[index] * (command.velocity_desired[index] - data->qvel[address.dof]);
      bool saturated = false;
      data->qfrc_applied[address.dof] = clampActuatorForce(
        model_, data, address.actuator, address.dof, torque, &saturated);
      if (saturated) {++torque_speed_saturation_by_leg[leg];}
    }
  }
}
