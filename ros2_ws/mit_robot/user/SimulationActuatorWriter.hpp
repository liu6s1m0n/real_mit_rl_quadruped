/**
 * @file SimulationActuatorWriter.hpp
 * @brief The single MuJoCo actuator write path used by the GUI and regression tests.
 */
#ifndef MYMIT_ROBOT_USER_SIMULATION_ACTUATOR_WRITER_HPP_
#define MYMIT_ROBOT_USER_SIMULATION_ACTUATOR_WRITER_HPP_

#include <array>
#include <cstddef>

#include <mujoco/mujoco.h>

#include "model/robot_types.hpp"

class RobotRunner;

/** Counts target positions that are at a configured DM1 joint limit. */
std::size_t countSimulationTargetJointLimitHits(const RobotRunner & runner);

/** Writes RobotRunner joint commands with the same actuator semantics everywhere. */
class SimulationActuatorWriter
{
public:
  explicit SimulationActuatorWriter(const mjModel * model);

  /**
   * @brief Write one controller frame to MuJoCo.
   *
   * XML motor controls are held at zero while the complete feedforward/PD torque
   * is applied through qfrc_applied. This leaves one actuator path only.
   */
  void write(
    const RobotRunner & runner, mjData * data,
    std::array<std::size_t, kNumLegs> & torque_speed_saturation_by_leg) const;

private:
  struct JointAddress
  {
    int qpos = -1;
    int dof = -1;
    int actuator = -1;
  };

  using JointAddresses =
    std::array<std::array<JointAddress, kJointsPerLeg>, kNumLegs>;

  const mjModel * model_ = nullptr;
  JointAddresses addresses_{};
};

#endif  // MYMIT_ROBOT_USER_SIMULATION_ACTUATOR_WRITER_HPP_
