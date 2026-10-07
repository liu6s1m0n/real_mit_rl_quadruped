/**
 * @file SimulationActuatorWriter.hpp
 * @brief The MuJoCo actuator write path used by the GUI and sim2sim runner.
 */
#ifndef MYMIT_ROBOT_USER_SIMULATION_ACTUATOR_WRITER_HPP_
#define MYMIT_ROBOT_USER_SIMULATION_ACTUATOR_WRITER_HPP_

#include <array>
#include <cstddef>

#include <mujoco/mujoco.h>

#include "model/robot_types.hpp"

class RobotRunner;

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
    bool motors_enabled = true) const;

private:
  struct JointAddress
  {
    int qpos = -1;
    int dof = -1;
    int actuator = -1;
  };

  struct HeldCommand
  {
    double position = 0.0;
    double velocity = 0.0;
    double kp = 0.0;
    double kd = 0.0;
    double torque_feedforward = 0.0;
    bool enabled = false;
  };

  using JointAddresses =
    std::array<std::array<JointAddress, kJointsPerLeg>, kNumLegs>;

  const mjModel * model_ = nullptr;
  JointAddresses addresses_{};
  mutable std::array<std::array<HeldCommand, kJointsPerLeg>, kNumLegs>
  held_commands_{};
  mutable std::size_t write_count_ = 0;
};

#endif  // MYMIT_ROBOT_USER_SIMULATION_ACTUATOR_WRITER_HPP_
