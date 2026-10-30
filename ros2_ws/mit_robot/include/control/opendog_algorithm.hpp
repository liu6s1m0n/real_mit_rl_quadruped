#ifndef MYMIT_ROBOT_CONTROL_OPENDOG_ALGORITHM_HPP_
#define MYMIT_ROBOT_CONTROL_OPENDOG_ALGORITHM_HPP_

#include <memory>
#include <string>

#include "config/control_config.h"
#include "control/algorithm_backend.hpp"
#include "teleop/operator_command.hpp"

class OpenDogAlgorithm final : public AlgorithmBackend
{
public:
  explicit OpenDogAlgorithm(
    std::string config_directory,
    double control_period = control_config::control_time_step);
  ~OpenDogAlgorithm() override;

  // OpenDog-only command event; a latched request retains its sequence.
  void setMotionRequest(teleop::Motion motion, std::uint64_t sequence);

  struct Timing
  {
    double compute_ms = 0.0;
    double wbc_ms = 0.0;
    double mpc_ms = 0.0;
    double policy_age_ms = 0.0;
    double horizon_remaining_ms = 0.0;
    std::size_t active_mode = 15;
    std::size_t planned_mode = 15;
    bool command_finite = true;
    double max_position_jump = 0.0;
    double max_velocity_jump = 0.0;
    double max_torque_jump = 0.0;
    bool initial_policy = false;
  };
  const Timing & timing() const noexcept;

  // Loaded from OpenDog reference/task files, independent of MIT presets.
  struct CommandSettings
  {
    float standing_height = 0.0F;
    float joint_kp = 0.0F;
    float joint_kd = 0.0F;
    teleop::VelocityProfile velocity{0.18F, 0.36F, 0.30F, 0.20F, 0.30F};
  };
  const CommandSettings & commandSettings() const noexcept;

  bool configure() override;
  bool start() override;
  bool compute(const ControlInputSnapshot & input, ControlOutput & output) override;
  void stop() noexcept override;
  void reset() noexcept override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

#endif  // MYMIT_ROBOT_CONTROL_OPENDOG_ALGORITHM_HPP_
