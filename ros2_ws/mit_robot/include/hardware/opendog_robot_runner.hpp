#pragma once

#include "control/opendog_algorithm.hpp"
#include "hardware/hardware_robot_runner.hpp"

// Owns OpenDog command state and its independent estimator / SQP-MPC / WBC.
// Deliberately has no RobotRunner, ControlFSM or MIT controller dependency.
class OpenDogRobotRunner final : public HardwareRobotRunner
{
public:
  OpenDogRobotRunner(const std::string & config_directory, float period);
  bool configure() override;
  bool start() override {return algorithm_.start();}
  bool compute(const ControlInputSnapshot & input, ControlOutput & output) override;
  void stop() noexcept override {algorithm_.stop();}
  void reset() noexcept override;
  void cancelPendingRequests() noexcept override {stand_up_pending_ = false;}
  void acceptCommands(
    const teleop::OperatorCommandArbiter & arbiter,
    const std::vector<teleop::OperatorCommand> & commands) override;
  bool handleCommands(
    teleop::OperatorCommandArbiter & arbiter,
    const ImuData<float> & imu, bool active) override;
  bool requestStandUp() override;
  const char * stateLabel() const noexcept override;
  void reportPublishTiming(double snapshot_age_ms, double motor_age_ms, bool ok) const override;
  const DesiredState<float> & desiredState() const noexcept {return desired_;}

private:
  void holdCurrentPose(ControlMode mode);
  bool computeJointInitialization(const ControlInputSnapshot & input, ControlOutput & output);
  OpenDogAlgorithm algorithm_;
  DesiredState<float> desired_;
  StateEstimate<float> estimate_;
  std::uint64_t motion_sequence_ = 0;
  std::uint64_t output_sequence_ = 0;
  std::array<Vec3<float>, kNumLegs> initial_joint_positions_{};
  float initialization_start_time_ = 0.0F;
  bool initialization_active_ = false;
  bool initialization_started_ = false;
  bool initialization_complete_ = false;
  bool stand_up_pending_ = false;
};
