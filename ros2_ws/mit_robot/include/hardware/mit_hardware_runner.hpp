#pragma once

#include "RobotRunner.hpp"
#include "control/current_mit_backend.hpp"
#include "hardware/hardware_robot_runner.hpp"

// Only this hardware runner constructs the MIT estimator / ControlFSM / MPC / WBC.
class MitHardwareRunner final : public HardwareRobotRunner
{
public:
  explicit MitHardwareRunner(float period)
  : runner_(period), backend_(runner_) {}
  bool configure() override {return backend_.configure();}
  bool start() override {return backend_.start();}
  bool compute(const ControlInputSnapshot & input, ControlOutput & output) override;
  void stop() noexcept override {backend_.stop();}
  void reset() noexcept override;
  bool handleCommands(
    teleop::OperatorCommandArbiter & arbiter,
    const ImuData<float> & imu, bool active) override;
  void afterCompute(teleop::OperatorCommandArbiter & arbiter) override;
  void cancelPendingRequests() noexcept override {stand_up_pending_ = false;}
  bool requestStandUp() override {return runner_.requestStandUp();}
  const char * stateLabel() const noexcept override;
  const RobotRunner * mitRunner() const noexcept override {return &runner_;}

private:
  RobotRunner runner_;
  CurrentMitBackend backend_;
  FSM_StateName state_before_control_ = FSM_StateName::PASSIVE;
  bool stand_up_pending_ = false;
};
