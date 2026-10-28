#pragma once

#include <memory>
#include <string>
#include <vector>

#include "control/algorithm_backend.hpp"
#include "teleop/operator_command.hpp"

class RobotRunner;

// Hardware transport sees this boundary only. Each implementation owns its
// controller, command state and mode transitions; no controller is shared.
class HardwareRobotRunner : public AlgorithmBackend
{
public:
  virtual void acceptCommands(
    const teleop::OperatorCommandArbiter &,
    const std::vector<teleop::OperatorCommand> &) {}
  virtual bool handleCommands(
    teleop::OperatorCommandArbiter & arbiter,
    const ImuData<float> & imu, bool active) = 0;
  virtual void afterCompute(teleop::OperatorCommandArbiter &) {}
  virtual void cancelPendingRequests() noexcept {}
  virtual bool requestStandUp() = 0;
  virtual const char * stateLabel() const noexcept = 0;
  // Existing MIT-only observer remains source compatible.
  virtual const RobotRunner * mitRunner() const noexcept {return nullptr;}
  virtual void reportPublishTiming(double, double, bool) const {}
};

std::unique_ptr<HardwareRobotRunner> makeHardwareRobotRunner(
  AlgorithmBackendId backend, const std::string & config_directory, float period);
