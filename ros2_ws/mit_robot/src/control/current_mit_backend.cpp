#include "control/current_mit_backend.hpp"

#include "RobotRunner.hpp"

CurrentMitBackend::CurrentMitBackend(RobotRunner & runner)
: runner_(runner)
{}

bool CurrentMitBackend::configure()
{
  return true;
}

bool CurrentMitBackend::start()
{
  return true;
}

bool CurrentMitBackend::compute(
  const ControlInputSnapshot & input, ControlOutput & output)
{
  if (!runner_.updateHardwareFeedback(input.imu, input.joints, input.timestamp) ||
    !runner_.run())
  {
    output.valid = false;
    return false;
  }
  output.commands = runner_.jointCommands();
  output.estimate = runner_.stateEstimate();
  output.backend = AlgorithmBackendId::CurrentMit;
  output.valid = true;
  return true;
}

void CurrentMitBackend::stop() noexcept {}

void CurrentMitBackend::reset() noexcept
{
  runner_.reset();
}
