#ifndef MYMIT_ROBOT_CONTROL_CURRENT_MIT_BACKEND_HPP_
#define MYMIT_ROBOT_CONTROL_CURRENT_MIT_BACKEND_HPP_

#include "control/algorithm_backend.hpp"

class RobotRunner;

class CurrentMitBackend final : public AlgorithmBackend
{
public:
  explicit CurrentMitBackend(RobotRunner & runner);

  bool configure() override;
  bool start() override;
  bool compute(const ControlInputSnapshot & input, ControlOutput & output) override;
  void stop() noexcept override;
  void reset() noexcept override;

private:
  RobotRunner & runner_;
};

#endif  // MYMIT_ROBOT_CONTROL_CURRENT_MIT_BACKEND_HPP_
