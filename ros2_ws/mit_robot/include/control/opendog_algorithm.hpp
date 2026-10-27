#ifndef MYMIT_ROBOT_CONTROL_OPENDOG_ALGORITHM_HPP_
#define MYMIT_ROBOT_CONTROL_OPENDOG_ALGORITHM_HPP_

#include <memory>
#include <string>

#include "config/control_config.h"
#include "control/algorithm_backend.hpp"

class OpenDogAlgorithm final : public AlgorithmBackend
{
public:
  explicit OpenDogAlgorithm(
    std::string config_directory,
    double control_period = control_config::control_time_step);
  ~OpenDogAlgorithm() override;

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
