#ifndef MYMIT_ROBOT_CONTROL_ALGORITHM_BACKEND_HPP_
#define MYMIT_ROBOT_CONTROL_ALGORITHM_BACKEND_HPP_

#include <array>
#include <cstdint>

#include "model/robot_types.hpp"

enum class AlgorithmBackendId : std::uint8_t
{
  CurrentMit = 0,
  OpenDog = 1
};

struct ControlInputSnapshot
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuData<float> imu{};
  std::array<JointState<float>, kNumLegs> joints{};
  std::array<FootContactState<float>, kNumLegs> contacts{};
  DesiredState<float> desired{};
  float timestamp = 0.0F;
};

struct ControlOutput
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::array<JointCommand<float>, kNumLegs> commands{};
  StateEstimate<float> estimate{};
  AlgorithmBackendId backend = AlgorithmBackendId::CurrentMit;
  bool valid = false;
};

class AlgorithmBackend
{
public:
  virtual ~AlgorithmBackend() = default;

  virtual bool configure() = 0;
  virtual bool start() = 0;
  virtual bool compute(const ControlInputSnapshot & input, ControlOutput & output) = 0;
  virtual void stop() noexcept = 0;
  virtual void reset() noexcept = 0;
};

#endif  // MYMIT_ROBOT_CONTROL_ALGORITHM_BACKEND_HPP_
