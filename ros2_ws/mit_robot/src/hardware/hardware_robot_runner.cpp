#include "hardware/hardware_robot_runner.hpp"

#include <stdexcept>

#include "hardware/mit_hardware_runner.hpp"
#if defined(MYMIT_ENABLE_OPENDOG)
#include "hardware/opendog_robot_runner.hpp"
#endif

std::unique_ptr<HardwareRobotRunner> makeHardwareRobotRunner(
  AlgorithmBackendId backend, const std::string & config_directory, float period)
{
  switch (backend) {
    case AlgorithmBackendId::CurrentMit:
      return std::make_unique<MitHardwareRunner>(period);
    case AlgorithmBackendId::OpenDog:
#if defined(MYMIT_ENABLE_OPENDOG)
      return std::make_unique<OpenDogRobotRunner>(config_directory, period);
#else
      throw std::invalid_argument("此构建未启用 OpenDog");
#endif
  }
  throw std::invalid_argument("未知算法后端");
}
