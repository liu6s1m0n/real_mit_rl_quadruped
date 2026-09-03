// DM1 真机入口。ROS 2 工程结构保持不变；底层设备调用集中在
// Dm1HardwareDriver.cpp，控制算法集中在 RobotRunner。
#include <array>
#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "Dm1HardwareDriver.hpp"
#include "HardwareBridge.hpp"

namespace
{
std::atomic_bool g_stop_requested{false};

void handleSignal(int) noexcept
{
  g_stop_requested.store(true);
}

struct Arguments
{
  std::string calibration_path;
  std::string can_interface = "can0";
  std::string imu_device = "/dev/ttyUSB0";
  float control_time_step = 0.002F;
  float zero_tolerance_rad = 0.05F;
  bool enable_output = false;
  bool stand_up = false;
};

[[noreturn]] void usageError(const std::string & message)
{
  throw std::invalid_argument(
    message +
    "\nusage: mymit_robot_hardware --calibration FILE [--can can0] "
    "[--imu /dev/ttyUSB0] [--control-dt 0.002] "
    "[--zero-tolerance 0.05] "
    "[--enable-output] [--stand-up]");
}

Arguments parseArguments(int argc, char ** argv)
{
  Arguments result;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    auto requireValue = [&](const char * option) -> std::string {
        if (++i >= argc) {usageError(std::string("missing value for ") + option);}
        return argv[i];
      };
    if (argument == "--calibration") {
      result.calibration_path = requireValue("--calibration");
    } else if (argument == "--can") {
      result.can_interface = requireValue("--can");
    } else if (argument == "--imu") {
      result.imu_device = requireValue("--imu");
    } else if (argument == "--control-dt") {
      result.control_time_step = std::stof(requireValue("--control-dt"));
    } else if (argument == "--zero-tolerance") {
      result.zero_tolerance_rad = std::stof(requireValue("--zero-tolerance"));
    } else if (argument == "--enable-output") {
      result.enable_output = true;
    } else if (argument == "--stand-up") {
      result.stand_up = true;
    } else if (argument == "--help" || argument == "-h") {
      usageError("DM1 hardware bridge options");
    } else {
      usageError("unknown argument: " + argument);
    }
  }
  if (result.calibration_path.empty()) {usageError("--calibration is required");}
  if (result.stand_up && !result.enable_output) {
    usageError("--stand-up requires --enable-output");
  }
  return result;
}

dm1_hardware::Dm1MitInterface::CalibrationArray loadCalibration(
  const std::string & path)
{
  constexpr std::array<const char *, kNumJoints> expected_names{
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf"};

  std::ifstream input(path);
  if (!input) {throw std::runtime_error("cannot open calibration file: " + path);}
  dm1_hardware::Dm1MitInterface::CalibrationArray result{};
  std::string line;
  std::size_t index = 0;
  while (std::getline(input, line)) {
    const auto first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos || line[first] == '#') {continue;}
    if (index >= result.size()) {
      throw std::runtime_error("calibration file contains more than 12 motors");
    }
    std::istringstream values(line);
    std::string name;
    unsigned int id = 0;
    int direction = 0;
    float zero = 0.0F;
    std::string extra;
    if (!(values >> name >> id >> direction >> zero) || (values >> extra)) {
      throw std::runtime_error(
        "calibration row must be: JOINT_NAME MOTOR_ID DIRECTION ZERO_RAD");
    }
    if (name != expected_names[index]) {
      throw std::runtime_error(
        "calibration joint order mismatch at row " + std::to_string(index + 1));
    }
    if (id == 0 || id > 255) {
      throw std::runtime_error("motor ID must be in [1,255]");
    }
    result[index] = dm1_hardware::MotorCalibration{
      static_cast<std::uint8_t>(id), direction, zero};
    ++index;
  }
  if (index != result.size()) {
    throw std::runtime_error("calibration file must contain exactly 12 motors");
  }
  return result;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Arguments arguments = parseArguments(argc, argv);
    const auto calibration = loadCalibration(arguments.calibration_path);
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    Dm1HardwareDriver driver(arguments.can_interface, arguments.imu_device);
    HardwareBridge::Options options;
    options.control_time_step = arguments.control_time_step;
    options.startup_zero_tolerance_rad = arguments.zero_tolerance_rad;
    options.enable_output = arguments.enable_output;
    options.request_stand_up = arguments.stand_up;
    HardwareBridge bridge(driver, calibration, options);

    if (!arguments.enable_output) {
      std::fprintf(
        stderr,
        "DM1 hardware bridge is read-only; add --enable-output only after "
        "checking motor IDs, directions and mechanical zeros.\n");
    }
    return bridge.run(g_stop_requested);
  } catch (const std::exception & error) {
    std::fprintf(stderr, "DM1 hardware startup failed: %s\n", error.what());
    return 1;
  }
}
