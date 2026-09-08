// DM1 真机入口。ROS 2 工程结构保持不变；底层设备调用集中在
// Dm1HardwareDriver.cpp，控制算法集中在 RobotRunner。
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "Dm1HardwareDriver.hpp"
#include "HardwareBridge.hpp"
#include "sensor/imu_log.hpp"

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
  std::string can0 = "can0";
  std::string can1 = "can1";
  std::string imu_device = "auto";
  float control_time_step = 0.002F;
  float zero_tolerance_rad = 0.05F;
  bool enable_output = false;
  bool stand_up = false;
  bool set_zero = false;
  bool imu_only = false;
  bool motor_only = false;
  std::optional<dm1_hardware::MotorAddress> poll_feedback_address;
};

[[noreturn]] void usageError(const std::string & message)
{
  throw std::invalid_argument(
          message +
          "\nusage: hardware_main --calibration FILE [--can0 can0] [--can1 can1] "
          "[--imu auto|DEVICE] [--control-dt 0.002] "
          "[--zero-tolerance 0.05] (automatic-zero upper bound; direct start is 0.02) "
          "[--enable-output] [--stand-up] [--set-zero] [--imu-only | --motor-only] "
          "[--poll-feedback BUS CAN_ID]");
}

std::string resolveImuDevice(const std::string & requested)
{
  if (requested != "auto") {return requested;}

  const std::filesystem::path by_id_directory("/dev/serial/by-id");
  std::error_code error;
  std::vector<std::string> candidates;
  for (std::filesystem::directory_iterator iterator(by_id_directory, error), end;
    !error && iterator != end; iterator.increment(error))
  {
    const std::string name = iterator->path().filename().string();
    if (name.find("DM-IMU") != std::string::npos) {
      candidates.push_back(iterator->path().string());
    }
  }
  std::sort(candidates.begin(), candidates.end());
  if (candidates.size() == 1) {
    imu_log::print(
      imu_log::Level::Info, "[DM IMU] auto-selected serial device: %s\n",
      candidates.front().c_str());
    return candidates.front();
  }
  if (candidates.empty()) {
    throw std::runtime_error(
            "unable to auto-detect DM IMU under /dev/serial/by-id; "
            "use --imu DEVICE");
  }
  throw std::runtime_error(
          "multiple DM IMU devices detected; select one explicitly with --imu DEVICE");
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
    } else if (argument == "--can0" || argument == "--can") {
      result.can0 = requireValue(argument.c_str());
    } else if (argument == "--can1") {
      result.can1 = requireValue("--can1");
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
    } else if (argument == "--set-zero") {
      result.set_zero = true;
    } else if (argument == "--imu-only") {
      result.imu_only = true;
    } else if (argument == "--motor-only") {
      result.motor_only = true;
    } else if (argument == "--poll-feedback") {
      const std::string bus_name = requireValue("--poll-feedback BUS");
      const std::string can_id_text = requireValue("--poll-feedback CAN_ID");
      std::uint8_t bus = 0;
      if (bus_name == "can0" || bus_name == "0") {
        bus = 0;
      } else if (bus_name == "can1" || bus_name == "1") {
        bus = 1;
      } else {
        usageError("--poll-feedback BUS must be can0/0 or can1/1");
      }
      try {
        const auto parsed = std::stoul(can_id_text, nullptr, 0);
        if (parsed == 0 || parsed > 0x0F) {
          usageError("--poll-feedback physical CAN_ID must be in [1,0x0f]");
        }
        result.poll_feedback_address = dm1_hardware::MotorAddress{
          bus, static_cast<std::uint16_t>(parsed), 0};
      } catch (const std::exception &) {
        usageError("--poll-feedback CAN_ID must be an integer");
      }
    } else if (argument == "--help" || argument == "-h") {
      usageError("DM1 hardware bridge options");
    } else {
      usageError("unknown argument: " + argument);
    }
  }
  if (!result.imu_only && result.calibration_path.empty()) {
    usageError("--calibration is required unless --imu-only is used");
  }
  if ((result.imu_only || result.motor_only) && (result.enable_output || result.stand_up)) {
    usageError("read-only modes cannot be combined with --enable-output or --stand-up");
  }
  if (result.imu_only && result.motor_only) {
    usageError("--imu-only and --motor-only are mutually exclusive");
  }
  if (result.poll_feedback_address.has_value() && !result.motor_only) {
    usageError("--poll-feedback requires --motor-only");
  }
  if (result.stand_up && !result.enable_output) {
    usageError("--stand-up requires --enable-output");
  }
  if (result.set_zero && (result.imu_only || result.motor_only)) {
    usageError("--set-zero requires the complete hardware mode");
  }
  if (result.set_zero && (result.enable_output || result.stand_up)) {
    usageError("--set-zero cannot be combined with --enable-output or --stand-up");
  }
  return result;
}

int runMotorOnly(
  const std::string & can0, const std::string & can1,
  const dm1_hardware::Dm1MitInterface::CalibrationArray & calibration,
  float control_time_step,
  const std::optional<dm1_hardware::MotorAddress> & poll_feedback_address)
{
  DmMotorDriver motors(can0, can1, calibration);
  if (!motors.open()) {
    throw std::runtime_error("unable to open SocketCAN buses: " + can0 + ", " + can1);
  }
  imu_log::print(
    imu_log::Level::Warning,
    poll_feedback_address.has_value() ?
    "Motor-only mode active; periodically sending zero-gain MIT feedback polls; "
    "no IMU, enable or zero commands will be sent. Press Ctrl+C to stop.\n" :
    "Motor-only mode active; no IMU, enable, zero or MIT commands will be sent. Press Ctrl+C to stop.\n");
  std::optional<dm1_hardware::MitFrame> poll_frame;
  if (poll_feedback_address.has_value()) {
    dm1_hardware::MitFrame frame{};
    frame.bus = poll_feedback_address->bus;
    frame.can_id = poll_feedback_address->can_id;
    // A zero-gain MIT frame is used as a periodic feedback poll.  It does not
    // enable the motor or request a non-zero position/velocity/torque command.
    poll_frame = frame;
    imu_log::print(
      imu_log::Level::Info,
      "Periodically sending zero-gain MIT feedback polls to bus%u CAN ID 0x%03x; "
      "no motor enable command will be sent.\n",
      static_cast<unsigned int>(frame.bus), static_cast<unsigned int>(frame.can_id));
  }
  const auto start = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(std::max(0.01F, control_time_step)));
  auto next_cycle = start;
  auto last_poll_report = start;
  std::uint64_t last_report_sequence = 0;
  std::uint64_t sent_poll_count = 0;
  while (!g_stop_requested.load()) {
    next_cycle += period;
    if (poll_frame.has_value() && !motors.sendMit(*poll_frame)) {
      throw std::runtime_error("failed to send periodic MIT feedback poll");
    }
    if (poll_frame.has_value()) {
      ++sent_poll_count;
    }
    const double now_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
    dm1_hardware::Dm1MitInterface::FeedbackArray feedback{};
    const bool fresh = motors.latest(feedback, now_s);
    if (poll_feedback_address.has_value()) {
      const auto report_now = std::chrono::steady_clock::now();
      const auto report_age = std::chrono::duration<double>(report_now - last_poll_report);
      if (report_age.count() >= 0.1) {
        const dm1_hardware::MotorFeedback * polled_motor = nullptr;
        for (const auto & motor : feedback) {
          if (motor.bus == poll_feedback_address->bus &&
            motor.can_id == poll_feedback_address->can_id)
          {
            polled_motor = &motor;
            break;
          }
        }
        if (polled_motor == nullptr || polled_motor->sequence == 0) {
          imu_log::print(
            imu_log::Level::Warning,
            "[POLL] bus%u CAN=0x%03x sent=%llu status=NO_FEEDBACK seq=0\n",
            static_cast<unsigned int>(poll_feedback_address->bus),
            static_cast<unsigned int>(poll_feedback_address->can_id),
            static_cast<unsigned long long>(sent_poll_count));
        } else {
          const auto sequence_delta = polled_motor->sequence - last_report_sequence;
          imu_log::print(
            sequence_delta > 0 ? imu_log::Level::Info : imu_log::Level::Warning,
            "[POLL] bus%u CAN=0x%03x status=%s seq=%llu (+%llu/%.2fs) "
            "q=%+.4f dq=%+.4f tau=%+.4f sent=%llu\n",
            static_cast<unsigned int>(polled_motor->bus),
            static_cast<unsigned int>(polled_motor->can_id),
            sequence_delta > 0 ? "UPDATING" : "NO_UPDATE",
            static_cast<unsigned long long>(polled_motor->sequence),
            static_cast<unsigned long long>(sequence_delta), report_age.count(),
            polled_motor->position, polled_motor->velocity, polled_motor->torque,
            static_cast<unsigned long long>(sent_poll_count));
          last_report_sequence = polled_motor->sequence;
        }
        last_poll_report = report_now;
      }
    } else {
      imu_log::print(
        fresh ? imu_log::Level::Info : imu_log::Level::Warning,
        "motor feedback %s:\n", fresh ? "fresh" : "stale/incomplete");
      for (std::size_t i = 0; i < feedback.size(); ++i) {
        const auto & motor = feedback[i];
        imu_log::print(
          imu_log::Level::Info,
          "  %02zu bus%u can=0x%03x seq=%llu q=%+.4f dq=%+.4f tau=%+.4f\n",
          i, static_cast<unsigned int>(motor.bus), static_cast<unsigned int>(motor.can_id),
          static_cast<unsigned long long>(motor.sequence), motor.position,
          motor.velocity, motor.torque);
      }
    }
    std::this_thread::sleep_until(next_cycle);
  }
  motors.close();
  return 0;
}

int runImuOnly(const std::string & device, float control_time_step)
{
  HardwareImu imu(device, 921600);
  if (!imu.open()) {
    throw std::runtime_error("unable to open IMU serial port: " + device);
  }

  imu_log::print(
    imu_log::Level::Info,
    "IMU-only mode active; press Ctrl+C to stop. No motor CAN commands will be sent.\n");
  const auto start = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(std::max(0.001F, control_time_step)));
  auto next_cycle = start;
  while (!g_stop_requested.load()) {
    next_cycle += period;
    ImuData<float> sample;
    const float timestamp = static_cast<float>(
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
    // readAt() is non-blocking: the receive thread owns the serial port and this
    // loop only copies the newest validated sample.
    static_cast<void>(imu.readAt(sample, timestamp));
    std::this_thread::sleep_until(next_cycle);
  }
  imu.close();
  return 0;
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
    std::string bus_name;
    std::string can_id_text;
    std::string master_id_text;
    unsigned int can_id = 0;
    unsigned int master_id = 0;
    int direction = 0;
    float zero = 0.0F;
    std::string extra;
    if (!(values >> name >> bus_name >> can_id_text >> master_id_text >> direction >> zero) ||
      (values >> extra))
    {
      throw std::runtime_error(
              "calibration row must be: JOINT_NAME BUS CAN_ID MASTER_ID DIRECTION ZERO_RAD");
    }
    if (name != expected_names[index]) {
      throw std::runtime_error(
              "calibration joint order mismatch at row " + std::to_string(index + 1));
    }
    std::uint8_t bus = 0;
    if (bus_name == "can0" || bus_name == "0") {
      bus = 0;
    } else if (bus_name == "can1" || bus_name == "1") {
      bus = 1;
    } else {
      throw std::runtime_error("motor bus must be can0/0 or can1/1");
    }
    try {
      can_id = std::stoul(can_id_text, nullptr, 0);
      master_id = std::stoul(master_id_text, nullptr, 0);
    } catch (const std::exception &) {
      throw std::runtime_error("CAN ID and master ID must be integer values");
    }
    if (can_id == 0 || can_id > 0x0F || master_id == 0 || master_id > 0x7FF) {
      throw std::runtime_error(
              "physical CAN IDs must be in [1,0x0f] and master IDs in [1,0x7ff]");
    }
    result[index] = dm1_hardware::MotorCalibration{
      dm1_hardware::MotorAddress{
        bus, static_cast<std::uint16_t>(can_id), static_cast<std::uint16_t>(master_id)},
      direction, zero};
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
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    if (arguments.imu_only) {
      return runImuOnly(resolveImuDevice(arguments.imu_device), arguments.control_time_step);
    }

    const auto calibration = loadCalibration(arguments.calibration_path);
    if (arguments.motor_only) {
      return runMotorOnly(
        arguments.can0, arguments.can1, calibration, arguments.control_time_step,
        arguments.poll_feedback_address);
    }

    Dm1HardwareDriver driver(
      arguments.can0, arguments.can1, resolveImuDevice(arguments.imu_device), calibration);
    HardwareBridge::Options options;
    options.control_time_step = arguments.control_time_step;
    options.startup_zero_tolerance_rad = arguments.zero_tolerance_rad;
    options.enable_output = arguments.enable_output;
    options.request_stand_up = arguments.stand_up;
    options.set_zero = arguments.set_zero;
    HardwareBridge bridge(driver, calibration, options);

    if (arguments.set_zero) {
      imu_log::print(
        imu_log::Level::Warning,
        "DM1 maintenance mode: --set-zero writes all motor zero parameters; "
        "output remains disabled.\n");
    } else if (arguments.enable_output) {
      imu_log::print(
        imu_log::Level::Warning,
        "DM1 control startup: offsets below 0.020 rad are accepted directly; "
        "offsets in [0.020, %.3f) rad are zeroed and verified before enable.\n",
        arguments.zero_tolerance_rad);
    } else {
      imu_log::print(
        imu_log::Level::Info,
        "DM1 hardware bridge is read-only at startup; it will not write motor "
        "parameters. Add --set-zero for explicit maintenance or "
        "--enable-output for control.\n");
    }
    const int result = bridge.run(g_stop_requested);
    return result;
  } catch (const std::exception & error) {
    imu_log::print(
      imu_log::Level::Error, "DM1 hardware startup failed: %s\n", error.what());
    return 1;
  }
}
