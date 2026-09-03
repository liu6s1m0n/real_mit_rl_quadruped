#include "HardwareBridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <thread>

HardwareBridge::HardwareBridge(
  dm1_hardware::Dm1HardwareIo & hardware,
  const dm1_hardware::Dm1MitInterface::CalibrationArray & calibration,
  const Options & options)
: hardware_(hardware), calibration_(calibration), options_(options),
  runner_(options.control_time_step),
  mit_(hardware, calibration, options.feedback_timeout_s, false)
{
  if (!std::isfinite(options_.control_time_step) ||
    options_.control_time_step <= 0.0F)
  {
    throw std::invalid_argument("HardwareBridge control period is invalid");
  }
  if (options_.request_stand_up && !options_.enable_output) {
    throw std::invalid_argument("--stand-up requires --enable-output");
  }
  if (!std::isfinite(options_.startup_zero_tolerance_rad) ||
    options_.startup_zero_tolerance_rad <= 0.0F ||
    !std::isfinite(options_.startup_stationary_velocity_rad_s) ||
    options_.startup_stationary_velocity_rad_s <= 0.0F)
  {
    throw std::invalid_argument("startup zero/stationary tolerance is invalid");
  }
  for (const auto & item : calibration_) {
    if (std::abs(item.zero_position) > 1.0e-4F) {
      throw std::invalid_argument(
        "startup auto-zero requires every calibration zero_position to be 0");
    }
  }
}

bool HardwareBridge::stationaryProneSample(
  const dm1_hardware::HardwareSample & sample,
  bool require_zero_position) const noexcept
{
  if (!sample.imu.valid || !sample.imu.orientation_valid ||
    !sample.imu.orientation_world_from_body.coeffs().allFinite() ||
    !sample.imu.angular_velocity_body.allFinite() ||
    sample.imu.angular_velocity_body.norm() > 0.15F)
  {
    return false;
  }

  Eigen::Quaternionf orientation = sample.imu.orientation_world_from_body;
  if (!std::isfinite(orientation.norm()) || orientation.norm() < 0.5F) {
    return false;
  }
  orientation.normalize();
  const Mat3<float> rotation = orientation.toRotationMatrix();
  const float roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const float pitch = std::asin(std::clamp(-rotation(2, 0), -1.0F, 1.0F));
  if (std::abs(roll) > 0.25F || std::abs(pitch) > 0.25F) {return false;}

  std::array<bool, kNumJoints> seen{};
  for (const auto & motor : sample.motors) {
    std::size_t index = kNumJoints;
    for (std::size_t candidate = 0; candidate < calibration_.size(); ++candidate) {
      if (calibration_[candidate].id == motor.id) {
        index = candidate;
        break;
      }
    }
    if (index >= kNumJoints || seen[index] ||
      !std::isfinite(motor.position) || !std::isfinite(motor.velocity) ||
      std::abs(motor.velocity) > options_.startup_stationary_velocity_rad_s ||
      (require_zero_position &&
      std::abs(motor.position) > options_.startup_zero_tolerance_rad))
    {
      return false;
    }
    seen[index] = true;
  }
  return std::all_of(seen.begin(), seen.end(), [](bool value) {return value;});
}

bool HardwareBridge::performStartupZeroCalibration(
  const std::chrono::steady_clock::time_point & start)
{
  auto readSample = [&](dm1_hardware::HardwareSample & sample) {
      const double now_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      if (!hardware_.read(sample, now_s)) {return false;}
      sample.imu.timestamp = static_cast<float>(now_s);
      return mit_.updateFeedback(sample.motors, now_s);
    };

  dm1_hardware::HardwareSample initial;
  if (!readSample(initial) || !stationaryProneSample(initial, false)) {
    std::fprintf(
      stderr,
      "Startup zero rejected: DM1 must be motionless, level and prone with "
      "12 healthy motor feedback frames.\n");
    return false;
  }

  bool zero_reset_required = false;
  for (const auto & motor : initial.motors) {
    zero_reset_required = zero_reset_required ||
      std::abs(motor.position) > options_.startup_zero_tolerance_rad;
  }
  if (!zero_reset_required) {
    std::fprintf(stderr, "DM1 startup motor zeros accepted; no reset required.\n");
    return true;
  }

  hardware_.disableAll();
  std::fprintf(stderr, "DM1 startup offset detected; resetting all 12 motor zeros.\n");
  for (const auto & item : calibration_) {
    if (!hardware_.setMotorZero(item.id)) {
      std::fprintf(
        stderr, "Failed to reset motor ID %u zero.\n",
        static_cast<unsigned int>(item.id));
      return false;
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  for (int attempt = 0; attempt < 20; ++attempt) {
    dm1_hardware::HardwareSample verification;
    if (readSample(verification) && stationaryProneSample(verification, true)) {
      std::fprintf(stderr, "DM1 startup motor zero reset verified.\n");
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::fprintf(stderr, "DM1 motor zero reset could not be verified.\n");
  return false;
}

int HardwareBridge::run(const std::atomic_bool & stop_requested)
{
  hardware_.disableAll();
  if (!hardware_.open()) {
    hardware_.disableAll();
    return 2;
  }

  const auto start = std::chrono::steady_clock::now();
  if (!performStartupZeroCalibration(start)) {
    mit_.disable();
    hardware_.close();
    return 7;
  }
  auto next_cycle = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(options_.control_time_step));
  bool stand_up_accepted = false;
  int result = 0;

  while (!stop_requested.load()) {
    next_cycle += period;
    const auto cycle_start = std::chrono::steady_clock::now();
    const double now_s = std::chrono::duration<double>(cycle_start - start).count();
    dm1_hardware::HardwareSample sample;

    if (!hardware_.read(sample, now_s) ||
      !mit_.updateFeedback(sample.motors, now_s))
    {
      std::fprintf(stderr, "DM1 feedback read/validation failed; motors disabled.\n");
      result = 3;
      break;
    }

    const float timestamp = static_cast<float>(now_s);
    // 控制器内部用桥的相对单调时钟；电机原始时间戳已经在上面的
    // updateFeedback() 中用于检查驱动是否真的返回了新帧。
    sample.imu.timestamp = timestamp;
    const auto joints = mit_.jointStates(timestamp);
    if (!runner_.updateHardwareFeedback(sample.imu, joints, timestamp) ||
      !runner_.run())
    {
      std::fprintf(stderr, "DM1 estimator/controller rejected the hardware frame.\n");
      result = 4;
      break;
    }

    if (options_.request_stand_up && !stand_up_accepted) {
      stand_up_accepted = runner_.requestStandUp();
    }

    if (options_.enable_output) {
      if (!mit_.send(runner_.jointCommands(), now_s)) {
        std::fprintf(stderr, "DM1 MIT command validation/send failed.\n");
        result = 5;
        break;
      }
    }

    const auto finished = std::chrono::steady_clock::now();
    if (finished > next_cycle + period) {
      std::fprintf(stderr, "DM1 control loop missed more than one period.\n");
      result = 6;
      break;
    }
    std::this_thread::sleep_until(next_cycle);
  }

  mit_.disable();
  hardware_.close();
  return result;
}
