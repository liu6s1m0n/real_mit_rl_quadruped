#include "HardwareBridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <stdexcept>
#include <thread>

#include "sensor/imu_log.hpp"

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
  if (options_.set_zero && options_.enable_output) {
    throw std::invalid_argument("--set-zero cannot be combined with --enable-output");
  }
  if (!std::isfinite(options_.startup_zero_tolerance_rad) ||
    options_.startup_zero_tolerance_rad <= 0.0F ||
    !std::isfinite(options_.direct_startup_zero_tolerance_rad) ||
    options_.direct_startup_zero_tolerance_rad <= 0.0F ||
    options_.direct_startup_zero_tolerance_rad >= options_.startup_zero_tolerance_rad ||
    !std::isfinite(options_.startup_stationary_velocity_rad_s) ||
    options_.startup_stationary_velocity_rad_s <= 0.0F ||
    options_.max_consecutive_cycle_overruns <= 0 ||
    options_.max_cycle_overruns_in_window <= 0 ||
    !std::isfinite(options_.cycle_overrun_window_s) || options_.cycle_overrun_window_s <= 0.0)
  {
    throw std::invalid_argument(
            "hardware startup thresholds or cycle-overrun option is invalid");
  }
  if (options_.set_zero) {
    for (const auto & item : calibration_) {
      if (std::abs(item.zero_position) > 1.0e-4F) {
        throw std::invalid_argument(
                "--set-zero requires every calibration zero_position to be 0");
      }
    }
  }
}

bool HardwareBridge::motorPositionsWithin(
  const dm1_hardware::HardwareSample & sample,
  float tolerance_rad) const noexcept
{
  if (!std::isfinite(tolerance_rad) || tolerance_rad < 0.0F) {return false;}
  std::array<bool, kNumJoints> seen{};
  for (const auto & motor : sample.motors) {
    std::size_t index = kNumJoints;
    for (std::size_t candidate = 0; candidate < calibration_.size(); ++candidate) {
      if (calibration_[candidate].address.bus == motor.bus &&
        calibration_[candidate].address.can_id == motor.can_id)
      {
        index = candidate;
        break;
      }
    }
    if (index >= kNumJoints || seen[index] || !std::isfinite(motor.position) ||
      std::abs(
        calibration_[index].direction *
        (motor.position - calibration_[index].zero_position)) >= tolerance_rad)
    {
      return false;
    }
    seen[index] = true;
  }
  return std::all_of(seen.begin(), seen.end(), [](bool value) {return value;});
}

bool HardwareBridge::stationaryProneSample(
  const dm1_hardware::HardwareSample & sample) const noexcept
{
  if (!sample.imu.valid || !sample.imu.acceleration_valid ||
    !sample.imu.orientation_valid ||
    !sample.imu.orientation_world_from_body.coeffs().allFinite() ||
    !sample.imu.acceleration_body.allFinite() ||
    !sample.imu.angular_velocity_body.allFinite() ||
    sample.imu.angular_velocity_body.norm() > 0.15F)
  {
    return false;
  }

  const float orientation_norm = sample.imu.orientation_world_from_body.norm();
  if (!std::isfinite(orientation_norm) || orientation_norm < 0.5F) {
    return false;
  }
  const float gravity = sample.imu.acceleration_body.norm();
  const float horizontal_gravity = std::hypot(
    sample.imu.acceleration_body.x(), sample.imu.acceleration_body.y());
  if (!std::isfinite(gravity) || gravity < 7.0F || gravity > 12.0F ||
    sample.imu.acceleration_body.z() <= 0.0F ||
    horizontal_gravity > gravity * std::sin(0.25F))
  {
    return false;
  }

  std::array<bool, kNumJoints> seen{};
  for (const auto & motor : sample.motors) {
    std::size_t index = kNumJoints;
    for (std::size_t candidate = 0; candidate < calibration_.size(); ++candidate) {
      if (calibration_[candidate].address.bus == motor.bus &&
        calibration_[candidate].address.can_id == motor.can_id)
      {
        index = candidate;
        break;
      }
    }
    if (index >= kNumJoints || seen[index] ||
      !std::isfinite(motor.position) || !std::isfinite(motor.velocity) ||
      std::abs(motor.velocity) > options_.startup_stationary_velocity_rad_s)
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
      if (!hardware_.pollMotors()) {return false;}
      if (!hardware_.read(sample, now_s)) {return false;}
      sample.imu.timestamp = static_cast<float>(now_s);
      return mit_.updateFeedback(sample.motors, now_s);
    };

  dm1_hardware::HardwareSample initial;
  if (!readSample(initial) ||
    !stationaryProneSample(initial))
  {
    imu_log::print(
      imu_log::Level::Error,
      options_.set_zero ?
      "Startup zero rejected: --set-zero requires a motionless, level robot "
      "with 12 healthy motor feedback frames.\n" :
      "Startup check rejected: robot must be motionless, level and already at "
      "the configured zero with 12 healthy motor feedback frames.\n");
    return false;
  }

  if (options_.set_zero) {
    // --set-zero is an explicit maintenance operation.  Once the robot has
    // passed the motion/level/feedback checks, always write and verify all
    // twelve motors, even when the current offset is already small.
  } else if (!motorPositionsWithin(initial, options_.startup_zero_tolerance_rad)) {
    imu_log::print(
      imu_log::Level::Error,
      "Startup rejected: robot initial position error is at least %.3f rad; "
      "place all joints within the configured zero window.\n",
      options_.startup_zero_tolerance_rad);
    return false;
  } else if (!options_.enable_output) {
    imu_log::print(
      imu_log::Level::Info,
      motorPositionsWithin(initial, options_.direct_startup_zero_tolerance_rad) ?
      "DM1 read-only startup accepted below %.3f rad; no motor parameters changed.\n" :
      "DM1 read-only startup accepted within %.3f rad; output remains disabled and "
      "automatic zero maintenance is unavailable without --enable-output.\n",
      motorPositionsWithin(initial, options_.direct_startup_zero_tolerance_rad) ?
      options_.direct_startup_zero_tolerance_rad : options_.startup_zero_tolerance_rad);
    return true;
  } else if (motorPositionsWithin(initial, options_.direct_startup_zero_tolerance_rad)) {
    imu_log::print(
      imu_log::Level::Info,
      "DM1 startup offset is below %.3f rad; enabling without changing motor parameters.\n",
      options_.direct_startup_zero_tolerance_rad);
    return true;
  }

  hardware_.disableAll();
  if (options_.set_zero) {
    imu_log::print(
      imu_log::Level::Warning,
      "DM1 explicit zero maintenance: resetting all 12 motor zeros.\n");
  } else {
    imu_log::print(
      imu_log::Level::Warning,
      "DM1 startup offset is in [%.3f, %.3f) rad; resetting all 12 motor zeros before enable.\n",
      options_.direct_startup_zero_tolerance_rad, options_.startup_zero_tolerance_rad);
  }
  for (const auto & item : calibration_) {
    const auto & address = item.address;
    if (!hardware_.setMotorZero(address)) {
      imu_log::print(
        imu_log::Level::Error,
        "Failed to reset motor bus %u CAN ID %u zero.\n",
        static_cast<unsigned int>(address.bus),
        static_cast<unsigned int>(address.can_id));
      return false;
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  for (int attempt = 0; attempt < 20; ++attempt) {
    dm1_hardware::HardwareSample verification;
    if (readSample(verification) && stationaryProneSample(verification) &&
      motorPositionsWithin(verification, options_.direct_startup_zero_tolerance_rad)) {
      if (options_.set_zero) {
        imu_log::print(
          imu_log::Level::Warning,
          "DM1 explicit motor zero reset verified.\n");
      } else {
        imu_log::print(
          imu_log::Level::Warning,
          "DM1 automatic motor zero reset verified below %.3f rad.\n",
          options_.direct_startup_zero_tolerance_rad);
      }
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  imu_log::print(
    imu_log::Level::Error,
    "DM1 motor zero reset could not be verified.\n");
  return false;
}

int HardwareBridge::run(const std::atomic_bool & stop_requested)
{
  if (!hardware_.open()) {
    hardware_.disableAll();
    return 2;
  }

  const auto start = std::chrono::steady_clock::now();
  if (!performStartupZeroCalibration(start)) {
    hardware_.close();
    return 7;
  }
  if (options_.set_zero) {
    hardware_.close();
    return 0;
  }
  if (options_.enable_output && !hardware_.enableAll()) {
    imu_log::print(
      imu_log::Level::Error,
      "DM1 motor enable failed; motors disabled.\n");
    hardware_.close();
    return 8;
  }
  auto next_cycle = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(options_.control_time_step));
  bool stand_up_accepted = false;
  int result = 0;
  int consecutive_cycle_overruns = 0;
  std::deque<std::chrono::steady_clock::time_point> cycle_overruns;

  while (!stop_requested.load()) {
    next_cycle += period;
    const auto cycle_start = std::chrono::steady_clock::now();
    const double now_s = std::chrono::duration<double>(cycle_start - start).count();
    dm1_hardware::HardwareSample sample;

    if (!options_.enable_output && !hardware_.pollMotors()) {
      imu_log::print(
        imu_log::Level::Error,
        "DM1 motor feedback poll failed; motors disabled.\n");
      result = 3;
      break;
    }
    if (!hardware_.read(sample, now_s) ||
      !mit_.updateFeedback(sample.motors, now_s))
    {
      imu_log::print(
        imu_log::Level::Error,
        "DM1 feedback read/validation failed; motors disabled.\n");
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
      imu_log::print(
        imu_log::Level::Error,
        "DM1 estimator/controller rejected the hardware frame.\n");
      result = 4;
      break;
    }

    if (options_.request_stand_up && !stand_up_accepted) {
      stand_up_accepted = runner_.requestStandUp();
    }

    if (options_.enable_output) {
      if (!mit_.send(runner_.jointCommands(), now_s)) {
        imu_log::print(
          imu_log::Level::Error,
          "DM1 MIT command validation/send failed.\n");
        result = 5;
        break;
      }
    }

    const auto finished = std::chrono::steady_clock::now();
    if (finished > next_cycle) {
      ++consecutive_cycle_overruns;
      cycle_overruns.push_back(finished);
      while (!cycle_overruns.empty() &&
        std::chrono::duration<double>(finished - cycle_overruns.front()).count() >
        options_.cycle_overrun_window_s)
      {
        cycle_overruns.pop_front();
      }
      imu_log::print(
        imu_log::Level::Warning,
        "DM1 control cycle missed its deadline (%d consecutive, %zu in window).\n",
        consecutive_cycle_overruns, cycle_overruns.size());
      if (consecutive_cycle_overruns >= options_.max_consecutive_cycle_overruns ||
        static_cast<int>(cycle_overruns.size()) >= options_.max_cycle_overruns_in_window)
      {
        imu_log::print(
          imu_log::Level::Error,
          "DM1 control loop overrun policy tripped; motors disabled.\n");
        result = 6;
        break;
      }
    } else {
      consecutive_cycle_overruns = 0;
    }
    std::this_thread::sleep_until(next_cycle);
  }

  hardware_.close();
  return result;
}
