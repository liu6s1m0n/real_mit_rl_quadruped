#include "hardware/hardware_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <stdexcept>
#include <thread>

#include "common/console_log.hpp"

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
  if (options_.keyboard_control && options_.enable_output) {
    throw std::invalid_argument("--keyboard-control cannot be combined with --enable-output");
  }
  if (options_.keyboard_control && (options_.request_stand_up || options_.set_zero)) {
    throw std::invalid_argument(
            "--keyboard-control cannot be combined with --stand-up or --set-zero");
  }
  if (options_.set_zero && options_.enable_output) {
    throw std::invalid_argument("--set-zero cannot be combined with --enable-output");
  }
  if (!std::isfinite(options_.startup_zero_tolerance_rad) ||
    options_.startup_zero_tolerance_rad <= 0.0F ||
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

int HardwareBridge::runKeyboardControl(
  const std::chrono::steady_clock::time_point & start,
  const std::atomic_bool & stop_requested)
{
  teleop::KeyboardTeleop keyboard;
  // Opening CAN does not imply that physical motors are disabled.  Make the
  // keyboard path fail-closed before accepting any input.
  hardware_.disableAll();
  if (!keyboard.start()) {
    imu_log::print(
      imu_log::Level::Error,
      "Keyboard control requires a usable TTY; motors remain disabled.\n");
    return 9;
  }
  teleop::OperatorCommandArbiter arbiter;
  bool output_enabled = false;
  bool control_active = false;
  bool stand_up_pending = false;
  bool keyboard_fault_reported = false;
  auto next_cycle = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(options_.control_time_step));
  int result = 0;
  int consecutive_cycle_overruns = 0;
  std::deque<std::chrono::steady_clock::time_point> cycle_overruns;

  imu_log::print(
    imu_log::Level::Info,
    "Keyboard control active: motors start locked. Press H for help; U/0 enable/disable; "
    "Esc disables and exits.\n");

  while (!stop_requested.load()) {
    next_cycle += period;
    const auto cycle_start = std::chrono::steady_clock::now();
    const double now_s = std::chrono::duration<double>(cycle_start - start).count();
    arbiter.applyBatch(keyboard.consume());
    if (keyboard.available() && !keyboard.healthy() && !keyboard_fault_reported) {
      keyboard_fault_reported = true;
      arbiter.markFault();
      imu_log::print(
        imu_log::Level::Error,
        "Keyboard input thread stopped; motors remain disabled and hardware control exits.\n");
      if (output_enabled) {hardware_.disableAll();}
      result = 9;
      break;
    }
    arbiter.expireMotion(std::chrono::steady_clock::now());
    if (arbiter.takeHelpRequest()) {
      imu_log::print(
        imu_log::Level::Info,
        "Keys: Shift+U enable, 0 disable, 1 stand, 2 prone down, W/S forward/back, "
        "A/D left/right, Q/E rotate, Space stop, Esc exit, H help.\n");
    }
    if (arbiter.quitRequested()) {
      if (output_enabled) {hardware_.disableAll();}
      break;
    }
    if (arbiter.state() == teleop::MotorOutputState::Locked ||
      arbiter.state() == teleop::MotorOutputState::Fault)
    {
      if (output_enabled) {
        hardware_.disableAll();
        output_enabled = false;
      }
      control_active = false;
      stand_up_pending = false;
    }

    dm1_hardware::HardwareSample sample;
    // With drives unlocked but motion control inactive, send only the driver's
    // zero-gain feedback poll. No RobotRunner/MIT-PD command is produced.
    if ((!control_active && !hardware_.pollMotors()) ||
      !hardware_.read(sample, now_s) ||
      !mit_.updateFeedback(sample.motors, now_s))
    {
      imu_log::print(
        imu_log::Level::Error,
        "DM1 feedback read/validation failed during keyboard control; motors disabled.\n");
      hardware_.disableAll();
      result = 3;
      break;
    }
    const bool enable_requested =
      arbiter.state() == teleop::MotorOutputState::Enabling &&
      arbiter.takeEnableRequest();
    if (enable_requested) {
      const bool safe_to_enable = stationaryLevelSample(sample) &&
        motorPositionsWithin(sample, options_.startup_zero_tolerance_rad) &&
        mit_.feedbackValid();
      if (!safe_to_enable || !hardware_.enableAll()) {
        imu_log::print(
          imu_log::Level::Error,
          "Keyboard unlock rejected: feedback, IMU, level/stationary, or zero-window "
          "check failed. Motor zeros were not changed.\n");
        arbiter.markFault();
        hardware_.disableAll();
        result = 8;
        break;
      }
      arbiter.markEnabled();
      output_enabled = true;
      imu_log::print(
        imu_log::Level::Info,
        "DM1 motors unlocked; no motion command will be sent until Stand.\n");
    }
    if (arbiter.takeStopRequest()) {
      if (control_active) {
        runner_.setLocomotionVelocityCommand(0.0F, 0.0F, 0.0F);
        if (runner_.standingReady()) {runner_.setControlMode(ControlMode::BalanceStand);}
      }
    }
    if (arbiter.takeStandUpRequest()) {
      if (!control_active) {
        runner_.prepareForMotionControl();
        control_active = true;
      }
      stand_up_pending = true;
    }
    if (arbiter.takeProneDownRequest()) {
      if (!control_active || !runner_.requestProneDown()) {
        imu_log::print(
          imu_log::Level::Warning,
          "Prone-down rejected: press 1 and wait for BalanceStand.\n");
      } else {
        arbiter.stopMotion();
        runner_.setLocomotionVelocityCommand(0.0F, 0.0F, 0.0F);
        stand_up_pending = false;
        imu_log::print(
          imu_log::Level::Info,
          "Prone-down request accepted: MPC/WBC descent active.\n");
      }
    }
    if (arbiter.motionActive()) {
      if (!control_active || !runner_.standingReady()) {
        imu_log::print(
          imu_log::Level::Warning,
          "Motion rejected: press 1 and wait for BalanceStand first.\n");
        arbiter.stopMotion();
        if (control_active) {
          runner_.setLocomotionVelocityCommand(0.0F, 0.0F, 0.0F);
        }
      } else {
        const auto velocity = teleop::velocityForMotion(arbiter.motion());
        runner_.setLocomotionVelocityCommand(
          velocity.forward, velocity.lateral, velocity.yaw);
        runner_.setControlMode(ControlMode::Locomotion);
      }
    }
    const float timestamp = static_cast<float>(now_s);
    sample.imu.timestamp = timestamp;
    const bool feedback_valid = !control_active || runner_.updateHardwareFeedback(
      sample.imu, mit_.jointStates(timestamp), timestamp);
    const bool control_valid = feedback_valid && (!control_active || runner_.run());
    if (control_active && control_valid && stand_up_pending && runner_.requestStandUp()) {
      stand_up_pending = false;
      imu_log::print(imu_log::Level::Info, "Stand-up request accepted.\n");
    }
    if (control_active && !control_valid) {
      imu_log::print(
        imu_log::Level::Error,
        "RobotRunner rejected a keyboard-control frame; motors disabled.\n");
      hardware_.disableAll();
      result = 4;
      break;
    }
    if (control_active && arbiter.state() == teleop::MotorOutputState::Enabled &&
      output_enabled &&
      !mit_.send(runner_.jointCommands(), now_s))
    {
      imu_log::print(
        imu_log::Level::Error,
        "DM1 MIT command validation/send failed; motors disabled.\n");
      arbiter.markFault();
      hardware_.disableAll();
      result = 5;
      break;
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
      if (consecutive_cycle_overruns >= options_.max_consecutive_cycle_overruns ||
        static_cast<int>(cycle_overruns.size()) >= options_.max_cycle_overruns_in_window)
      {
        imu_log::print(
          imu_log::Level::Error,
          "DM1 keyboard control loop overrun policy tripped; motors disabled.\n");
        hardware_.disableAll();
        result = 6;
        break;
      }
    } else {
      consecutive_cycle_overruns = 0;
    }
    std::this_thread::sleep_until(next_cycle);
  }
  keyboard.stop();
  hardware_.disableAll();
  return result;
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

bool HardwareBridge::stationaryLevelSample(
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
    !stationaryLevelSample(initial))
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

  if (!options_.set_zero &&
    !motorPositionsWithin(initial, options_.startup_zero_tolerance_rad))
  {
    imu_log::print(
      imu_log::Level::Error,
      "Startup rejected: robot initial position error is at least %.3f rad; "
      "place all joints within the configured zero window or run --set-zero explicitly.\n",
      options_.startup_zero_tolerance_rad);
    return false;
  }

  if (!options_.set_zero) {
    imu_log::print(
      imu_log::Level::Info,
      "DM1 startup accepted within %.3f rad; no motor parameters changed.\n",
      options_.startup_zero_tolerance_rad);
    return true;
  }

  hardware_.disableAll();
  if (options_.set_zero) {
    imu_log::print(
      imu_log::Level::Warning,
      "DM1 explicit zero maintenance: resetting all 12 motor zeros.\n");
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
    if (readSample(verification) && stationaryLevelSample(verification) &&
      motorPositionsWithin(verification, options_.startup_zero_tolerance_rad))
    {
      if (options_.set_zero) {
        imu_log::print(
          imu_log::Level::Warning,
          "DM1 explicit motor zero reset verified.\n");
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
  // Opening CAN does not imply that a previous process left the drives
  // disabled. Establish the fail-closed state before any startup poll/read.
  hardware_.disableAll();

  const auto start = std::chrono::steady_clock::now();
  if (!performStartupZeroCalibration(start)) {
    hardware_.close();
    return 7;
  }
  if (options_.set_zero) {
    hardware_.close();
    return 0;
  }
  if (options_.keyboard_control) {
    try {
      const int result = runKeyboardControl(start, stop_requested);
      hardware_.close();
      return result;
    } catch (...) {
      hardware_.close();
      throw;
    }
  }
  auto next_cycle = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(options_.control_time_step));
  bool stand_up_accepted = false;
  int result = 0;
  int consecutive_cycle_overruns = 0;
  std::deque<std::chrono::steady_clock::time_point> cycle_overruns;
  bool output_enabled = false;

  while (!stop_requested.load()) {
    next_cycle += period;
    const auto cycle_start = std::chrono::steady_clock::now();
    double now_s = std::chrono::duration<double>(cycle_start - start).count();
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
      if (!output_enabled) {
        // The first controller frame must be complete and protocol-safe before
        // any physical output is enabled.
        if (!mit_.validateCommands(runner_.jointCommands(), now_s) ||
          !hardware_.enableAll())
        {
          imu_log::print(
            imu_log::Level::Error,
            "DM1 initial command validation or motor enable failed; motors disabled.\n");
          result = 8;
          break;
        }
        output_enabled = true;

        // Do not send the pre-enable frame. Refresh feedback and run the
        // controller once more so the first enabled frame is based on a fresh
        // post-enable state estimate.
        const auto fresh_cycle_start = std::chrono::steady_clock::now();
        now_s = std::chrono::duration<double>(fresh_cycle_start - start).count();
        if (!hardware_.pollMotors() || !hardware_.read(sample, now_s) ||
          !mit_.updateFeedback(sample.motors, now_s))
        {
          imu_log::print(
            imu_log::Level::Error,
            "DM1 post-enable feedback refresh failed; motors disabled.\n");
          result = 3;
          break;
        }
        const float fresh_timestamp = static_cast<float>(now_s);
        sample.imu.timestamp = fresh_timestamp;
        if (!runner_.updateHardwareFeedback(
            sample.imu, mit_.jointStates(fresh_timestamp), fresh_timestamp) ||
          !runner_.run() || !mit_.validateCommands(runner_.jointCommands(), now_s))
        {
          imu_log::print(
            imu_log::Level::Error,
            "DM1 post-enable controller frame validation failed; motors disabled.\n");
          result = 5;
          break;
        }
      }
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
