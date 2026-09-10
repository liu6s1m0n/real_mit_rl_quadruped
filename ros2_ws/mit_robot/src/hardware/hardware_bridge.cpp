// ============================================================
// DM1 真机硬件桥接实现。
// 本文件负责硬件反馈读取、启动安全检查、RobotRunner 控制循环以及
// 键盘遥操作路径；任何异常路径都保持电机失能。
// ============================================================

#include "hardware/hardware_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <stdexcept>
#include <thread>

#include "common/console_log.hpp"

namespace
{
constexpr std::array<const char *, kNumJoints> kJointNames{
  "FR_hip", "FR_thigh", "FR_calf",
  "FL_hip", "FL_thigh", "FL_calf",
  "RR_hip", "RR_thigh", "RR_calf",
  "RL_hip", "RL_thigh", "RL_calf"};
constexpr auto kStartupSequenceTimeout = std::chrono::milliseconds(100);
}

// 构造硬件桥：绑定底层设备、关节标定和 RobotRunner，随后校验运行参数。
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
    throw std::invalid_argument("HardwareBridge 控制周期无效");
  }
  if (options_.request_stand_up && !options_.enable_output) {
    throw std::invalid_argument("--stand-up 必须与 --enable-output 一起使用");
  }
  if (options_.keyboard_control && options_.enable_output) {
    throw std::invalid_argument("--keyboard-control 不能与 --enable-output 同时使用");
  }
  if (options_.keyboard_control && (options_.request_stand_up || options_.set_zero)) {
    throw std::invalid_argument(
            "--keyboard-control 不能与 --stand-up 或 --set-zero 同时使用");
  }
  if (options_.set_zero && options_.enable_output) {
    throw std::invalid_argument("--set-zero 不能与 --enable-output 同时使用");
  }
  if (!std::isfinite(options_.startup_zero_tolerance_rad) ||
    options_.startup_zero_tolerance_rad <= 0.0F ||
    !std::isfinite(options_.startup_stationary_velocity_rad_s) ||
    options_.startup_stationary_velocity_rad_s <= 0.0F ||
    options_.startup_stable_samples <= 0 ||
    options_.max_consecutive_cycle_overruns <= 0 ||
    options_.max_cycle_overruns_in_window <= 0 ||
    !std::isfinite(options_.cycle_overrun_window_s) || options_.cycle_overrun_window_s <= 0.0)
  {
    throw std::invalid_argument(
            "硬件启动阈值或控制周期超时参数无效");
  }
  if (options_.set_zero) {
    for (const auto & item : calibration_) {
      if (std::abs(item.zero_position) > 1.0e-4F) {
        throw std::invalid_argument(
                "--set-zero 要求所有标定参数的 zero_position 均为 0");
      }
    }
  }
}

// ---------- 键盘遥操作控制 ----------

// 运行键盘遥操作循环：按键状态先经过命令仲裁，再决定电机解锁、站立和运动。
int HardwareBridge::runKeyboardControl(
  const std::chrono::steady_clock::time_point & start,
  const std::atomic_bool & stop_requested)
{
  teleop::KeyboardTeleop keyboard;
  // 打开 CAN 并不代表物理电机已经失能；接受键盘输入前先建立失效安全状态。
  hardware_.disableAll();
  if (!keyboard.start()) {
    imu_log::print(
      imu_log::Level::Error,
      "键盘控制需要可用的 TTY；电机保持失能。\n");
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
    "键盘控制已启动：电机初始处于锁定状态。按 H 查看帮助；按 Shift+U/0 使能/失能；"
    "按 Esc 失能并退出。\n");

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
        "键盘输入线程已停止；电机保持失能，硬件控制即将退出。\n");
      if (output_enabled) {hardware_.disableAll();}
      result = 9;
      break;
    }
    arbiter.expireMotion(std::chrono::steady_clock::now());
    if (arbiter.takeHelpRequest()) {
      imu_log::print(
        imu_log::Level::Info,
        "按键：Shift+U 使能，0 失能，1 站立，2 趴下，W/S 前进/后退，"
        "A/D 左移/右移，Q/E 旋转，空格停止，Esc 退出，H 帮助。\n");
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
    // 电机尚未解锁时，只发送驱动器的零增益反馈轮询，不生成
    // RobotRunner/MIT-PD 控制命令。
    if ((!control_active && !hardware_.pollMotors()) ||
      !hardware_.read(sample, now_s) ||
      !mit_.updateFeedback(sample.motors, now_s))
    {
      imu_log::print(
        imu_log::Level::Error,
        "键盘控制期间读取或校验 DM1 反馈失败；电机已失能。\n");
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
        if (!motorPositionsWithin(sample, options_.startup_zero_tolerance_rad)) {
          reportInvalidMotorPositions(sample, options_.startup_zero_tolerance_rad);
        }
        imu_log::print(
          imu_log::Level::Error,
          "键盘解锁被拒绝：反馈、IMU、水平/静止状态或零位窗口检查失败；"
          "电机零位未改变。\n");
        arbiter.markFault();
        hardware_.disableAll();
        result = 8;
        break;
      }
      arbiter.markEnabled();
      output_enabled = true;
      // 解锁后立即启动 RobotRunner 的零位初始化轨迹；首帧控制命令在本周期末发送。
      runner_.reset();
      control_active = true;
      imu_log::print(
        imu_log::Level::Info,
        "DM1 电机已使能；正在将电机平滑回到零位，完成后可按 1 站立。\n");
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
          "趴下请求被拒绝：请按 1 并等待进入 BalanceStand。\n");
      } else {
        arbiter.stopMotion();
        runner_.setLocomotionVelocityCommand(0.0F, 0.0F, 0.0F);
        stand_up_pending = false;
        imu_log::print(
          imu_log::Level::Info,
          "趴下请求已接受：MPC/WBC 下降控制已激活。\n");
      }
    }
    if (arbiter.motionActive()) {
      if (!control_active || !runner_.standingReady()) {
        imu_log::print(
          imu_log::Level::Warning,
          "运动请求被拒绝：请先按 1 并等待进入 BalanceStand。\n");
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
      imu_log::print(imu_log::Level::Info, "站立请求已接受。\n");
    }
    if (control_active && !control_valid) {
      imu_log::print(
        imu_log::Level::Error,
        "RobotRunner 拒绝了键盘控制帧；电机已失能。\n");
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
        "DM1 MIT 指令校验或发送失败；电机已失能。\n");
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
          "DM1 键盘控制循环超时策略已触发；电机已失能。\n");
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

// ---------- 启动反馈与零位检查 ----------

// 检查每个标定电机都已反馈，且当前位置处于指定的零位误差窗口内。
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

// 打印每个超出零位窗口或没有有效反馈的电机，帮助人工调整机械位置。
void HardwareBridge::reportInvalidMotorPositions(
  const dm1_hardware::HardwareSample & sample,
  float tolerance_rad) const
{
  for (std::size_t index = 0; index < calibration_.size(); ++index) {
    const auto & calibration = calibration_[index];
    const dm1_hardware::MotorFeedback * feedback = nullptr;
    for (const auto & motor : sample.motors) {
      if (motor.bus == calibration.address.bus &&
        motor.can_id == calibration.address.can_id)
      {
        feedback = &motor;
        break;
      }
    }

    if (feedback == nullptr) {
      imu_log::print(
        imu_log::Level::Error,
        "电机位置不合理：%s（总线 %u，CAN ID %u）没有收到有效反馈；请人工调整后重试。\n",
        kJointNames[index], static_cast<unsigned int>(calibration.address.bus),
        static_cast<unsigned int>(calibration.address.can_id));
      continue;
    }

    const float model_position = calibration.direction *
      (feedback->position - calibration.zero_position);
    const float position_error = std::abs(model_position);
    if (!std::isfinite(model_position) || !std::isfinite(position_error) ||
      position_error >= tolerance_rad)
    {
      imu_log::print(
        imu_log::Level::Error,
        "电机位置不合理：%s（总线 %u，CAN ID %u）当前误差为 %+.4f rad，"
        "允许范围为 ±%.3f rad；请人工调整后重试。\n",
        kJointNames[index], static_cast<unsigned int>(calibration.address.bus),
        static_cast<unsigned int>(calibration.address.can_id), model_position,
        tolerance_rad);
    }
  }
}

// 检查 IMU 姿态、重力和角速度，以及全部电机位置和速度是否满足启动条件。
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

// 执行启动采样；普通启动只验证零位，--set-zero 则在验证通过后写入并复核零位。
bool HardwareBridge::performStartupZeroCalibration(
  const std::chrono::steady_clock::time_point & start,
  const std::atomic_bool & stop_requested)
{
  auto readSample = [&](dm1_hardware::HardwareSample & sample) {
      const double now_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      if (!hardware_.pollMotors()) {return false;}
      if (!hardware_.read(sample, now_s)) {return false;}
      sample.imu.timestamp = static_cast<float>(now_s);
      return mit_.updateFeedback(sample.motors, now_s);
    };

  dm1_hardware::HardwareSample previous;
  bool have_previous = false;
  int stable_samples = 0;
  const auto sequence_deadline = std::chrono::steady_clock::now() + kStartupSequenceTimeout;
  auto sequencesAdvanced = [&](const dm1_hardware::HardwareSample & sample) {
      if (sample.imu.sequence == 0 ||
        (have_previous && sample.imu.sequence <= previous.imu.sequence))
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
        if (index >= kNumJoints || seen[index] || motor.sequence == 0) {return false;}
        if (have_previous) {
          // The transport normally emits calibration order, but startup
          // validation must compare by physical address rather than relying
          // on array order.
          bool previous_found = false;
          std::uint64_t previous_sequence = 0;
          for (const auto & previous_motor : previous.motors) {
            if (previous_motor.bus == motor.bus && previous_motor.can_id == motor.can_id) {
              previous_found = true;
              previous_sequence = previous_motor.sequence;
              break;
            }
          }
          if (!previous_found || motor.sequence <= previous_sequence) {return false;}
        }
        seen[index] = true;
      }
      return std::all_of(seen.begin(), seen.end(), [](bool value) {return value;});
    };
  while (stable_samples < options_.startup_stable_samples) {
    if (stop_requested.load() || std::chrono::steady_clock::now() >= sequence_deadline) {
      imu_log::print(
        imu_log::Level::Error,
        "启动检查被拒绝：在 %d ms 内未收到连续更新的 IMU/电机反馈。\n",
        static_cast<int>(kStartupSequenceTimeout.count()));
      return false;
    }
    dm1_hardware::HardwareSample sample;
    if (!readSample(sample)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    if (!sequencesAdvanced(sample)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    if (!stationaryLevelSample(sample)) {
      imu_log::print(
        imu_log::Level::Error,
        options_.set_zero ?
        "启动零位检查被拒绝：--set-zero 要求连续 %d 帧静止、水平且健康的反馈。\n" :
        "启动检查被拒绝：机器人必须连续 %d 帧静止、水平，且反馈健康。\n",
        options_.startup_stable_samples);
      return false;
    }
    if (!options_.set_zero &&
      !motorPositionsWithin(sample, options_.startup_zero_tolerance_rad))
    {
      imu_log::print(
        imu_log::Level::Error,
        "启动被拒绝：以下电机位置超出 ±%.3f rad，请人工调整后重试。\n",
        options_.startup_zero_tolerance_rad);
      reportInvalidMotorPositions(sample, options_.startup_zero_tolerance_rad);
      return false;
    }
    previous = sample;
    have_previous = true;
    ++stable_samples;
  }

  imu_log::print(
    imu_log::Level::Info,
    "DM1 启动稳定窗口通过：连续 %d 帧静止、水平且反馈健康。\n",
    options_.startup_stable_samples);

  if (!options_.set_zero) {
    imu_log::print(
      imu_log::Level::Info,
      "DM1 启动检查通过，位置误差在 %.3f rad 以内；未修改电机参数。\n",
      options_.startup_zero_tolerance_rad);
    return true;
  }

  hardware_.disableAll();
  if (options_.set_zero) {
    imu_log::print(
      imu_log::Level::Warning,
      "DM1 显式零位维护：正在重置全部 12 个电机的零位。\n");
  }
  for (const auto & item : calibration_) {
    if (stop_requested.load()) {return false;}
    const auto & address = item.address;
    if (!hardware_.setMotorZero(address)) {
      imu_log::print(
        imu_log::Level::Error,
        "重置电机零位失败：总线 %u，CAN ID %u。\n",
        static_cast<unsigned int>(address.bus),
        static_cast<unsigned int>(address.can_id));
      return false;
    }
  }

  if (stop_requested.load()) {return false;}
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const auto verification_deadline =
    std::chrono::steady_clock::now() + kStartupSequenceTimeout;
  while (!stop_requested.load() && std::chrono::steady_clock::now() < verification_deadline) {
    dm1_hardware::HardwareSample verification;
    if (readSample(verification) && sequencesAdvanced(verification)) {
      // Advance the baseline even when this fresh sample is otherwise unsafe;
      // a repeated copy of that unsafe frame must not pass on the next try.
      previous = verification;
      if (stationaryLevelSample(verification) &&
        motorPositionsWithin(verification, options_.startup_zero_tolerance_rad))
      {
        if (options_.set_zero) {
          imu_log::print(
            imu_log::Level::Warning,
            "DM1 显式电机零位重置已验证。\n");
        }
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  imu_log::print(
    imu_log::Level::Error,
    "DM1 电机零位重置未能通过验证。\n");
  return false;
}

// ---------- 主硬件控制循环 ----------

// 打开设备、完成启动安全检查，并按固定周期运行 RobotRunner 和 MIT 输出。
int HardwareBridge::run(const std::atomic_bool & stop_requested)
{
  if (!hardware_.open()) {
    hardware_.disableAll();
    return 2;
  }
  // 打开 CAN 不代表上一个进程退出时留下的驱动器仍处于失能状态；
  // 在任何启动轮询或读取前先建立失效安全状态。
  hardware_.disableAll();

  const auto start = std::chrono::steady_clock::now();
  if (!performStartupZeroCalibration(start, stop_requested)) {
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
        "DM1 电机反馈轮询失败；电机已失能。\n");
      result = 3;
      break;
    }
    if (!hardware_.read(sample, now_s) ||
      !mit_.updateFeedback(sample.motors, now_s))
    {
      imu_log::print(
        imu_log::Level::Error,
        "DM1 反馈读取或校验失败；电机已失能。\n");
      result = 3;
      break;
    }

    const float timestamp = static_cast<float>(now_s);
    // 控制器内部使用桥的相对单调时钟；电机原始时间戳已经在上面的
    // updateFeedback() 中用于检查驱动器是否确实返回了新帧。
    sample.imu.timestamp = timestamp;
    const auto joints = mit_.jointStates(timestamp);
    if (!runner_.updateHardwareFeedback(sample.imu, joints, timestamp) ||
      !runner_.run())
    {
      imu_log::print(
        imu_log::Level::Error,
        "DM1 状态估计器/控制器拒绝了硬件数据帧。\n");
      result = 4;
      break;
    }

    if (options_.request_stand_up && !stand_up_accepted) {
      stand_up_accepted = runner_.requestStandUp();
    }

    if (options_.enable_output) {
      if (!output_enabled) {
        // 首个控制器帧必须完整且通过协议安全检查后，才能使能物理输出。
        if (!mit_.validateCommands(runner_.jointCommands(), now_s) ||
          !hardware_.enableAll())
        {
          imu_log::print(
            imu_log::Level::Error,
            "DM1 初始指令校验或电机使能失败；电机已失能。\n");
          result = 8;
          break;
        }
        output_enabled = true;
        // 使能后只发送刷新反馈并重新生成的控制帧，确保首个输出帧开始执行平滑回零轨迹。
        imu_log::print(
          imu_log::Level::Info,
          "DM1 电机已使能；正在将电机平滑回到零位。\n");

        // 不发送使能前生成的控制帧；重新读取反馈并再次运行控制器，
        // 让首个使能后的控制帧基于最新的状态估计。
        const auto fresh_cycle_start = std::chrono::steady_clock::now();
        now_s = std::chrono::duration<double>(fresh_cycle_start - start).count();
        if (!hardware_.pollMotors() || !hardware_.read(sample, now_s) ||
          !mit_.updateFeedback(sample.motors, now_s))
        {
          imu_log::print(
            imu_log::Level::Error,
            "DM1 电机使能后的反馈刷新失败；电机已失能。\n");
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
            "DM1 电机使能后的控制器数据帧校验失败；电机已失能。\n");
          result = 5;
          break;
        }
      }
      if (!mit_.send(runner_.jointCommands(), now_s)) {
        imu_log::print(
          imu_log::Level::Error,
          "DM1 MIT 指令校验或发送失败。\n");
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
        "DM1 控制周期超过截止时间（连续 %d 次，时间窗口内 %zu 次）。\n",
        consecutive_cycle_overruns, cycle_overruns.size());
      if (consecutive_cycle_overruns >= options_.max_consecutive_cycle_overruns ||
        static_cast<int>(cycle_overruns.size()) >= options_.max_cycle_overruns_in_window)
      {
        imu_log::print(
          imu_log::Level::Error,
          "DM1 控制循环超时策略已触发；电机已失能。\n");
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

/*
返回值含义
返回值	含义
0	正常退出，或零位维护成功
2	硬件打开失败
3	电机/IMU 反馈读取或校验失败
4	RobotRunner 拒绝硬件帧
5	MIT 命令校验或发送失败
6	控制周期超时策略触发
7	启动零位、水平或静止检查失败
8	首次使能或键盘解锁失败
9	键盘 TTY、键盘线程或输入路径失败
*/
