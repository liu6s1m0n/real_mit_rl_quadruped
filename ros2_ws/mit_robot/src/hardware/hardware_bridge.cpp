// ============================================================
// DM1 真机硬件桥接实现。
// 本文件负责硬件反馈读取、启动安全检查、RobotRunner 控制循环以及
// 键盘遥操作路径；显式退出仍负责关闭电机输出。
// ============================================================

#include "hardware/hardware_bridge.hpp"
#include "hardware/periodic_mit_sender.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <fstream>
#include <filesystem>
#include <iomanip>
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
// 状态估计/控制与独立 MIT 发送保持 500 Hz；未使能反馈轮询为 125 Hz。
constexpr std::uint32_t kMotorFrameDivider = 4;

const char * fsmStateLabel(FSM_StateName state) noexcept
{
  switch (state) {
    case FSM_StateName::PASSIVE: return "PASSIVE";
    case FSM_StateName::JOINT_PD: return "JOINT_PD";
    case FSM_StateName::STAND_UP: return "STAND_UP";
    case FSM_StateName::RECOVERY_STAND: return "RECOVERY_STAND";
    case FSM_StateName::BALANCE_STAND: return "BALANCE_STAND";
    case FSM_StateName::LOCOMOTION: return "LOCOMOTION";
    case FSM_StateName::LIE_DOWN: return "LIE_DOWN";
    default: return "INVALID";
  }
}

void reportMotorFaults(const dm1_hardware::HardwareSample & sample)
{
  static robot_log::Throttle log(1000);
  bool reporting = false;
  for (std::size_t joint = 0; joint < sample.motors.size(); ++joint) {
    const auto & motor = sample.motors[joint];
    if (motor.fault_code == 0 && motor.temperature_c <= 120.0F &&
      motor.rotor_temperature_c <= 120.0F &&
      (!motor.voltage_valid || (motor.voltage_v >= 18.0F && motor.voltage_v <= 60.0F)))
    {
      continue;
    }
    if (!reporting && !log.ready()) {return;}
    reporting = true;
    imu_log::print(
      imu_log::Level::Error,
      "[DM1][MOTOR-FAULT] joint=%s bus=%u can=0x%03x fault=0x%02x "
      "mos=%.1f rotor=%.1f voltage=%.2f voltage_valid=%s；仅报错，不自动失能。\n",
      kJointNames[joint], static_cast<unsigned int>(motor.bus),
      static_cast<unsigned int>(motor.can_id), static_cast<unsigned int>(motor.fault_code),
      static_cast<double>(motor.temperature_c), static_cast<double>(motor.rotor_temperature_c),
      static_cast<double>(motor.voltage_v), motor.voltage_valid ? "true" : "false");
  }
}

bool publishCommands(
  dm1_hardware::Dm1MitInterface & mit, dm1_hardware::PeriodicMitSender & sender,
  const dm1_hardware::Dm1MitInterface::CommandArray & commands, double now_s,
  dm1_hardware::PeriodicMitSender::Frames * published = nullptr)
{
  dm1_hardware::PeriodicMitSender::Frames frames{};
  if (!mit.prepareFrames(commands, now_s, frames) || !sender.publish(frames)) {return false;}
  if (published) {*published = frames;}
  return true;
}

void reportSendFailures(const dm1_hardware::PeriodicMitSender & sender)
{
  static robot_log::Throttle log(1000);
  if (sender.failedFrames() > 0 && log.ready()) {
    imu_log::print(
      imu_log::Level::Warning,
      "DM1 独立发送累计失败=%llu；同轮其余电机仍发送，失败电机下轮重试。\n",
      static_cast<unsigned long long>(sender.failedFrames()));
  }
}

void reportMotorTorque(const dm1_hardware::HardwareSample & sample)
{
  // 单个电机力矩提示暂时关闭；故障、发送失败和控制失败仍由独立告警路径输出。
  (void)sample;
}
}

// 构造硬件桥：绑定底层设备、关节标定和 RobotRunner，随后校验运行参数。
HardwareBridge::HardwareBridge(
  dm1_hardware::Dm1HardwareIo & hardware,
  const dm1_hardware::Dm1MitInterface::CalibrationArray & calibration,
  const Options & options)
: hardware_(hardware), calibration_(calibration), options_(options),
  runner_(options.control_time_step),
  // 默认 88 N*m 关节力矩饱和上限（电机峰值 97 N*m）：超限关节按比例缩小后
  // 照常发送，不丢弃整帧；可用 hardware_main --torque-limit 覆盖。
  mit_(hardware, calibration, options.feedback_timeout_s, false, options.torque_limit_nm)
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
  if ((!options_.set_zero &&
    (!std::isfinite(options_.startup_zero_tolerance_rad) ||
    options_.startup_zero_tolerance_rad <= 0.0F ||
    !std::isfinite(options_.startup_stationary_velocity_rad_s) ||
    options_.startup_stationary_velocity_rad_s <= 0.0F ||
    options_.startup_stable_samples <= 0)) ||
    options_.max_consecutive_cycle_overruns <= 0 ||
    options_.max_cycle_overruns_in_window <= 0 ||
    !std::isfinite(options_.cycle_overrun_window_s) || options_.cycle_overrun_window_s <= 0.0)
  {
    throw std::invalid_argument(
            "硬件启动阈值或控制周期超时参数无效");
  }
}

// ---------- 键盘遥操作控制 ----------

// 运行键盘遥操作循环：按键状态先经过命令仲裁，再决定电机解锁、站立和运动。
int HardwareBridge::runKeyboardControl(
  const std::chrono::steady_clock::time_point & start,
  const std::atomic_bool & stop_requested)
{
  teleop::KeyboardTeleop keyboard;
  // 每次键盘控制启动都新建一个带时间戳的 CSV，记录全部 12 个关节的
  // q（实际位置）、q_des（期望位置）和 tau（反馈力矩），用于步态分析。
  std::time_t log_ts = std::time(nullptr);
  char log_name[128];
  std::strftime(log_name, sizeof(log_name), "dm1_march_%Y%m%d_%H%M%S", std::localtime(&log_ts));
  const char * configured_log_dir = std::getenv("DM1_LOG_DIR");
  const std::filesystem::path log_dir(configured_log_dir ? configured_log_dir : "dm1_diagnostics/real_tests");
  std::error_code log_error;
  std::filesystem::create_directories(log_dir, log_error);
  const auto log_path = log_dir / (std::string(log_name) + "_" +
    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".csv");
  std::ofstream csv_log;
  if (!log_error) {csv_log.open(log_path);}
  if (!csv_log.is_open()) {
    imu_log::print(imu_log::Level::Error, "诊断 CSV 无法写入 %s；仅报错。\n", log_path.c_str());
  }
  if (csv_log.is_open()) {
    csv_log << "t_s,fsm,body_roll,body_pitch,body_yaw,body_z";
    for (const auto * leg : {"FR", "FL", "RR", "RL"}) {
      csv_log << ',' << leg << "_foot_body_x," << leg << "_foot_body_y," <<
        leg << "_foot_body_z," << leg << "_foot_world_x," <<
        leg << "_foot_world_y," << leg << "_foot_world_z";
    }
    for (const auto * name : kJointNames) {
      csv_log << ',' << name << "_q," << name << "_q_des," << name << "_tau,"
              << name << "_dq," << name << "_dq_des," << name << "_ff,"
              << name << "_kp," << name << "_kd," << name << "_tau_pos,"
              << name << "_tau_vel," << name << "_tau_total," << name << "_q_published,"
              << name << "_dq_published," << name << "_ff_published,"
              << name << "_kp_published," << name << "_kd_published";
    }
    csv_log << ",publish_attempted,publish_ok,published_valid,published_age_s,tx_failed_frames,"
            << "imu_sequence,imu_timestamp,imu_qw,imu_qx,imu_qy,imu_qz,"
            << "imu_ax,imu_ay,imu_az,imu_gx,imu_gy,imu_gz,imu_gravity,"
            << "est_pos_x,est_pos_y,est_pos_z,est_vel_world_x,est_vel_world_y,"
            << "est_vel_world_z,est_vel_body_x,est_vel_body_y,est_vel_body_z,"
            << "est_omega_body_x,est_omega_body_y,est_omega_body_z,"
            << "est_acc_world_x,est_acc_world_y,est_acc_world_z,model_body_z_contact\n";
    imu_log::print(imu_log::Level::Info, "关节数据记录到 %s（published 表示已入发送队列，不代表电机执行确认）\n", log_path.c_str());
  }
  // 打开 CAN 并不代表物理电机已经失能；接受键盘输入前先建立失效安全状态。
  hardware_.disableAll();
  if (!options_.command_source && !keyboard.start()) {
    imu_log::print(
      imu_log::Level::Error,
      "键盘控制需要可用的 TTY；电机保持失能。\n");
    return 9;
  }
  teleop::OperatorCommandArbiter arbiter;
  bool output_enabled = false;
  bool control_active = false;
  bool stand_up_pending = false;
  dm1_hardware::PeriodicMitSender sender(hardware_, stop_requested);
  dm1_hardware::PeriodicMitSender::Frames published_frames{};
  double last_published_s = -1.0;
  bool keyboard_fault_reported = false;
  auto next_cycle = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(options_.control_time_step));
  int result = 0;
  int consecutive_cycle_overruns = 0;
  std::deque<std::chrono::steady_clock::time_point> cycle_overruns;
  std::uint32_t motor_frame_phase = 0;
  // 独立发送线程以500Hz持续续发；通信/控制/电机故障不触发自动失能。
  const auto report_control_failure = [&](const char * reason) {
      static robot_log::Throttle failure_log(5000);
      if (failure_log.ready()) {
        imu_log::print(
          imu_log::Level::Warning,
          "DM1 控制帧已跳过：stage=%s fsm=%s output_enabled=%s；%s。\n",
          reason, fsmStateLabel(runner_.currentStateName()),
          output_enabled ? "true" : "false",
          output_enabled ? "独立线程持续保持上一有效命令，不失能" : "等待下一次正常周期");
      }
    };
  imu_log::print(
    imu_log::Level::Info,
    "键盘控制已启动：电机初始处于锁定状态。按 Shift+U/0 使能/失能，1 站立，"
    "2 原地踏步，P 趴下；按 H 查看帮助，Esc 失能并退出。\n");

  while (!stop_requested.load()) {
    reportSendFailures(sender);
    next_cycle += period;
    const auto cycle_start = std::chrono::steady_clock::now();
    const double now_s = std::chrono::duration<double>(cycle_start - start).count();
    auto commands = options_.command_source ? options_.command_source() : keyboard.consume();
    const bool input_failed = std::any_of(commands.begin(), commands.end(), [](const auto & command) {
        return command.type == teleop::CommandType::KeyboardFailure;
      }) || (keyboard.available() && !keyboard.healthy());
    commands.erase(std::remove_if(commands.begin(), commands.end(), [](const auto & command) {
        return command.type == teleop::CommandType::KeyboardFailure;
      }), commands.end());
    arbiter.applyBatch(commands);
    if (input_failed && !keyboard_fault_reported) {
      keyboard_fault_reported = true;
      imu_log::print(
        imu_log::Level::Warning,
        "键盘输入线程已停止；持续保持上一有效命令，可用Ctrl+C退出。\n");
    }
    arbiter.expireMotion(std::chrono::steady_clock::now());
    if (arbiter.takeHelpRequest()) {
      imu_log::print(
        imu_log::Level::Info,
        "按键：Shift+U 使能，0 失能，1 站立，2 原地踏步，3 双腿支撑测试，"
        "P 趴下，W/S 前进/后退，A/D 左移/右移，Q/E 旋转，"
        "空格停止，Esc 退出，H 帮助。\n");
    }
    if (arbiter.quitRequested()) {
      if (output_enabled) {
        sender.pause();
        hardware_.disableAll();
      }
      break;
    }
    if (arbiter.state() == teleop::MotorOutputState::Locked ||
      arbiter.state() == teleop::MotorOutputState::Fault)
    {
      if (output_enabled) {
        sender.pause();
        hardware_.disableAll();
        output_enabled = false;
      }
      control_active = false;
      stand_up_pending = false;
      sender.pause();
      last_published_s = -1.0;
    }

    dm1_hardware::HardwareSample sample;
    // 电机尚未解锁时，只发送驱动器的零增益反馈轮询，不生成
    // RobotRunner/MIT-PD 控制命令。
    const bool send_motor_frame = (motor_frame_phase++ % kMotorFrameDivider) == 0;
    if (!control_active && send_motor_frame && !hardware_.pollMotors()) {
      report_control_failure("pollMotors");
      std::this_thread::sleep_until(next_cycle);
      continue;
    }
    const bool hardware_read_valid = hardware_.read(sample, now_s);
    reportMotorFaults(sample);
    if (!hardware_read_valid) {
      report_control_failure("hardware.read(IMU/电机快照)");
      std::this_thread::sleep_until(next_cycle);
      continue;
    }
    if (!mit_.updateFeedback(sample.motors, now_s)) {
      report_control_failure("mit.updateFeedback(12路反馈校验)");
      std::this_thread::sleep_until(next_cycle);
      continue;
    }
    reportMotorTorque(sample);
    if (keyboard_fault_reported) {
      // 仍读取电机故障，输入断开不再生成新动作，也不自动失能。
      std::this_thread::sleep_until(next_cycle);
      continue;
    }
    const bool enable_requested =
      arbiter.state() == teleop::MotorOutputState::Enabling;
    if (enable_requested) {
      arbiter.takeEnableRequest();
      const bool safe_to_enable = stationaryLevelSample(sample) &&
        mit_.feedbackValid();
      if (!safe_to_enable || !hardware_.enableAll()) {
        static robot_log::Throttle enable_error_log(1000);
        if (enable_error_log.ready()) {
          imu_log::print(
            imu_log::Level::Error,
            "键盘解锁未确认：反馈、IMU、水平/静止状态或硬件使能检查失败；"
            "仅报错，下轮重试，不发送失能帧。\n");
        }
        next_cycle = std::chrono::steady_clock::now();
        std::this_thread::sleep_until(next_cycle + period);
        continue;
      }
      arbiter.markEnabled();
      output_enabled = true;
      sender.arm();
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
        runner_.setLocomotionGait(
          arbiter.motion() == teleop::Motion::TwoLegStand ?
          GaitType::TWO_LEG_STAND : GaitType::TROT);
        runner_.setControlMode(ControlMode::Locomotion);
      }
    }
    const float timestamp = static_cast<float>(now_s);
    sample.imu.timestamp = timestamp;
    // 在 updateHardwareFeedback 之前快照校准后的关节反馈，用于 CSV 记录。
    const auto log_js = mit_.jointStates(timestamp);
    const bool feedback_valid = !control_active || runner_.updateHardwareFeedback(
      sample.imu, log_js, timestamp);
    if (control_active && !feedback_valid) {
      report_control_failure("状态估计器拒绝硬件反馈");
      std::this_thread::sleep_until(next_cycle);
      continue;
    }

    const FSM_StateName state_before_control = runner_.currentStateName();
    const bool control_valid = !control_active || runner_.run();
    if (control_active && control_valid && arbiter.motionActive() &&
      state_before_control == FSM_StateName::LOCOMOTION &&
      runner_.currentStateName() == FSM_StateName::BALANCE_STAND)
    {
      arbiter.stopMotion();
      runner_.setLocomotionVelocityCommand(0.0F, 0.0F, 0.0F);
      imu_log::print(
        imu_log::Level::Warning,
        "Locomotion 安全回退已取消当前运动请求；保持 BalanceStand，需重新按运动键。\n");
    }
    if (control_active && control_valid && stand_up_pending && runner_.requestStandUp()) {
      stand_up_pending = false;
      imu_log::print(imu_log::Level::Info, "站立请求已接受。\n");
    }
    if (control_active && !control_valid) {
      report_control_failure("RobotRunner 拒绝控制帧");
      std::this_thread::sleep_until(next_cycle);
      continue;
    }
    const bool publish_attempted = control_active &&
      arbiter.state() == teleop::MotorOutputState::Enabled && output_enabled;
    const bool publish_ok = publish_attempted &&
      publishCommands(mit_, sender, runner_.jointCommands(), now_s, &published_frames);
    if (publish_ok) {last_published_s = now_s;}
    if (publish_attempted && !publish_ok) {
      report_control_failure("MIT命令校验/发布失败");
    }
    // 每个控制周期写一行 CSV：12 个关节的实际位置、期望位置和反馈力矩。
    if (control_active && control_valid && options_.control_observer) {
      options_.control_observer(runner_);
    }
    if (control_active && csv_log.is_open()) {
      csv_log << std::fixed << std::setprecision(9) << now_s
              << ',' << fsmStateLabel(runner_.currentStateName());
      const auto & estimate = runner_.stateEstimate();
      csv_log << ',' << estimate.rpy.x() << ',' << estimate.rpy.y()
              << ',' << estimate.rpy.z() << ',' << estimate.position_world.z();
      float model_body_z = 0.0F;
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        const auto leg_id = static_cast<LegId>(leg);
        Vec3<float> foot_from_hip;
        computeLegJacobianAndPosition(runner_.quadruped(), log_js[leg].position,
          static_cast<Mat3<float> *>(nullptr), &foot_from_hip, leg_id);
        const Vec3<float> foot_body =
          runner_.quadruped().hipLocation(leg_id) + foot_from_hip;
        const Vec3<float> foot_world = estimate.position_world +
          estimate.rotation_world_from_body * foot_body;
        csv_log << ',' << foot_body.x() << ',' << foot_body.y() << ',' << foot_body.z()
                << ',' << foot_world.x() << ',' << foot_world.y() << ',' << foot_world.z();
        model_body_z += (runner_.quadruped().leg(leg_id).foot_radius -
          (estimate.rotation_world_from_body *
          foot_body).z()) / kNumLegs;
      }
      const auto & cmds = runner_.jointCommands();
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
          const std::size_t idx = leg * kJointsPerLeg + joint;
          const auto ji = static_cast<Eigen::Index>(joint);
          const float q = log_js[leg].position[ji];
          const float q_des = cmds[leg].position_desired[ji];
          const float tau = idx < sample.motors.size() ?
            sample.motors[idx].torque : 0.0F;
          csv_log << ',' << q << ',' << q_des << ',' << tau
                  << ',' << log_js[leg].velocity[ji]
                  << ',' << cmds[leg].velocity_desired[ji]
                  << ',' << cmds[leg].torque_feedforward[ji]
                  << ',' << cmds[leg].kp[ji] << ',' << cmds[leg].kd[ji];
          const float tau_pos = cmds[leg].kp[ji] * (q_des - q);
          const float tau_vel = cmds[leg].kd[ji] * (cmds[leg].velocity_desired[ji] - log_js[leg].velocity[ji]);
          csv_log << ',' << tau_pos << ',' << tau_vel << ',' << (tau_pos + tau_vel + cmds[leg].torque_feedforward[ji]);
          // prepareFrames 按关节交错 FR/RR/FL/RL；此排列也是其逆排列。
          constexpr std::array<std::size_t, kNumLegs> frame_leg_order{0, 2, 1, 3};
          const auto & frame = published_frames[joint * kNumLegs + frame_leg_order[leg]];
          const auto & calibration = calibration_[idx];
          const float published_q = calibration.direction * (frame.position - calibration.zero_position);
          for (const float value : {published_q, calibration.direction * frame.velocity,
            calibration.direction * frame.torque, frame.kp, frame.kd})
          {
            csv_log << ',' << (last_published_s >= 0 ? value : std::nanf(""));
          }
        }
      }
      const auto & imu = sample.imu;
      csv_log << ',' << publish_attempted << ',' << publish_ok << ',' << (last_published_s >= 0)
              << ',' << (last_published_s >= 0 ? now_s - last_published_s : std::nan(""))
              << ',' << sender.failedFrames() << ',' << imu.sequence << ',' << imu.timestamp
              << ',' << imu.orientation_world_from_body.w()
              << ',' << imu.orientation_world_from_body.x() << ',' << imu.orientation_world_from_body.y()
              << ',' << imu.orientation_world_from_body.z() << ',' << imu.acceleration_body.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ",", ","))
              << ',' << imu.angular_velocity_body.x() << ',' << imu.angular_velocity_body.y()
              << ',' << imu.angular_velocity_body.z() << ',' << imu.gravity_magnitude
              << ',' << estimate.position_world.x() << ',' << estimate.position_world.y()
              << ',' << estimate.position_world.z() << ',' << estimate.velocity_world.x()
              << ',' << estimate.velocity_world.y() << ',' << estimate.velocity_world.z()
              << ',' << estimate.velocity_body.x() << ',' << estimate.velocity_body.y()
              << ',' << estimate.velocity_body.z() << ',' << estimate.angular_velocity_body.x()
              << ',' << estimate.angular_velocity_body.y() << ',' << estimate.angular_velocity_body.z()
              << ',' << estimate.acceleration_world.x() << ',' << estimate.acceleration_world.y()
              << ',' << estimate.acceleration_world.z()
              << ',' << model_body_z << '\n';
      if (motor_frame_phase % 50 == 0) {csv_log.flush();}
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
          imu_log::Level::Warning,
          "DM1 键盘控制周期超过截止时间：fsm=%s elapsed_ms=%.3f late_ms=%.3f "
          "send_motor_frame=%s；已重新同步周期。\n",
          fsmStateLabel(runner_.currentStateName()),
          std::chrono::duration<double, std::milli>(finished - cycle_start).count(),
          std::chrono::duration<double, std::milli>(finished - next_cycle).count(),
          send_motor_frame ? "true" : "false");
        consecutive_cycle_overruns = 0;
        cycle_overruns.clear();
      }
      // 周期性硬件控制不依赖键盘事件；一次较慢的控制周期不能让后续周期
      // 永远落后于时间表，否则会连续忙循环并再次放大 CAN/USB 调度压力。
      next_cycle = finished;
    } else {
      consecutive_cycle_overruns = 0;
    }
    std::this_thread::sleep_until(next_cycle);
  }
  sender.shutdown();
  keyboard.stop();
  sender.pause();
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

// 执行启动采样；--set-zero 直接写入全部电机零位，普通启动验证硬件稳定性。
bool HardwareBridge::performStartupZeroCalibration(
  const std::chrono::steady_clock::time_point & start,
  const std::atomic_bool & stop_requested)
{
  if (options_.set_zero) {
    imu_log::print(
      imu_log::Level::Warning,
      "DM1 零位维护：跳过反馈/姿态/位置检查，直接写入全部电机零位。\n");
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
    imu_log::print(imu_log::Level::Warning, "DM1 全部电机零位写入完成。\n");
    return true;
  }
  const bool read_only_diagnostic =
    !options_.enable_output && !options_.keyboard_control;
  const bool single_feedback_round =
    read_only_diagnostic || options_.keyboard_control;
  const int required_stable_samples = single_feedback_round ? 1 : options_.startup_stable_samples;
  const auto sequence_timeout = single_feedback_round ?
    std::chrono::milliseconds(500) : kStartupSequenceTimeout;
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
  const auto sequence_deadline = std::chrono::steady_clock::now() + sequence_timeout;
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
  while (stable_samples < required_stable_samples) {
    if (stop_requested.load() || std::chrono::steady_clock::now() >= sequence_deadline) {
      if (single_feedback_round) {
        imu_log::print(
          imu_log::Level::Error,
          "启动检查被拒绝：未收到 IMU 和 12 个电机的完整首轮反馈。\n");
      } else {
        imu_log::print(
          imu_log::Level::Error,
          "启动检查被拒绝：在 %d ms 内未收到连续更新的 IMU/电机反馈。\n",
          static_cast<int>(sequence_timeout.count()));
      }
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
        "启动检查被拒绝：机器人必须连续 %d 帧静止、水平，且反馈健康。\n",
        required_stable_samples);
      return false;
    }
    if (options_.enable_output &&
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

  if (read_only_diagnostic) {
    imu_log::print(
      imu_log::Level::Info,
      "DM1 12 路电机首轮反馈通过；将持续监测 IMU，电机不再要求连续上报。\n");
  } else if (options_.keyboard_control) {
    imu_log::print(
      imu_log::Level::Info,
      "DM1 键盘控制首轮反馈通过；等待 Shift+U 解锁。\n");
  } else {
    imu_log::print(
      imu_log::Level::Info,
      "DM1 启动稳定窗口通过：连续 %d 帧静止、水平且反馈健康。\n",
      required_stable_samples);
  }

  imu_log::print(
    imu_log::Level::Info,
    "DM1 启动稳定性检查通过；使能后将执行平滑回零，未修改电机参数。\n");
  return true;
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
  if (!options_.enable_output) {
    auto next_cycle = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(options_.control_time_step));
    while (!stop_requested.load()) {
      next_cycle += period;
      const double now_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      ImuData<float> imu_sample;
      if (!hardware_.readImu(imu_sample, now_s)) {
        imu_log::print(
          imu_log::Level::Error,
          "DM1 IMU 实时读取失败；电机保持失能。\n");
        hardware_.disableAll();
        hardware_.close();
        return 3;
      }
      std::this_thread::sleep_until(next_cycle);
    }
    hardware_.close();
    return 0;
  }
  auto next_cycle = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(options_.control_time_step));
  bool stand_up_accepted = false;
  int result = 0;
  int consecutive_cycle_overruns = 0;
  std::deque<std::chrono::steady_clock::time_point> cycle_overruns;
  bool output_enabled = false;
  dm1_hardware::PeriodicMitSender sender(hardware_, stop_requested);
  std::size_t consecutive_output_failures = 0;
  std::uint32_t motor_frame_phase = 0;
  while (!stop_requested.load()) {
    reportSendFailures(sender);
    next_cycle += period;
    const auto cycle_start = std::chrono::steady_clock::now();
    double now_s = std::chrono::duration<double>(cycle_start - start).count();
    dm1_hardware::HardwareSample sample;
    const bool send_motor_frame = (motor_frame_phase++ % kMotorFrameDivider) == 0;

    const bool hardware_read_valid = hardware_.read(sample, now_s);
    reportMotorFaults(sample);
    const bool motor_feedback_valid = hardware_read_valid &&
      mit_.updateFeedback(sample.motors, now_s);
    if (!motor_feedback_valid) {
      ++consecutive_output_failures;
      if (consecutive_output_failures == 1 ||
        (consecutive_output_failures % 500) == 0)
      {
        imu_log::print(
          imu_log::Level::Warning,
          "DM1 输入失败：stage=%s fsm=%s now=%.6f consecutive=%zu；%s。\n",
          hardware_read_valid ? "mit.updateFeedback(12路反馈校验)" :
          "hardware.read(IMU/电机快照)", fsmStateLabel(runner_.currentStateName()),
          now_s, consecutive_output_failures,
          output_enabled ? "独立线程持续保持上一有效命令，不失能" : "尚未使能");
      }
      std::this_thread::sleep_until(next_cycle);
      continue;
    }
    reportMotorTorque(sample);

    const float timestamp = static_cast<float>(now_s);
    // 控制器内部使用桥的相对单调时钟；电机原始时间戳已经在上面的
    // updateFeedback() 中用于检查驱动器是否确实返回了新帧。
    sample.imu.timestamp = timestamp;
    const auto joints = mit_.jointStates(timestamp);
    const bool runner_feedback_valid =
      runner_.updateHardwareFeedback(sample.imu, joints, timestamp);
    const bool runner_control_valid = runner_feedback_valid && runner_.run();
    if (!runner_control_valid) {
      ++consecutive_output_failures;
      if (consecutive_output_failures == 1 ||
        (consecutive_output_failures % 500) == 0)
      {
        imu_log::print(
          imu_log::Level::Warning,
          "DM1 控制链失败：stage=%s fsm=%s now=%.6f consecutive=%zu；%s。\n",
          runner_feedback_valid ? "RobotRunner.run" : "updateHardwareFeedback",
          fsmStateLabel(runner_.currentStateName()), now_s,
          consecutive_output_failures,
          output_enabled ? "独立线程持续保持上一有效命令，不失能" : "尚未使能");
      }
      std::this_thread::sleep_until(next_cycle);
      continue;
    }
    consecutive_output_failures = 0;

    if (options_.request_stand_up && !stand_up_accepted) {
      stand_up_accepted = runner_.requestStandUp();
    }

    if (!output_enabled) {
      static robot_log::Throttle enable_error_log(1000);
      // 首个控制器帧必须完整且通过协议安全检查后，才能使能物理输出。
      if (!mit_.validateCommands(runner_.jointCommands(), now_s)) {
        if (enable_error_log.ready()) {
          imu_log::print(imu_log::Level::Error,
            "DM1 初始控制指令校验失败；仅报错，等待有效命令，不发送失能帧。\n");
        }
        std::this_thread::sleep_until(next_cycle);
        continue;
      }
      if (!hardware_.enableAll()) {
        if (enable_error_log.ready()) {
          imu_log::print(imu_log::Level::Error,
            "DM1 硬件使能未确认；仅报错，下轮重试，不发送失能帧。\n");
        }
        next_cycle = std::chrono::steady_clock::now();
        std::this_thread::sleep_until(next_cycle + period);
        continue;
      }
      output_enabled = true;
      sender.arm();
      imu_log::print(
        imu_log::Level::Info,
        "DM1 电机已使能；正在将电机平滑回到零位。\n");
    }
    if (!publishCommands(mit_, sender, runner_.jointCommands(), now_s)) {
      static robot_log::Throttle publish_log(1000);
      if (publish_log.ready()) {
        imu_log::print(
          imu_log::Level::Warning,
          "DM1 新控制帧未发布；独立发送持续保持上一有效命令，不失能。\n");
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
        "DM1 控制周期超过截止时间：fsm=%s elapsed_ms=%.3f late_ms=%.3f "
        "send_motor_frame=%s consecutive=%d window=%zu。\n",
        fsmStateLabel(runner_.currentStateName()),
        std::chrono::duration<double, std::milli>(finished - cycle_start).count(),
        std::chrono::duration<double, std::milli>(finished - next_cycle).count(),
        send_motor_frame ? "true" : "false", consecutive_cycle_overruns,
        cycle_overruns.size());
      // 丢弃已经错过的旧截止时间，避免一次较慢的 WBC 周期让后续周期
      // 永远落后并被连续计为超时；真正持续超过周期仍由下面的阈值保护。
      next_cycle = finished;
      if (consecutive_cycle_overruns >= options_.max_consecutive_cycle_overruns ||
        static_cast<int>(cycle_overruns.size()) >= options_.max_cycle_overruns_in_window)
      {
        imu_log::print(
          imu_log::Level::Warning,
          "DM1 控制循环超时策略已触发；已丢弃过期周期。\n");
        result = 6;
        consecutive_cycle_overruns = 0;
        cycle_overruns.clear();
      }
    } else {
      consecutive_cycle_overruns = 0;
    }
    std::this_thread::sleep_until(next_cycle);
  }

  if (result != 0 || stop_requested.load()) {
    imu_log::print(
      imu_log::Level::Warning,
      "DM1 硬件循环即将 close()：result=%d stop_requested=%s output_enabled=%s。\n",
      result, stop_requested.load() ? "true" : "false",
      output_enabled ? "true" : "false");
  }
  sender.shutdown();
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
