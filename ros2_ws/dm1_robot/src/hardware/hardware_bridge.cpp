/*! @file hardware_bridge.cpp
 *  @brief DM1 真机硬件桥接实现。
 *
 *  本文件负责硬件反馈读取、启动安全检查、OpenDog 控制循环以及
 *  键盘遥操作路径；显式退出仍负责关闭电机输出。
 */

#include "hardware/hardware_bridge.hpp"
#include "hardware/periodic_mit_sender.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "common/console_log.hpp"

namespace
{
/// 12 个关节名，顺序与标定文件和关节数组一致，用于日志和 CSV 表头。
constexpr std::array<const char *, kNumJoints> kJointNames{
  "FR_hip", "FR_thigh", "FR_calf",
  "FL_hip", "FL_thigh", "FL_calf",
  "RR_hip", "RR_thigh", "RR_calf",
  "RL_hip", "RL_thigh", "RL_calf"};
// 状态估计/控制与独立 MIT 发送保持 500 Hz；未使能反馈轮询为 125 Hz。
constexpr std::uint32_t kMotorFrameDivider = control_config::hardware::motor_frame_divider;

// DM1 足端半径，m；与 dm1_opendog.urdf 的足端碰撞球一致。
constexpr float kFootRadius = 0.02F;

// 机身系下的足端位置，仅用于 CSV 诊断。几何参数取自 dm1_opendog.urdf：
// 髋 roll 轴为 X，大腿/小腿 pitch 轴为 Y；关节角即电机角（OpenDog 机械零位）。
Vec3<float> footPositionBody(LegId leg, const Vec3<float> & q)
{
  const bool left = leg == LegId::FL || leg == LegId::RL;
  const bool rear = leg == LegId::RR || leg == LegId::RL;
  const float side = left ? 1.0F : -1.0F;
  const float longitudinal = rear ? -1.0F : 1.0F;
  const Vec3<float> hip_location(
    longitudinal * 0.16175F, side * 0.059997F, -0.061498F);
  const Vec3<float> hip_to_thigh(longitudinal * 0.049998F, side * 0.019503F, 0.0F);
  const Vec3<float> thigh_to_calf(-0.2F, side * 0.07825F, 0.0F);
  const Vec3<float> calf_to_foot(-0.17362F, 0.0F, -0.14312F);
  const Mat3<float> rotation_hip =
    Eigen::AngleAxisf(q[0], Vec3<float>::UnitX()).toRotationMatrix();
  const Mat3<float> rotation_thigh = rotation_hip *
    Eigen::AngleAxisf(q[1], Vec3<float>::UnitY()).toRotationMatrix();
  const Mat3<float> rotation_calf = rotation_thigh *
    Eigen::AngleAxisf(q[2], Vec3<float>::UnitY()).toRotationMatrix();
  return hip_location + rotation_hip * hip_to_thigh +
         rotation_thigh * thigh_to_calf + rotation_calf * calf_to_foot;
}

/// 每秒最多报告一次故障码、温度或电压异常的电机；只报错，不失能。
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

/// 校验并转换命令，交给发送线程；published 非空时返回实际发布的帧。
bool publishCommands(
  dm1_hardware::Dm1MitInterface & mit, dm1_hardware::PeriodicMitSender & sender,
  const dm1_hardware::Dm1MitInterface::CommandArray & commands,
  dm1_hardware::PeriodicMitSender::Frames * published = nullptr)
{
  dm1_hardware::PeriodicMitSender::Frames frames{};
  if (!mit.prepareFrames(commands, frames) || !sender.publish(frames)) {return false;}
  if (published) {*published = frames;}
  return true;
}

/// 每秒最多报告一次发送线程累计的失败帧数。
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

/// 后台 CSV 写入线程，避免格式化和写盘阻塞 500 Hz 控制循环。
/// 只有不可变的数据快照会进入该线程，线程内不调用任何硬件/控制接口。
class AsyncCsvLog
{
public:
  AsyncCsvLog()
  : worker_([this]() {run();}) {}
  ~AsyncCsvLog()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    ready_.notify_one();
    worker_.join();  // 在 CSV 文件流析构前写完队列中的全部记录。
  }
  /// 追加一条记录；record 在后台线程中执行写入。
  void append(std::function<void()> record)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      records_.push_back(std::move(record));
    }
    ready_.notify_one();
  }

private:
  /// 批量取出队列中的记录依次执行；停止且队列为空时退出。
  void run()
  {
    for (;; ) {
      std::deque<std::function<void()>> records;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock, [this]() {return stopping_ || !records_.empty();});
        if (records_.empty() && stopping_) {return;}
        records.swap(records_);
      }
      for (auto & record : records) {record();}
    }
  }
  std::mutex mutex_;                           ///< 保护 records_ 和 stopping_。
  std::condition_variable ready_;              ///< 有新记录或停止时唤醒。
  std::deque<std::function<void()>> records_;  ///< 待写入的记录。
  bool stopping_ = false;                      ///< 请求退出。
  std::thread worker_;                         ///< 写入线程，最后构造。
};
}

// 构造硬件桥：绑定底层设备、关节标定和唯一选中的算法 runner。
HardwareBridge::HardwareBridge(
  dm1_hardware::Dm1HardwareIo & hardware,
  const dm1_hardware::Dm1MitInterface::CalibrationArray & calibration,
  const Options & options)
: hardware_(hardware), calibration_(calibration), options_(options),
  runner_(options.algorithm_config_directory, options.control_time_step),
  // 默认 88 N*m 关节力矩饱和上限（电机峰值 97 N*m）：超限关节按比例缩小后
  // 照常发送，不丢弃整帧；可用 hardware_main --torque-limit 覆盖。
  mit_(calibration, options.torque_limit_nm)
{}

// ---------- 键盘遥操作控制 ----------

// 运行键盘遥操作循环：按键状态先经过命令仲裁，再决定电机解锁、站立和运动。
int HardwareBridge::runKeyboardControl(
  const std::chrono::steady_clock::time_point & start,
  const std::atomic_bool & stop_requested)
{
  teleop::KeyboardTeleop keyboard;
  ControlOutput algorithm_output;
  // 每次键盘控制启动都新建一个带时间戳的 CSV，记录全部 12 个关节的
  // q（实际位置）、q_des（期望位置）和 tau（反馈力矩），用于步态分析。
  std::time_t log_ts = std::time(nullptr);
  char log_name[128];
  std::strftime(log_name, sizeof(log_name), "dm1_march_%Y%m%d_%H%M%S", std::localtime(&log_ts));
  const char * configured_log_dir = std::getenv("DM1_LOG_DIR");
  const std::filesystem::path log_dir(configured_log_dir ? configured_log_dir :
    "dm1_diagnostics/real_tests");
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
    csv_log << ",opendog_diag_valid,opendog_active_mode,opendog_planned_mode";
    for (const auto * leg : {"FR", "FL", "RR", "RL"}) {
      csv_log << ',' << "opendog_" << leg << "_contact"
              << ',' << "opendog_" << leg << "_force_des_x"
              << ',' << "opendog_" << leg << "_force_des_y"
              << ',' << "opendog_" << leg << "_force_des_z"
              << ',' << "opendog_" << leg << "_force_wbc_x"
              << ',' << "opendog_" << leg << "_force_wbc_y"
              << ',' << "opendog_" << leg << "_force_wbc_z";
    }
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
    for (const auto * name : kJointNames) {
      csv_log << ',' << name << "_fault_code," << name << "_mos_temperature_c,"
              << name << "_rotor_temperature_c," << name << "_voltage_v,"
              << name << "_voltage_valid," << name << "_feedback_sequence,"
              << name << "_feedback_timestamp," << name << "_feedback_age_s,"
              << name << "_health_valid";
    }
    csv_log << ",publish_attempted,publish_ok,published_valid,published_age_s,tx_failed_frames,"
            << "imu_sequence,imu_timestamp,imu_qw,imu_qx,imu_qy,imu_qz,"
            << "imu_ax,imu_ay,imu_az,imu_gx,imu_gy,imu_gz,imu_gravity,"
            << "est_pos_x,est_pos_y,est_pos_z,est_vel_world_x,est_vel_world_y,"
            << "est_vel_world_z,est_vel_body_x,est_vel_body_y,est_vel_body_z,"
            << "est_omega_body_x,est_omega_body_y,est_omega_body_z,"
            << "est_acc_world_x,est_acc_world_y,est_acc_world_z,model_body_z_contact\n";
    imu_log::print(
      imu_log::Level::Info, "关节数据记录到 %s（published 表示已入发送队列，不代表电机执行确认）\n",
      log_path.c_str());
  }
  // 打开 CAN 并不代表物理电机已经失能；接受键盘输入前先建立失效安全状态。
  hardware_.disableAll();
  if (!options_.command_source && !keyboard.start()) {
    imu_log::print(
      imu_log::Level::Error,
      "键盘控制需要可用的 TTY；电机保持失能。\n");
    return 9;
  }
  AsyncCsvLog csv_worker;
  teleop::OperatorCommandArbiter arbiter;
  bool output_enabled = false;
  bool control_active = false;
  dm1_hardware::PeriodicMitSender sender(hardware_, stop_requested);
  dm1_hardware::PeriodicMitSender::Frames published_frames{};
  double last_published_s = -1.0;
  bool keyboard_fault_reported = false;
  auto next_cycle = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(options_.control_time_step));
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
          reason, runner_.stateLabel(),
          output_enabled ? "true" : "false",
          output_enabled ? "独立线程持续保持上一有效命令，不失能" : "等待下一次正常周期");
      }
    };
  imu_log::print(
    imu_log::Level::Info,
    "键盘控制已启动：电机初始处于锁定状态。按 Shift+U/0 使能/失能，1 站立，"
    "2 原地踏步，P 趴下；按 H 查看帮助，Esc 失能并退出。\n");

  while (!stop_requested.load()) {
    // 1. 收集并仲裁本周期的操作员命令
    reportSendFailures(sender);
    next_cycle += period;
    const auto cycle_start = std::chrono::steady_clock::now();
    const double now_s = std::chrono::duration<double>(cycle_start - start).count();
    auto commands = options_.command_source ? options_.command_source() : keyboard.consume();
    const bool input_failed = std::any_of(
      commands.begin(), commands.end(), [](const auto & command) {
        return command.type == teleop::CommandType::KeyboardFailure;
      }) || (keyboard.available() && !keyboard.healthy());
    commands.erase(
      std::remove_if(
        commands.begin(), commands.end(), [](const auto & command) {
          return command.type == teleop::CommandType::KeyboardFailure;
        }), commands.end());
    arbiter.applyBatch(commands);
    runner_.acceptCommands(arbiter, commands);
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
        "OpenDog：2 连续原地踏步，3 左前/右后单次踏步，4 右前/左后单次踏步。\n");
      imu_log::print(
        imu_log::Level::Info,
        "按键：Shift+U 使能，0 失能，1 站立，2 原地踏步，3 双腿支撑测试，"
        "P 趴下，W/S 前进/后退，A/D 左移/右移，Q/E 旋转，"
        "空格停止，Esc 退出，H 帮助。\n");
    }
    // 2. 退出或锁定：先暂停发送线程，再失能电机
    if (arbiter.quitRequested()) {
      if (output_enabled) {
        sender.pause();
        hardware_.disableAll();
      }
      break;
    }
    if (arbiter.state() == teleop::MotorOutputState::Locked) {
      sender.pause();
      if (output_enabled) {
        hardware_.disableAll();
        output_enabled = false;
      }
      control_active = false;
      runner_.cancelPendingRequests();
      last_published_s = -1.0;
    }

    // 3. 读取并校验 IMU 与 12 路电机反馈
    dm1_hardware::HardwareSample sample;
    // 电机尚未解锁时，只发送驱动器的零增益反馈轮询，不生成
    // OpenDog/MIT-PD 控制命令。
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
    if (!mit_.updateFeedback(sample.motors)) {
      report_control_failure("mit.updateFeedback(12路反馈校验)");
      std::this_thread::sleep_until(next_cycle);
      continue;
    }
    if (keyboard_fault_reported) {
      // 仍读取电机故障，输入断开不再生成新动作，也不自动失能。
      std::this_thread::sleep_until(next_cycle);
      continue;
    }
    // 4. 处理使能请求：必须静止、水平且反馈有效，才使能电机并复位算法
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
      // 算法各自复位其初始化轨迹和命令状态。
      runner_.reset();
      control_active = true;
      imu_log::print(
        imu_log::Level::Info,
        "DM1 电机已使能；正在将电机平滑回到零位，完成后可按 1 站立。\n");
    }
    // 5. 更新期望状态并运行算法
    control_active = runner_.handleCommands(arbiter, sample.imu, control_active);
    const float timestamp = static_cast<float>(now_s);
    sample.imu.timestamp = timestamp;
    // 快照已转换到模型坐标的关节反馈，同时作为算法输入和 CSV 记录。
    const auto log_js = mit_.jointStates(timestamp);
    bool control_valid = !control_active;
    if (control_active) {
      ControlInputSnapshot input;
      input.imu = sample.imu;
      input.joints = log_js;
      input.timestamp = timestamp;
      control_valid = runner_.compute(input, algorithm_output);
    }
    if (control_active && !control_valid) {
      report_control_failure("算法 runner 拒绝控制帧");
      std::this_thread::sleep_until(next_cycle);
      continue;
    }
    // 6. 校验命令并交给独立发送线程，记录数据新鲜度
    const bool publish_attempted = control_active &&
      arbiter.state() == teleop::MotorOutputState::Enabled && output_enabled;
    const bool publish_ok = publish_attempted &&
      publishCommands(mit_, sender, algorithm_output.commands, &published_frames);
    if (publish_ok) {last_published_s = now_s;}
    if (publish_attempted) {
      const double publish_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      double motor_age_ms = 0.0;
      for (const auto & motor : sample.motors) {
        motor_age_ms = std::max(motor_age_ms, 1000.0 * (publish_time - motor.timestamp));
      }
      runner_.reportPublishTiming(1000.0 * (publish_time - timestamp), motor_age_ms, publish_ok);
    }
    if (publish_attempted && !publish_ok) {
      report_control_failure("MIT命令校验/发布失败");
    }
    // 7. 每个控制周期提交快照；CSV 格式化、足端诊断和写盘在独立线程执行。
    if (control_active && csv_log.is_open()) {
      csv_worker.append(
        [&csv_log, & calibration = calibration_,
        now_s, state = runner_.stateLabel(), algorithm_output, log_js, sample,
        published_frames, last_published_s, publish_attempted, publish_ok,
        failed_frames = sender.failedFrames(), motor_frame_phase]() {
          csv_log << std::fixed << std::setprecision(9) << now_s
                  << ',' << state;
          const auto & estimate = algorithm_output.estimate;
          csv_log << ',' << estimate.rpy.x() << ',' << estimate.rpy.y()
                  << ',' << estimate.rpy.z() << ',' << estimate.position_world.z();
          const auto & opendog = algorithm_output.opendog;
          csv_log << ',' << opendog.valid << ',' << opendog.active_mode
                  << ',' << opendog.planned_mode;
          for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            csv_log << ',' << opendog.contact[leg];
            for (std::size_t axis = 0; axis < 3; ++axis) {
              csv_log << ',' << opendog.force_des[leg][axis];
            }
            for (std::size_t axis = 0; axis < 3; ++axis) {
              csv_log << ',' << opendog.force_wbc[leg][axis];
            }
          }
          float model_body_z = 0.0F;
          for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            const auto leg_id = static_cast<LegId>(leg);
            const Vec3<float> foot_body = footPositionBody(leg_id, log_js[leg].position);
            const Vec3<float> foot_world = estimate.position_world +
            estimate.rotation_world_from_body * foot_body;
            csv_log << ',' << foot_body.x() << ',' << foot_body.y() << ',' << foot_body.z()
                    << ',' << foot_world.x() << ',' << foot_world.y() << ',' << foot_world.z();
            model_body_z += (kFootRadius -
            (estimate.rotation_world_from_body *
            foot_body).z()) / kNumLegs;
          }
          const auto & cmds = algorithm_output.commands;
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
              const float tau_vel = cmds[leg].kd[ji] *
              (cmds[leg].velocity_desired[ji] - log_js[leg].velocity[ji]);
              csv_log << ',' << tau_pos << ',' << tau_vel << ',' <<
                (tau_pos + tau_vel + cmds[leg].torque_feedforward[ji]);
              // prepareFrames 按关节交错 FR/RR/FL/RL；此排列也是其逆排列。
              constexpr std::array<std::size_t, kNumLegs> frame_leg_order{0, 2, 1, 3};
              const auto & frame = published_frames[joint * kNumLegs + frame_leg_order[leg]];
              const auto & joint_calibration = calibration[idx];
              const float published_q = joint_calibration.direction *
              (frame.position - joint_calibration.zero_position);
              for (const float value : {published_q, joint_calibration.direction * frame.velocity,
                joint_calibration.direction * frame.torque, frame.kp, frame.kd})
              {
                csv_log << ',' << (last_published_s >= 0 ? value : std::nanf(""));
              }
            }
          }
          for (const auto & motor : sample.motors) {
            const double feedback_age_s = now_s - motor.timestamp;
            csv_log << ',' << motor.fault_code << ',' << motor.temperature_c
                    << ',' << motor.rotor_temperature_c << ',' << motor.voltage_v
                    << ',' << motor.voltage_valid << ',' << motor.sequence
                    << ',' << motor.timestamp << ',' << feedback_age_s
                    << ',' << motor.health_valid;
          }
          const auto & imu = sample.imu;
          csv_log << ',' << publish_attempted << ',' << publish_ok << ',' << (last_published_s >= 0)
                  << ',' << (last_published_s >= 0 ? now_s - last_published_s : std::nan(""))
                  << ',' << failed_frames << ',' << imu.sequence << ',' << imu.timestamp
                  << ',' << imu.orientation_world_from_body.w()
                  << ',' << imu.orientation_world_from_body.x() << ',' <<
            imu.orientation_world_from_body.y()
                  << ',' << imu.orientation_world_from_body.z() << ',' <<
            imu.acceleration_body.transpose().format(
            Eigen::IOFormat(
              Eigen::StreamPrecision, 0, ",",
              ","))
                  << ',' << imu.angular_velocity_body.x() << ',' << imu.angular_velocity_body.y()
                  << ',' << imu.angular_velocity_body.z() << ',' << imu.gravity_magnitude
                  << ',' << estimate.position_world.x() << ',' << estimate.position_world.y()
                  << ',' << estimate.position_world.z() << ',' << estimate.velocity_world.x()
                  << ',' << estimate.velocity_world.y() << ',' << estimate.velocity_world.z()
                  << ',' << estimate.velocity_body.x() << ',' << estimate.velocity_body.y()
                  << ',' << estimate.velocity_body.z() << ',' << estimate.angular_velocity_body.x()
                  << ',' << estimate.angular_velocity_body.y() << ',' <<
            estimate.angular_velocity_body.z()
                  << ',' << estimate.acceleration_world.x() << ',' << estimate.acceleration_world.y()
                  << ',' << estimate.acceleration_world.z()
                  << ',' << model_body_z << '\n';
          if (motor_frame_phase % 50 == 0) {csv_log.flush();}
        });
    }
    // 8. 周期超时统计：连续或窗口内超时次数达到阈值时告警并重新同步
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
          "poll_slot=%s command_published=%s；已重新同步周期。\n",
          runner_.stateLabel(),
          std::chrono::duration<double, std::milli>(finished - cycle_start).count(),
          std::chrono::duration<double, std::milli>(finished - next_cycle).count(),
          send_motor_frame ? "true" : "false", publish_ok ? "true" : "false");
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
  // 正常退出：停止发送线程和键盘线程，最后失能全部电机。
  sender.shutdown();
  keyboard.stop();
  sender.pause();
  hardware_.disableAll();
  return 0;
}

// ---------- 启动反馈与零位检查 ----------

// 检查 IMU 姿态、重力和角速度，以及全部电机位置和速度是否满足启动条件。
bool HardwareBridge::stationaryLevelSample(
  const dm1_hardware::HardwareSample & sample) const noexcept
{
  if (!sample.imu.valid ||
    sample.imu.angular_velocity_body.norm() >
    control_config::hardware::startup_maximum_angular_speed)
  {
    return false;
  }
  const float gravity = sample.imu.acceleration_body.norm();
  const float horizontal_gravity = std::hypot(
    sample.imu.acceleration_body.x(), sample.imu.acceleration_body.y());
  if (gravity < control_config::hardware::startup_gravity_minimum ||
    gravity > control_config::hardware::startup_gravity_maximum ||
    sample.imu.acceleration_body.z() <= 0.0F ||
    horizontal_gravity > gravity * std::sin(control_config::hardware::startup_maximum_tilt))
  {
    return false;
  }

  // 电机地址和健康已由 Dm1MitInterface::updateFeedback 校验，这里只检查静止。
  return std::all_of(
    sample.motors.begin(), sample.motors.end(), [this](const auto & motor) {
      return std::abs(motor.velocity) <= options_.startup_stationary_velocity_rad_s;
    });
}

// 等待一轮静止、水平的完整反馈；--set-zero 时把当前位置写为全部电机零位。
bool HardwareBridge::performStartupZeroCalibration(
  const std::chrono::steady_clock::time_point & start,
  const std::atomic_bool & stop_requested)
{
  if (options_.set_zero) {
    imu_log::print(
      imu_log::Level::Warning,
      "DM1 零位维护：跳过反馈/姿态/位置检查，直接写入全部电机零位。"
      "必须先悬空并摆成 OpenDog 零位：髋水平、大腿电机接口对准螺丝、小腿伸直顶限位。\n");
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
  // 键盘控制和只读监测都只要求一轮完整、静止、水平的 IMU/电机反馈。
  const bool read_only_diagnostic = !options_.keyboard_control;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  dm1_hardware::HardwareSample sample;
  while (true) {
    if (stop_requested.load() || std::chrono::steady_clock::now() >= deadline) {
      imu_log::print(
        imu_log::Level::Error,
        "启动检查被拒绝：未收到 IMU 和 12 个电机的完整首轮反馈。\n");
      return false;
    }
    const double now_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
    // updateFeedback 校验 12 路地址唯一、序号非零且健康；IMU 序号非零表示已收到真实帧。
    if (hardware_.pollMotors() && hardware_.read(sample, now_s) &&
      mit_.updateFeedback(sample.motors) && sample.imu.sequence != 0)
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!stationaryLevelSample(sample)) {
    imu_log::print(
      imu_log::Level::Error, "启动检查被拒绝：机器人必须静止、水平，且反馈健康。\n");
    return false;
  }

  if (read_only_diagnostic) {
    imu_log::print(
      imu_log::Level::Info,
      "DM1 12 路电机首轮反馈通过；将持续监测 IMU，电机不再要求连续上报。\n");
  } else {
    imu_log::print(
      imu_log::Level::Info,
      "DM1 键盘控制首轮反馈通过；等待 Shift+U 解锁。\n");
  }

  imu_log::print(
    imu_log::Level::Info,
    "DM1 启动稳定性检查通过；使能后将执行平滑回零，未修改电机参数。\n");
  return true;
}

// ---------- 主硬件控制循环 ----------

// 打开设备、完成启动检查，并按固定周期运行一个算法后端和 MIT 输出。
int HardwareBridge::run(const std::atomic_bool & stop_requested)
{
  if (!runner_.configure()) {
    imu_log::print(
      imu_log::Level::Error,
      "DM1 OpenDog 算法初始化失败：config=%s。\n",
      options_.algorithm_config_directory.c_str());
    return 8;
  }
  if (!hardware_.open()) {
    runner_.stop();
    hardware_.disableAll();
    return 2;
  }
  // 打开 CAN 不代表上一个进程退出时留下的驱动器仍处于失能状态；
  // 在任何启动轮询或读取前先建立失效安全状态。
  hardware_.disableAll();

  const auto start = std::chrono::steady_clock::now();
  if (!performStartupZeroCalibration(start, stop_requested)) {
    runner_.stop();
    hardware_.close();
    return 7;
  }
  if (options_.set_zero) {
    runner_.stop();
    hardware_.close();
    return 0;
  }
  if (options_.keyboard_control) {
    try {
      const int result = runKeyboardControl(start, stop_requested);
      runner_.stop();
      hardware_.close();
      return result;
    } catch (...) {
      runner_.stop();
      hardware_.close();
      throw;
    }
  }
  // 只读模式：电机保持失能，按控制周期持续监测 IMU。
  {
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
        runner_.stop();
        hardware_.close();
        return 3;
      }
      std::this_thread::sleep_until(next_cycle);
    }
    runner_.stop();
    hardware_.close();
    return 0;
  }
}

/*
 * run() 返回值含义：
 *   0  正常退出，或零位维护成功
 *   2  硬件打开失败
 *   3  电机/IMU 反馈读取或校验失败
 *   7  启动零位、水平或静止检查失败
 *   8  首次使能或键盘解锁失败
 *   9  键盘 TTY、键盘线程或输入路径失败
 */
