// ============================================================
// DM1 真机程序入口。
// 本文件只负责：命令行解析、设备/标定加载、运行模式选择和生命周期管理；
// 具体 CAN/IMU 设备访问集中在 Dm1HardwareDriver，控制算法集中在
// HardwareBridge -> user/RobotRunner。
// ============================================================
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

#include "hardware/dm1_hardware_driver.hpp"
#include "hardware/hardware_bridge.hpp"
#include "common/console_log.hpp"

// ---------- 本文件内部辅助函数（匿名命名空间） ----------
namespace
{

// 收到 Ctrl+C 或终止信号后，所有运行循环都会在下一个周期安全退出。
std::atomic_bool g_stop_requested{false};

// 信号处理函数只设置停止标志，不直接访问 CAN、串口等非异步信号安全资源。
void handleSignal(int) noexcept
{
  g_stop_requested.store(true);
}

// 命令行参数的临时保存结构。
// 解析完成后，main() 会将其中与控制相关的字段复制到 HardwareBridge::Options。
struct Arguments
{
  // 12 个电机的 CAN 地址、方向和零位标定文件。
  std::string calibration_path;
  // 两条 SocketCAN 总线名称。
  std::string can0 = "can0";
  std::string can1 = "can1";
  // IMU 串口路径；默认使用自动检测。
  std::string imu_device = "auto";
  // 控制周期和启动零位允许误差，单位分别为 s 和 rad。
  float control_time_step = 0.002F;
  float zero_tolerance_rad = 0.05F;
  // 运行模式开关。
  bool enable_output = false;
  bool keyboard_control = false;
  bool stand_up = false;
  bool set_zero = false;
  bool imu_only = false;
  bool motor_only = false;
  // 仅在 motor-only 模式下可选：指定需要周期性轮询的单个电机。
  std::optional<dm1_hardware::MotorAddress> poll_feedback_address;
  // 单电机使能测试请求：关节索引、总线和物理 CAN ID 必须与标定一致。
  struct SingleMotorRequest
  {
    std::size_t joint_index = 0;
    dm1_hardware::MotorAddress address{};
  };
  std::optional<SingleMotorRequest> single_motor;
  // 只有显式指定时，单电机模式才会发送 MIT 动作帧。
  bool slow_move_test = false;
};

// 报告命令行错误并抛出异常；该函数不会返回。
[[noreturn]] void usageError(const std::string & message)
{
  throw std::invalid_argument(
          message +
          "\n用法: hardware_main --calibration FILE [--can0 can0] [--can1 can1] "
          "[--imu auto|DEVICE] [--control-dt 0.002] "
          "[--zero-tolerance 0.05]（启动时零位允许误差窗口） "
          "[--enable-output | --keyboard-control] [--stand-up] [--set-zero] "
          "[--imu-only | --motor-only] "
          "[--poll-feedback BUS CAN_ID] "
          "[--single-motor JOINT_INDEX BUS CAN_ID --enable-output] "
          "[--slow-move-test]");
}

std::string resolveImuDevice(const std::string & requested)
{
  // 用户明确给出设备路径时，直接使用该路径，不做自动选择。
  if (requested != "auto") {return requested;}

  // /dev/serial/by-id 下的名称通常比 /dev/ttyUSB0 更稳定，适合自动识别。
  const std::filesystem::path by_id_directory("/dev/serial/by-id");
  std::error_code error;
  std::vector<std::string> candidates;

  // 查找名称中包含 DM-IMU 的串口设备。
  for (std::filesystem::directory_iterator iterator(by_id_directory, error), end;
    !error && iterator != end; iterator.increment(error))
  {
    const std::string name = iterator->path().filename().string();
    if (name.find("DM-IMU") != std::string::npos) {
      candidates.push_back(iterator->path().string());
    }
  }
  // 排序后再处理，保证设备选择和日志顺序具有确定性。
  std::sort(candidates.begin(), candidates.end());

  // 自动模式要求恰好找到一个设备，避免误选多个 IMU 中的某一个。
  if (candidates.size() == 1) {
    imu_log::print(
      imu_log::Level::Info, "[DM IMU] 已自动选择串口设备: %s\n",
      candidates.front().c_str());
    return candidates.front();
  }
  if (candidates.empty()) {
    // 目录不存在、没有匹配设备等情况都会走到这里。
    throw std::runtime_error(
            "无法在 /dev/serial/by-id 下自动检测到 DM IMU；请使用 --imu DEVICE");
  }
  // 找到多个 DM IMU 时必须由用户通过 --imu 明确指定。
  throw std::runtime_error(
          "检测到多个 DM IMU 设备；请使用 --imu DEVICE 明确指定");
}

Arguments parseArguments(int argc, char ** argv)
{
  Arguments result;

  // 从 argv[1] 开始解析；argv[0] 是可执行文件名称。
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];

    // 读取需要值的选项，例如 --imu /dev/ttyUSB0。
    auto requireValue = [&](const char * option) -> std::string {
        if (++i >= argc) {usageError(std::string("选项缺少参数: ") + option);}
        return argv[i];
      };
    // 标定文件路径。
    if (argument == "--calibration") {
      result.calibration_path = requireValue("--calibration");
    // --can 是旧参数名，保留它作为 --can0 的兼容别名。
    } else if (argument == "--can0" || argument == "--can") {
      result.can0 = requireValue(argument.c_str());
    // 第二条 CAN 总线名称。
    } else if (argument == "--can1") {
      result.can1 = requireValue("--can1");
    // IMU 设备路径或 auto。
    } else if (argument == "--imu") {
      result.imu_device = requireValue("--imu");
    // 控制周期，使用 stof 将命令行字符串转换为 float。
    } else if (argument == "--control-dt") {
      result.control_time_step = std::stof(requireValue("--control-dt"));
    // 启动时电机位置允许偏离标定零位的窗口。
    } else if (argument == "--zero-tolerance") {
      result.zero_tolerance_rad = std::stof(requireValue("--zero-tolerance"));
    // 允许完整控制模式真正向电机输出命令。
    } else if (argument == "--enable-output") {
      result.enable_output = true;
    // 使用键盘命令控制电机解锁、站立和运动。
    } else if (argument == "--keyboard-control") {
      result.keyboard_control = true;
    // 完整控制模式启动后自动请求站立。
    } else if (argument == "--stand-up") {
      result.stand_up = true;
    // 维护模式：将当前电机位置写入驱动器零位参数。
    } else if (argument == "--set-zero") {
      result.set_zero = true;
    // 只打开并读取 IMU，不需要标定文件，也不打开电机 CAN。
    } else if (argument == "--imu-only") {
      result.imu_only = true;
    // 只打开并诊断电机 CAN，不启动 IMU 和 RobotRunner。
    } else if (argument == "--motor-only") {
      result.motor_only = true;
    // 读取 --poll-feedback 后面的总线名称和物理 CAN ID。
    } else if (argument == "--poll-feedback") {
      const std::string bus_name = requireValue("--poll-feedback BUS");
      const std::string can_id_text = requireValue("--poll-feedback CAN_ID");
      std::uint8_t bus = 0;
      // 总线既可以写成 can0/can1，也可以写成 0/1。
      if (bus_name == "can0" || bus_name == "0") {
        bus = 0;
      } else if (bus_name == "can1" || bus_name == "1") {
        bus = 1;
      } else {
        usageError("--poll-feedback 的 BUS 必须是 can0/0 或 can1/1");
      }
      try {
        // stoul 的进制参数为 0，因此同时支持十进制和 0x 前缀十六进制。
        const auto parsed = std::stoul(can_id_text, nullptr, 0);
        if (parsed == 0 || parsed > 0x0F) {
          usageError("--poll-feedback 的物理 CAN_ID 必须在 [1,0x0f] 范围内");
        }
        result.poll_feedback_address = dm1_hardware::MotorAddress{
          // 诊断轮询帧只需要 bus 和物理 CAN ID；master_id 在此处不参与发送。
          bus, static_cast<std::uint16_t>(parsed), 0};
      } catch (const std::exception &) {
        usageError("--poll-feedback 的 CAN_ID 必须是整数");
      }
    } else if (argument == "--single-motor") {
      const std::string joint_index_text = requireValue("--single-motor JOINT_INDEX");
      const std::string bus_name = requireValue("--single-motor BUS");
      const std::string can_id_text = requireValue("--single-motor CAN_ID");
      std::size_t joint_index = 0;
      std::uint8_t bus = 0;
      try {
        const auto parsed = std::stoul(joint_index_text, nullptr, 0);
        if (parsed >= kNumJoints) {
          usageError("--single-motor 的 JOINT_INDEX 必须在 [0,11] 范围内");
        }
        joint_index = static_cast<std::size_t>(parsed);
      } catch (const std::exception &) {
        usageError("--single-motor 的 JOINT_INDEX 必须是整数");
      }
      if (bus_name == "can0" || bus_name == "0") {
        bus = 0;
      } else if (bus_name == "can1" || bus_name == "1") {
        bus = 1;
      } else {
        usageError("--single-motor 的 BUS 必须是 can0/0 或 can1/1");
      }
      try {
        const auto parsed = std::stoul(can_id_text, nullptr, 0);
        if (parsed == 0 || parsed > 0x0F) {
          usageError("--single-motor 的物理 CAN_ID 必须在 [1,0x0f] 范围内");
        }
        result.single_motor = Arguments::SingleMotorRequest{
          joint_index,
          dm1_hardware::MotorAddress{
            bus, static_cast<std::uint16_t>(parsed), 0}};
      } catch (const std::exception &) {
        usageError("--single-motor 的 CAN_ID 必须是整数");
      }
    } else if (argument == "--slow-move-test") {
      result.slow_move_test = true;
    } else if (argument == "--help" || argument == "-h") {
      // usageError 会抛出异常，因此这里打印用法后立即结束当前流程。
      usageError("DM1 硬件桥接参数");
    } else {
      usageError("未知参数: " + argument);
    }
  }

  // 除 IMU-only 外，其余模式都需要完整的 12 电机标定信息。
  if (!result.imu_only && result.calibration_path.empty()) {
    usageError("除非使用 --imu-only，否则必须提供 --calibration");
  }
  // 两种只读诊断模式不能执行电机输出或启动控制器。
  if ((result.imu_only || result.motor_only) &&
    (result.enable_output || result.keyboard_control || result.stand_up))
  {
    usageError(
      "只读模式不能与 --enable-output、--keyboard-control 或 --stand-up 同时使用");
  }
  // IMU-only 和 motor-only 是两条互斥的诊断路径。
  if (result.imu_only && result.motor_only) {
    usageError("--imu-only 和 --motor-only 不能同时使用");
  }
  // 单电机反馈轮询依赖 motor-only 的 DmMotorDriver。
  if (result.poll_feedback_address.has_value() && !result.motor_only) {
    usageError("--poll-feedback 必须与 --motor-only 一起使用");
  }
  if (result.single_motor.has_value()) {
    if (!result.enable_output) {
      usageError("--single-motor 必须与 --enable-output 一起使用");
    }
    if (result.motor_only || result.imu_only || result.keyboard_control ||
      result.stand_up || result.set_zero || result.poll_feedback_address.has_value())
    {
      usageError(
        "--single-motor 不能与 --motor-only、--imu-only、--keyboard-control、"
        "--stand-up、--set-zero 或 --poll-feedback 同时使用");
    }
  } else if (result.slow_move_test) {
    usageError("--slow-move-test 必须与 --single-motor 一起使用");
  }
  // 站立会生成实际关节命令，因此必须允许输出。
  if (result.stand_up && !result.enable_output) {
    usageError("--stand-up 必须与 --enable-output 一起使用");
  }
  // 自动输出和键盘输出不能同时拥有电机控制权。
  if (result.enable_output && result.keyboard_control) {
    usageError("--enable-output 和 --keyboard-control 不能同时使用");
  }
  // 写零位需要完整硬件模式，不能和单设备诊断模式混用。
  if (result.set_zero && (result.imu_only || result.motor_only)) {
    usageError("--set-zero 必须使用完整硬件模式");
  }
  // 写零位是独立维护操作，不能同时执行运动控制。
  if (result.set_zero &&
    (result.enable_output || result.keyboard_control || result.stand_up))
  {
    usageError(
      "--set-zero 不能与 --enable-output、--keyboard-control 或 --stand-up 同时使用");
  }
  return result;
}

int runMotorOnly(
  const std::string & can0, const std::string & can1,
  const dm1_hardware::Dm1MitInterface::CalibrationArray & calibration,
  float control_time_step,
  const std::optional<dm1_hardware::MotorAddress> & poll_feedback_address)
{
  // 电机诊断模式直接使用 DmMotorDriver，不创建 IMU 和 HardwareBridge。
  DmMotorDriver motors(can0, can1, calibration);
  if (!motors.open()) {
    throw std::runtime_error("无法打开 SocketCAN 总线: " + can0 + ", " + can1);
  }
  imu_log::print(
    imu_log::Level::Warning,
    poll_feedback_address.has_value() ?
    "电机诊断模式已启动；将周期性发送零增益 MIT 反馈轮询；不会读取 IMU、使能电机或发送零位命令。按 Ctrl+C 停止。\n" :
    "电机诊断模式已启动；不会读取 IMU，也不会发送使能、零位或 MIT 控制命令。按 Ctrl+C 停止。\n");
  std::optional<dm1_hardware::MitFrame> poll_frame;
  if (poll_feedback_address.has_value()) {
    // 默认构造的 MIT 帧各控制量均为 0，只用于触发指定电机返回反馈。
    dm1_hardware::MitFrame frame{};
    frame.bus = poll_feedback_address->bus;
    frame.can_id = poll_feedback_address->can_id;
    // 使用零增益 MIT 帧周期性请求反馈；它不会使能电机，也不会请求非零的
    // 位置、速度或力矩控制。
    poll_frame = frame;
    imu_log::print(
      imu_log::Level::Info,
      "正在向总线%u CAN ID 0x%03x 周期性发送零增益 MIT 反馈轮询；"
      "不会发送电机使能命令。\n",
      static_cast<unsigned int>(frame.bus), static_cast<unsigned int>(frame.can_id));
  }
  // 诊断轮询至少每 10 ms 执行一次，避免配置过小导致无意义的高频输出。
  const auto start = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(std::max(0.01F, control_time_step)));
  auto next_cycle = start;
  auto last_poll_report = start;
  std::uint64_t last_report_sequence = 0;
  std::uint64_t sent_poll_count = 0;

  // 该循环只发送可选的反馈轮询并打印反馈，不运行控制算法。
  while (!g_stop_requested.load()) {
    next_cycle += period;
    if (poll_frame.has_value() && !motors.sendMit(*poll_frame)) {
      throw std::runtime_error("周期性 MIT 反馈轮询发送失败");
    }
    if (poll_frame.has_value()) {
      ++sent_poll_count;
    }
    const double now_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
    dm1_hardware::Dm1MitInterface::FeedbackArray feedback{};
    // latest() 返回当前缓存的 12 路反馈，并报告数据是否完整且新鲜。
    const bool fresh = motors.latest(feedback, now_s);
    if (poll_feedback_address.has_value()) {
      // 指定单个电机时，每 100 ms 只输出该电机的状态摘要。
      const auto report_now = std::chrono::steady_clock::now();
      const auto report_age = std::chrono::duration<double>(report_now - last_poll_report);
      if (report_age.count() >= 0.1) {
        const dm1_hardware::MotorFeedback * polled_motor = nullptr;
        // 在完整反馈数组中找到用户指定的 bus/CAN ID。
        for (const auto & motor : feedback) {
          if (motor.bus == poll_feedback_address->bus &&
            motor.can_id == poll_feedback_address->can_id)
          {
            polled_motor = &motor;
            break;
          }
        }
        if (polled_motor == nullptr || polled_motor->sequence == 0) {
          // 没有反馈或序号仍为 0，说明该电机尚未返回有效帧。
          imu_log::print(
            imu_log::Level::Warning,
            "[轮询] 总线%u CAN=0x%03x 已发送=%llu 状态=NO_FEEDBACK 序号=0\n",
            static_cast<unsigned int>(poll_feedback_address->bus),
            static_cast<unsigned int>(poll_feedback_address->can_id),
            static_cast<unsigned long long>(sent_poll_count));
        } else {
          // sequence 增加表示轮询后确实收到了新反馈帧。
          const auto sequence_delta = polled_motor->sequence - last_report_sequence;
          imu_log::print(
            sequence_delta > 0 ? imu_log::Level::Info : imu_log::Level::Warning,
            "[轮询] 总线%u CAN=0x%03x 状态=%s 序号=%llu (+%llu/%.2fs) "
            "q=%+.4f dq=%+.4f tau=%+.4f 已发送=%llu\n",
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
      // 未指定单个电机时，打印全部 12 个电机的最新状态。
      imu_log::print(
        fresh ? imu_log::Level::Info : imu_log::Level::Warning,
        "电机反馈: %s:\n", fresh ? "新鲜" : "过期或不完整");
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
  // 退出前关闭 CAN 驱动，释放总线资源。
  motors.close();
  return 0;
}

int runSingleMotor(
  const std::string & can0, const std::string & can1,
  const dm1_hardware::Dm1MitInterface::CalibrationArray & calibration,
  const dm1_hardware::MotorAddress & address, bool slow_move_test)
{
  // 单电机测试只打开目标总线，且 DmMotorDriver 的析构只禁用这个目标。
  DmMotorDriver motors(can0, can1, calibration);
  if (!motors.openSingle(address)) {
    throw std::runtime_error(
            "无法打开目标电机所在的 SocketCAN 总线或目标地址未登记");
  }
  imu_log::print(
    imu_log::Level::Warning,
    "单电机测试已启动：只操作 bus=%u CAN ID=0x%03x；不会使能其他电机。"
    "按 Ctrl+C 禁用目标电机并退出。\n",
    static_cast<unsigned int>(address.bus), static_cast<unsigned int>(address.can_id));

  if (!motors.enableOne(address)) {
    throw std::runtime_error("目标电机使能命令发送失败");
  }
  const can_frame enable_frame = DmMotorDriver::encodeCommandFrame(address, 0xFC);
  imu_log::print(
    imu_log::Level::Info,
    "使能帧发送成功: bus=%u CAN_ID=0x%03x DLC=%u data="
    "[%02x %02x %02x %02x %02x %02x %02x %02x]\n"
    "%s",
    static_cast<unsigned int>(address.bus), static_cast<unsigned int>(enable_frame.can_id),
    static_cast<unsigned int>(enable_frame.can_dlc), enable_frame.data[0], enable_frame.data[1],
    enable_frame.data[2], enable_frame.data[3], enable_frame.data[4], enable_frame.data[5],
    enable_frame.data[6], enable_frame.data[7],
    slow_move_test ?
    "已先切换 MIT 模式并重复发送 5 次 0xFC；将执行显式慢速动作测试："
    "目标增量 30 deg（0.5236 rad），时长 5 s，Kp=20.0，Kd=0.2，前馈力矩=0。\n" :
    "已先切换 MIT 模式并重复发送 5 次 0xFC；本模式不会发送任何 MIT 位置、速度、"
    "增益或力矩帧。\n");

  if (slow_move_test) {
    // 先用零增益 MIT 帧获得当前位置；该帧不产生位置/速度/力矩控制。
    const auto feedback_start = std::chrono::steady_clock::now();
    dm1_hardware::MotorFeedback feedback{};
    bool feedback_received = false;
    bool enable_retried = false;
    while (!g_stop_requested.load() &&
      std::chrono::steady_clock::now() - feedback_start < std::chrono::milliseconds(500))
    {
      dm1_hardware::MitFrame poll{};
      poll.bus = address.bus;
      poll.can_id = address.can_id;
      poll.master_id = address.master_id;
      if (!motors.sendMit(poll)) {
        throw std::runtime_error("读取目标电机当前位置的零增益帧发送失败");
      }
      const double now_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - feedback_start).count();
      if (motors.latestOne(address, feedback, now_s)) {
        if (motors.isEnabled(address)) {
          feedback_received = true;
          break;
        }
        if (!enable_retried) {
          imu_log::print(
            imu_log::Level::Warning,
            "目标电机反馈仍为 Disabled，重新发送 MIT 模式和 5 次 0xFC 使能命令。\n");
          if (!motors.enableOne(address)) {
            throw std::runtime_error("重新发送目标电机使能命令失败");
          }
          enable_retried = true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!feedback_received) {
      throw std::runtime_error("未收到目标电机反馈，未开始慢速动作测试");
    }

    constexpr float kMoveDeltaRad = 0.5235988F;
    constexpr float kMoveKp = 20.0F;
    constexpr float kMoveKd = 0.2F;
    const float start_position = feedback.position;
    const float target_position = start_position + kMoveDeltaRad;
    imu_log::print(
      imu_log::Level::Info,
      "当前位置 q=%+.5f rad；目标 q=%+.5f rad（增量 30 deg）；仅沿正方向缓慢移动。\n",
      start_position, target_position);

    const auto move_start = std::chrono::steady_clock::now();
    auto next_report = move_start;
    while (!g_stop_requested.load()) {
      const auto elapsed = std::chrono::steady_clock::now() - move_start;
      const float phase = std::min(
        1.0F, static_cast<float>(std::chrono::duration<double>(elapsed).count() / 5.0));
      dm1_hardware::MitFrame command{};
      command.bus = address.bus;
      command.can_id = address.can_id;
      command.master_id = address.master_id;
      command.position = start_position + kMoveDeltaRad * phase;
      command.velocity = 0.0F;
      command.kp = kMoveKp;
      command.kd = kMoveKd;
      command.torque = 0.0F;
      if (!motors.sendMit(command)) {
        throw std::runtime_error("慢速 MIT 动作帧发送失败");
      }
      if (std::chrono::steady_clock::now() >= next_report) {
        imu_log::print(
          imu_log::Level::Info,
          "慢速 MIT: q=%+.5f rad dq=0 Kp=%.2f Kd=%.2f tau=0\n",
          command.position, command.kp, command.kd);
        next_report += std::chrono::milliseconds(100);
      }
      if (phase >= 1.0F) {break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return motors.close(), 0;
  }

  // 默认单电机模式使能后完全不再发送 CAN 控制帧；只等待 Ctrl+C。
  const auto period = std::chrono::milliseconds(100);
  while (!g_stop_requested.load()) {
    std::this_thread::sleep_for(period);
  }
  if (motors.disableOne(address)) {
    const can_frame disable_frame = DmMotorDriver::encodeCommandFrame(address, 0xFD);
    imu_log::print(
      imu_log::Level::Info,
      "禁用帧发送成功: bus=%u CAN_ID=0x%03x DLC=%u data="
      "[%02x %02x %02x %02x %02x %02x %02x %02x]\n",
      static_cast<unsigned int>(address.bus), static_cast<unsigned int>(disable_frame.can_id),
      static_cast<unsigned int>(disable_frame.can_dlc), disable_frame.data[0], disable_frame.data[1],
      disable_frame.data[2], disable_frame.data[3], disable_frame.data[4], disable_frame.data[5],
      disable_frame.data[6], disable_frame.data[7]);
  } else {
    imu_log::print(imu_log::Level::Error, "目标电机禁用帧发送失败。\n");
  }
  motors.close();
  return 0;
}

int runImuOnly(const std::string & device, float control_time_step)
{
  // IMU-only 模式完全绕过电机驱动和控制器，只验证 IMU 串口接收链路。
  HardwareImu imu(device, 921600);
  if (!imu.open()) {
    throw std::runtime_error("无法打开 IMU 串口: " + device);
  }

  imu_log::print(
    imu_log::Level::Info,
    "IMU-only 模式已启动；按 Ctrl+C 停止。不会发送任何电机 CAN 命令。\n");
  const auto start = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    // IMU 诊断循环最小周期为 1 ms。
    std::chrono::duration<double>(std::max(0.001F, control_time_step)));
  auto next_cycle = start;
  while (!g_stop_requested.load()) {
    next_cycle += period;
    ImuData<float> sample;
    // 时间戳使用相对于本次 IMU-only 启动时刻的单调时间。
    const float timestamp = static_cast<float>(
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
    // readAt() 是非阻塞函数：串口由接收线程负责读取，本循环只复制最新的
    // 已校验数据。
    // readAt() 非阻塞地复制最新已校验数据；此模式不运行控制器，因此忽略返回值。
    static_cast<void>(imu.readAt(sample, timestamp));
    std::this_thread::sleep_until(next_cycle);
  }
  // 退出时停止 IMU 接收线程并关闭串口。
  imu.close();
  return 0;
}

dm1_hardware::Dm1MitInterface::CalibrationArray loadCalibration(
  const std::string & path)
{
  // 标定数组下标必须和 RobotRunner/DM1 模型的 12 关节顺序一致。
  constexpr std::array<const char *, kNumJoints> expected_names{
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf"};

  std::ifstream input(path);
  if (!input) {throw std::runtime_error("无法打开标定文件: " + path);}
  dm1_hardware::Dm1MitInterface::CalibrationArray result{};
  std::string line;
  std::size_t index = 0;
  // 每个有效数据行对应一个电机；空行和 # 注释行会被跳过。
  while (std::getline(input, line)) {
    const auto first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos || line[first] == '#') {continue;}
    if (index >= result.size()) {
      throw std::runtime_error("标定文件包含超过 12 个电机");
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
    // 标定文件每行格式：关节名 总线 物理CAN ID Master ID 方向 零位(rad)。
    if (!(values >> name >> bus_name >> can_id_text >> master_id_text >> direction >> zero) ||
      (values >> extra))
    {
      throw std::runtime_error(
              "标定文件每行格式必须为: 关节名 总线 CAN_ID MASTER_ID 方向 零位_RAD");
    }
    // 不仅检查关节名，还检查它在文件中的固定顺序。
    if (name != expected_names[index]) {
      throw std::runtime_error(
              "标定文件第 " + std::to_string(index + 1) + " 行的关节顺序不正确");
    }
    std::uint8_t bus = 0;
    // 将可读的总线名称转换成驱动使用的数值编号。
    if (bus_name == "can0" || bus_name == "0") {
      bus = 0;
    } else if (bus_name == "can1" || bus_name == "1") {
      bus = 1;
    } else {
      throw std::runtime_error("电机总线必须是 can0/0 或 can1/1");
    }
    try {
      // 支持十进制和十六进制的 CAN/master ID。
      can_id = std::stoul(can_id_text, nullptr, 0);
      master_id = std::stoul(master_id_text, nullptr, 0);
    } catch (const std::exception &) {
      throw std::runtime_error("CAN ID 和 master ID 必须是整数");
    }
    // 物理 CAN ID 使用 DM1 的 4-bit 地址范围；master ID 使用 11-bit 范围。
    if (can_id == 0 || can_id > 0x0F || master_id == 0 || master_id > 0x7FF) {
      throw std::runtime_error(
              "物理 CAN ID 必须在 [1,0x0f] 范围内，master ID 必须在 [1,0x7ff] 范围内");
    }
    result[index] = dm1_hardware::MotorCalibration{
      dm1_hardware::MotorAddress{
        bus, static_cast<std::uint16_t>(can_id), static_cast<std::uint16_t>(master_id)},
      direction, zero};
    // 下一个有效行写入下一个关节槽位。
    ++index;
  }
  // 必须恰好提供 12 个电机，不能少也不能多。
  if (index != result.size()) {
    throw std::runtime_error("标定文件必须恰好包含 12 个电机");
  }
  return result;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    // 先解析并验证参数，再打开任何硬件设备。
    const Arguments arguments = parseArguments(argc, argv);
    // 注册终止信号；各运行循环通过原子标志完成收尾。
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    if (arguments.imu_only) {
      // IMU-only 不需要标定文件，并且直接返回，不进入完整硬件路径。
      return runImuOnly(resolveImuDevice(arguments.imu_device), arguments.control_time_step);
    }

    // motor-only 和完整模式都需要先加载 12 电机标定。
    const auto calibration = loadCalibration(arguments.calibration_path);
    if (arguments.single_motor.has_value()) {
      const auto & request = arguments.single_motor.value();
      const auto & configured = calibration[request.joint_index].address;
      // CLI 中的总线和物理 ID 必须与关节索引对应的标定完全一致。
      if (configured.bus != request.address.bus ||
        configured.can_id != request.address.can_id)
      {
        throw std::runtime_error(
                "--single-motor 的关节索引、总线和 CAN_ID 与标定文件不一致");
      }
      return runSingleMotor(
        arguments.can0, arguments.can1, calibration, configured, arguments.slow_move_test);
    }
    if (arguments.motor_only) {
      // motor-only 只检查 CAN/电机反馈，不创建 IMU 或 RobotRunner。
      return runMotorOnly(
        arguments.can0, arguments.can1, calibration, arguments.control_time_step,
        arguments.poll_feedback_address);
    }

    // 完整模式的底层硬件适配器，统一管理两条 CAN 总线和一个 IMU 串口。
    Dm1HardwareDriver driver(
      arguments.can0, arguments.can1, resolveImuDevice(arguments.imu_device), calibration);
    HardwareBridge::Options options;
    // 将命令行参数转换成控制桥使用的运行配置。
    options.control_time_step = arguments.control_time_step;
    options.startup_zero_tolerance_rad = arguments.zero_tolerance_rad;
    options.enable_output = arguments.enable_output;
    options.keyboard_control = arguments.keyboard_control;
    options.request_stand_up = arguments.stand_up;
    options.set_zero = arguments.set_zero;
    // HardwareBridge 构造函数还会检查参数组合和安全阈值。
    HardwareBridge bridge(driver, calibration, options);

    // 启动前打印当前模式及其安全含义，便于操作者确认。
    if (arguments.set_zero) {
      imu_log::print(
        imu_log::Level::Warning,
        "DM1 维护模式: --set-zero 将写入所有电机零位参数；输出保持关闭。\n");
    } else if (arguments.keyboard_control) {
      imu_log::print(
        imu_log::Level::Warning,
        "DM1 键盘控制: 电机启动时处于锁定状态；不会修改电机零位参数。"
        "只有机器人静止、水平且处于零位窗口内时，才能使用 Shift+U 解锁。\n");
    } else if (arguments.enable_output) {
      imu_log::print(
        imu_log::Level::Warning,
        "DM1 控制启动: 所有关节必须处于 %.3f rad 零位窗口内；"
        "启动过程不会修改电机参数。\n",
        arguments.zero_tolerance_rad);
    } else {
      imu_log::print(
        imu_log::Level::Info,
        "DM1 硬件桥接在启动时为只读模式；不会写入电机参数。"
        "使用 --set-zero 进入维护模式，或使用 --enable-output 启动控制。\n");
    }
    // 进入完整硬件桥：打开设备、执行启动检查，然后运行控制循环。
    const int result = bridge.run(g_stop_requested);
    return result;
  } catch (const std::exception & error) {
    // 参数、文件或设备初始化异常统一转换为错误日志和返回码 1。
    imu_log::print(
      imu_log::Level::Error, "DM1 硬件启动失败: %s\n", error.what());
    return 1;
  }
}
