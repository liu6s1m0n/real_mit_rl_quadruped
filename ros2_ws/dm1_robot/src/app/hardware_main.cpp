/*! @file hardware_main.cpp
 *  @brief DM1 真机程序入口。
 *
 *  本文件只负责：命令行解析、设备/标定加载、运行模式选择和生命周期管理；
 *  具体 CAN/IMU 设备访问集中在 Dm1HardwareDriver，控制算法集中在
 *  HardwareBridge -> OpenDogRobotRunner。
 */
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "hardware/dm1_hardware_driver.hpp"
#include "hardware/hardware_bridge.hpp"
#include "hardware/motor_calibration.hpp"
#include "common/console_log.hpp"

// ---------- 本文件内部辅助函数（匿名命名空间） ----------
namespace
{

// 默认 OpenDog 配置目录；CMake 安装时会定义为 share 目录下的绝对路径。
#ifdef DM1_ROBOT_ALGORITHM_CONFIG_DIR
constexpr const char * kDefaultAlgorithmConfigDirectory =
  DM1_ROBOT_ALGORITHM_CONFIG_DIR;
#else
constexpr const char * kDefaultAlgorithmConfigDirectory = "config/opendog";
#endif

// 收到 Ctrl+C 或终止信号后，所有运行循环都会在下一个周期安全退出。
std::atomic_bool g_stop_requested{false};

/// 按额定 30 Nm / 峰值 97 Nm 给力矩分级，用于诊断日志。
const char * torqueStatus(float torque) noexcept
{
  const float magnitude = std::abs(torque);
  return magnitude > 97.0F ? "OVER_PEAK" :
         (magnitude > 30.0F ? "OVER_CONTINUOUS" : "NORMAL");
}

// 信号处理函数只设置停止标志，不直接访问 CAN、串口等非异步信号安全资源。
void handleSignal(int) noexcept
{
  g_stop_requested.store(true);
}

/// 命令行参数的临时保存结构。
/// 解析完成后，main() 会将其中与控制相关的字段复制到 HardwareBridge::Options。
struct Arguments
{
  /// 12 个电机的 CAN 地址、方向和零位标定文件。
  std::string calibration_path;
  std::string can0 = "can0";  ///< 前腿 SocketCAN 总线名称。
  std::string can1 = "can1";  ///< 后腿 SocketCAN 总线名称。
  /// IMU 串口路径；默认使用自动检测。
  std::string imu_device = "auto";
  /// 控制周期，单位 s。
  float control_time_step = float(control_config::control_time_step);
  /// 关节力矩饱和上限；默认 88 Nm（电机峰值 97 Nm）。超限关节按比例缩小后
  /// 照常发送，不丢弃整帧。
  float torque_limit_nm = control_config::hardware::torque_limit;
  bool keyboard_control = false;  ///< --keyboard-control：键盘控制模式。
  bool set_zero = false;          ///< --set-zero：写入电机零位的维护模式。
  std::string algorithm_config_directory = kDefaultAlgorithmConfigDirectory;  ///< OpenDog 配置目录。
  bool imu_only = false;          ///< --imu-only：只检查 IMU 串口。
  bool motor_only = false;        ///< --motor-only：只检查 CAN/电机反馈。
  /// 仅在 motor-only 模式下可选：指定需要周期性轮询的单个电机。
  std::optional<dm1_hardware::MotorAddress> poll_feedback_address;
};

// 报告命令行错误并抛出异常；该函数不会返回。
[[noreturn]] void usageError(const std::string & message)
{
  throw std::invalid_argument(
          message +
          "\n用法: hardware_main --calibration FILE [--can0 can0] [--can1 can1] "
          "[--imu auto|DEVICE] [--control-dt 0.002] "
          "[--torque-limit 88]（0~97 Nm，关节力矩饱和上限） "
          "[--algorithm-config DIR] "
          "[--keyboard-control | --set-zero] "
          "[--imu-only | --motor-only] "
          "[--poll-feedback BUS CAN_ID]");
}

/// 解析 IMU 串口路径；"auto" 时在 /dev/serial/by-id 中查找唯一的 DM-IMU 设备。
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
    if (name.find("DM-IMU") != std::string::npos) {// 匹配 DM-IMU 串口设备。
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

/// 解析并校验命令行参数；非法参数或互斥模式组合会抛出异常。
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
      // 电机力矩上限；站立吃力时可在 0~97 Nm 内临时放宽。
    } else if (argument == "--torque-limit") {
      result.torque_limit_nm = std::stof(requireValue("--torque-limit"));
    } else if (argument == "--algorithm") {
      // 兼容旧命令：OpenDog 是唯一算法，只接受 opendog。
      if (requireValue("--algorithm") != "opendog") {
        usageError("--algorithm 只接受 opendog（MIT 算法已删除）");
      }
    } else if (argument == "--algorithm-config") {
      result.algorithm_config_directory = requireValue("--algorithm-config DIR");
      // 使用键盘命令控制电机解锁、站立和运动。
    } else if (argument == "--keyboard-control") {
      result.keyboard_control = true;
      // 维护模式：将当前电机位置写入驱动器零位参数。
    } else if (argument == "--set-zero") {
      result.set_zero = true;
      // 只打开并读取 IMU，不需要标定文件，也不打开电机 CAN。
    } else if (argument == "--imu-only") {
      result.imu_only = true;
      // 只打开并诊断电机 CAN，不启动 IMU 和控制算法。
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
  if ((result.imu_only || result.motor_only) && result.keyboard_control) {
    usageError("只读模式不能与 --keyboard-control 同时使用");
  }
  // IMU-only 和 motor-only 是两条互斥的诊断路径。
  if (result.imu_only && result.motor_only) {
    usageError("--imu-only 和 --motor-only 不能同时使用");
  }
  // 单电机反馈轮询依赖 motor-only 的 DmMotorDriver。
  if (result.poll_feedback_address.has_value() && !result.motor_only) {
    usageError("--poll-feedback 必须与 --motor-only 一起使用");
  }
  if (!(result.control_time_step > 0.0F && result.control_time_step <= 0.02F)) {
    usageError("--control-dt 必须在 (0, 0.02] s 范围内");
  }
  // 力矩上限必须落在 DM1 允许的 0~97 Nm 区间。
  if (!std::isfinite(result.torque_limit_nm) || result.torque_limit_nm < 0.0F ||
    result.torque_limit_nm > 97.0F)
  {
    usageError("--torque-limit 必须在 [0, 97] Nm 范围内");
  }
  // 写零位需要完整硬件模式，不能和单设备诊断模式混用。
  if (result.set_zero && (result.imu_only || result.motor_only)) {
    usageError("--set-zero 必须使用完整硬件模式");
  }
  // 写零位是独立维护操作，不能同时执行运动控制。
  if (result.set_zero && result.keyboard_control) {
    usageError("--set-zero 不能与 --keyboard-control 同时使用");
  }
  return result;
}

/// motor-only 诊断：只打开 CAN，周期打印电机反馈，不使能电机。
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
            std::abs(polled_motor->torque) > 97.0F ? imu_log::Level::Error :
            (std::abs(polled_motor->torque) > 30.0F || sequence_delta == 0 ?
            imu_log::Level::Warning : imu_log::Level::Info),
            "[轮询] 总线%u CAN=0x%03x 状态=%s 序号=%llu (+%llu/%.2fs) "
            "q=%+.4f dq=%+.4f tau=%+.4f Nm torque=%s 已发送=%llu\n",
            static_cast<unsigned int>(polled_motor->bus),
            static_cast<unsigned int>(polled_motor->can_id),
            sequence_delta > 0 ? "UPDATING" : "NO_UPDATE",
            static_cast<unsigned long long>(polled_motor->sequence),
            static_cast<unsigned long long>(sequence_delta), report_age.count(),
            polled_motor->position, polled_motor->velocity, polled_motor->torque,
            torqueStatus(polled_motor->torque),
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
          std::abs(motor.torque) > 97.0F ? imu_log::Level::Error :
          (std::abs(motor.torque) > 30.0F ? imu_log::Level::Warning : imu_log::Level::Info),
          "  %02zu bus%u can=0x%03x seq=%llu q=%+.4f dq=%+.4f "
          "tau=%+.4f Nm torque=%s\n",
          i, static_cast<unsigned int>(motor.bus), static_cast<unsigned int>(motor.can_id),
          static_cast<unsigned long long>(motor.sequence), motor.position,
          motor.velocity, motor.torque, torqueStatus(motor.torque));
      }
    }
    std::this_thread::sleep_until(next_cycle);
  }
  // 退出前关闭 CAN 驱动，释放总线资源。
  motors.close();
  return 0;
}

/// imu-only 诊断：只打开 IMU 串口，周期打印解算后的 IMU 数据。
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
  const auto print_period = std::chrono::milliseconds(100);
  auto next_cycle = start;
  auto next_print = start;
  std::uint64_t last_printed_sequence = 0;
  while (!g_stop_requested.load()) {
    next_cycle += period;
    ImuData<float> sample;
    // 时间戳使用相对于本次 IMU-only 启动时刻的单调时间。
    const float timestamp = static_cast<float>(
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
    // readAt() 是非阻塞函数：串口由接收线程负责读取，本循环只复制最新的
    // 已校验数据。
    // readAt() 非阻塞地复制最新已校验数据；IMU-only 模式按固定频率输出诊断数据。
    const bool valid = imu.readAt(sample, timestamp);
    const auto now = std::chrono::steady_clock::now();
    if (valid && now >= next_print && sample.sequence != last_printed_sequence) {
      // 以 ZYX 顺序从当前姿态四元数恢复 roll/pitch/yaw，并转换为角度输出。
      const auto yaw_pitch_roll =
        sample.orientation_world_from_body.toRotationMatrix().eulerAngles(2, 1, 0);
      imu_log::print(
        imu_log::Level::Info,
        "[DM IMU] seq=%llu t=%.3f acc=(%.3f, %.3f, %.3f) "
        "gyro=(%.3f, %.3f, %.3f) rpy_deg=(%.3f, %.3f, %.3f)\n",
        static_cast<unsigned long long>(sample.sequence), sample.timestamp,
        sample.acceleration_body.x(), sample.acceleration_body.y(),
        sample.acceleration_body.z(), sample.angular_velocity_body.x(),
        sample.angular_velocity_body.y(), sample.angular_velocity_body.z(),
        static_cast<double>(yaw_pitch_roll[2] * 57.29577951308232F),
        static_cast<double>(yaw_pitch_roll[1] * 57.29577951308232F),
        static_cast<double>(yaw_pitch_roll[0] * 57.29577951308232F));
      last_printed_sequence = sample.sequence;
      next_print += print_period;
      if (next_print < now) {next_print = now + print_period;}
    }
    std::this_thread::sleep_until(next_cycle);
  }
  // 退出时停止 IMU 接收线程并关闭串口。
  imu.close();
  return 0;
}


}  // namespace

/// 程序入口：按命令行选择 imu-only、motor-only 或完整硬件桥模式。
int main(int argc, char ** argv)
{
  bool ros_initialized = false;
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
    const auto calibration = dm1_hardware::loadCalibration(arguments.calibration_path);
    if (arguments.motor_only) {
      // motor-only 只检查 CAN/电机反馈，不创建 IMU 或控制算法。
      return runMotorOnly(
        arguments.can0, arguments.can1, calibration, arguments.control_time_step,
        arguments.poll_feedback_address);
    }

    // OpenDog 的 OCS2/MPC-MRT 接口和状态估计器属于 ROS 2 算法层。
    rclcpp::init(0, nullptr);
    ros_initialized = true;

    // 完整模式的底层硬件适配器，统一管理两条 CAN 总线和一个 IMU 串口。
    Dm1HardwareDriver driver(
      arguments.can0, arguments.can1, resolveImuDevice(arguments.imu_device), calibration);
    HardwareBridge::Options options;
    // 将命令行参数转换成控制桥使用的运行配置。
    options.control_time_step = arguments.control_time_step;
    options.torque_limit_nm = arguments.torque_limit_nm;
    options.keyboard_control = arguments.keyboard_control;
    options.set_zero = arguments.set_zero;
    options.algorithm_config_directory = arguments.algorithm_config_directory;
    // HardwareBridge 构造函数还会检查参数组合和安全阈值。
    HardwareBridge bridge(driver, calibration, options);

    // 启动前打印当前模式及其安全含义，便于操作者确认。
    if (arguments.set_zero) {
      imu_log::print(
        imu_log::Level::Warning,
        "DM1 维护模式: --set-zero 将写入所有电机零位参数；输出保持关闭。"
        "机器人必须悬空并摆成 OpenDog 零位：髋水平、大腿电机接口对准螺丝、小腿伸直顶限位。\n");
    } else if (arguments.keyboard_control) {
      imu_log::print(
        imu_log::Level::Warning,
        "DM1 键盘控制: 电机启动时处于锁定状态；不会修改电机零位参数。"
        "只有机器人静止、水平且 12 路反馈健康时，才能使用 Shift+U 解锁；"
        "键盘模式不检查当前关节是否处于零位窗口。\n");
    } else {
      imu_log::print(
        imu_log::Level::Info,
        "DM1 硬件桥接在启动时为只读模式；不会写入电机参数。"
        "使用 --set-zero 进入维护模式，或使用 --keyboard-control 启动控制。\n");
    }
    imu_log::print(
      imu_log::Level::Info,
      "DM1 关节力矩饱和上限=%.1f Nm（超过则按比例截断后仍发送；连续额定 30、峰值 97；"
      "可用 --torque-limit 调整）。\n",
      static_cast<double>(arguments.torque_limit_nm));
    // 进入完整硬件桥：打开设备、执行启动检查，然后运行控制循环。
    const int result = bridge.run(g_stop_requested);
    if (ros_initialized) {
      rclcpp::shutdown();
    }
    return result;
  } catch (const std::exception & error) {
    // 参数、文件或设备初始化异常统一转换为错误日志和返回码 1。
    imu_log::print(
      imu_log::Level::Error, "DM1 硬件启动失败: %s\n", error.what());
    if (ros_initialized) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
