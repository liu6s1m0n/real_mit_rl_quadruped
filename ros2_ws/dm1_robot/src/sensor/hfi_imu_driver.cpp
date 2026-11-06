/*! @file hfi_imu_driver.cpp
 *  @brief HandsFree HFI-A9 IMU 后台串口接收线程。
 *
 *  这里只做传输层同步、帧校验、单位与符号换算；启动基准、安装偏差等处理在 imu.cpp，
 *  与 DM IMU 共用。
 */

#include "sensor/hfi_imu_driver.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
// Linux/POSIX 串口相关接口
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <utility>
#include <vector>
#include <unistd.h>

#include "common/console_log.hpp"

namespace
{
constexpr std::uint8_t kHeader0 = 0xAA;            ///< 帧头第 1 字节。
constexpr std::uint8_t kHeader1 = 0x55;            ///< 帧头第 2 字节。
constexpr std::size_t kPayloadOffset = 7;          ///< 浮点数据起始偏移（[7..10] 为时间戳）。
constexpr float kStandardGravity = 9.80665F;       ///< 加速度原始单位 g → m/s^2。
/// 设备加速度为重力矢量（静止 z≈-1 g），取反得到比力；2026-11-05 实物验证。
constexpr float kAccelerationScale = -kStandardGravity;
constexpr int kReadTimeoutMs = 20;                 ///< 单次读取超时时间，ms。

/// 把波特率数值转换为 termios 常量；不支持时返回 0。
int baudConstant(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return 0;
  }
}

/// CRC16/MODBUS（多项式 0xA001 反射，初值 0xFFFF），与厂商驱动一致。
std::uint16_t crc16Modbus(const std::uint8_t * data, std::size_t size)
{
  std::uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U ? static_cast<std::uint16_t>((crc >> 1) ^ 0xA001U) :
        static_cast<std::uint16_t>(crc >> 1);
    }
  }
  return crc;
}

/// 从字节流按本机字节序读取一个 float（HFI 为小端）。
float readFloat(const std::uint8_t * data)
{
  float value = 0.0F;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

/// 校验帧头、长度字段和 CRC；CRC 覆盖 frame[2] 起的 LEN+1 字节，低字节在前。
bool validFrame(const std::uint8_t * frame, std::size_t size, std::uint8_t length)
{
  if (frame == nullptr || size != static_cast<std::size_t>(length) + 5U ||
    frame[0] != kHeader0 || frame[1] != kHeader1 || frame[2] != length)
  {
    return false;
  }
  const auto expected = crc16Modbus(frame + 2, static_cast<std::size_t>(length) + 1U);
  const auto received = static_cast<std::uint16_t>(frame[length + 3]) |
    static_cast<std::uint16_t>(frame[length + 4]) << 8;
  return expected == received;
}

}  // namespace

bool HfiImuDriver::decodeInertialFrame(
  const std::uint8_t * frame, std::size_t size,
  std::array<float, 3> & angular_velocity, std::array<float, 3> & acceleration) noexcept
{
  if (!validFrame(frame, size, kInertialLength)) {return false;}
  // 浮点序号：[0] 时间戳位、[1..3] 角速度、[4..6] 加速度、[7..9] 磁场（未使用）。
  for (int axis = 0; axis < 3; ++axis) {
    angular_velocity[axis] = readFloat(frame + kPayloadOffset + (1 + axis) * sizeof(float));
    acceleration[axis] = kAccelerationScale *
      readFloat(frame + kPayloadOffset + (4 + axis) * sizeof(float));
    if (!std::isfinite(angular_velocity[axis]) || !std::isfinite(acceleration[axis])) {
      return false;
    }
  }
  return true;
}

bool HfiImuDriver::decodeEulerFrame(
  const std::uint8_t * frame, std::size_t size, std::array<float, 3> & rpy_degrees) noexcept
{
  if (!validFrame(frame, size, kEulerLength)) {return false;}
  // 浮点序号：[0] 时间戳位、[1..3] roll/pitch/yaw（度）。
  for (int axis = 0; axis < 3; ++axis) {
    rpy_degrees[axis] = readFloat(frame + kPayloadOffset + (1 + axis) * sizeof(float));
    if (!std::isfinite(rpy_degrees[axis])) {return false;}
  }
  // 与厂商 ROS 驱动 quaternion_from_euler(roll, -pitch, -yaw) 一致。
  rpy_degrees[1] = -rpy_degrees[1];
  rpy_degrees[2] = -rpy_degrees[2];
  return true;
}

HfiImuDriver::HfiImuDriver(std::string serial_device, int baudrate)
: serial_device_(std::move(serial_device)), baudrate_(baudrate)
{
}

HfiImuDriver::~HfiImuDriver()
{
  stop();
}

bool HfiImuDriver::start()
{
  if (running_.load()) {return true;}
  const int speed = baudConstant(baudrate_);
  if (serial_device_.empty()) {
    imu_log::print(imu_log::Level::Error, "[HFI IMU] serial device path is empty\n");
    return false;
  }
  if (speed == 0) {
    imu_log::print(
      imu_log::Level::Error, "[HFI IMU] unsupported baudrate: %d\n", baudrate_);
    return false;
  }

  serial_fd_ = ::open(
    serial_device_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (serial_fd_ < 0) {
    const int error = errno;
    const auto level =
      error == ENOENT || error == ENODEV || error == EIO ?
      imu_log::Level::Warning : imu_log::Level::Error;
    imu_log::print(
      level, "[HFI IMU] open(%s) failed: %s\n",
      serial_device_.c_str(), std::strerror(error));
    return false;
  }
  // 原始模式，8N1，无流控，非阻塞读取
  termios configuration{};
  if (tcgetattr(serial_fd_, &configuration) != 0) {
    const int error = errno;
    imu_log::print(
      imu_log::Level::Error, "[HFI IMU] tcgetattr failed: %s\n", std::strerror(error));
    stop();
    return false;
  }
  cfmakeraw(&configuration);
  configuration.c_cflag |= CLOCAL | CREAD;
  configuration.c_cflag &= ~(CSTOPB | CRTSCTS | CSIZE);
  configuration.c_cflag |= CS8;
  cfsetispeed(&configuration, speed);
  cfsetospeed(&configuration, speed);
  configuration.c_cc[VMIN] = 0;
  configuration.c_cc[VTIME] = 0;
  if (tcsetattr(serial_fd_, TCSANOW, &configuration) != 0) {
    const int error = errno;
    imu_log::print(
      imu_log::Level::Error, "[HFI IMU] tcsetattr failed: %s\n", std::strerror(error));
    stop();
    return false;
  }
  tcflush(serial_fd_, TCIFLUSH);  // 清空输入缓冲区
  imu_log::print(
    imu_log::Level::Info, "[HFI IMU] receiver started: %s, %d baud\n",
    serial_device_.c_str(), baudrate_);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_sample_ = false;
  }
  running_.store(true);
  try {
    receive_thread_ = std::thread(&HfiImuDriver::receiveLoop, this);
  } catch (...) {
    running_.store(false);
    stop();
    return false;
  }
  return true;
}

// 先让线程退出再关闭 fd，避免接收线程读到已关闭的描述符。
void HfiImuDriver::stop() noexcept
{
  running_.store(false);
  if (receive_thread_.joinable()) {receive_thread_.join();}
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool HfiImuDriver::latest(
  ImuRawSample & sample, std::chrono::milliseconds maximum_age) const
{
  if (maximum_age.count() <= 0) {return false;}
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_sample_) {return false;}
  const auto age = std::chrono::steady_clock::now() - latest_sample_.received_at;
  if (age > maximum_age) {return false;}
  sample = latest_sample_;
  return true;
}

// 持续接收：0x2C 惯性帧与 0x14 欧拉角帧各收到一次后合成一个样本发布；
// 超过 1 s 没有有效样本时每秒打印一次原因。
void HfiImuDriver::receiveLoop() noexcept
{
  std::uint64_t sample_count = 0;
  const auto start_time = std::chrono::steady_clock::now();
  auto last_valid_sample = start_time;
  auto last_wait_log = start_time;
  std::size_t window_bytes = 0;
  std::size_t window_crc_failures = 0;
  std::vector<std::uint8_t> stream;
  stream.reserve(4096);
  ImuRawSample pending;
  bool have_inertial = false;
  bool have_euler = false;

  while (running_.load()) {
    pollfd descriptor{serial_fd_, POLLIN, 0};
    const int poll_result = ::poll(&descriptor, 1, kReadTimeoutMs);
    if (poll_result > 0 && (descriptor.revents & POLLIN) != 0) {
      std::array<std::uint8_t, 4096> chunk{};
      const ssize_t result = ::read(serial_fd_, chunk.data(), chunk.size());
      if (result > 0) {
        stream.insert(stream.end(), chunk.begin(), chunk.begin() + result);
        window_bytes += static_cast<std::size_t>(result);
      }
    } else if (poll_result > 0) {
      // POLLERR/POLLHUP：设备掉线，避免空转占满 CPU；由上层 tryRecover() 重开。
      std::this_thread::sleep_for(std::chrono::milliseconds(kReadTimeoutMs));
    }

    // 按 AA 55 LEN 对齐；长度未知或 CRC 失败时只丢弃一个字节重新同步。
    std::size_t offset = 0;
    while (stream.size() - offset >= 3) {
      if (stream[offset] != kHeader0 || stream[offset + 1] != kHeader1) {
        ++offset;
        continue;
      }
      const std::uint8_t length = stream[offset + 2];
      if (length != kInertialLength && length != kEulerLength) {
        ++offset;
        continue;
      }
      const std::size_t frame_size = static_cast<std::size_t>(length) + 5U;
      if (stream.size() - offset < frame_size) {break;}
      // 先解到临时量，校验失败时不污染已收到的另一半数据。
      const std::uint8_t * frame = stream.data() + offset;
      std::array<float, 3> first{};
      std::array<float, 3> second{};
      const bool decoded = length == kInertialLength ?
        decodeInertialFrame(frame, frame_size, first, second) :
        decodeEulerFrame(frame, frame_size, first);
      if (!decoded) {
        ++window_crc_failures;
        ++offset;
        continue;
      }
      offset += frame_size;
      if (length == kInertialLength) {
        pending.angular_velocity = first;
        pending.acceleration = second;
        have_inertial = true;
      } else {
        pending.rpy_degrees = first;
        have_euler = true;
      }
      if (!(have_inertial && have_euler)) {continue;}

      have_inertial = have_euler = false;
      const auto received_at = std::chrono::steady_clock::now();
      last_valid_sample = received_at;
      window_bytes = 0;
      window_crc_failures = 0;
      pending.received_at = received_at;
      pending.sequence = ++sample_count;
      // 接收线程只报告首帧；周期性终端输出会阻塞串口热路径。
      if (sample_count == 1) {
        imu_log::print(
          imu_log::Level::Info,
          "[HFI IMU] status=UPDATING seq=1 | acc=(%.3f, %.3f, %.3f) "
          "gyro=(%.3f, %.3f, %.3f) rpy_deg=(%.3f, %.3f, %.3f)\n",
          pending.acceleration[0], pending.acceleration[1], pending.acceleration[2],
          pending.angular_velocity[0], pending.angular_velocity[1],
          pending.angular_velocity[2], pending.rpy_degrees[0], pending.rpy_degrees[1],
          pending.rpy_degrees[2]);
      }
      std::lock_guard<std::mutex> lock(mutex_);
      latest_sample_ = pending;
      has_sample_ = true;
    }
    stream.erase(stream.begin(), stream.begin() + static_cast<std::ptrdiff_t>(offset));
    // 长时间没有帧头时防止缓存无限增长。
    if (stream.size() > 8192) {stream.erase(stream.begin(), stream.end() - 2);}

    const auto now = std::chrono::steady_clock::now();
    if (running_.load() && now - last_valid_sample >= std::chrono::seconds(1) &&
      now - last_wait_log >= std::chrono::seconds(1))
    {
      if (window_bytes == 0) {
        imu_log::print(
          imu_log::Level::Warning,
          "[HFI IMU] status=NO_DATA seq=%llu; no serial bytes received; "
          "check device path and baudrate.\n",
          static_cast<unsigned long long>(sample_count));
      } else {
        imu_log::print(
          imu_log::Level::Warning,
          "[HFI IMU] status=INVALID_FRAME seq=%llu; received %zu raw bytes "
          "(%zu CRC failures) but no valid 0x2C+0x14 frame pair; check baudrate, "
          "or the device is not an HFI-A9 (B6/B9 use 0x55 0x5x frames).\n",
          static_cast<unsigned long long>(sample_count), window_bytes,
          window_crc_failures);
      }
      last_wait_log = now;
      window_bytes = 0;
      window_crc_failures = 0;
    }
  }
}
