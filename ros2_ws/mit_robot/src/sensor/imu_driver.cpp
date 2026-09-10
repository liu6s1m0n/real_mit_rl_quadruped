// DM IMU 后台串口接收线程。
// 这里只做传输层同步、帧校验和原始浮点数解码；单位转换和姿态处理在 imu.cpp。

#include "sensor/imu_driver.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstring>
/*Linux/POSIX 串口相关接口*/
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <utility>
#include <unistd.h>

#include "dm_imu/bsp_crc.h"
#include "common/console_log.hpp"

namespace
{
constexpr std::size_t kRecordSize = 19; /*每个数据记录固定 19 字节*/
constexpr std::size_t kPacketSize = ImuDriver::kFrameSize; /*一整帧由三个记录组成*/
constexpr std::uint8_t kFrameHeader = 0x55; /*帧头*/
constexpr std::uint8_t kFrameFlag = 0xAA;   /*帧标志*/
constexpr std::uint8_t kSlaveId = 0x01;     /*从机ID*/
constexpr int kReadTimeoutMs = 20;          /*读取超时时间（毫秒）*/

int baudConstant(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return 0;
  }
}

float readFloat(const std::uint8_t * data)
{
  float value = 0.0F;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

bool validRecord(
  const std::array<std::uint8_t, kPacketSize> & packet,
  std::size_t offset, std::uint8_t register_id, bool check_header)
{
  // The original DM driver uses the first record header for stream alignment,
  // then validates each fixed 19-byte record by its CRC. Keep that behavior:
  // some firmware versions do not repeat exactly the same header fields in
  // records 2 and 3, and the trailing byte is not part of the CRC input.
  if (check_header &&
    (packet[offset] != kFrameHeader || packet[offset + 1] != kFrameFlag ||
    packet[offset + 2] != kSlaveId || packet[offset + 3] != register_id))
  {
    return false;
  }
  const auto expected = Get_CRC16(packet.data() + offset, 16);
  const auto received = static_cast<std::uint16_t>(packet[offset + 16]) |
    static_cast<std::uint16_t>(packet[offset + 17]) << 8;
  return expected == received;
}

bool decodePacket(
  const std::array<std::uint8_t, kPacketSize> & packet,
  DmImuRawSample & sample)
{
  if (!validRecord(packet, 0, 0x01, true) ||
    !validRecord(packet, kRecordSize, 0x02, false) ||
    !validRecord(packet, 2 * kRecordSize, 0x03, false))
  {
    return false;
  }

  for (int axis = 0; axis < 3; ++axis) {
    sample.acceleration[axis] = readFloat(packet.data() + 4 + axis * sizeof(float));
    sample.angular_velocity[axis] = readFloat(
      packet.data() + kRecordSize + 4 + axis * sizeof(float));
    sample.rpy_degrees[axis] = readFloat(
      packet.data() + 2 * kRecordSize + 4 + axis * sizeof(float));
    if (!std::isfinite(sample.acceleration[axis]) ||
      !std::isfinite(sample.angular_velocity[axis]) ||
      !std::isfinite(sample.rpy_degrees[axis]))
    {
      return false;
    }
  }
  return true;
}

bool readUntil(
  int fd, std::uint8_t * data, std::size_t size,
  const std::chrono::steady_clock::time_point & deadline,
  std::size_t & bytes_seen)
{
  std::size_t received = 0;
  while (received < size) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    if (remaining <= 0) {return false;}

    pollfd descriptor{fd, POLLIN, 0};
    const int poll_result = ::poll(&descriptor, 1, static_cast<int>(remaining));
    if (poll_result <= 0 ||
      (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
    {
      return false;
    }
    const ssize_t result = ::read(fd, data + received, size - received);
    if (result > 0) {
      received += static_cast<std::size_t>(result);
      bytes_seen += static_cast<std::size_t>(result);
    } else if (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      return false;
    }
  }
  return true;
}

bool readPacket(int fd, DmImuRawSample & sample, std::size_t & bytes_seen)
{
  ImuDriver::Frame packet{};
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(kReadTimeoutMs);
  std::size_t candidate_size = 0;
  for (;; ) {
    if (!readUntil(fd, packet.data() + candidate_size, 1, deadline, bytes_seen)) {
      return false;
    }
    if (candidate_size == 0 && packet[0] != kFrameHeader) {continue;}
    ++candidate_size;
    if (candidate_size < packet.size()) {continue;}
    if (decodePacket(packet, sample)) {return true;}

    // Preserve a possible frame header inside a corrupted candidate. Without
    // this one-byte resynchronization, a valid frame immediately following a
    // bad frame would be discarded together with the bad candidate.
    const auto header = std::find(
      packet.begin() + 1, packet.end(), kFrameHeader);
    if (header == packet.end()) {
      candidate_size = 0;
    } else {
      const auto shift = static_cast<std::size_t>(header - packet.begin());
      std::memmove(packet.data(), packet.data() + shift, packet.size() - shift);
      candidate_size = packet.size() - shift;
    }
  }
}

}  // namespace

bool ImuDriver::decodeFrame(const Frame & frame, DmImuRawSample & sample) noexcept
{
  return decodePacket(frame, sample);
}

ImuDriver::ImuDriver(std::string serial_device, int baudrate)
: serial_device_(std::move(serial_device)), baudrate_(baudrate)
{
}

ImuDriver::~ImuDriver()
{
  stop();
}

bool ImuDriver::start()
{
  if (running_.load()) {return true;}
  const int speed = baudConstant(baudrate_);
  if (serial_device_.empty()) {
    imu_log::print(imu_log::Level::Error, "[DM IMU] serial device path is empty\n");
    return false;
  }
  if (speed == 0) {
    imu_log::print(
      imu_log::Level::Error, "[DM IMU] unsupported baudrate: %d\n", baudrate_);
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
      level, "[DM IMU] open(%s) failed: %s\n",
      serial_device_.c_str(), std::strerror(error));
    return false;
  }
  /* 配置串口参数 */
  termios configuration{};
  if (tcgetattr(serial_fd_, &configuration) != 0) {
    const int error = errno;
    imu_log::print(
      imu_log::Level::Error, "[DM IMU] tcgetattr failed: %s\n", std::strerror(error));
    stop();
    return false;
  }
  /* 设置串口为原始模式，8N1，无流控，非阻塞读取 */
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
      imu_log::Level::Error, "[DM IMU] tcsetattr failed: %s\n", std::strerror(error));
    stop();
    return false;
  }
  tcflush(serial_fd_, TCIFLUSH);/*清空输入缓冲区*/
  imu_log::print(
    imu_log::Level::Info, "[DM IMU] receiver started: %s, %d baud\n",
    serial_device_.c_str(), baudrate_);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_sample_ = false;
  }
  running_.store(true);
  try {
    receive_thread_ = std::thread(&ImuDriver::receiveLoop, this);
  } catch (...) {
    running_.store(false);
    stop();
    return false;
  }
  return true;
}

void ImuDriver::stop() noexcept
{
  running_.store(false);
  if (receive_thread_.joinable()) {receive_thread_.join();}
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool ImuDriver::latest(
  DmImuRawSample & sample, std::chrono::milliseconds maximum_age) const
{
  if (maximum_age.count() <= 0) {return false;}
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_sample_) {return false;}
  const auto age = std::chrono::steady_clock::now() - latest_sample_.received_at;
  if (age > maximum_age) {return false;}
  sample = latest_sample_;
  return true;
}

void ImuDriver::receiveLoop() noexcept
{
  std::uint64_t frame_count = 0;
  const auto start_time = std::chrono::steady_clock::now();
  auto last_data_log = start_time;
  std::uint64_t last_logged_sequence = 0;
  auto last_wait_log = start_time;
  std::size_t invalid_window_bytes = 0;
  while (running_.load()) {
    DmImuRawSample sample;
    std::size_t bytes_seen = 0;
    if (readPacket(serial_fd_, sample, bytes_seen)) {
      const auto received_at = std::chrono::steady_clock::now();
      sample.received_at = received_at;
      sample.sequence = ++frame_count;
      if (frame_count == 1 || received_at - last_data_log >= std::chrono::milliseconds(100)) {
        const double report_seconds = std::max(
          0.0, std::chrono::duration<double>(received_at - last_data_log).count());
        const std::uint64_t sequence_delta = frame_count - last_logged_sequence;
        imu_log::print(
          imu_log::Level::Info,
          "[DM IMU] status=UPDATING seq=%llu (+%llu/%.2fs) | acc=(%.3f, %.3f, %.3f) "
          "gyro=(%.3f, %.3f, %.3f) rpy_deg=(%.3f, %.3f, %.3f)\n",
          static_cast<unsigned long long>(frame_count),
          static_cast<unsigned long long>(sequence_delta), report_seconds,
          sample.acceleration[0], sample.acceleration[1], sample.acceleration[2],
          sample.angular_velocity[0], sample.angular_velocity[1],
          sample.angular_velocity[2], sample.rpy_degrees[0], sample.rpy_degrees[1],
          sample.rpy_degrees[2]);
        last_data_log = received_at;
        last_logged_sequence = frame_count;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_sample_ = sample;
        has_sample_ = true;
      }
    } else if (running_.load()) {
      invalid_window_bytes += bytes_seen;
      const auto now = std::chrono::steady_clock::now();
      if (now - last_wait_log >= std::chrono::seconds(1)) {
        if (invalid_window_bytes == 0) {
          imu_log::print(
            imu_log::Level::Warning,
            "[DM IMU] status=NO_DATA seq=%llu; no serial bytes received; "
            "check IMU output mode, baudrate and device path.\n",
            static_cast<unsigned long long>(frame_count));
        } else {
          imu_log::print(
            imu_log::Level::Warning,
            "[DM IMU] status=INVALID_FRAME seq=%llu; received %zu raw bytes, "
            "but no valid 57-byte frame; check baudrate or frame format.\n",
            static_cast<unsigned long long>(frame_count), invalid_window_bytes);
        }
        last_wait_log = now;
        invalid_window_bytes = 0;
      }
    }
  }
}
