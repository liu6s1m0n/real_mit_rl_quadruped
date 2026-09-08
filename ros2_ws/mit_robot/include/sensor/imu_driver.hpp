/*! @file imu_driver.hpp
 *  @brief DM IMU 的后台串口接收驱动。
 */

#ifndef MYMIT_ROBOT_SENSOR_IMU_DRIVER_HPP_
#define MYMIT_ROBOT_SENSOR_IMU_DRIVER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

/** 一帧已经完成协议校验、但尚未转换为控制层格式的 DM IMU 原始数据。 */
struct DmImuRawSample
{
  std::array<float, 3> acceleration{};
  std::array<float, 3> angular_velocity{};
  std::array<float, 3> rpy_degrees{};
  std::chrono::steady_clock::time_point received_at{};
  std::uint64_t sequence = 0;
};

/**
 * @brief 只接收 DM IMU 数据的后台串口驱动。
 *
 * start() 成功后由内部线程独占串口，调用 latest() 只复制最近一次有效帧，
 * 不会向设备发送配置命令，也不会让控制线程等待串口数据。
 */
class ImuDriver final
{
public:
  static constexpr std::size_t kFrameSize = 57;
  using Frame = std::array<std::uint8_t, kFrameSize>;

  ImuDriver(std::string serial_device, int baudrate);
  ~ImuDriver();

  ImuDriver(const ImuDriver &) = delete;
  ImuDriver & operator=(const ImuDriver &) = delete;

  /** @brief 打开串口并启动接收线程。 */
  bool start();

  /** @brief 停止接收线程并关闭串口。 */
  void stop() noexcept;

  /**
   * @brief 复制最近有效帧。
   * @param sample 输出原始数据。
   * @param maximum_age 最大允许数据年龄。
   */
  bool latest(
    DmImuRawSample & sample,
    std::chrono::milliseconds maximum_age = std::chrono::milliseconds(50)) const;

  /** @brief 校验并解码一个完整的 57 字节 DM IMU 帧。 */
  static bool decodeFrame(const Frame & frame, DmImuRawSample & sample) noexcept;

private:
  void receiveLoop() noexcept;

  std::string serial_device_;
  int baudrate_ = 921600;
  int serial_fd_ = -1;
  std::thread receive_thread_;
  mutable std::mutex mutex_;
  DmImuRawSample latest_sample_;
  bool has_sample_ = false;
  std::atomic_bool running_{false};
};

#endif  // MYMIT_ROBOT_SENSOR_IMU_DRIVER_HPP_
