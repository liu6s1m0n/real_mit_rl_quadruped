/*! @file imu_driver.hpp
 *  @brief IMU 原始样本、串口接收接口，以及 DM IMU 的后台串口接收驱动。
 */

#ifndef DM1_ROBOT_SENSOR_IMU_DRIVER_HPP_
#define DM1_ROBOT_SENSOR_IMU_DRIVER_HPP_

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
  std::array<float, 3> acceleration{};                   ///< 传感器系加速度，m/s^2。
  std::array<float, 3> angular_velocity{};               ///< 传感器系角速度，rad/s。
  std::array<float, 3> rpy_degrees{};                    ///< IMU 内部解算的 roll/pitch/yaw，度。
  std::chrono::steady_clock::time_point received_at{};  ///< 接收完成时间。
  std::uint64_t sequence = 0;                            ///< 接收序号，从 1 开始递增。
};

/// 各型号 IMU 共用的原始样本；HFI 驱动也按上面的单位和坐标语义输出。
using ImuRawSample = DmImuRawSample;

/**
 * @brief IMU 串口接收驱动的公共接口；HardwareImu 只通过它读取原始样本。
 *
 * 实现：ImuDriver（DM IMU）、HfiImuDriver（HandsFree HFI-A9）。
 */
class ImuReceiver
{
public:
  virtual ~ImuReceiver() = default;

  /** @brief 打开串口并启动接收线程。 */
  virtual bool start() = 0;

  /** @brief 停止接收线程并关闭串口。 */
  virtual void stop() noexcept = 0;

  /**
   * @brief 复制最近有效样本。
   * @param sample 输出原始数据。
   * @param maximum_age 最大允许数据年龄。
   */
  virtual bool latest(
    ImuRawSample & sample,
    std::chrono::milliseconds maximum_age = std::chrono::milliseconds(50)) const = 0;
};

/**
 * @brief 只接收 DM IMU 数据的后台串口驱动。
 *
 * start() 成功后由内部线程独占串口，调用 latest() 只复制最近一次有效帧，
 * 不会向设备发送配置命令，也不会让控制线程等待串口数据。
 */
class ImuDriver final : public ImuReceiver
{
public:
  static constexpr std::size_t kFrameSize = 57;        ///< 一帧 DM IMU 数据的字节数。
  using Frame = std::array<std::uint8_t, kFrameSize>;  ///< 一帧原始字节。

  /// @param serial_device 串口设备路径；@param baudrate 串口波特率。
  ImuDriver(std::string serial_device, int baudrate);
  /// 析构时停止接收线程。
  ~ImuDriver() override;

  ImuDriver(const ImuDriver &) = delete;
  ImuDriver & operator=(const ImuDriver &) = delete;

  /** @brief 打开串口并启动接收线程。 */
  bool start() override;

  /** @brief 停止接收线程并关闭串口。 */
  void stop() noexcept override;

  /**
   * @brief 复制最近有效帧。
   * @param sample 输出原始数据。
   * @param maximum_age 最大允许数据年龄。
   */
  bool latest(
    DmImuRawSample & sample,
    std::chrono::milliseconds maximum_age = std::chrono::milliseconds(50)) const override;

  /** @brief 校验并解码一个完整的 57 字节 DM IMU 帧。 */
  static bool decodeFrame(const Frame & frame, DmImuRawSample & sample) noexcept;

private:
  /// 接收线程主循环：读串口、按帧头同步并解码，更新 latest_sample_。
  void receiveLoop() noexcept;

  std::string serial_device_;        ///< 串口设备路径。
  int baudrate_ = 921600;            ///< 串口波特率。
  int serial_fd_ = -1;               ///< 串口文件描述符，-1 表示未打开。
  std::thread receive_thread_;       ///< 后台接收线程。
  mutable std::mutex mutex_;         ///< 保护 latest_sample_ 和 has_sample_。
  DmImuRawSample latest_sample_;     ///< 最近一次有效帧。
  bool has_sample_ = false;          ///< 是否已收到过有效帧。
  std::atomic_bool running_{false};  ///< 接收线程运行标志。
};

#endif  // DM1_ROBOT_SENSOR_IMU_DRIVER_HPP_
