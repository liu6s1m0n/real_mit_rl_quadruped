/*! @file hfi_imu_driver.hpp
 *  @brief HandsFree HFI-A9 IMU 的后台串口接收驱动。
 *
 *  协议移植自厂商 handsfree_ros_imu（demo/c++/cpp_driver_a9.cpp 与
 *  scripts/ros_driver_a9.py）。设备持续交替输出两种帧：
 *    - 0x2C 帧（49 字节）：时间戳 + 角速度 + 加速度 + 磁场；
 *    - 0x14 帧（25 字节）：时间戳 + roll/pitch/yaw。
 *  帧格式：AA 55 LEN ... CRC_lo CRC_hi，总长 LEN+5；CRC16/MODBUS 覆盖 LEN 起的 LEN+1 字节。
 */

#ifndef DM1_ROBOT_SENSOR_HFI_IMU_DRIVER_HPP_
#define DM1_ROBOT_SENSOR_HFI_IMU_DRIVER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "sensor/imu_driver.hpp"

/**
 * @brief 只接收 HFI-A9 数据的后台串口驱动。
 *
 * 收到一对 0x2C（惯性）+ 0x14（欧拉角）帧后合成一个 ImuRawSample，
 * 输出语义与 DM IMU 一致：加速度 m/s^2、角速度 rad/s、右手系 ZYX 欧拉角（度）。
 * 不向设备发送任何配置命令。
 */
class HfiImuDriver final : public ImuReceiver
{
public:
  static constexpr std::uint8_t kInertialLength = 0x2C;  ///< 惯性帧 LEN 字段。
  static constexpr std::uint8_t kEulerLength = 0x14;     ///< 欧拉角帧 LEN 字段。
  static constexpr std::size_t kInertialFrameSize = kInertialLength + 5;  ///< 49 字节。
  static constexpr std::size_t kEulerFrameSize = kEulerLength + 5;        ///< 25 字节。

  /// @param serial_device 串口设备路径；@param baudrate 串口波特率。
  HfiImuDriver(std::string serial_device, int baudrate);
  /// 析构时停止接收线程。
  ~HfiImuDriver() override;

  HfiImuDriver(const HfiImuDriver &) = delete;
  HfiImuDriver & operator=(const HfiImuDriver &) = delete;

  bool start() override;          ///< 打开串口并启动接收线程。
  void stop() noexcept override;  ///< 停止接收线程并关闭串口。
  bool latest(
    ImuRawSample & sample,
    std::chrono::milliseconds maximum_age = std::chrono::milliseconds(50)) const override;

  /**
   * @brief 校验并解码一个 49 字节 0x2C 惯性帧。
   * @param angular_velocity 输出传感器系角速度，rad/s。
   * @param acceleration 输出传感器系比力，m/s^2。设备原始输出为以 g 为单位的重力矢量
   *        （水平静止时 z≈-1），这里乘以 -g，使静止时 z≈+9.81，与 DM IMU 和 MuJoCo 一致。
   */
  static bool decodeInertialFrame(
    const std::uint8_t * frame, std::size_t size,
    std::array<float, 3> & angular_velocity, std::array<float, 3> & acceleration) noexcept;

  /**
   * @brief 校验并解码一个 25 字节 0x14 欧拉角帧。
   *
   * 设备 pitch/yaw 与右手系相反（厂商 ROS 驱动按 roll, -pitch, -yaw 构造四元数），
   * 这里直接输出取反后的值，使其与角速度、加速度处于同一右手坐标系。
   * 2026-11-05 实物手持倾斜验证：X 端抬起 pitch_dev=+34.6°、Y 侧抬起 roll_dev=+42.9°、
   * 逆时针转 yaw_dev 减小；取反后与陀螺仪符号及比力方向一致（残差 <0.5°）。
   * @param rpy_degrees 输出 roll/pitch/yaw，度。
   */
  static bool decodeEulerFrame(
    const std::uint8_t * frame, std::size_t size, std::array<float, 3> & rpy_degrees) noexcept;

private:
  /// 接收线程主循环：读串口、按 AA 55 同步、配对两种帧并更新 latest_sample_。
  void receiveLoop() noexcept;

  std::string serial_device_;        ///< 串口设备路径。
  int baudrate_ = 921600;            ///< 串口波特率。
  int serial_fd_ = -1;               ///< 串口文件描述符，-1 表示未打开。
  std::thread receive_thread_;       ///< 后台接收线程。
  mutable std::mutex mutex_;         ///< 保护 latest_sample_ 和 has_sample_。
  ImuRawSample latest_sample_;       ///< 最近一次合成的有效样本。
  bool has_sample_ = false;          ///< 是否已收到过有效样本。
  std::atomic_bool running_{false};  ///< 接收线程运行标志。
};

#endif  // DM1_ROBOT_SENSOR_HFI_IMU_DRIVER_HPP_
