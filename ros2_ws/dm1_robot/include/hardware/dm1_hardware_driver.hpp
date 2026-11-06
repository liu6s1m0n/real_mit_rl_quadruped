/*! @file dm1_hardware_driver.hpp
 *  @brief DM1 厂商 CAN/IMU SDK 的集中接入点。
 *
 *  CAN 与 IMU 都在这里集中管理；控制器只看到统一的快照和 MIT 帧。
 */
#ifndef DM1_ROBOT_HARDWARE_DM1_HARDWARE_DRIVER_HPP_
#define DM1_ROBOT_HARDWARE_DM1_HARDWARE_DRIVER_HPP_

#include <string>

#include "hardware/dm1_hardware_io.hpp"
#include "hardware/dm_motor_driver.hpp"
#include "sensor/imu.hpp"

/**
 * @brief Dm1HardwareIo 的真机实现：组合 DmMotorDriver 和 HardwareImu。
 *
 * IMU 串口断开后会按时间间隔自动尝试重连。
 */
class Dm1HardwareDriver final : public dm1_hardware::Dm1HardwareIo
{
public:
  /// transport 为 nullptr 时使用 SocketCAN；仿真时传入替代的 CAN 传输层。
  /// imu_type 选择 IMU 型号；仿真注入的是 DM 帧，因此默认 DM。
  Dm1HardwareDriver(
    std::string can0, std::string can1, std::string imu_device,
    const dm1_hardware::Dm1MitInterface::CalibrationArray & calibration,
    DmMotorDriver::CanTransport * transport = nullptr, ImuType imu_type = ImuType::Dm);

  bool open() override;  ///< 打开两条 CAN 和 IMU 串口。
  bool read(dm1_hardware::HardwareSample & sample, double now_s) override;  ///< 读取 IMU 和电机反馈。
  bool pollMotors() override;  ///< 发送零增益帧刷新电机反馈。
  /** @brief 读取一帧 IMU 数据并转换为控制层格式。 */
  bool readImu(ImuData<float> & sample, double now_s);
  bool setMotorZero(const dm1_hardware::MotorAddress & address) override;  ///< 写入电机零位。
  bool sendMit(const dm1_hardware::MitFrame & frame) override;  ///< 发送一帧 MIT 命令。
  bool enableAll() override;  ///< 使能全部电机。
  void disableAll() noexcept override;  ///< 失能全部电机。
  void close() noexcept override;  ///< 失能电机并关闭设备。

private:
  std::string can0_;            ///< 前腿 CAN 设备名（can0）。
  std::string can1_;            ///< 后腿 CAN 设备名（can1）。
  std::string imu_device_;      ///< IMU 串口设备路径。
  ImuType imu_type_;            ///< IMU 型号。
  HardwareImu imu_reader_;      ///< IMU 读取与姿态解算。
  DmMotorDriver motor_driver_;  ///< 电机驱动。
  /// IMU 串口掉线（USB 重新枚举/瞬断）后自动重连用的时间基准，单位 s。
  double last_imu_ok_time_s_ = 0.0;           ///< 最近一次成功读取 IMU 的时间。
  double last_imu_recovery_time_s_ = -1.0e9;  ///< 最近一次尝试重连的时间。
};

#endif  // DM1_ROBOT_HARDWARE_DM1_HARDWARE_DRIVER_HPP_
