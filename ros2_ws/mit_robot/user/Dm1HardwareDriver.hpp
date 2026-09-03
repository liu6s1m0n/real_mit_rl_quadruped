/*! @file Dm1HardwareDriver.hpp
 *  @brief DM1 厂商 CAN/IMU SDK 的集中接入点。
 *
 *  当前仓库没有实际设备 SDK。本类默认拒绝 open()，防止占位代码误使能真机。
 *  用户只需在对应 .cpp 的四个函数中接入驱动，不要把 SDK 调用放进控制器。
 */
#ifndef MYMIT_ROBOT_USER_DM1_HARDWARE_DRIVER_HPP_
#define MYMIT_ROBOT_USER_DM1_HARDWARE_DRIVER_HPP_

#include <string>

#include "hardware/dm1_hardware_io.hpp"

class Dm1HardwareDriver final : public dm1_hardware::Dm1HardwareIo
{
public:
  Dm1HardwareDriver(std::string can_interface, std::string imu_device);

  bool open() override;
  bool read(dm1_hardware::HardwareSample & sample, double now_s) override;
  bool setMotorZero(std::uint8_t motor_id) override;
  bool sendMit(const dm1_hardware::MitFrame & frame) override;
  void disableAll() noexcept override;
  void close() noexcept override;

private:
  std::string can_interface_;
  std::string imu_device_;
  bool opened_ = false;
};

#endif  // MYMIT_ROBOT_USER_DM1_HARDWARE_DRIVER_HPP_
