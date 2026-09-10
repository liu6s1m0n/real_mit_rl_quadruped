/*! @file dm1_hardware_driver.hpp
 *  @brief DM1 厂商 CAN/IMU SDK 的集中接入点。
 *
 *  CAN 与 IMU 都在这里集中管理；控制器只看到统一的快照和 MIT 帧。
 */
#ifndef MYMIT_ROBOT_HARDWARE_DM1_HARDWARE_DRIVER_HPP_
#define MYMIT_ROBOT_HARDWARE_DM1_HARDWARE_DRIVER_HPP_

#include <string>

#include "hardware/dm1_hardware_io.hpp"
#include "hardware/dm_motor_driver.hpp"
#include "sensor/imu.hpp"

class Dm1HardwareDriver final : public dm1_hardware::Dm1HardwareIo
{
public:
  Dm1HardwareDriver(
    std::string can0, std::string can1, std::string imu_device,
    const dm1_hardware::Dm1MitInterface::CalibrationArray & calibration);

  bool open() override;
  bool read(dm1_hardware::HardwareSample & sample, double now_s) override;
  bool pollMotors() override;
  /** @brief Read and convert one DM IMU frame into the control-layer format. */
  bool readImu(ImuData<float> & sample, double now_s);
  bool setMotorZero(const dm1_hardware::MotorAddress & address) override;
  bool sendMit(const dm1_hardware::MitFrame & frame) override;
  bool enableAll() override;
  void disableAll() noexcept override;
  void close() noexcept override;

private:
  std::string can0_;
  std::string can1_;
  std::string imu_device_;
  HardwareImu imu_reader_;
  DmMotorDriver motor_driver_;
  bool opened_ = false;
};

#endif  // MYMIT_ROBOT_HARDWARE_DM1_HARDWARE_DRIVER_HPP_
