#include "Dm1HardwareDriver.hpp"

#include <cmath>
#include <cstdio>
#include <utility>

Dm1HardwareDriver::Dm1HardwareDriver(
  std::string can_interface, std::string imu_device)
: can_interface_(std::move(can_interface)), imu_device_(std::move(imu_device)),
  imu_reader_(imu_device_, 921600)
{
}

bool Dm1HardwareDriver::open()
{
  if (!imu_reader_.open()) {
    std::fprintf(
      stderr, "Unable to open/configure IMU serial port: %s\n", imu_device_.c_str());
    opened_ = false;
    return false;
  }
  std::fprintf(
    stderr, "Reading DM IMU directly from '%s' at 921600 baud; CAN '%s' is not connected yet.\n",
    imu_device_.c_str(), can_interface_.c_str());
  opened_ = true;
  return true;
}

bool Dm1HardwareDriver::read(
  dm1_hardware::HardwareSample & sample, double now_s)
{
  sample = dm1_hardware::HardwareSample{};
  if (!readImu(sample.imu, now_s)) {return false;}

  // A complete HardwareSample also requires 12 valid motor feedback frames.
  // Do not fabricate them just to make an IMU-only setup pass the safety gate.
  return false;
}

bool Dm1HardwareDriver::readImu(ImuData<float> & sample, double now_s)
{
  return opened_ && imu_reader_.readAt(sample, static_cast<float>(now_s));
}

bool Dm1HardwareDriver::sendMit(const dm1_hardware::MitFrame & frame)
{
  // HARDWARE INTEGRATION POINT 4:
  // 按达妙协议打包 frame.id/position/velocity/kp/kd/torque 并发送一帧。
  // 任何编码或发送错误必须返回 false，HardwareBridge 会立即 disableAll()。
  static_cast<void>(frame);
  return opened_ && false;
}

bool Dm1HardwareDriver::setMotorZero(std::uint8_t motor_id)
{
  // HARDWARE INTEGRATION POINT 3:
  // 电机保持失能，在机器狗确认处于机械趴卧且静止时，发送达妙
  // “当前位置设为零位”命令。等待设备确认后返回 true。
  static_cast<void>(motor_id);
  return opened_ && false;
}

void Dm1HardwareDriver::disableAll() noexcept
{
  // HARDWARE INTEGRATION POINT 5:
  // 向所有已知电机发送失能帧。此函数必须 noexcept、可重复调用，并且在
  // 部分初始化失败时也安全。
}

void Dm1HardwareDriver::close() noexcept
{
  disableAll();
  imu_reader_.close();
  // Close the future CAN file descriptor or vendor SDK handle here.
  opened_ = false;
}
