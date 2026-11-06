/*! @file dm1_hardware_driver.cpp
 *  @brief Dm1HardwareDriver 实现：IMU 与双 CAN 电机的打开、读取和失能。
 */
#include "common/control_config.h"
#include "hardware/dm1_hardware_driver.hpp"

#include <cmath>
#include <cstdio>
#include <thread>
#include <utility>

#include "common/console_log.hpp"

Dm1HardwareDriver::Dm1HardwareDriver(
  std::string can0, std::string can1, std::string imu_device,
  const dm1_hardware::Dm1MitInterface::CalibrationArray & calibration,
  DmMotorDriver::CanTransport * transport, ImuType imu_type)
: can0_(std::move(can0)), can1_(std::move(can1)), imu_device_(std::move(imu_device)),
  imu_type_(imu_type), imu_reader_(imu_device_, 921600, imu_type_), motor_driver_(can0_, can1_, calibration, transport)
{
}

// 先打开 IMU，再打开 CAN；任一步失败都回收已打开的设备。
// 成功返回时电机保持失能，并已收到 12 路新鲜反馈。
bool Dm1HardwareDriver::open()
{
  if (!imu_reader_.open()) {
    imu_log::print(
      imu_log::Level::Error,
      "Unable to open/configure IMU serial port: %s\n", imu_device_.c_str());
    return false;
  }
  if (!motor_driver_.open()) {
    imu_log::print(
      imu_log::Level::Error,
      "Unable to open SocketCAN buses '%s' and '%s'.\n", can0_.c_str(), can1_.c_str());
    imu_reader_.close();
    return false;
  }
  // 打开 SocketCAN 不会清除驱动器之前的使能状态。
  // 启动阶段第一次轮询反馈之前，先失能全部电机。
  imu_log::print(
    imu_log::Level::Info,
    "DM1 启动阶段：主动发送一次全电机失能帧，然后开始反馈轮询。\n");
  motor_driver_.disableAll();
  // MIT 反馈只在收到 MIT 帧后才返回。在电机保持失能的状态下轮询全部
  // 12 个电机；冷启动时只等接收线程会一直超时。
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  bool feedback_seen = false;
  const auto poll_start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() < deadline) {
    if (!motor_driver_.pollAll()) {
      imu_log::print(
        imu_log::Level::Error,
        "DM1 启动失败原因：启动阶段电机反馈轮询失败；随后执行 close()。\n");
      motor_driver_.close();
      imu_reader_.close();
      return false;
    }
    dm1_hardware::Dm1MitInterface::FeedbackArray feedback{};
    const double now_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - poll_start).count();
    if (motor_driver_.latest(feedback, now_s)) {feedback_seen = true; break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  if (!feedback_seen) {
    imu_log::print(
      imu_log::Level::Warning,
      "DM1 启动失败原因：未收到 12 路新鲜电机反馈；随后执行 close()。\n");
    motor_driver_.close();
    imu_reader_.close();
    return false;
  }
  imu_log::print(
    imu_log::Level::Info,
    "Reading %s IMU from '%s' at 921600 baud; CAN buses '%s'/'%s' stay disabled until unlocked with Shift+U.\n",
    imuTypeName(imu_type_), imu_device_.c_str(), can0_.c_str(), can1_.c_str());
  return true;
}

// 读取 IMU 和电机反馈，两者都成功才返回 true。
bool Dm1HardwareDriver::read(
  dm1_hardware::HardwareSample & sample, double now_s)
{
  sample = dm1_hardware::HardwareSample{};
  // readAt()/latest() 各自打印具体原因，这里只负责区分是哪一段失败。
  if (!readImu(sample.imu, now_s)) {return false;}
  return motor_driver_.latest(sample.motors, now_s);
}

bool Dm1HardwareDriver::pollMotors()
{
  return motor_driver_.pollAll();
}

// 读取 IMU；失败时按配置的间隔尝试重连。
bool Dm1HardwareDriver::readImu(ImuData<float> & sample, double now_s)
{
  if (imu_reader_.readAt(sample, static_cast<float>(now_s))) {
    last_imu_ok_time_s_ = now_s;
    return true;
  }
  // IMU 串口掉线（实测是 USB Hub/设备重新枚举）后自动重连：连续失败超过
  // 0.5 s 才尝试，且每 2 s 最多一次，避免和正常抖动抢串口。
  constexpr double kImuLossBeforeRecoveryS = control_config::hardware::imu_loss_before_recovery;
  constexpr double kImuRecoveryIntervalS = control_config::hardware::imu_recovery_interval;
  if (now_s - last_imu_ok_time_s_ > kImuLossBeforeRecoveryS &&
    now_s - last_imu_recovery_time_s_ > kImuRecoveryIntervalS)
  {
    last_imu_recovery_time_s_ = now_s;
    imu_reader_.tryRecover();
  }
  return false;
}

bool Dm1HardwareDriver::sendMit(const dm1_hardware::MitFrame & frame)
{
  return motor_driver_.sendMit(frame);
}

bool Dm1HardwareDriver::setMotorZero(const dm1_hardware::MotorAddress & address)
{
  return motor_driver_.setZero(address);
}

bool Dm1HardwareDriver::enableAll()
{
  return motor_driver_.enableAll();
}

void Dm1HardwareDriver::disableAll() noexcept
{
  motor_driver_.disableAll();
}

// 关闭 IMU 与电机驱动；电机驱动关闭时会先失能全部电机。
void Dm1HardwareDriver::close() noexcept
{
  imu_reader_.close();
  motor_driver_.close();
}
