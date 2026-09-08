#include "Dm1HardwareDriver.hpp"

#include <cmath>
#include <cstdio>
#include <thread>
#include <utility>

#include "sensor/imu_log.hpp"

Dm1HardwareDriver::Dm1HardwareDriver(
  std::string can0, std::string can1, std::string imu_device,
  const dm1_hardware::Dm1MitInterface::CalibrationArray & calibration)
: can0_(std::move(can0)), can1_(std::move(can1)), imu_device_(std::move(imu_device)),
  imu_reader_(imu_device_, 921600), motor_driver_(can0_, can1_, calibration)
{
}

bool Dm1HardwareDriver::open()
{
  if (!imu_reader_.open()) {
    imu_log::print(
      imu_log::Level::Error,
      "Unable to open/configure IMU serial port: %s\n", imu_device_.c_str());
    opened_ = false;
    return false;
  }
  if (!motor_driver_.open()) {
    imu_log::print(
      imu_log::Level::Error,
      "Unable to open SocketCAN buses '%s' and '%s'.\n", can0_.c_str(), can1_.c_str());
    imu_reader_.close();
    opened_ = false;
    return false;
  }
  // MIT feedback is returned in response to an MIT frame.  Poll all twelve
  // motors while they remain disabled; waiting on the receive thread alone
  // would always time out on a cold start.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  bool feedback_seen = false;
  const auto poll_start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() < deadline) {
    if (!motor_driver_.pollAll()) {
      imu_log::print(imu_log::Level::Error, "Unable to poll all DM motors during startup.\n");
      motor_driver_.close();
      imu_reader_.close();
      opened_ = false;
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
      "SocketCAN opened but 12 fresh motor feedback streams were not seen.\n");
    motor_driver_.close();
    imu_reader_.close();
    opened_ = false;
    return false;
  }
  imu_log::print(
    imu_log::Level::Info,
    "Reading DM IMU from '%s' at 921600 baud; CAN buses '%s'/'%s' are read-only until output is enabled.\n",
    imu_device_.c_str(), can0_.c_str(), can1_.c_str());
  opened_ = true;
  return true;
}

bool Dm1HardwareDriver::read(
  dm1_hardware::HardwareSample & sample, double now_s)
{
  sample = dm1_hardware::HardwareSample{};
  if (!readImu(sample.imu, now_s)) {return false;}
  return motor_driver_.latest(sample.motors, now_s);
}

bool Dm1HardwareDriver::pollMotors()
{
  return opened_ && motor_driver_.pollAll();
}

bool Dm1HardwareDriver::readImu(ImuData<float> & sample, double now_s)
{
  return opened_ && imu_reader_.readAt(sample, static_cast<float>(now_s));
}

bool Dm1HardwareDriver::sendMit(const dm1_hardware::MitFrame & frame)
{
  return opened_ && motor_driver_.sendMit(frame);
}

bool Dm1HardwareDriver::setMotorZero(const dm1_hardware::MotorAddress & address)
{
  return opened_ && motor_driver_.setZero(address);
}

bool Dm1HardwareDriver::enableAll()
{
  return opened_ && motor_driver_.enableAll();
}

void Dm1HardwareDriver::disableAll() noexcept
{
  motor_driver_.disableAll();
}

void Dm1HardwareDriver::close() noexcept
{
  imu_reader_.close();
  motor_driver_.close();
  opened_ = false;
}
