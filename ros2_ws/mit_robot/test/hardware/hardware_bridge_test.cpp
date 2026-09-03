#include <atomic>
#include <cstdint>

#include <gtest/gtest.h>

#include "HardwareBridge.hpp"

namespace
{
class StartupZeroHardware final : public dm1_hardware::Dm1HardwareIo
{
public:
  explicit StartupZeroHardware(std::atomic_bool & stop_requested)
  : stop_requested_(stop_requested) {}

  bool open() override {return true;}

  bool read(dm1_hardware::HardwareSample & sample, double now_s) override
  {
    ++read_count;
    sample.imu.orientation_world_from_body = Eigen::Quaternionf::Identity();
    sample.imu.angular_velocity_body.setZero();
    sample.imu.acceleration_body << 0.0F, 0.0F, 9.81F;
    sample.imu.orientation_valid = true;
    sample.imu.acceleration_valid = true;
    sample.imu.valid = true;
    for (std::size_t index = 0; index < sample.motors.size(); ++index) {
      auto & motor = sample.motors[index];
      motor.id = static_cast<std::uint8_t>(index + 1);
      motor.position = zeroed ? 0.0F : 0.1F;
      motor.temperature_c = 25.0F;
      motor.voltage_v = 24.0F;
      motor.timestamp = now_s;
    }
    if (read_count >= 3) {stop_requested_.store(true);}
    return true;
  }

  bool setMotorZero(std::uint8_t) override
  {
    ++zero_count;
    if (zero_count == static_cast<int>(kNumJoints)) {zeroed = true;}
    return true;
  }

  bool sendMit(const dm1_hardware::MitFrame &) override {return true;}
  void disableAll() noexcept override {++disable_count;}
  void close() noexcept override {++close_count;}

  int read_count = 0;
  int zero_count = 0;
  int disable_count = 0;
  int close_count = 0;
  bool zeroed = false;

private:
  std::atomic_bool & stop_requested_;
};

dm1_hardware::Dm1MitInterface::CalibrationArray calibration()
{
  dm1_hardware::Dm1MitInterface::CalibrationArray result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index].id = static_cast<std::uint8_t>(index + 1);
  }
  return result;
}
}  // namespace

TEST(HardwareBridgeTest, ResetsAllMotorZerosBeforeControlLoop)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  HardwareBridge::Options options;
  options.control_time_step = 0.002F;
  options.startup_zero_tolerance_rad = 0.05F;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 0);
  EXPECT_TRUE(hardware.zeroed);
  EXPECT_EQ(hardware.zero_count, static_cast<int>(kNumJoints));
  EXPECT_GE(hardware.disable_count, 2);
  EXPECT_EQ(hardware.close_count, 1);
}
