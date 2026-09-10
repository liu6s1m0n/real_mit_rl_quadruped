#include <atomic>
#include <cstdint>

#include <gtest/gtest.h>

#include "hardware/hardware_bridge.hpp"

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
    sample.imu.acceleration_body << (tilted ? 3.0F : 0.0F), 0.0F, 9.34F;
    sample.imu.orientation_valid = true;
    sample.imu.acceleration_valid = true;
    sample.imu.valid = true;
    for (std::size_t index = 0; index < sample.motors.size(); ++index) {
      auto & motor = sample.motors[index];
      motor.bus = static_cast<std::uint8_t>(index / 6);
      motor.can_id = static_cast<std::uint16_t>(index % 6 + 1);
      motor.position = zeroed ? 0.0F : startup_offset;
      motor.sequence = static_cast<std::uint64_t>(read_count);
      motor.temperature_c = 25.0F;
      motor.rotor_temperature_c = 25.0F;
      motor.health_valid = true;
      motor.timestamp = now_s;
    }
    if (read_count >= 3) {stop_requested_.store(true);}
    return true;
  }

  bool setMotorZero(const dm1_hardware::MotorAddress &) override
  {
    ++zero_count;
    if (zero_count == static_cast<int>(kNumJoints)) {zeroed = true;}
    return true;
  }

  bool sendMit(const dm1_hardware::MitFrame &) override {return true;}
  bool pollMotors() override
  {
    ++poll_count;
    ++action_sequence;
    if (first_poll_sequence == 0) {first_poll_sequence = action_sequence;}
    return true;
  }
  bool enableAll() override
  {
    ++enable_count;
    if (stop_after_enable) {stop_requested_.store(true);}
    return true;
  }
  void disableAll() noexcept override
  {
    ++disable_count;
    ++action_sequence;
    if (first_disable_sequence == 0) {first_disable_sequence = action_sequence;}
  }
  void close() noexcept override {++close_count; disableAll();}

  int read_count = 0;
  int zero_count = 0;
  int disable_count = 0;
  int close_count = 0;
  int poll_count = 0;
  int enable_count = 0;
  int action_sequence = 0;
  int first_disable_sequence = 0;
  int first_poll_sequence = 0;
  bool zeroed = false;
  bool tilted = false;
  bool stop_after_enable = false;
  float startup_offset = 0.1F;

private:
  std::atomic_bool & stop_requested_;
};

dm1_hardware::Dm1MitInterface::CalibrationArray calibration()
{
  dm1_hardware::Dm1MitInterface::CalibrationArray result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    const auto can_id = static_cast<std::uint16_t>(index % 6 + 1);
    result[index].address = dm1_hardware::MotorAddress{
      static_cast<std::uint8_t>(index / 6), can_id,
      static_cast<std::uint16_t>(can_id + 0x10)};
  }
  return result;
}
}  // namespace

TEST(HardwareBridgeTest, ReadOnlyStartupDoesNotResetMotorZeros)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  HardwareBridge::Options options;
  options.control_time_step = 0.002F;
  options.startup_zero_tolerance_rad = 0.05F;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 7);
  EXPECT_FALSE(hardware.zeroed);
  EXPECT_EQ(hardware.zero_count, 0);
  EXPECT_GE(hardware.disable_count, 1);
  EXPECT_EQ(hardware.close_count, 1);
  EXPECT_EQ(hardware.poll_count, 1);
  ASSERT_GT(hardware.first_poll_sequence, 0);
  EXPECT_LT(hardware.first_disable_sequence, hardware.first_poll_sequence);
}

TEST(HardwareBridgeTest, SetZeroExplicitlyResetsAllMotorZeros)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  HardwareBridge::Options options;
  options.control_time_step = 0.002F;
  options.startup_zero_tolerance_rad = 0.05F;
  options.set_zero = true;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 0);
  EXPECT_TRUE(hardware.zeroed);
  EXPECT_EQ(hardware.zero_count, static_cast<int>(kNumJoints));
  EXPECT_GE(hardware.disable_count, 2);
  EXPECT_EQ(hardware.close_count, 1);
  EXPECT_EQ(hardware.read_count, 2);
  EXPECT_EQ(hardware.poll_count, 2);
}

TEST(HardwareBridgeTest, SmallOffsetEnablesWithoutWritingMotorZeros)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.startup_offset = 0.01F;
  hardware.stop_after_enable = true;
  HardwareBridge::Options options;
  options.enable_output = true;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 0);
  EXPECT_EQ(hardware.zero_count, 0);
  EXPECT_EQ(hardware.enable_count, 1);
}

TEST(HardwareBridgeTest, OffsetWithinWindowEnablesWithoutWritingMotorZeros)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.startup_offset = 0.03F;
  hardware.stop_after_enable = true;
  HardwareBridge::Options options;
  options.enable_output = true;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 0);
  EXPECT_EQ(hardware.zero_count, 0);
  EXPECT_EQ(hardware.enable_count, 1);
}

TEST(HardwareBridgeTest, InvalidInitialMitCommandNeverEnablesMotors)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.startup_offset = 13.0F;
  hardware.stop_after_enable = true;
  auto invalid_calibration = calibration();
  for (auto & item : invalid_calibration) {
    item.zero_position = 13.0F;
  }
  HardwareBridge::Options options;
  options.enable_output = true;
  HardwareBridge bridge(hardware, invalid_calibration, options);

  EXPECT_EQ(bridge.run(stop_requested), 8);
  EXPECT_EQ(hardware.enable_count, 0);
}

TEST(HardwareBridgeTest, SetZeroWritesEvenWhenOffsetIsSmall)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.startup_offset = 0.01F;
  HardwareBridge::Options options;
  options.set_zero = true;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 0);
  EXPECT_EQ(hardware.zero_count, static_cast<int>(kNumJoints));
}

TEST(HardwareBridgeTest, LargeOffsetRejectsOutputStartup)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.startup_offset = 0.05F;
  HardwareBridge::Options options;
  options.enable_output = true;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 7);
  EXPECT_EQ(hardware.zero_count, 0);
  EXPECT_EQ(hardware.enable_count, 0);
}

TEST(HardwareBridgeTest, UsesRawStartupGravityInsteadOfRelativeQuaternion)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.zeroed = true;
  hardware.tilted = true;
  HardwareBridge::Options options;
  options.control_time_step = 0.002F;
  options.startup_zero_tolerance_rad = 0.2F;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 7);
  EXPECT_EQ(hardware.zero_count, 0);
  EXPECT_EQ(hardware.close_count, 1);
}
