#include <array>
#include <atomic>
#include <cstdint>
#include <cmath>

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
    sample.imu.acceleration_body <<
      (tilted || read_count == transient_tilt_sample ? 3.0F : 0.0F), 0.0F, 9.34F;
    sample.imu.orientation_valid = true;
    sample.imu.acceleration_valid = true;
    sample.imu.valid = true;
    sample.imu.sequence =
      (repeat_sequence || (zeroed && repeat_after_zero)) ? 1U :
      static_cast<std::uint64_t>(read_count);
    for (std::size_t index = 0; index < sample.motors.size(); ++index) {
      auto & motor = sample.motors[index];
      motor.bus = static_cast<std::uint8_t>(index / 6);
      motor.can_id = static_cast<std::uint16_t>(index % 6 + 1);
      motor.position = zeroed ? 0.0F :
        (seen_positions[index] ? last_positions[index] : startup_offset);
      motor.velocity = read_count == transient_sample ? transient_velocity : 0.0F;
      motor.sequence =
        (repeat_sequence || (zeroed && repeat_after_zero)) ? 1U :
        static_cast<std::uint64_t>(read_count);
      motor.temperature_c = 25.0F;
      motor.rotor_temperature_c = 25.0F;
      motor.health_valid = true;
      motor.timestamp = now_s;
    }
    if (fail_read_after > 0 && read_count >= fail_read_after) {
      if (failed_read_count++ == 0) {send_count_at_first_failed_read = send_count;}
      if (damage_on_failed_read) {sample.motors[0].fault_code = 1;}
      if (stop_after_failed_reads > 0 && failed_read_count >= stop_after_failed_reads) {
        stop_requested_.store(true);
      }
      return false;
    }
    return true;
  }

  bool setMotorZero(const dm1_hardware::MotorAddress &) override
  {
    ++zero_count;
    if (zero_count == static_cast<int>(kNumJoints)) {zeroed = true;}
    return true;
  }

  bool sendMit(const dm1_hardware::MitFrame & frame) override
  {
    ++send_count;
    const auto index = static_cast<std::size_t>(frame.bus) * 6 + frame.can_id - 1;
    if (index < last_positions.size()) {
      if (!seen_positions[index]) {
        first_positions[index] = frame.position;
        seen_positions[index] = true;
      }
      last_positions[index] = frame.position;
    }
    if (stop_after_send_count > 0 && send_count >= stop_after_send_count) {
      stop_requested_.store(true);
    }
    return true;
  }
  bool readImu(ImuData<float> & sample, double now_s) override
  {
    dm1_hardware::HardwareSample combined;
    const bool result = read(combined, now_s);
    sample = combined.imu;
    ++imu_read_count;
    if (stop_after_imu_reads > 0 && imu_read_count >= stop_after_imu_reads) {
      stop_requested_.store(true);
    }
    return result;
  }
  bool pollMotors() override
  {
    ++poll_count;
    if (stop_after_poll > 0 && poll_count >= stop_after_poll) {
      stop_requested_.store(true);
    }
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
  int send_count = 0;
  int imu_read_count = 0;
  int enable_count = 0;
  int fail_read_after = 0;
  int failed_read_count = 0;
  int stop_after_failed_reads = 0;
  int send_count_at_first_failed_read = 0;
  int action_sequence = 0;
  int first_disable_sequence = 0;
  int first_poll_sequence = 0;
  bool zeroed = false;
  bool tilted = false;
  bool stop_after_enable = false;
  int transient_sample = 0;
  float transient_velocity = 0.0F;
  int transient_tilt_sample = 0;
  int stop_after_poll = 0;
  int stop_after_send_count = 0;
  int stop_after_imu_reads = 0;
  bool repeat_sequence = false;
  bool repeat_after_zero = false;
  bool damage_on_failed_read = false;
  float startup_offset = 0.1F;
  std::array<float, kNumJoints> first_positions{};
  std::array<float, kNumJoints> last_positions{};
  std::array<bool, kNumJoints> seen_positions{};

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
  hardware.stop_after_imu_reads = 1;
  HardwareBridge::Options options;
  options.control_time_step = 0.002F;
  options.startup_stable_samples = 1;
  options.startup_zero_tolerance_rad = 0.05F;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 0);
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

TEST(HardwareBridgeTest, StartupRejectsMotionInMiddleOfStableWindow)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.startup_offset = 0.0F;
  hardware.transient_sample = 2;
  hardware.transient_velocity = 0.2F;
  HardwareBridge::Options options;
  options.enable_output = true;
  options.startup_stable_samples = 3;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 7);
  EXPECT_EQ(hardware.read_count, 2);
  EXPECT_EQ(hardware.enable_count, 0);
  EXPECT_EQ(hardware.zero_count, 0);
}

TEST(HardwareBridgeTest, StartupRejectsTiltInMiddleOfStableWindow)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.startup_offset = 0.0F;
  hardware.transient_tilt_sample = 2;
  HardwareBridge::Options options;
  options.enable_output = true;
  options.startup_stable_samples = 3;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 7);
  EXPECT_EQ(hardware.read_count, 2);
  EXPECT_EQ(hardware.enable_count, 0);
}

TEST(HardwareBridgeTest, StartupRejectsRepeatedFeedbackSnapshot)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.startup_offset = 0.0F;
  hardware.repeat_sequence = true;
  HardwareBridge::Options options;
  options.enable_output = true;
  options.startup_stable_samples = 2;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 7);
  EXPECT_GT(hardware.read_count, 1);
  EXPECT_EQ(hardware.enable_count, 0);
}

TEST(HardwareBridgeTest, SetZeroRejectsStalePostWriteFeedback)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.startup_offset = 0.01F;
  hardware.repeat_after_zero = true;
  hardware.stop_after_poll = 3;
  HardwareBridge::Options options;
  options.set_zero = true;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 7);
  EXPECT_EQ(hardware.zero_count, static_cast<int>(kNumJoints));
  EXPECT_EQ(hardware.read_count, 3);
}

TEST(HardwareBridgeTest, LargeOffsetRejectsOutputStartup)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.startup_offset = 0.1F;
  HardwareBridge::Options options;
  options.enable_output = true;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 7);
  EXPECT_EQ(hardware.zero_count, 0);
  EXPECT_EQ(hardware.enable_count, 0);
}

TEST(HardwareBridgeTest, KeyboardStartupAcceptsSingleFeedbackRound)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  HardwareBridge::Options options;
  options.keyboard_control = true;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 9);
  EXPECT_EQ(hardware.poll_count, 1);
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

TEST(HardwareBridgeTest, StandUpKeepsSendingLastSafeCommandOnReadFailure)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.startup_offset = 0.0F;
  hardware.fail_read_after = 70;
  hardware.stop_after_failed_reads = 12;
  HardwareBridge::Options options;
  options.control_time_step = 0.02F;
  options.startup_stable_samples = 1;
  options.enable_output = true;
  options.request_stand_up = true;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 0);
  EXPECT_GT(hardware.send_count, hardware.send_count_at_first_failed_read);
}

TEST(HardwareBridgeTest, StandUpDisablesForReportedMotorFault)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.startup_offset = 0.0F;
  hardware.fail_read_after = 70;
  hardware.damage_on_failed_read = true;
  HardwareBridge::Options options;
  options.control_time_step = 0.02F;
  options.startup_stable_samples = 1;
  options.enable_output = true;
  options.request_stand_up = true;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 3);
  EXPECT_GE(hardware.disable_count, 2);
}
