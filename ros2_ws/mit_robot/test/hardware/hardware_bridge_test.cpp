#include <array>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "hardware/hardware_bridge.hpp"
#include "hardware/dm_motor_driver.hpp"
#include "hardware/periodic_mit_sender.hpp"

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
    std::unique_lock<std::mutex> lock(mutex_);
    ++read_count;
    if (stop_after_read_count > 0 && read_count >= stop_after_read_count) {
      disable_count_before_stop = disable_count;
      stop_requested_.store(true);
    }
    if (read_count == stall_read_at) {
      sends_before_stall = send_count;
      lock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(stall_ms));
      lock.lock();
      sends_during_stall = send_count - sends_before_stall;
      stop_requested_.store(true);
    }
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
      if (stale_motor_feedback && read_count >= 70) {
        if (read_count == 70) {disable_count_at_stale_feedback = disable_count;}
        motor.sequence = 70;
        motor.timestamp = 0.0;
      }
    }
    if (fail_read_after > 0 && read_count >= fail_read_after) {
      if (failed_read_count++ == 0) {
        send_count_at_first_failed_read = send_count;
        disable_count_at_first_failed_read = disable_count;
      }
      if (damage_on_failed_read) {
        sample.motors[0].fault_code = injected_fault_code;
        sample.motors[0].temperature_c = injected_mos_temperature;
        sample.motors[0].rotor_temperature_c = injected_rotor_temperature;
        sample.motors[0].voltage_valid = injected_voltage != 0.0F;
        sample.motors[0].voltage_v = injected_voltage;
      }
      if (communication_fault) {sample.motors[0].fault_code = 0x0D;}
      if (stop_after_failed_reads > 0 && failed_read_count >= stop_after_failed_reads) {
        disable_count_before_stop = disable_count;
        stop_requested_.store(true);
      }
      return feedback_read_succeeds;
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
    std::lock_guard<std::mutex> lock(mutex_);
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
      if (!stop_requested_.load()) {disable_count_before_stop = disable_count;}
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
    if (enable_count <= failed_enable_attempts) {
      disable_count_at_enable_failure = disable_count;
      return false;
    }
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
  int stall_read_at = 0;
  int stall_ms = 0;
  int sends_before_stall = 0;
  int sends_during_stall = 0;
  int zero_count = 0;
  int disable_count = 0;
  int disable_count_at_stale_feedback = 0, disable_count_before_stop = 0;
  int close_count = 0;
  int poll_count = 0;
  int send_count = 0;
  int imu_read_count = 0;
  int enable_count = 0;
  int failed_enable_attempts = 0, disable_count_at_enable_failure = 0;
  int fail_read_after = 0;
  int failed_read_count = 0;
  int stop_after_read_count = 0;
  int stop_after_failed_reads = 0;
  int send_count_at_first_failed_read = 0;
  int disable_count_at_first_failed_read = 0;
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
  bool feedback_read_succeeds = false;
  std::uint32_t injected_fault_code = 0x0A;
  float injected_mos_temperature = 25.0F, injected_rotor_temperature = 25.0F;
  float injected_voltage = 0.0F;
  bool communication_fault = false;
  bool stale_motor_feedback = false;
  float startup_offset = 0.1F;
  std::array<float, kNumJoints> first_positions{};
  std::array<float, kNumJoints> last_positions{};
  std::array<bool, kNumJoints> seen_positions{};

private:
  std::mutex mutex_;
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

TEST(HardwareBridgeTest, CsvTracksPublishedFramesAndNeverInventsUnpublishedTargets)
{
  struct LogDirectory
  {
    std::filesystem::path path = std::filesystem::temp_directory_path() /
      ("dm1_csv_contract_" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::optional<std::string> old;
    LogDirectory()
    {
      if (const char * value = std::getenv("DM1_LOG_DIR")) {old = value;}
      setenv("DM1_LOG_DIR", path.c_str(), 1);
    }
    ~LogDirectory()
    {
      if (old) {setenv("DM1_LOG_DIR", old->c_str(), 1);} else {unsetenv("DM1_LOG_DIR");}
      std::filesystem::remove_all(path);
    }
  };
  const auto split = [](const std::string & line) {
      std::istringstream input(line);
      std::vector<std::string> fields;
      for (std::string field; std::getline(input, field, ','); ) {
        fields.push_back(field);
      }
      return fields;
    };
  for (bool rejected : {false, true}) {
    SCOPED_TRACE(rejected);
    LogDirectory logs;
    std::atomic_bool stop{false};
    StartupZeroHardware hardware(stop);
    hardware.startup_offset = rejected ? -0.1F : 0.02F;
    hardware.stop_after_read_count = 30;
    auto motor_calibration = calibration();
    if (!rejected) {
      for (std::size_t joint = 0; joint < kNumJoints; ++joint) {
        motor_calibration[joint].direction = joint % 2 ? -1 : 1;
      }
    }
    HardwareBridge::Options options;
    options.keyboard_control = true;
    options.command_source = [sent = false]() mutable {
        if (sent) {return std::vector<teleop::OperatorCommand>{};}
        sent = true;
        return std::vector<teleop::OperatorCommand>{*teleop::decodeKey('U')};
      };
    HardwareBridge bridge(hardware, motor_calibration, options);
    ASSERT_EQ(bridge.run(stop), 0);
    ASSERT_TRUE(std::filesystem::is_directory(logs.path));
    std::vector<std::filesystem::path> files;
    for (const auto & file : std::filesystem::directory_iterator(logs.path)) {
      if (file.path().extension() == ".csv") {files.push_back(file.path());}
    }
    ASSERT_EQ(files.size(), 1U);
    std::ifstream input(files.front());
    std::string line;
    ASSERT_TRUE(static_cast<bool>(std::getline(input, line)));
    const auto header = split(line);
    std::map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < header.size(); ++i) {
      columns[header[i]] = i;
    }
    std::size_t rows = 0, accepted = 0, held = 0;
    std::vector<std::string> last_published;
    for (; std::getline(input, line); ) {
      const auto fields = split(line);
      ASSERT_EQ(fields.size(), header.size());
      ++rows;
      const auto number = [&](const std::string & name) {
          return std::stod(fields.at(columns.at(name)));
        };
      const bool published = number("publish_ok") != 0;
      const bool valid = number("published_valid") != 0;
      if (rejected) {
        EXPECT_FALSE(published);
        EXPECT_FALSE(valid);
        EXPECT_TRUE(std::isnan(number("FR_calf_q_published")));
        EXPECT_TRUE(std::isnan(number("published_age_s")));
      } else {
        EXPECT_TRUE(valid);
        const char * legs[] = {"FR", "FL", "RR", "RL"};
        const char * joints[] = {"hip", "thigh", "calf"};
        for (const auto * leg : legs) {
          for (const auto * joint : joints) {
            const std::string name = std::string(leg) + "_" + joint;
            const double pos = number(name + "_kp") *
              (number(name + "_q_des") - number(name + "_q"));
            const double vel = number(name + "_kd") *
              (number(name + "_dq_des") - number(name + "_dq"));
            EXPECT_NEAR(number(name + "_tau_pos"), pos, 1.0e-5);
            EXPECT_NEAR(number(name + "_tau_vel"), vel, 1.0e-5);
            EXPECT_NEAR(number(name + "_tau_total"), pos + vel + number(name + "_ff"), 1.0e-5);
            if (published) {
              EXPECT_NEAR(number(name + "_q_published"), number(name + "_q_des"), 1.0e-6);
            } else if (!last_published.empty()) {
              EXPECT_EQ(
                fields.at(columns.at(name + "_q_published")),
                last_published.at(columns.at(name + "_q_published")));
            }
          }
        }
        if (published) {++accepted; last_published = fields;} else {
          ++held; EXPECT_GT(number("published_age_s"), 0.0);
        }
      }
    }
    EXPECT_GT(rows, 5U);
    if (!rejected) {EXPECT_GT(accepted, 5U); EXPECT_GT(held, 0U);}
  }
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
  EXPECT_GE(hardware.disable_count, 1);
  EXPECT_EQ(hardware.close_count, 1);
  EXPECT_EQ(hardware.read_count, 0);
  EXPECT_EQ(hardware.poll_count, 0);
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
  hardware.stop_after_read_count = 90;
  auto invalid_calibration = calibration();
  for (auto & item : invalid_calibration) {
    item.zero_position = 13.0F;
  }
  HardwareBridge::Options options;
  options.enable_output = true;
  HardwareBridge bridge(hardware, invalid_calibration, options);

  EXPECT_EQ(bridge.run(stop_requested), 0);
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

TEST(HardwareBridgeTest, SetZeroIgnoresPostWriteFeedback)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.startup_offset = 0.01F;
  hardware.repeat_after_zero = true;
  hardware.stop_after_poll = 3;
  HardwareBridge::Options options;
  options.set_zero = true;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 0);
  EXPECT_EQ(hardware.zero_count, static_cast<int>(kNumJoints));
  EXPECT_EQ(hardware.read_count, 0);
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

TEST(HardwareBridgeTest, EnabledOutputKeepsSendingLastSafeCommandOnReadFailure)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.startup_offset = 0.0F;
  hardware.fail_read_after = 70;
  hardware.stop_after_failed_reads = 12;  // 超过旧100ms期限仍保持。
  HardwareBridge::Options options;
  options.control_time_step = 0.02F;
  options.startup_stable_samples = 1;
  options.enable_output = true;
  HardwareBridge bridge(hardware, calibration(), options);

  EXPECT_EQ(bridge.run(stop_requested), 0);
  EXPECT_GT(hardware.send_count, hardware.send_count_at_first_failed_read);
}

TEST(HardwareBridgeTest, MotorFaultsOnlyReportAndKeepSendingUntilExplicitStop)
{
  for (bool keyboard : {false, true}) {
    for (bool read_succeeds : {false, true}) {
      for (int fault = 0; fault < 6; ++fault) {
        SCOPED_TRACE(
          ::testing::Message() << "keyboard=" << keyboard <<
            " read_succeeds=" << read_succeeds << " fault=" << fault);
        std::atomic_bool stop_requested{false};
        StartupZeroHardware hardware(stop_requested);
        hardware.startup_offset = 0.0F;
        hardware.fail_read_after = 70;
        hardware.stop_after_failed_reads = 20;
        hardware.damage_on_failed_read = true;
        hardware.feedback_read_succeeds = read_succeeds;
        hardware.injected_fault_code = fault == 0 ? 0x09 : (fault == 1 ? 0x0A : 0);
        if (fault == 2) {hardware.injected_mos_temperature = 125.0F;}
        if (fault == 3) {hardware.injected_rotor_temperature = 125.0F;}
        if (fault == 4) {hardware.injected_voltage = 65.0F;}
        if (fault == 5) {hardware.injected_voltage = 16.0F;}
        HardwareBridge::Options options;
        options.control_time_step = 0.002F;
        options.startup_stable_samples = 1;
        options.enable_output = !keyboard;
        options.keyboard_control = keyboard;
        if (keyboard) {
          options.command_source = [requested = false]() mutable {
              std::vector<teleop::OperatorCommand> batch;
              if (!requested) {
                teleop::OperatorCommand command;
                command.type = teleop::CommandType::EnableMotors;
                batch.push_back(command);
                requested = true;
              }
              return batch;
            };
        }
        HardwareBridge bridge(hardware, calibration(), options);
        EXPECT_EQ(bridge.run(stop_requested), 0);
        EXPECT_EQ(hardware.enable_count, 1);
        EXPECT_GT(hardware.send_count, hardware.send_count_at_first_failed_read);
        EXPECT_EQ(hardware.disable_count_before_stop, hardware.disable_count_at_first_failed_read);
      }
    }
  }
}

TEST(HardwareBridgeTest, SenderContinuesWhileControlThreadIsBlocked)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.startup_offset = 0.0F;
  hardware.stall_read_at = 70;
  hardware.stall_ms = 50;
  HardwareBridge::Options options;
  options.control_time_step = 0.002F;
  options.startup_stable_samples = 1;
  options.enable_output = true;
  HardwareBridge bridge(hardware, calibration(), options);
  EXPECT_EQ(bridge.run(stop_requested), 0);
  EXPECT_GE(hardware.sends_during_stall, static_cast<int>(kNumJoints * 2));
}

TEST(HardwareBridgeTest, FailedEnableReportsAndRetriesWithoutDisableRollback)
{
  for (bool keyboard : {false, true}) {
    SCOPED_TRACE(keyboard);
    std::atomic_bool stop_requested{false};
    StartupZeroHardware hardware(stop_requested);
    hardware.startup_offset = 0.0F;
    hardware.failed_enable_attempts = 1;
    hardware.stop_after_send_count = 100;
    HardwareBridge::Options options;
    options.control_time_step = 0.002F;
    options.startup_stable_samples = 1;
    options.enable_output = !keyboard;
    options.keyboard_control = keyboard;
    if (keyboard) {
      options.command_source = [requested = false]() mutable {
          std::vector<teleop::OperatorCommand> batch;
          if (!requested) {
            teleop::OperatorCommand command;
            command.type = teleop::CommandType::EnableMotors;
            batch.push_back(command);
            requested = true;
          }
          return batch;
        };
    }
    HardwareBridge bridge(hardware, calibration(), options);
    EXPECT_EQ(bridge.run(stop_requested), 0);
    EXPECT_EQ(hardware.enable_count, 2);
    EXPECT_GE(hardware.send_count, 100);
    EXPECT_EQ(hardware.disable_count_before_stop, hardware.disable_count_at_enable_failure);
  }
}

TEST(DmMotorDriverReportOnlyTest, EnableFailuresNeverSendAutomaticDisableFrames)
{
  class FailingCan final : public DmMotorDriver::CanTransport
  {
public:
    bool open(std::uint8_t, Receiver) override {return true;}
    void close(std::uint8_t) noexcept override {}
    bool write(std::uint8_t bus, const can_frame & frame) override
    {
      const bool special = std::all_of(
        frame.data, frame.data + 7,
        [](std::uint8_t byte) {return byte == 0xff;});
      if (special && frame.data[7] == 0xfd) {++disable_frames;}
      if (failure == 0 && special && frame.data[7] == 0xfc &&
        bus == 0 && frame.can_id == 1) {return false;}
      if (failure == 1 && !special) {return false;}
      // failure=2: writes succeed, but no feedback confirms enable.
      return true;
    }
    int failure = 0;
    int disable_frames = 0;
  };
  for (int failure = 0; failure < 3; ++failure) {
    SCOPED_TRACE(failure);
    FailingCan transport;
    transport.failure = failure;
    DmMotorDriver driver("virtual0", "virtual1", calibration(), &transport);
    ASSERT_TRUE(driver.open());
    EXPECT_FALSE(driver.enableAll());
    EXPECT_EQ(transport.disable_frames, 0);
    driver.disableAll();
    EXPECT_EQ(transport.disable_frames, static_cast<int>(kNumJoints));
    driver.close();
  }
}

TEST(HardwareBridgeTest, KeyboardFailureKeepsOutputUntilExplicitStop)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.startup_offset = 0.0F;
  hardware.stop_after_read_count = 90;
  HardwareBridge::Options options;
  options.control_time_step = 0.002F;
  options.keyboard_control = true;
  int disable_count_before_failure = -1;
  options.command_source = [&, enabled = false, failed = false]() mutable {
      std::vector<teleop::OperatorCommand> batch;
      teleop::OperatorCommand command;
      if (!enabled) {
        command.type = teleop::CommandType::EnableMotors;
        enabled = true;
        batch.push_back(command);
      } else if (hardware.read_count >= 70 && !failed) {
        command.type = teleop::CommandType::KeyboardFailure;
        failed = true;
        disable_count_before_failure = hardware.disable_count;
        batch.push_back(command);
      }
      return batch;
    };
  HardwareBridge bridge(hardware, calibration(), options);
  EXPECT_EQ(bridge.run(stop_requested), 0);
  EXPECT_EQ(hardware.enable_count, 1);
  EXPECT_GE(disable_count_before_failure, 0);
  EXPECT_EQ(hardware.disable_count_before_stop, disable_count_before_failure);
  EXPECT_GT(hardware.send_count, 100);
}

namespace
{
class SenderTransport final : public dm1_hardware::MitTransport
{
public:
  bool sendMit(const dm1_hardware::MitFrame & frame) override
  {
    ++attempts[static_cast<std::size_t>(frame.bus) * 6 + frame.can_id - 1];
    if (disabled.load()) {++sends_after_disable;}
    if (frame.bus == 0 && slow_bus_us > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(slow_bus_us));
    }
    if (throw_on_send) {throw std::runtime_error("injected send failure");}
    return frame.can_id != failed_id;
  }
  void disableAll() noexcept override {disabled.store(true); ++disable_count;}
  std::array<std::atomic<int>, kNumJoints> attempts{};
  std::atomic<int> disable_count{0};
  std::atomic<int> sends_after_disable{0};
  std::atomic<bool> disabled{false};
  int failed_id = 0;
  bool throw_on_send = false;
  int slow_bus_us = 0;
};

dm1_hardware::PeriodicMitSender::Frames senderFrames()
{
  dm1_hardware::PeriodicMitSender::Frames frames{};
  for (std::size_t i = 0; i < frames.size(); ++i) {
    frames[i].bus = static_cast<std::uint8_t>(i / 6);
    frames[i].can_id = static_cast<std::uint16_t>(i % 6 + 1);
  }
  return frames;
}
}

TEST(PeriodicMitSenderTest, PauseQuiescesOutputAndReenableUsesOnlyNewFrames)
{
  using namespace std::chrono_literals;
  std::atomic_bool stop{false};
  SenderTransport transport;
  dm1_hardware::PeriodicMitSender sender(transport, stop);
  EXPECT_FALSE(sender.publish(senderFrames()));
  sender.arm();
  ASSERT_TRUE(sender.publish(senderFrames()));
  std::this_thread::sleep_for(40ms);
  const auto pause_started = std::chrono::steady_clock::now();
  sender.pause();
  EXPECT_LT(std::chrono::steady_clock::now() - pause_started, 100ms);
  const int count = transport.attempts.back().load();
  EXPECT_GE(count, 2);
  std::this_thread::sleep_for(25ms);
  EXPECT_EQ(transport.attempts.back().load(), count);
  sender.arm();
  std::this_thread::sleep_for(16ms);
  EXPECT_EQ(transport.attempts.back().load(), count);
  ASSERT_TRUE(sender.publish(senderFrames()));
  std::this_thread::sleep_for(25ms);
  sender.shutdown();
  EXPECT_GT(transport.attempts.back().load(), count);
}

TEST(PeriodicMitSenderTest, OldCommandsContinueWithoutAutomaticDisable)
{
  std::atomic_bool stop{false};
  SenderTransport transport;
  dm1_hardware::PeriodicMitSender sender(transport, stop);
  sender.arm();
  ASSERT_TRUE(sender.publish(senderFrames()));
  std::this_thread::sleep_for(std::chrono::milliseconds(180));
  EXPECT_GE(transport.attempts.back().load(), 10);
  EXPECT_EQ(transport.disable_count.load(), 0);
  EXPECT_EQ(transport.sends_after_disable.load(), 0);
  EXPECT_TRUE(sender.publish(senderFrames()));
}

TEST(PeriodicMitSenderTest, EnabledWithoutFirstCommandDoesNotAutomaticallyDisable)
{
  std::atomic_bool stop{false};
  SenderTransport transport;
  dm1_hardware::PeriodicMitSender sender(transport, stop);
  sender.arm();
  std::this_thread::sleep_for(std::chrono::milliseconds(180));
  EXPECT_EQ(transport.disable_count.load(), 0);
  EXPECT_EQ(transport.attempts[0].load(), 0);
}

TEST(PeriodicMitSenderTest, FailedMotorDoesNotStarveOthersOrDisable)
{
  using namespace std::chrono_literals;
  std::atomic_bool stop{false};
  SenderTransport transport;
  transport.failed_id = 1;
  dm1_hardware::PeriodicMitSender sender(transport, stop);
  sender.arm();
  for (int i = 0; i < 20; ++i) {
    sender.publish(senderFrames());
    std::this_thread::sleep_for(10ms);
  }
  sender.shutdown();
  EXPECT_GE(transport.attempts.back().load(), 5);
  EXPECT_GT(sender.failedFrames(), 0U);
  EXPECT_EQ(transport.disable_count.load(), 0);
  EXPECT_EQ(transport.sends_after_disable.load(), 0);
}

TEST(PeriodicMitSenderTest, StopRequestDisablesWithoutWaitingForControlThread)
{
  std::atomic_bool stop{false};
  SenderTransport transport;
  dm1_hardware::PeriodicMitSender sender(transport, stop);
  sender.arm();
  ASSERT_TRUE(sender.publish(senderFrames()));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  stop.store(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  sender.shutdown();
  EXPECT_EQ(transport.disable_count.load(), 1);
  EXPECT_EQ(transport.sends_after_disable.load(), 0);
}

TEST(PeriodicMitSenderTest, TransportExceptionKeepsRetryingWithoutDisabling)
{
  std::atomic_bool stop{false};
  SenderTransport transport;
  transport.throw_on_send = true;
  dm1_hardware::PeriodicMitSender sender(transport, stop);
  sender.arm();
  ASSERT_TRUE(sender.publish(senderFrames()));
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  sender.shutdown();
  EXPECT_GT(sender.failedFrames(), 0U);
  EXPECT_GT(transport.attempts.back().load(), 0);
  EXPECT_EQ(transport.disable_count.load(), 0);
}

TEST(PeriodicMitSenderTest, BothBusesRunNearFiveHundredHz)
{
  using Clock = std::chrono::steady_clock;
  std::atomic_bool stop{false};
  SenderTransport transport;
  dm1_hardware::PeriodicMitSender sender(transport, stop);
  sender.arm();
  ASSERT_TRUE(sender.publish(senderFrames()));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  std::array<int, kNumJoints> before{};
  for (std::size_t i = 0; i < before.size(); ++i) {
    before[i] = transport.attempts[i].load();
  }
  const auto start = Clock::now();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  sender.pause();
  const double elapsed_s = std::chrono::duration<double>(Clock::now() - start).count();
  for (std::size_t i = 0; i < before.size(); ++i) {
    const double hz = (transport.attempts[i].load() - before[i]) / elapsed_s;
    EXPECT_GT(hz, 400.0) << "joint=" << i;
    EXPECT_LT(hz, 550.0) << "joint=" << i;
  }
  EXPECT_EQ(transport.disable_count.load(), 0);
}

TEST(PeriodicMitSenderTest, SlowBusDoesNotDelayOtherBus)
{
  std::atomic_bool stop{false};
  SenderTransport transport;
  transport.slow_bus_us = 3000;
  dm1_hardware::PeriodicMitSender sender(transport, stop);
  sender.arm();
  ASSERT_TRUE(sender.publish(senderFrames()));
  std::this_thread::sleep_for(std::chrono::milliseconds(180));
  sender.pause();
  EXPECT_GT(transport.attempts.back().load(), 60);
  EXPECT_GT(transport.attempts.back().load(), transport.attempts[5].load() * 4);
  EXPECT_EQ(transport.disable_count.load(), 0);
}

TEST(HardwareBridgeTest, StaleRepeatedMotorFeedbackDoesNotStopSendingOrDisable)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.startup_offset = 0.0F;
  hardware.stale_motor_feedback = true;
  hardware.stop_after_send_count = 900;
  HardwareBridge::Options options;
  options.control_time_step = 0.002F;
  options.startup_stable_samples = 1;
  options.enable_output = true;
  HardwareBridge bridge(hardware, calibration(), options);
  EXPECT_EQ(bridge.run(stop_requested), 0);
  EXPECT_GT(hardware.read_count, 70);
  EXPECT_GE(hardware.send_count, 900);
  // 启动与最终显式停机可以失能，中间不能因陈旧/重复反馈失能再使能。
  EXPECT_EQ(hardware.enable_count, 1);
  EXPECT_EQ(hardware.disable_count_before_stop, hardware.disable_count_at_stale_feedback);
}

TEST(HardwareBridgeTest, CommunicationFaultKeepsSendingBeyondOldThreeSecondTimeout)
{
  std::atomic_bool stop_requested{false};
  StartupZeroHardware hardware(stop_requested);
  hardware.startup_offset = 0.0F;
  hardware.fail_read_after = 70;
  hardware.communication_fault = true;
  hardware.stop_after_failed_reads = 1600;
  HardwareBridge::Options options;
  options.control_time_step = 0.002F;
  options.startup_stable_samples = 1;
  options.enable_output = true;
  HardwareBridge bridge(hardware, calibration(), options);
  EXPECT_EQ(bridge.run(stop_requested), 0);
  EXPECT_EQ(hardware.failed_read_count, 1600);
  EXPECT_GT(hardware.send_count - hardware.send_count_at_first_failed_read, 1000);
}
