#include <array>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "hardware/dm1_mit_interface.hpp"

namespace
{
class MockTransport final : public dm1_hardware::MitTransport
{
public:
  bool sendMit(const dm1_hardware::MitFrame & frame) override
  {
    frames.push_back(frame);
    return send_ok;
  }

  void disableAll() noexcept override {++disable_count;}

  std::vector<dm1_hardware::MitFrame> frames;
  int disable_count = 0;
  bool send_ok = true;
};

dm1_hardware::Dm1MitInterface::CalibrationArray calibration()
{
  dm1_hardware::Dm1MitInterface::CalibrationArray result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    const auto can_id = static_cast<std::uint16_t>(index % 6 + 1);
    result[index].address = dm1_hardware::MotorAddress{
      static_cast<std::uint8_t>(index / 6), can_id,
      static_cast<std::uint16_t>(can_id + 0x10)};
    result[index].direction = index == 0 ? -1 : 1;
  }
  return result;
}

dm1_hardware::Dm1MitInterface::FeedbackArray feedback(double timestamp)
{
  dm1_hardware::Dm1MitInterface::FeedbackArray result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index].bus = static_cast<std::uint8_t>(index / 6);
    result[index].can_id = static_cast<std::uint16_t>(index % 6 + 1);
    result[index].temperature_c = 25.0F;
    result[index].rotor_temperature_c = 25.0F;
    result[index].sequence = index + 1;
    result[index].health_valid = true;
    result[index].timestamp = timestamp;
  }
  return result;
}

dm1_hardware::Dm1MitInterface::CommandArray validCommands(float timestamp)
{
  dm1_hardware::Dm1MitInterface::CommandArray result{};
  for (std::size_t leg = 0; leg < result.size(); ++leg) {
    result[leg].leg = static_cast<LegId>(leg);
    result[leg].enabled = true;
    result[leg].timestamp = timestamp;
    result[leg].kp.setConstant(1.0F);
    result[leg].kd.setConstant(0.1F);
  }
  return result;
}
}  // namespace

TEST(Dm1MitInterfaceTest, ConvertsFeedbackAndSendsTwelveCalibratedFrames)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(transport, calibration());
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0), 1.0));

  const auto joints = interface.jointStates(1.0F);
  ASSERT_EQ(joints[0].leg, LegId::FR);
  EXPECT_TRUE(joints[0].valid);

  dm1_hardware::Dm1MitInterface::CommandArray commands{};
  for (std::size_t leg = 0; leg < commands.size(); ++leg) {
    commands[leg].leg = static_cast<LegId>(leg);
    commands[leg].enabled = true;
    commands[leg].timestamp = 1.0F;
    commands[leg].kp.setConstant(1.0F);
    commands[leg].kd.setConstant(0.1F);
  }
  commands[0].position_desired.x() = 0.1F;
  ASSERT_TRUE(interface.send(commands, 1.0));
  ASSERT_EQ(transport.frames.size(), kNumJoints);
  EXPECT_EQ(transport.frames.front().can_id, 1);
  EXPECT_FLOAT_EQ(transport.frames.front().position, -0.1F);
}

TEST(Dm1MitInterfaceTest, ValidatesCommandLimitsBeforeSending)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(transport, calibration());
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0), 1.0));

  dm1_hardware::Dm1MitInterface::CommandArray commands{};
  for (std::size_t leg = 0; leg < commands.size(); ++leg) {
    commands[leg].leg = static_cast<LegId>(leg);
    commands[leg].enabled = true;
    commands[leg].timestamp = 1.0F;
  }
  commands[0].position_desired[0] = 2.0F;

  EXPECT_FALSE(interface.validateCommands(commands, 1.0));
  EXPECT_TRUE(transport.frames.empty());
  EXPECT_EQ(transport.disable_count, 0);
}

TEST(Dm1MitInterfaceTest, RejectsProtocolGainBoundsBeforeEnable)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(transport, calibration());
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0), 1.0));

  auto commands = validCommands(1.0F);
  commands[0].kp[0] = dm1_hardware::mit_protocol::kKpMax + 1.0F;
  EXPECT_FALSE(interface.validateCommands(commands, 1.0));
  EXPECT_EQ(transport.disable_count, 0);

  commands = validCommands(1.0F);
  commands[0].kd[0] = dm1_hardware::mit_protocol::kKdMax + 0.1F;
  EXPECT_FALSE(interface.validateCommands(commands, 1.0));
  EXPECT_EQ(transport.disable_count, 0);
}

TEST(Dm1MitInterfaceTest, RejectsCalibratedMotorPositionOutsideWireRange)
{
  MockTransport transport;
  auto calibrated = calibration();
  calibrated[0].zero_position = 12.0F;
  dm1_hardware::Dm1MitInterface interface(transport, calibrated);
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0), 1.0));

  auto commands = validCommands(1.0F);
  commands[0].position_desired[0] = -1.0F;
  EXPECT_FALSE(interface.validateCommands(commands, 1.0));
  EXPECT_EQ(transport.disable_count, 0);
}

TEST(Dm1MitInterfaceTest, RejectsNonFiniteCalibrationZero)
{
  MockTransport transport;
  auto invalid = calibration();
  invalid[0].zero_position = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(
    dm1_hardware::Dm1MitInterface(transport, invalid), std::invalid_argument);
}

TEST(Dm1MitInterfaceTest, RejectsStaleFeedbackAndDisablesAllMotors)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(transport, calibration(), 0.05);
  EXPECT_FALSE(interface.updateFeedback(feedback(1.0), 1.1));
  EXPECT_GT(transport.disable_count, 0);
}

TEST(Dm1MitInterfaceTest, AllowsARepeatedSequenceWhileFeedbackIsFresh)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(transport, calibration());
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0), 1.0));
  EXPECT_TRUE(interface.updateFeedback(feedback(1.01), 1.01));
  EXPECT_EQ(transport.disable_count, 0);
}

TEST(Dm1MitInterfaceTest, AcceptsProtocolFeedbackWithoutBusVoltage)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(transport, calibration());
  auto samples = feedback(1.0);
  for (auto & sample : samples) {
    sample.voltage_valid = false;
    sample.voltage_v = 0.0F;
  }

  EXPECT_TRUE(interface.updateFeedback(samples, 1.0));
  EXPECT_TRUE(interface.feedbackValid());
  EXPECT_EQ(transport.disable_count, 0);
}

TEST(Dm1MitInterfaceTest, RejectsFeedbackAfterItsTimeout)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(transport, calibration(), 0.05);
  const auto first = feedback(1.0);
  ASSERT_TRUE(interface.updateFeedback(first, 1.0));
  EXPECT_FALSE(interface.updateFeedback(first, 1.06));
  EXPECT_GT(transport.disable_count, 0);
}

TEST(Dm1MitInterfaceTest, RejectsFeedbackWithoutAReceiverSequence)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(transport, calibration());
  auto invalid = feedback(1.0);
  invalid[5].sequence = 0;
  EXPECT_FALSE(interface.updateFeedback(invalid, 1.0));
  EXPECT_FALSE(interface.feedbackValid());
  EXPECT_GT(transport.disable_count, 0);
}

TEST(Dm1MitInterfaceTest, RejectsUnsupportedCanBus)
{
  MockTransport transport;
  auto invalid = calibration();
  invalid[0] = dm1_hardware::MotorCalibration{
    dm1_hardware::MotorAddress{2, 1, 0x11}, -1, 0.0F};
  EXPECT_THROW(
    dm1_hardware::Dm1MitInterface(transport, invalid), std::invalid_argument);
}

TEST(Dm1MitInterfaceTest, RejectsPhysicalIdThatCannotFitInFeedbackNibble)
{
  MockTransport transport;
  auto invalid = calibration();
  invalid[0].address.can_id = 0x10;
  EXPECT_THROW(
    dm1_hardware::Dm1MitInterface(transport, invalid), std::invalid_argument);
}
