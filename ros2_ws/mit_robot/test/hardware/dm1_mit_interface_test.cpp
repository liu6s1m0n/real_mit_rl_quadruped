#include <array>
#include <cmath>
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

// 超过当前上限时只截断该关节的 kp/kd/前馈力矩，整批 12 帧仍然发送。
TEST(Dm1MitInterfaceTest, ClampsPositiveAndNegativeTotalTorquePerJoint)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(
    transport, calibration(), 0.05, false, 30.0F);
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0), 1.0));

  auto commands = validCommands(1.0F);
  commands[0].kp[1] = 100.0F;
  commands[0].kd[1] = 0.2F;
  commands[0].position_desired[1] = 0.5F;
  commands[0].velocity_desired[1] = 0.4F;
  // 预计总力矩 = 100*0.5 + 0.2*0.4 = 50.08 Nm，超过 30 Nm 上限。
  ASSERT_TRUE(interface.validateCommands(commands, 1.0));
  ASSERT_TRUE(interface.send(commands, 1.0));
  ASSERT_EQ(transport.frames.size(), kNumJoints);
  const auto & positive = transport.frames[4];  // bus0/CAN2 = leg0/joint1
  // 目标位置/速度按原样保留：截断只作用于增益和前馈，不改变目标轨迹。
  EXPECT_FLOAT_EQ(positive.position, 0.5F);
  EXPECT_FLOAT_EQ(positive.velocity, 0.4F);
  // 等比缩小后，预计总力矩正好压到上限。
  EXPECT_NEAR(
    positive.kp * 0.5F + positive.kd * 0.4F + positive.torque, 30.0F, 1.0e-4F);
  EXPECT_GT(positive.kp, 0.0F);
  EXPECT_LT(positive.kp, 100.0F);
  for (std::size_t index = 0; index < transport.frames.size(); ++index) {
    if (index != 4) {
      EXPECT_FLOAT_EQ(transport.frames[index].kp, 1.0F);
      EXPECT_FLOAT_EQ(transport.frames[index].kd, 0.1F);
      EXPECT_FLOAT_EQ(transport.frames[index].torque, 0.0F);
    }
  }

  transport.frames.clear();
  commands[0].position_desired[1] = -0.5F;
  commands[0].velocity_desired[1] = -0.4F;  // -50.08 Nm -> -30 Nm
  ASSERT_TRUE(interface.validateCommands(commands, 1.0));
  ASSERT_TRUE(interface.send(commands, 1.0));
  ASSERT_EQ(transport.frames.size(), kNumJoints);
  const auto & negative = transport.frames[4];
  EXPECT_FLOAT_EQ(negative.position, -0.5F);
  EXPECT_FLOAT_EQ(negative.velocity, -0.4F);
  EXPECT_NEAR(
    negative.kp * -0.5F + negative.kd * -0.4F + negative.torque, -30.0F, 1.0e-4F);
  // 正负方向的截断比例一致（同为 30/50.08）。
  EXPECT_NEAR(negative.kp, positive.kp, 1.0e-4F);
}

TEST(Dm1MitInterfaceTest, ClampsTorqueFeedforwardToMitProtocolRange)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(
    transport, calibration(), 0.05, false, 97.0F);
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0), 1.0));

  auto commands = validCommands(1.0F);
  commands[0].kp[1] = 100.0F;
  commands[0].position_desired[1] = -1.2F;
  commands[0].torque_feedforward[1] = 200.0F;
  ASSERT_TRUE(interface.validateCommands(commands, 1.0));
  ASSERT_TRUE(interface.send(commands, 1.0));
  ASSERT_EQ(transport.frames.size(), kNumJoints);

  const auto & frame = transport.frames[4];
  EXPECT_LE(std::abs(frame.torque), dm1_hardware::mit_protocol::kTorqueMax);
  EXPECT_NEAR(frame.torque, dm1_hardware::mit_protocol::kTorqueMax, 1.0e-4F);
  EXPECT_NEAR(frame.kp, 60.0F, 1.0e-4F);
  EXPECT_NEAR(frame.kp * -1.2F + frame.torque, 48.0F, 1.0e-4F);
}

TEST(Dm1MitInterfaceTest, RejectsNonFiniteTorqueFeedforward)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(transport, calibration());
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0), 1.0));

  auto commands = validCommands(1.0F);
  commands[0].torque_feedforward[0] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(interface.validateCommands(commands, 1.0));
  EXPECT_TRUE(transport.frames.empty());
}

// 指令速度只是前馈目标，放宽到电机空载 60 rpm；超过才拒绝。
TEST(Dm1MitInterfaceTest, UsesNoLoadVelocityLimit)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(transport, calibration());
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0), 1.0));

  auto commands = validCommands(1.0F);
  commands[0].velocity_desired[0] = 5.0F;
  EXPECT_TRUE(interface.validateCommands(commands, 1.0));

  commands[0].velocity_desired[0] = 7.0F;
  EXPECT_FALSE(interface.validateCommands(commands, 1.0));
}

// 软件温度上限放到 120 摄氏度；100~120 度是警告区，不再直接失能。
TEST(Dm1MitInterfaceTest, ToleratesMotorsUpTo120c)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(transport, calibration());

  auto hot = feedback(1.0);
  for (auto & sample : hot) {
    sample.temperature_c = 110.0F;
  }
  EXPECT_TRUE(interface.updateFeedback(hot, 1.0));

  auto too_hot = feedback(1.1);
  for (auto & sample : too_hot) {
    sample.temperature_c = 125.0F;
  }
  EXPECT_FALSE(interface.updateFeedback(too_hot, 1.1));
}

// 反馈/CAN 暂时不可用时，"保持上一帧"通道必须放行，否则电机收不到帧会按
// 自己的超时失能，整条腿瞬间瘫掉。物理边界仍然照常检查。
TEST(Dm1MitInterfaceTest, HoldingLastCommandSkipsFeedbackFreshnessButKeepsBounds)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(transport, calibration());
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0), 1.0));
  auto commands = validCommands(1.0F);

  // 5 s 后反馈已过期：严格通道拒绝，保持通道放行。
  EXPECT_FALSE(interface.validateCommands(commands, 5.0));
  EXPECT_TRUE(interface.validateCommands(commands, 5.0, false));

  for (auto & command : commands) {
    command.timestamp = 5.0F;
  }
  EXPECT_TRUE(interface.send(commands, 5.0, false));
  EXPECT_EQ(transport.frames.size(), kNumJoints);

  // 保持通道不是免检通道：几何越界依旧拒绝。
  commands[0].position_desired[0] = 2.0F;
  EXPECT_FALSE(interface.validateCommands(commands, 5.0, false));
}

TEST(Dm1MitInterfaceTest, AcceptsSameSourceFloatTimestampAtDoubleBoundary)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(transport, calibration());
  constexpr double now_s = 32.006239;
  const float same_source_timestamp = static_cast<float>(now_s);
  ASSERT_GT(static_cast<double>(same_source_timestamp), now_s);
  ASSERT_TRUE(interface.updateFeedback(feedback(now_s), now_s));

  const auto commands = validCommands(same_source_timestamp);
  EXPECT_TRUE(interface.validateCommands(commands, now_s));

  auto future_commands = commands;
  future_commands[0].timestamp = static_cast<float>(now_s + 0.01);
  EXPECT_FALSE(interface.validateCommands(future_commands, now_s));
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

TEST(Dm1MitInterfaceTest, AcceptsStaleMotorFeedback)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(transport, calibration(), 0.05);
  EXPECT_TRUE(interface.updateFeedback(feedback(1.0), 1.1));
  EXPECT_TRUE(interface.feedbackValid());
  EXPECT_EQ(transport.disable_count, 0);
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

TEST(Dm1MitInterfaceTest, AllowsShortFeedbackDelayWithinRelaxedTimeout)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(transport, calibration(), 0.10);
  const auto first = feedback(1.0);
  ASSERT_TRUE(interface.updateFeedback(first, 1.0));
  EXPECT_TRUE(interface.updateFeedback(first, 1.06));
  EXPECT_EQ(transport.disable_count, 0);
}

TEST(Dm1MitInterfaceTest, RejectsFeedbackWithoutDisablingOnMissingSequence)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(transport, calibration());
  auto invalid = feedback(1.0);
  invalid[5].sequence = 0;
  EXPECT_FALSE(interface.updateFeedback(invalid, 1.0));
  EXPECT_FALSE(interface.feedbackValid());
  EXPECT_EQ(transport.disable_count, 0);
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
