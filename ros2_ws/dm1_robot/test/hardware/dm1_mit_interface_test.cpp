/*! @file dm1_mit_interface_test.cpp
 *  @brief Dm1MitInterface 反馈校验、命令校验和 MIT 帧转换测试。
 */
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "hardware/dm1_mit_interface.hpp"

namespace
{
// 收集 prepareFrames() 生成的一批帧，替代真实发送线程。
/// 收集发送帧的模拟传输层。
struct FrameSink
{
  std::vector<dm1_hardware::MitFrame> frames;
};

/// 准备一批 MIT 帧并写入 sink；命令校验失败时返回 false。
bool send(
  const dm1_hardware::Dm1MitInterface & interface, FrameSink & sink,
  const dm1_hardware::Dm1MitInterface::CommandArray & commands)
{
  dm1_hardware::Dm1MitInterface::FrameArray frames{};
  if (!interface.prepareFrames(commands, frames)) {return false;}
  sink.frames.assign(frames.begin(), frames.end());
  return true;
}

/// 测试用的 12 电机标定。
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

/// 生成一批健康的 12 路反馈。
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

/// 生成一组合法的四腿命令。
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

// 反馈按标定转换到模型坐标，并生成 12 帧已标定的 MIT 命令。
TEST(Dm1MitInterfaceTest, ConvertsFeedbackAndSendsTwelveCalibratedFrames)
{
  FrameSink transport;
  dm1_hardware::Dm1MitInterface interface(calibration());
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0)));

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
  ASSERT_TRUE(send(interface, transport, commands));
  ASSERT_EQ(transport.frames.size(), kNumJoints);
  EXPECT_EQ(transport.frames.front().can_id, 1);
  EXPECT_FLOAT_EQ(transport.frames.front().position, -0.1F);
}

// 越界关节目标被限幅，其余关节命令不受影响。
TEST(Dm1MitInterfaceTest, BoundsJointTargetsWithoutDroppingOtherCommands)
{
  FrameSink transport;
  dm1_hardware::Dm1MitInterface interface(calibration());
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0)));

  dm1_hardware::Dm1MitInterface::CommandArray commands{};
  for (std::size_t leg = 0; leg < commands.size(); ++leg) {
    commands[leg].leg = static_cast<LegId>(leg);
    commands[leg].enabled = true;
    commands[leg].timestamp = 1.0F;
  }
  commands[0].position_desired[0] = 2.0F;
  commands[0].position_desired[2] = -2.2991912079F;  // 实测回零时的膝关节目标（旧坐标 -0.079）。
  commands[1].position_desired[2] = -2.2585455489F;  // 实测站立开始时的越界（旧坐标 -0.039）。
  commands[2].position_desired[2] = 0.78F;
  commands[3].position_desired[1] = 0.2F;

  ASSERT_TRUE(interface.validateCommands(commands));
  ASSERT_TRUE(send(interface, transport, commands));
  ASSERT_EQ(transport.frames.size(), kNumJoints);
  EXPECT_FLOAT_EQ(transport.frames[0].position, -1.57F);  // 标定方向为 -1。
  EXPECT_FLOAT_EQ(transport.frames[8].position, -2.25F);
  EXPECT_FLOAT_EQ(transport.frames[9].position, 0.0F);  // 小腿上限 = 零位机械限位。
  EXPECT_FLOAT_EQ(transport.frames[10].position, -2.25F);
  EXPECT_FLOAT_EQ(transport.frames[7].position, 0.2F);
}

// 力矩饱和使用限幅后的目标位置计算。
TEST(Dm1MitInterfaceTest, UsesBoundedPositionForTorqueSaturation)
{
  FrameSink transport;
  dm1_hardware::Dm1MitInterface interface(calibration(), 30.0F);
  // FR 小腿停在趴卧角 -2.22（方向 +1、零位 0，原始读数即模型角）。
  constexpr float kProneCalf = -2.22F;
  auto prone_feedback = feedback(1.0);
  prone_feedback[2].position = kProneCalf;
  ASSERT_TRUE(interface.updateFeedback(prone_feedback));
  auto commands = validCommands(1.0F);
  commands[0].position_desired[2] = -2.2991912079F;  // 旧坐标 -0.079，越过折叠限位。
  commands[0].kp[2] = 500.0F;
  commands[0].kd[2] = 0.0F;
  commands[0].torque_feedforward[2] = -20.0F;
  ASSERT_TRUE(send(interface, transport, commands));
  ASSERT_EQ(transport.frames.size(), kNumJoints);
  const auto & frame = transport.frames[8];
  EXPECT_FLOAT_EQ(frame.position, -2.25F);
  EXPECT_NEAR(frame.kp * (frame.position - kProneCalf) + frame.torque, -30.0F, 1.0e-3F);
  // 期望值与实现走不同的 float 运算路径，相差数个 ULP。
  EXPECT_NEAR(frame.kp, 500.0F * 30.0F / 35.0F, 1.0e-3F);
}

// 非有限的目标位置在限幅之前就被拒绝。
TEST(Dm1MitInterfaceTest, RejectsNonFinitePositionBeforeBounding)
{
  FrameSink transport;
  dm1_hardware::Dm1MitInterface interface(calibration());
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0)));
  auto commands = validCommands(1.0F);
  for (const float q : {std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity()})
  {
    commands[0].position_desired[2] = q;
    EXPECT_FALSE(send(interface, transport, commands));
    EXPECT_TRUE(transport.frames.empty());
    ASSERT_TRUE(interface.updateFeedback(feedback(1.0)));
  }
}

// 超过当前上限时只截断该关节的 kp/kd/前馈力矩，整批 12 帧仍然发送。
// 正负方向的总力矩都按关节独立截断。
TEST(Dm1MitInterfaceTest, ClampsPositiveAndNegativeTotalTorquePerJoint)
{
  FrameSink transport;
  dm1_hardware::Dm1MitInterface interface(calibration(), 30.0F);
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0)));

  auto commands = validCommands(1.0F);
  commands[0].kp[1] = 100.0F;
  commands[0].kd[1] = 0.2F;
  commands[0].position_desired[1] = 0.5F;
  commands[0].velocity_desired[1] = 0.4F;
  // 预计总力矩 = 100*0.5 + 0.2*0.4 = 50.08 Nm，超过 30 Nm 上限。
  ASSERT_TRUE(interface.validateCommands(commands));
  ASSERT_TRUE(send(interface, transport, commands));
  ASSERT_EQ(transport.frames.size(), kNumJoints);
  const auto & positive = transport.frames[4];  // bus0/CAN2 对应 leg0/joint1
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
  ASSERT_TRUE(interface.validateCommands(commands));
  ASSERT_TRUE(send(interface, transport, commands));
  ASSERT_EQ(transport.frames.size(), kNumJoints);
  const auto & negative = transport.frames[4];
  EXPECT_FLOAT_EQ(negative.position, -0.5F);
  EXPECT_FLOAT_EQ(negative.velocity, -0.4F);
  EXPECT_NEAR(
    negative.kp * -0.5F + negative.kd * -0.4F + negative.torque, -30.0F, 1.0e-4F);
  // 正负方向的截断比例一致（同为 30/50.08）。
  EXPECT_NEAR(negative.kp, positive.kp, 1.0e-4F);
}

// 前馈力矩被截断到 MIT 协议编码范围内。
TEST(Dm1MitInterfaceTest, ClampsTorqueFeedforwardToMitProtocolRange)
{
  FrameSink transport;
  dm1_hardware::Dm1MitInterface interface(calibration(), 97.0F);
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0)));

  auto commands = validCommands(1.0F);
  commands[0].kp[1] = 100.0F;
  commands[0].position_desired[1] = -1.2F;
  commands[0].torque_feedforward[1] = 200.0F;
  ASSERT_TRUE(interface.validateCommands(commands));
  ASSERT_TRUE(send(interface, transport, commands));
  ASSERT_EQ(transport.frames.size(), kNumJoints);

  const auto & frame = transport.frames[4];
  EXPECT_LE(std::abs(frame.torque), dm1_hardware::mit_protocol::kTorqueMax);
  EXPECT_NEAR(frame.torque, dm1_hardware::mit_protocol::kTorqueMax, 1.0e-4F);
  EXPECT_NEAR(frame.kp, 60.0F, 1.0e-4F);
  EXPECT_NEAR(frame.kp * -1.2F + frame.torque, 48.0F, 1.0e-4F);
}

// 非有限的前馈力矩会被拒绝。
TEST(Dm1MitInterfaceTest, RejectsNonFiniteTorqueFeedforward)
{
  FrameSink transport;
  dm1_hardware::Dm1MitInterface interface(calibration());
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0)));

  auto commands = validCommands(1.0F);
  commands[0].torque_feedforward[0] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(interface.validateCommands(commands));
  EXPECT_TRUE(transport.frames.empty());
}

// 指令速度只是前馈目标，放宽到电机空载 60 rpm；超过才拒绝。
// 期望速度上限使用空载转速 60 rpm。
TEST(Dm1MitInterfaceTest, UsesNoLoadVelocityLimit)
{
  dm1_hardware::Dm1MitInterface interface(calibration());
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0)));

  auto commands = validCommands(1.0F);
  commands[0].velocity_desired[0] = 5.0F;
  EXPECT_TRUE(interface.validateCommands(commands));

  commands[0].velocity_desired[0] = 7.0F;
  EXPECT_FALSE(interface.validateCommands(commands));
}

// 软件温度上限放到 120 摄氏度；100~120 度是警告区，不再直接失能。
// 电机温度不超过 120 °C 时反馈仍然有效。
TEST(Dm1MitInterfaceTest, ToleratesMotorsUpTo120c)
{
  dm1_hardware::Dm1MitInterface interface(calibration());

  auto hot = feedback(1.0);
  for (auto & sample : hot) {
    sample.temperature_c = 110.0F;
  }
  EXPECT_TRUE(interface.updateFeedback(hot));

  auto too_hot = feedback(1.1);
  for (auto & sample : too_hot) {
    sample.temperature_c = 125.0F;
  }
  EXPECT_FALSE(interface.updateFeedback(too_hot));
}

// kp/kd 超出协议范围时拒绝命令。
TEST(Dm1MitInterfaceTest, RejectsProtocolGainBoundsBeforeEnable)
{
  dm1_hardware::Dm1MitInterface interface(calibration());
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0)));

  auto commands = validCommands(1.0F);
  commands[0].kp[0] = dm1_hardware::mit_protocol::kKpMax + 1.0F;
  EXPECT_FALSE(interface.validateCommands(commands));

  commands = validCommands(1.0F);
  commands[0].kd[0] = dm1_hardware::mit_protocol::kKdMax + 0.1F;
  EXPECT_FALSE(interface.validateCommands(commands));
}

// 标定后的电机位置超出协议编码范围时拒绝命令。
TEST(Dm1MitInterfaceTest, RejectsCalibratedMotorPositionOutsideWireRange)
{
  auto calibrated = calibration();
  calibrated[0].zero_position = 12.0F;
  dm1_hardware::Dm1MitInterface interface(calibrated);
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0)));

  auto commands = validCommands(1.0F);
  commands[0].position_desired[0] = -1.0F;
  EXPECT_FALSE(interface.validateCommands(commands));
}

// 标定零位非有限时校验失败。
TEST(Dm1MitInterfaceTest, RejectsNonFiniteCalibrationZero)
{
  auto invalid = calibration();
  invalid[0].zero_position = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(dm1_hardware::validateCalibration(invalid), std::invalid_argument);
}

// 接口层不按反馈年龄拒绝反馈，反馈新鲜度由上层监管。
TEST(Dm1MitInterfaceTest, AcceptsStaleMotorFeedback)
{
  dm1_hardware::Dm1MitInterface interface(calibration());
  EXPECT_TRUE(interface.updateFeedback(feedback(1.0)));
  EXPECT_TRUE(interface.feedbackValid());
}

// 序号未递增但时间戳更新的反馈仍被接受。
TEST(Dm1MitInterfaceTest, AllowsARepeatedSequenceWhileFeedbackIsFresh)
{
  dm1_hardware::Dm1MitInterface interface(calibration());
  ASSERT_TRUE(interface.updateFeedback(feedback(1.0)));
  EXPECT_TRUE(interface.updateFeedback(feedback(1.01)));
}

// 没有总线电压字段的反馈仍然有效。
TEST(Dm1MitInterfaceTest, AcceptsProtocolFeedbackWithoutBusVoltage)
{
  dm1_hardware::Dm1MitInterface interface(calibration());
  auto samples = feedback(1.0);
  for (auto & sample : samples) {
    sample.voltage_valid = false;
    sample.voltage_v = 0.0F;
  }

  EXPECT_TRUE(interface.updateFeedback(samples));
  EXPECT_TRUE(interface.feedbackValid());
}

// 同一批反馈被重复提交（反馈短时未更新）时仍被接受。
TEST(Dm1MitInterfaceTest, AllowsShortFeedbackDelayWithinRelaxedTimeout)
{
  dm1_hardware::Dm1MitInterface interface(calibration());
  const auto first = feedback(1.0);
  ASSERT_TRUE(interface.updateFeedback(first));
  EXPECT_TRUE(interface.updateFeedback(first));
}

// 序号为 0 的反馈被拒绝，但不触发失能。
TEST(Dm1MitInterfaceTest, RejectsFeedbackWithoutDisablingOnMissingSequence)
{
  dm1_hardware::Dm1MitInterface interface(calibration());
  auto invalid = feedback(1.0);
  invalid[5].sequence = 0;
  EXPECT_FALSE(interface.updateFeedback(invalid));
  EXPECT_FALSE(interface.feedbackValid());
}

// 总线编号不是 0/1 时标定校验失败。
TEST(Dm1MitInterfaceTest, RejectsUnsupportedCanBus)
{
  auto invalid = calibration();
  invalid[0] = dm1_hardware::MotorCalibration{
    dm1_hardware::MotorAddress{2, 1, 0x11}, -1, 0.0F};
  EXPECT_THROW(dm1_hardware::validateCalibration(invalid), std::invalid_argument);
}

// 物理 CAN ID 超过反馈帧 4 位字段时标定校验失败。
TEST(Dm1MitInterfaceTest, RejectsPhysicalIdThatCannotFitInFeedbackNibble)
{
  auto invalid = calibration();
  invalid[0].address.can_id = 0x10;
  EXPECT_THROW(dm1_hardware::validateCalibration(invalid), std::invalid_argument);
}
