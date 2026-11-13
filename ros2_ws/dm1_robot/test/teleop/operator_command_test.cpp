/*! @file operator_command_test.cpp
 *  @brief 按键解码和 OperatorCommandArbiter 仲裁规则测试。
 */
#include <chrono>
#include <vector>

#include <gtest/gtest.h>

#include "teleop/keyboard_teleop.hpp"
#include "teleop/operator_command.hpp"

namespace
{

using teleop::CommandType;
using teleop::MotorOutputState;

// 按键映射到正确的命令和速度方向。
TEST(OperatorCommandTest, MapsKeyboardKeysAndDirections)
{
  const auto enable = teleop::decodeKey('U');
  ASSERT_TRUE(enable.has_value());
  EXPECT_EQ(enable->type, CommandType::EnableMotors);

  const auto march = teleop::decodeKey('2');
  ASSERT_TRUE(march.has_value());
  EXPECT_EQ(march->type, CommandType::Motion);
  EXPECT_EQ(march->motion, teleop::Motion::MarchInPlace);
  const auto march_velocity = teleop::velocityForMotion(march->motion);
  EXPECT_FLOAT_EQ(march_velocity.forward, 0.0F);
  EXPECT_FLOAT_EQ(march_velocity.lateral, 0.0F);
  EXPECT_FLOAT_EQ(march_velocity.yaw, 0.0F);

  const auto zero_calibration = teleop::decodeKey('3');
  ASSERT_TRUE(zero_calibration.has_value());
  EXPECT_EQ(zero_calibration->type, CommandType::ZeroCalibration);
  const auto two_leg_reverse = teleop::decodeKey('4');
  ASSERT_TRUE(two_leg_reverse.has_value());
  EXPECT_EQ(two_leg_reverse->type, CommandType::Motion);
  EXPECT_EQ(two_leg_reverse->motion, teleop::Motion::TwoLegStandReverse);
  const auto two_leg_reverse_velocity =
    teleop::velocityForMotion(two_leg_reverse->motion);
  EXPECT_FLOAT_EQ(two_leg_reverse_velocity.forward, 0.0F);
  EXPECT_FLOAT_EQ(two_leg_reverse_velocity.lateral, 0.0F);
  EXPECT_FLOAT_EQ(two_leg_reverse_velocity.yaw, 0.0F);
  EXPECT_FALSE(teleop::decodeKey('5').has_value());

  const auto prone_down = teleop::decodeKey('P');
  ASSERT_TRUE(prone_down.has_value());
  EXPECT_EQ(prone_down->type, CommandType::ProneDown);

  const auto forward = teleop::decodeKey('w');
  ASSERT_TRUE(forward.has_value());
  EXPECT_EQ(forward->type, CommandType::Motion);
  EXPECT_EQ(forward->motion, teleop::Motion::Forward);
  const auto velocity = teleop::velocityForMotion(forward->motion);
  EXPECT_GT(velocity.forward, 0.0F);

  const auto backward = teleop::decodeKey('S');
  ASSERT_TRUE(backward.has_value());
  const auto backward_velocity = teleop::velocityForMotion(backward->motion);
  EXPECT_LT(backward_velocity.forward, 0.0F);
  const auto left = teleop::decodeKey('a');
  ASSERT_TRUE(left.has_value());
  EXPECT_GT(teleop::velocityForMotion(left->motion).lateral, 0.0F);
  const auto right = teleop::decodeKey('D');
  ASSERT_TRUE(right.has_value());
  EXPECT_LT(teleop::velocityForMotion(right->motion).lateral, 0.0F);
  const auto ccw = teleop::decodeKey('q');
  ASSERT_TRUE(ccw.has_value());
  EXPECT_GT(teleop::velocityForMotion(ccw->motion).yaw, 0.0F);
  const auto cw = teleop::decodeKey('E');
  ASSERT_TRUE(cw.has_value());
  EXPECT_LT(teleop::velocityForMotion(cw->motion).yaw, 0.0F);
}

// 只有电机已使能时才接受趴下请求。
TEST(OperatorCommandTest, ProneDownRequestRequiresEnabledMotors)
{
  teleop::OperatorCommandArbiter arbiter;
  arbiter.apply(*teleop::decodeKey('P'));
  EXPECT_FALSE(arbiter.takeProneDownRequest());

  arbiter.apply(*teleop::decodeKey('U'));
  arbiter.markEnabled();
  arbiter.apply(*teleop::decodeKey('p'));
  EXPECT_TRUE(arbiter.takeProneDownRequest());
  EXPECT_FALSE(arbiter.takeProneDownRequest());

  arbiter.apply(*teleop::decodeKey('2'));
  EXPECT_TRUE(arbiter.motionActive());
  EXPECT_EQ(arbiter.motion(), teleop::Motion::MarchInPlace);
}

// 3 键零点标定请求只在电机使能后有效，取走后清除，锁定时丢弃。
TEST(OperatorCommandTest, ZeroCalibrationRequestRequiresEnabledMotors)
{
  teleop::OperatorCommandArbiter arbiter;
  arbiter.apply(*teleop::decodeKey('3'));
  EXPECT_FALSE(arbiter.takeZeroCalibrationRequest());

  arbiter.apply(*teleop::decodeKey('U'));
  arbiter.markEnabled();
  arbiter.apply(*teleop::decodeKey('3'));
  EXPECT_TRUE(arbiter.takeZeroCalibrationRequest());
  EXPECT_FALSE(arbiter.takeZeroCalibrationRequest());

  arbiter.apply(*teleop::decodeKey('3'));
  arbiter.lock();
  EXPECT_FALSE(arbiter.takeZeroCalibrationRequest());
}

// 锁定状态拒绝运动命令，重复使能请求是幂等的。
TEST(OperatorCommandTest, LockedStateRejectsMotionAndEnableIsIdempotent)
{
  teleop::OperatorCommandArbiter arbiter;
  auto motion = *teleop::decodeKey('W');
  arbiter.apply(motion);
  EXPECT_EQ(arbiter.state(), MotorOutputState::Locked);
  EXPECT_FALSE(arbiter.motionActive());

  arbiter.apply(*teleop::decodeKey('U'));
  EXPECT_EQ(arbiter.state(), MotorOutputState::Enabling);
  EXPECT_TRUE(arbiter.takeEnableRequest());
  EXPECT_FALSE(arbiter.takeEnableRequest());
  arbiter.markEnabled();
  arbiter.apply(motion);
  EXPECT_TRUE(arbiter.motionActive());
}

// 失能和退出命令优先级最高。
TEST(OperatorCommandTest, DisableAndQuitHaveHighestPriority)
{
  teleop::OperatorCommandArbiter arbiter;
  arbiter.apply(*teleop::decodeKey('U'));
  arbiter.markEnabled();
  arbiter.apply(*teleop::decodeKey('W'));
  arbiter.apply(*teleop::decodeKey('0'));
  EXPECT_EQ(arbiter.state(), MotorOutputState::Locked);
  EXPECT_FALSE(arbiter.motionActive());
  EXPECT_TRUE(arbiter.takeStopRequest());

  arbiter.apply(*teleop::decodeKey('\x1b'));
  EXPECT_TRUE(arbiter.quitRequested());
  EXPECT_EQ(arbiter.state(), MotorOutputState::Locked);
}

// 同一批次中失能优先于使能。
TEST(OperatorCommandTest, DisableWinsOverEnableWithinOneBatch)
{
  teleop::OperatorCommandArbiter arbiter;
  std::vector<teleop::OperatorCommand> commands{
    *teleop::decodeKey('U'), *teleop::decodeKey('0')};

  arbiter.applyBatch(commands);

  EXPECT_EQ(arbiter.state(), MotorOutputState::Locked);
  EXPECT_FALSE(arbiter.takeEnableRequest());
  EXPECT_TRUE(arbiter.takeStopRequest());
}

// 只有批次中没有失能命令时使能才生效。
TEST(OperatorCommandTest, EnableWinsOnlyWhenNoDisableIsPresent)
{
  teleop::OperatorCommandArbiter arbiter;
  std::vector<teleop::OperatorCommand> commands{*teleop::decodeKey('U')};

  arbiter.applyBatch(commands);

  EXPECT_EQ(arbiter.state(), MotorOutputState::Enabling);
  EXPECT_TRUE(arbiter.takeEnableRequest());
}

// 运动 watchdog 超时只停止运动，不失能电机。
TEST(OperatorCommandTest, MotionWatchdogStopsWithoutDisablingMotors)
{
  teleop::OperatorCommandArbiter arbiter;
  arbiter.apply(*teleop::decodeKey('U'));
  arbiter.markEnabled();
  auto command = *teleop::decodeKey('W');
  command.watchdog = true;
  const auto now = std::chrono::steady_clock::now();
  command.received_at = now;
  arbiter.apply(command);
  arbiter.expireMotion(now + teleop::kMotionWatchdog + std::chrono::milliseconds(1));
  EXPECT_EQ(arbiter.state(), MotorOutputState::Enabled);
  EXPECT_FALSE(arbiter.motionActive());
  EXPECT_TRUE(arbiter.takeStopRequest());
}

// 终端运动命令锁存，直到显式停止。
TEST(OperatorCommandTest, TerminalMotionIsLatchedUntilExplicitStop)
{
  teleop::OperatorCommandArbiter arbiter;
  arbiter.apply(*teleop::decodeKey('U'));
  arbiter.markEnabled();
  const auto now = std::chrono::steady_clock::now();
  auto command = *teleop::decodeKey('W');
  command.received_at = now;
  arbiter.apply(command);

  arbiter.expireMotion(now + teleop::kMotionWatchdog + std::chrono::milliseconds(1));
  EXPECT_TRUE(arbiter.motionActive());
  EXPECT_EQ(arbiter.state(), MotorOutputState::Enabled);

  arbiter.apply(*teleop::decodeKey(' '));
  EXPECT_FALSE(arbiter.motionActive());
  EXPECT_TRUE(arbiter.takeStopRequest());
}

// 键盘故障回到可恢复的锁定状态。
TEST(OperatorCommandTest, KeyboardFailureReturnsToRecoverableLockedState)
{
  teleop::OperatorCommandArbiter arbiter;
  arbiter.apply(*teleop::decodeKey('U'));
  arbiter.markEnabled();
  arbiter.apply(teleop::OperatorCommand{teleop::CommandType::KeyboardFailure});
  EXPECT_EQ(arbiter.state(), MotorOutputState::Locked);
  EXPECT_TRUE(arbiter.takeStopRequest());

  arbiter.apply(*teleop::decodeKey('U'));
  EXPECT_EQ(arbiter.state(), MotorOutputState::Enabling);
  EXPECT_TRUE(arbiter.takeEnableRequest());
  arbiter.markEnabled();
  EXPECT_EQ(arbiter.state(), MotorOutputState::Enabled);
}

// GUI 运动选择不受键盘 watchdog 超时影响。
TEST(OperatorCommandTest, GuiMotionSelectionDoesNotExpireByKeyboardWatchdog)
{
  teleop::OperatorCommandArbiter arbiter;
  arbiter.apply(*teleop::decodeKey('U'));
  arbiter.markEnabled();
  auto command = *teleop::decodeKey('W');
  command.watchdog = false;
  const auto now = std::chrono::steady_clock::now();
  command.received_at = now;
  arbiter.apply(command);
  arbiter.expireMotion(now + teleop::kMotionWatchdog + std::chrono::milliseconds(1));
  EXPECT_TRUE(arbiter.motionActive());
  EXPECT_FALSE(arbiter.takeStopRequest());
}

// 窗口输入使用独立队列，不与终端命令混在一起。
TEST(OperatorCommandTest, WindowInputHasItsOwnQueue)
{
  teleop::KeyboardTeleop keyboard;
  keyboard.injectKey('W');

  EXPECT_TRUE(keyboard.consume().empty());
  const auto window_commands = keyboard.consumeWindow();
  ASSERT_EQ(window_commands.size(), 1U);
  EXPECT_EQ(window_commands.front().type, CommandType::Motion);
  EXPECT_FALSE(window_commands.front().watchdog);
}

}  // namespace
