#include <cmath>

#include <gtest/gtest.h>
#include <linux/can.h>

#include "hardware/dm_motor_driver.hpp"

namespace
{
std::uint16_t floatToUint(float value, float minimum, float maximum, unsigned bits)
{
  const float normalized = (value - minimum) / (maximum - minimum);
  return static_cast<std::uint16_t>(std::lround(
           normalized * static_cast<float>((1U << bits) - 1U)));
}
}

TEST(DmMotorDriverTest, DecodesEightByteMitFeedbackHealthFields)
{
  canfd_frame frame{};
  frame.can_id = 0x14;
  frame.len = 8;
  const auto q = floatToUint(1.25F, -12.566F, 12.566F, 16);
  const auto dq = floatToUint(-2.0F, -20.0F, 20.0F, 12);
  const auto tau = floatToUint(7.5F, -120.0F, 120.0F, 12);
  frame.data[0] = static_cast<std::uint8_t>((3U << 4) | 4U);
  frame.data[1] = static_cast<std::uint8_t>(q >> 8);
  frame.data[2] = static_cast<std::uint8_t>(q);
  frame.data[3] = static_cast<std::uint8_t>(dq >> 4);
  frame.data[4] = static_cast<std::uint8_t>(((dq & 0x0F) << 4) | (tau >> 8));
  frame.data[5] = static_cast<std::uint8_t>(tau);
  frame.data[6] = 61;
  frame.data[7] = 58;

  DmMotorDriver::DecodedFeedback decoded;
  ASSERT_TRUE(DmMotorDriver::decodeFeedback(frame, decoded));
  EXPECT_EQ(decoded.motor_id, 4);
  EXPECT_EQ(decoded.status, 3);
  EXPECT_FALSE(DmMotorDriver::isHealthyStatus(decoded.status));
  EXPECT_NEAR(decoded.position, 1.25F, 0.001F);
  EXPECT_NEAR(decoded.velocity, -2.0F, 0.02F);
  EXPECT_NEAR(decoded.torque, 7.5F, 0.06F);
  EXPECT_FLOAT_EQ(decoded.mos_temperature_c, 61.0F);
  EXPECT_FLOAT_EQ(decoded.rotor_temperature_c, 58.0F);
}

TEST(DmMotorDriverTest, UsesPhysicalCanIdForCommandsAndMasterIdForFeedback)
{
  const dm1_hardware::MotorAddress address{1, 0x01, 0x11};
  const auto mit = DmMotorDriver::encodeMitFrame(address, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
  const auto enable = DmMotorDriver::encodeCommandFrame(address, 0xFC);
  const auto disable = DmMotorDriver::encodeCommandFrame(address, 0xFD);
  const auto set_zero = DmMotorDriver::encodeCommandFrame(address, 0xFE);
  EXPECT_EQ(mit.can_id, 0x01U);
  EXPECT_EQ(enable.can_id, 0x01U);
  EXPECT_EQ(disable.can_id, 0x01U);
  EXPECT_EQ(set_zero.can_id, 0x01U);

  canfd_frame feedback_frame{};
  feedback_frame.len = 8;
  feedback_frame.data[0] = 0x01;
  DmMotorDriver::DecodedFeedback decoded;
  ASSERT_TRUE(DmMotorDriver::decodeFeedback(feedback_frame, decoded));
  EXPECT_TRUE(DmMotorDriver::matchesFeedbackAddress(address, 0x11, decoded));
  EXPECT_FALSE(DmMotorDriver::matchesFeedbackAddress(address, 0x01, decoded));
}

TEST(DmMotorDriverTest, EnabledStatusIsHealthy)
{
  EXPECT_TRUE(DmMotorDriver::isHealthyStatus(0));
  EXPECT_TRUE(DmMotorDriver::isHealthyStatus(1));
  EXPECT_FALSE(DmMotorDriver::isHealthyStatus(2));
}

TEST(DmMotorDriverTest, RejectsNonEightByteFeedback)
{
  canfd_frame frame{};
  frame.can_id = 0x14;
  frame.len = 6;
  DmMotorDriver::DecodedFeedback decoded;
  EXPECT_FALSE(DmMotorDriver::decodeFeedback(frame, decoded));
}

TEST(DmMotorDriverTest, RejectsUnsupportedCanBus)
{
  DmMotorDriver::CalibrationArray invalid{};
  for (std::size_t index = 0; index < invalid.size(); ++index) {
    invalid[index] = dm1_hardware::MotorCalibration{
      dm1_hardware::MotorAddress{
        static_cast<std::uint8_t>(index == 0 ? 2 : index / 6),
        static_cast<std::uint16_t>(index + 1),
        static_cast<std::uint16_t>(index + 0x11)},
      index == 0 ? -1 : 1, 0.0F};
  }
  EXPECT_THROW(DmMotorDriver("can0", "can1", invalid), std::invalid_argument);
}
