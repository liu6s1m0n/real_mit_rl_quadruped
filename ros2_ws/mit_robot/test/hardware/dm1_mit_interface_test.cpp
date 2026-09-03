#include <array>
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
    result[index].id = static_cast<std::uint8_t>(index + 1);
    result[index].direction = index == 0 ? -1 : 1;
  }
  return result;
}

dm1_hardware::Dm1MitInterface::FeedbackArray feedback(double timestamp)
{
  dm1_hardware::Dm1MitInterface::FeedbackArray result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index].id = static_cast<std::uint8_t>(index + 1);
    result[index].temperature_c = 25.0F;
    result[index].voltage_v = 24.0F;
    result[index].timestamp = timestamp;
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
  EXPECT_EQ(transport.frames.front().id, 1);
  EXPECT_FLOAT_EQ(transport.frames.front().position, -0.1F);
}

TEST(Dm1MitInterfaceTest, RejectsStaleFeedbackAndDisablesAllMotors)
{
  MockTransport transport;
  dm1_hardware::Dm1MitInterface interface(transport, calibration(), 0.05);
  EXPECT_FALSE(interface.updateFeedback(feedback(1.0), 1.1));
  EXPECT_GT(transport.disable_count, 0);
}
