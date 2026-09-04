#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "controller/RlPolicy.hpp"
#include "controller/frozen_dwaq_policy.hpp"
#include "controller/leg_controller.hpp"
#include "hardware/dm1_mit_interface.hpp"
#include "model/quadruped.hpp"
#include "model/robot_control_parameters.hpp"

namespace
{
class RecordingMitTransport final : public dm1_hardware::MitTransport
{
public:
  bool sendMit(const dm1_hardware::MitFrame & frame) override
  {
    frames.push_back(frame);
    return true;
  }
  void disableAll() noexcept override {disabled = true;}
  std::vector<dm1_hardware::MitFrame> frames;
  bool disabled = false;
};
}

TEST(Dm1Contract, HasSingleModelAndCanonicalJointOrder)
{
  const auto model = makeQuadruped<float>(RobotType::DM1);
  const auto parameters = makeRobotControlParameters<float>(RobotType::DM1);
  EXPECT_EQ(model.robotType(), RobotType::DM1);
  EXPECT_FLOAT_EQ(model.nominalBodyHeight(), 0.39F);
  EXPECT_EQ(parameters.rl_observation_size, kRlObservationSize);
  EXPECT_EQ(parameters.rl_history_length, kRlHistoryLength);
  EXPECT_FLOAT_EQ(parameters.rl_policy_period, 0.02F);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_EQ(static_cast<std::size_t>(model.leg(static_cast<LegId>(leg)).leg), leg);
    const bool is_rear = leg >= static_cast<std::size_t>(LegId::RR);
    EXPECT_TRUE(model.leg(static_cast<LegId>(leg)).joints.home_position.isApprox(
      Vec3<float>(0.0F, -0.597F, is_rear ? 1.468F : 1.432F)));
  }
}

TEST(Dm1Contract, AnalyticKinematicsIsFiniteAtHome)
{
  const auto model = makeQuadruped<float>(RobotType::DM1);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    Mat3<float> jacobian;
    Vec3<float> position;
    computeLegJacobianAndPosition(
      model, model.leg(static_cast<LegId>(leg)).joints.home_position,
      &jacobian, &position, static_cast<LegId>(leg));
    EXPECT_TRUE(jacobian.allFinite());
    EXPECT_TRUE(position.allFinite());
    EXPECT_LT(position.z(), 0.0F);
  }
}

TEST(Dm1Contract, PolicyInterfaceCarriesGoldenVectorShape)
{
  bool called = false;
  auto policy = std::make_shared<CallbackRlPolicy>(
    [&called](const std::array<float, kRlObservationSize> & observation,
      const std::array<float, kRlObservationSize * kRlHistoryLength> & history,
      std::array<float, kRlActionSize> & action) {
      called = true;
      EXPECT_FLOAT_EQ(observation[0], history[kRlObservationSize]);
      action.fill(0.0F);
      return true;
    }, RlPolicyMetadata{
      "model_3285", kDm1FlatCheckpointSha256, false, true, true});
  std::array<float, kRlObservationSize> observation{};
  std::array<float, kRlObservationSize * kRlHistoryLength> history{};
  std::array<float, kRlActionSize> action{};
  EXPECT_TRUE(policy->infer(observation, history, action));
  EXPECT_TRUE(called);
  EXPECT_TRUE(std::all_of(action.begin(), action.end(), [](float value) {
    return std::isfinite(value);
  }));
  EXPECT_FALSE(policy->metadata().supports_stairs);
}

TEST(Dm1Contract, FrozenModel3485AdapterProducesFiniteActions)
{
  FrozenDwaqPolicy policy;
  const RlPolicyMetadata metadata = policy.metadata();
  // Keep the legacy selector name because FSM_State_Locomotion is unchanged;
  // the active checkpoint is identified by the new SHA below.
  EXPECT_EQ(metadata.name, "model_3285");
  EXPECT_EQ(metadata.checkpoint_sha256, kDm1FlatCheckpointSha256);
  EXPECT_TRUE(metadata.frozen);
  EXPECT_TRUE(metadata.uses_vae_posterior_mean);
  EXPECT_FALSE(metadata.supports_stairs);

  std::array<float, kRlObservationSize> observation{};
  std::array<float, kRlObservationSize * kRlHistoryLength> history{};
  std::array<float, kRlActionSize> action{};
  EXPECT_TRUE(policy.infer(observation, history, action));
  EXPECT_TRUE(std::all_of(action.begin(), action.end(), [](float value) {
    return std::isfinite(value);
  }));
}

TEST(Dm1Contract, FrozenModel3485AdapterMatchesRepeatedInitialHistory)
{
  FrozenDwaqPolicy policy;
  std::array<float, kRlObservationSize> observation{};
  for (std::size_t index = 0; index < observation.size(); ++index) {
    observation[index] = 0.01F * static_cast<float>(index + 1);
  }
  std::array<float, kRlObservationSize * kRlHistoryLength> startup_history{};
  std::copy(
    observation.begin(), observation.end(), startup_history.end() - kRlObservationSize);
  std::array<float, kRlObservationSize * kRlHistoryLength> repeated_history{};
  for (std::size_t frame = 0; frame < kRlHistoryLength; ++frame) {
    std::copy(
      observation.begin(), observation.end(),
      repeated_history.begin() + frame * kRlObservationSize);
  }
  std::array<float, kRlActionSize> startup_action{};
  std::array<float, kRlActionSize> repeated_action{};
  ASSERT_TRUE(policy.infer(observation, startup_history, startup_action));
  ASSERT_TRUE(policy.infer(observation, repeated_history, repeated_action));
  for (std::size_t index = 0; index < kRlActionSize; ++index) {
    EXPECT_NEAR(startup_action[index], repeated_action[index], 1e-5F);
  }
}

TEST(Dm1Contract, HardwareRequiresCalibratedFeedbackAndClampsMitOutput)
{
  RecordingMitTransport transport;
  dm1_hardware::Dm1MitInterface::CalibrationArray calibration{};
  for (std::size_t index = 0; index < calibration.size(); ++index) {
    calibration[index] = dm1_hardware::MotorCalibration{
      static_cast<std::uint8_t>(index + 1), 1, 0.0F};
  }
  dm1_hardware::Dm1MitInterface hardware(transport, calibration);
  dm1_hardware::Dm1MitInterface::FeedbackArray feedback{};
  for (std::size_t index = 0; index < feedback.size(); ++index) {
    feedback[index] = dm1_hardware::MotorFeedback{
      static_cast<std::uint8_t>(index + 1), 0.0F, 0.0F, 0.0F,
      35.0F, 48.0F, 0, 1.0};
  }
  EXPECT_TRUE(hardware.updateFeedback(feedback, 1.0));

  dm1_hardware::Dm1MitInterface::CommandArray commands{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    commands[leg].leg = static_cast<LegId>(leg);
    commands[leg].enabled = true;
    commands[leg].timestamp = 1.0F;
    commands[leg].kp.setZero();
    commands[leg].kd.setZero();
  }
  EXPECT_TRUE(hardware.send(commands, 1.0));
  EXPECT_EQ(transport.frames.size(), kNumJoints);

  commands[0].torque_feedforward[0] = 31.0F;
  EXPECT_FALSE(hardware.send(commands, 1.0));
  EXPECT_TRUE(transport.disabled);
}
