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
#include "data/dm1_policy_4210_golden.hpp"

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
  EXPECT_FLOAT_EQ(parameters.rl_action_scale, 0.25F);
  EXPECT_FLOAT_EQ(parameters.rl_action_filter_time_constant, 0.05F);
  EXPECT_FLOAT_EQ(parameters.rl_max_action_delta, 0.15F);
  EXPECT_FLOAT_EQ(parameters.rl_max_target_velocity, 3.0F);
  EXPECT_FLOAT_EQ(parameters.rl_continuous_torque_limit, 30.0F);
  EXPECT_FLOAT_EQ(parameters.rl_peak_torque_limit, 97.0F);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_EQ(static_cast<std::size_t>(model.leg(static_cast<LegId>(leg)).leg), leg);
    const bool is_rear = leg >= static_cast<std::size_t>(LegId::RR);
    EXPECT_TRUE(
      model.leg(static_cast<LegId>(leg)).joints.home_position.isApprox(
        Vec3<float>(0.0F, -0.597F, is_rear ? 1.468F : 1.432F)));
    EXPECT_FLOAT_EQ(model.leg(static_cast<LegId>(leg)).joints.lower_limit[2], -0.03F);
    EXPECT_FLOAT_EQ(model.leg(static_cast<LegId>(leg)).joints.upper_limit[2], 2.72F);
  }
}

TEST(Dm1Contract, MotorFacingPdProfilesMatchTheDeployedContract)
{
  const auto parameters = makeRobotControlParameters<float>(RobotType::DM1);
  EXPECT_TRUE(parameters.initialization_kp.isApprox(
      Vec3<float>(100.0F, 100.0F, 100.0F)));
  EXPECT_TRUE(parameters.initialization_kd.isApprox(
      Vec3<float>(2.0F, 2.0F, 2.0F)));
  EXPECT_TRUE(parameters.prone_home_joint_kp.isApprox(
      Vec3<float>(100.0F, 100.0F, 100.0F)));
  EXPECT_TRUE(parameters.prone_home_joint_kd.isApprox(
      Vec3<float>(2.0F, 2.0F, 2.0F)));
  // 机身姿态环属于 BalanceStand 的 WBC 任务增益，与 RL 的直接关节 PD 不同；
  // 保留 MPC/WBC 已验证的阻尼，避免站立姿态在接触切换时欠阻尼。
  EXPECT_TRUE(parameters.balance_body_orientation_kp.isApprox(
      Vec3<float>(100.0F, 100.0F, 40.0F)));
  EXPECT_TRUE(parameters.balance_body_orientation_kd.isApprox(
      Vec3<float>(18.0F, 18.0F, 8.0F)));
  EXPECT_TRUE(parameters.balance_joint_kp.isApprox(
      Vec3<float>(100.0F, 100.0F, 100.0F)));
  EXPECT_TRUE(parameters.balance_joint_kd.isApprox(
      Vec3<float>(2.0F, 2.0F, 2.0F)));
  EXPECT_TRUE(parameters.stand_up_joint_kp.isApprox(
      Vec3<float>(100.0F, 100.0F, 100.0F)));
  EXPECT_TRUE(parameters.stand_up_joint_kd.isApprox(
      Vec3<float>(2.0F, 2.0F, 2.0F)));
  EXPECT_TRUE(parameters.locomotion_joint_kp.isApprox(
      Vec3<float>(100.0F, 100.0F, 100.0F)));
  EXPECT_TRUE(parameters.locomotion_joint_kd.isApprox(
      Vec3<float>(5.0F, 5.0F, 5.0F)));
}

// 所有会被写进 command.kp_joint/kd_joint、最终原样下发给 DM 电机的增益，
// 都必须落在 MIT 协议 0..500 / 0..5 范围内；越界会让 dm1_mit_interface
// 的 validateCommands() 直接 fail() 并 disableAll()，真机上等于整车失能。
TEST(Dm1Contract, MotorFacingJointGainsStayInsideMitProtocolRange)
{
  const auto parameters = makeRobotControlParameters<float>(RobotType::DM1);
  // 直接下发给电机的关节 PD：初始化/趴卧保持、BalanceStand、StandUp 收腿、
  // MPC/WBC 行走（LieDown 折叠复用），以及 RlControlStep() 中写死的 RL 增益。
  const std::array<Vec3<float>, 5> kp_sets{
    parameters.prone_home_joint_kp,
    parameters.balance_joint_kp,
    parameters.stand_up_joint_kp,
    parameters.locomotion_joint_kp,
    Vec3<float>(100.0F, 100.0F, 100.0F)};
  const std::array<Vec3<float>, 5> kd_sets{
    parameters.prone_home_joint_kd,
    parameters.balance_joint_kd,
    parameters.stand_up_joint_kd,
    parameters.locomotion_joint_kd,
    Vec3<float>(2.0F, 2.0F, 2.0F)};
  for (const auto & set : kp_sets) {
    EXPECT_TRUE((set.array() >= 0.0F).all());
    EXPECT_TRUE((set.array() <= dm1_hardware::mit_protocol::kKpMax).all())
      << "Kp set exceeds the DM MIT protocol limit";
  }
  for (const auto & set : kd_sets) {
    EXPECT_TRUE((set.array() >= 0.0F).all());
    EXPECT_TRUE((set.array() <= dm1_hardware::mit_protocol::kKdMax).all())
      << "Kd set exceeds the DM MIT protocol limit";
  }
  // 行走的位置增益必须与 RL/BalanceStand 一致，切换时只有阻尼不同。
  EXPECT_TRUE(parameters.locomotion_joint_kp.isApprox(
      parameters.balance_joint_kp));
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
  // 换模型要修改的地方：测试用 metadata 模型槽位名
  auto policy = std::make_shared<CallbackRlPolicy>(
    [&called](const std::array<float, kRlObservationSize> & observation,
    const std::array<float, kRlObservationSize * kRlHistoryLength> & history,
    std::array<float, kRlActionSize> & action) {
      called = true;
      EXPECT_FLOAT_EQ(observation[0], history[kRlObservationSize]);
      action.fill(0.0F);
      return true;
    }, RlPolicyMetadata{
    "model_4210", kDm1FlatCheckpointSha256, false, true, true});
  std::array<float, kRlObservationSize> observation{};
  std::array<float, kRlObservationSize * kRlHistoryLength> history{};
  std::array<float, kRlActionSize> action{};
  EXPECT_TRUE(policy->infer(observation, history, action));
  EXPECT_TRUE(called);
  EXPECT_TRUE(
    std::all_of(
      action.begin(), action.end(), [](float value) {
        return std::isfinite(value);
      }));
  EXPECT_FALSE(policy->metadata().supports_stairs);
}

TEST(Dm1Contract, RlReferenceMatchesIsaacTrainingContract)
{
  EXPECT_FLOAT_EQ(kDm1RlDefaultJointPosition[0], 0.0F);
  EXPECT_FLOAT_EQ(kDm1RlDefaultJointPosition[1], -0.520F);
  EXPECT_FLOAT_EQ(kDm1RlDefaultJointPosition[2], 1.330F);
  EXPECT_EQ(kDm1RlDefaultJointPosition.size(), kRlActionSize);
}

// 换模型要修改的地方：测试名称和 metadata 期望值
TEST(Dm1Contract, FrozenModel4210AdapterProducesFiniteActions)
{
  FrozenDwaqPolicy policy;
  const RlPolicyMetadata metadata = policy.metadata();
  EXPECT_EQ(metadata.name, "model_4210");
  EXPECT_EQ(metadata.checkpoint_sha256, kDm1FlatCheckpointSha256);
  EXPECT_TRUE(metadata.frozen);
  EXPECT_TRUE(metadata.uses_vae_posterior_mean);
  EXPECT_FALSE(metadata.supports_stairs);

  std::array<float, kRlObservationSize> observation{};
  std::array<float, kRlObservationSize * kRlHistoryLength> history{};
  std::array<float, kRlActionSize> action{};
  EXPECT_TRUE(policy.infer(observation, history, action));
  EXPECT_TRUE(
    std::all_of(
      action.begin(), action.end(), [](float value) {
        return std::isfinite(value);
      }));
}

TEST(Dm1Contract, FrozenModel4245AdapterProducesFiniteActions)
{
  FrozenDwaqPolicy policy(FrozenDwaqModel::Model4245);
  const RlPolicyMetadata metadata = policy.metadata();
  EXPECT_EQ(metadata.name, "model_4245");
  EXPECT_EQ(metadata.checkpoint_sha256, kDm1YawRecoveryCheckpointSha256);
  EXPECT_TRUE(metadata.frozen);
  EXPECT_TRUE(metadata.uses_vae_posterior_mean);
  EXPECT_FALSE(metadata.supports_stairs);

  std::array<float, kRlObservationSize> observation{};
  std::array<float, kRlObservationSize * kRlHistoryLength> history{};
  std::array<float, kRlActionSize> action{};
  EXPECT_TRUE(policy.infer(observation, history, action));
  EXPECT_TRUE(
    std::all_of(
      action.begin(), action.end(), [](float value) {
        return std::isfinite(value);
      }));
}

TEST(Dm1Contract, FrozenModel4210AdapterMatchesRepeatedInitialHistory)
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

TEST(Dm1Contract, FrozenModel4210ActorUsesCurrentObservation)
{
  FrozenDwaqPolicy policy;
  std::array<float, kRlObservationSize> observation_a{};
  std::array<float, kRlObservationSize> observation_b{};
  std::array<float, kRlObservationSize * kRlHistoryLength> history{};
  for (std::size_t index = 0; index < history.size(); ++index) {
    history[index] = 0.002F * static_cast<float>(index + 1);
  }
  for (std::size_t index = 0; index < observation_a.size(); ++index) {
    observation_a[index] = 0.01F * static_cast<float>(index + 1);
    observation_b[index] = observation_a[index];
  }
  observation_b[0] += 0.25F;

  std::array<float, kRlActionSize> action_a{};
  std::array<float, kRlActionSize> action_b{};
  ASSERT_TRUE(policy.infer(observation_a, history, action_a));
  ASSERT_TRUE(policy.infer(observation_b, history, action_b));
  EXPECT_TRUE(
    std::any_of(
      action_a.begin(), action_a.end(),
      [&action_b, index = std::size_t{0}](float value) mutable {
        const bool differs = std::abs(value - action_b[index]) > 1.0e-6F;
        ++index;
        return differs;
      }));
}

TEST(Dm1Contract, FrozenModel4210MatchesPythonGoldenVectors)
{
  FrozenDwaqPolicy policy;
  for (std::size_t case_index = 0;
    case_index < dm1_policy_4210_golden::kObservations.size(); ++case_index)
  {
    std::array<float, kRlObservationSize> observation{};
    std::array<float, kRlObservationSize * kRlHistoryLength> history{};
    std::copy(
      dm1_policy_4210_golden::kObservations[case_index].begin(),
      dm1_policy_4210_golden::kObservations[case_index].end(), observation.begin());
    std::copy(
      dm1_policy_4210_golden::kHistories[case_index].begin(),
      dm1_policy_4210_golden::kHistories[case_index].end(), history.begin());
    std::array<float, kRlActionSize> action{};
    ASSERT_TRUE(policy.infer(observation, history, action)) << "case " << case_index;
    for (std::size_t action_index = 0; action_index < kRlActionSize; ++action_index) {
      EXPECT_NEAR(
        action[action_index],
        dm1_policy_4210_golden::kExpectedActions[case_index][action_index], 1.0e-5F)
        << "case " << case_index << ", action " << action_index;
    }
  }
}

TEST(Dm1Contract, HardwareRequiresCalibratedFeedbackAndClampsMitOutput)
{
  RecordingMitTransport transport;
  dm1_hardware::Dm1MitInterface::CalibrationArray calibration{};
  for (std::size_t index = 0; index < calibration.size(); ++index) {
    const auto bus = static_cast<std::uint8_t>(index / 6);
    const auto can_id = static_cast<std::uint16_t>(index % 6 + 1);
    calibration[index] = dm1_hardware::MotorCalibration{
      dm1_hardware::MotorAddress{
        bus, can_id, static_cast<std::uint16_t>(can_id + 0x10)},
      1, 0.0F};
  }
  dm1_hardware::Dm1MitInterface hardware(transport, calibration);
  dm1_hardware::Dm1MitInterface::FeedbackArray feedback{};
  for (std::size_t index = 0; index < feedback.size(); ++index) {
    const auto bus = static_cast<std::uint8_t>(index / 6);
    const auto can_id = static_cast<std::uint16_t>(index % 6 + 1);
    feedback[index].bus = bus;
    feedback[index].can_id = can_id;
    feedback[index].temperature_c = 35.0F;
    feedback[index].voltage_v = 48.0F;
    feedback[index].timestamp = 1.0;
    feedback[index].rotor_temperature_c = 35.0F;
    feedback[index].sequence = index + 1;
    feedback[index].health_valid = true;
    feedback[index].voltage_valid = true;
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
  for (std::size_t index = 0; index < transport.frames.size(); ++index) {
    const auto can_id = static_cast<std::uint16_t>(index % 6 + 1);
    EXPECT_EQ(transport.frames[index].bus, index / 6);
    EXPECT_EQ(transport.frames[index].can_id, can_id);
    EXPECT_EQ(transport.frames[index].master_id, can_id + 0x10);
  }

  commands[0].torque_feedforward[0] = 31.0F;
  EXPECT_FALSE(hardware.send(commands, 1.0));
  EXPECT_TRUE(transport.disabled);
}
