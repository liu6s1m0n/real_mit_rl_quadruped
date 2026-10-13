#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "controller/RlPolicy.hpp"
#include "controller/frozen_dwaq_policy.hpp"
#include "controller/leg_controller.hpp"
#include "FSM/FSM_State_BalanceStand.h"
#include "FSM/FSM_State_Locomotion.h"
#include "hardware/dm1_mit_interface.hpp"
#include "model/quadruped.hpp"
#include "model/robot_control_parameters.hpp"
#include "model/robots/dm1.hpp"
#include "data/dm1_policy_4210_golden.hpp"
#include "fsm/fsm_test_support.hpp"

namespace
{
class DiagnosticEnvironment
{
public:
  explicit DiagnosticEnvironment(const char * value)
  {
    if (const char * old = std::getenv("DM1_CONTROL_DIAGNOSTICS")) {old_ = old;}
    setenv("DM1_CONTROL_DIAGNOSTICS", value, 1);
  }
  ~DiagnosticEnvironment()
  {
    if (old_) {setenv("DM1_CONTROL_DIAGNOSTICS", old_->c_str(), 1);}
    else {unsetenv("DM1_CONTROL_DIAGNOSTICS");}
  }
private:
  std::optional<std::string> old_;
};

std::vector<float> diagnosticVector(const std::string & log, const char * name)
{
  const std::string prefix = std::string(name) + "=[";
  const auto start = log.find(prefix);
  if (start == std::string::npos) {return {};}
  const auto end = log.find(']', start);
  std::istringstream values(log.substr(start + prefix.size(), end - start - prefix.size()));
  std::vector<float> result;
  for (float value; values >> value;) {result.push_back(value);}
  return result;
}

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

TEST(Dm1Contract, WbcDiagnosticsDistinguishRawTargetsFromStandingOverride)
{
  DiagnosticEnvironment diagnostics("1");
  test_support::FsmContext context(0.002F);
  for (auto & joint : context.joint_states) {
    joint.valid = true;
    ASSERT_TRUE(context.leg_controller.updateData(joint));
  }
  context.estimate.timestamp = 12.25F;
  context.estimate.position_world.z() = 0.32F;
  context.estimate.rpy.y() = -0.0583F;
  context.estimate.orientation_world_from_body = Eigen::AngleAxisf(-0.0583F, Vec3<float>::UnitY());
  context.estimate.rotation_world_from_body = context.estimate.orientation_world_from_body.toRotationMatrix();
  context.desired_state.body_position_world.z() = 0.32F;
  FSM_State_BalanceStand<float> standing(&context.data());
  standing.onEnter();
  ::testing::internal::CaptureStderr();
  standing.run();
  const std::string log = ::testing::internal::GetCapturedStderr();
  ASSERT_TRUE(context.leg_controller.legsEnabled()) << log;
  ASSERT_NE(log.find("[WBC][DIAGNOSTIC] t_s=12.25"), std::string::npos);
  EXPECT_NE(log.find("floating_weight=300 contacts=4"), std::string::npos);
  const auto raw = diagnosticVector(log, "q_wbc");
  const auto orientation_error = diagnosticVector(log, "task0_error");
  ASSERT_EQ(raw.size(), kNumJoints);
  ASSERT_EQ(orientation_error.size(), 3U);
  EXPECT_NEAR(orientation_error[1], 0.0583F * 0.25F, 1.0e-6F);
  EXPECT_LT(raw[7], context.joint_states[2].position[1] - 0.01F);
  EXPECT_FLOAT_EQ(context.leg_controller.commands[2].position_desired[1], context.joint_states[2].position[1]);
  context.desired_state.mode = ControlMode::Locomotion;
  context.data().locomotion_gait = GaitType::STAND;
  FSM_State_Locomotion<float> locomotion(&context.data());
  locomotion.onEnter();
  ::testing::internal::CaptureStderr();
  locomotion.run();
  const std::string locomotion_log = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(locomotion_log.find("floating_weight=3000 contacts=4"), std::string::npos);
  const auto walking_raw = diagnosticVector(locomotion_log, "q_wbc");
  ASSERT_EQ(walking_raw.size(), kNumJoints);
  EXPECT_FLOAT_EQ(context.leg_controller.commands[2].position_desired[1], context.joint_states[2].position[1]);
  EXPECT_GT(std::abs(walking_raw[7] - context.leg_controller.commands[2].position_desired[1]), 0.01F);
}

TEST(Dm1Contract, ModelLimitTargetsPassMitButAdjacentOutOfRangeTargetsFail)
{
  RecordingMitTransport transport;
  dm1_hardware::Dm1MitInterface::CalibrationArray calibration{};
  for (std::size_t index = 0; index < kNumJoints; ++index) {
    calibration[index].address = dm1_hardware::MotorAddress{
      static_cast<std::uint8_t>(index / 6),
      static_cast<std::uint16_t>(index % 6 + 1),
      static_cast<std::uint16_t>(index % 6 + 0x11)};
    calibration[index].direction = 1;
  }
  dm1_hardware::Dm1MitInterface mit(transport, calibration);
  const auto robot = makeQuadruped<float>(RobotType::DM1);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & limits = robot.leg(static_cast<LegId>(leg)).joints;
    for (int joint = 0; joint < 3; ++joint) {
      for (const float bound : {limits.lower_limit[joint], limits.upper_limit[joint]}) {
        dm1_hardware::Dm1MitInterface::CommandArray commands{};
        for (std::size_t index = 0; index < kNumLegs; ++index) {
          commands[index].leg = static_cast<LegId>(index);
          commands[index].enabled = true;
        }
        commands[leg].position_desired[joint] = bound;
        ASSERT_TRUE(mit.validateCommands(commands, 0.0, false));
        const float outside = bound == limits.lower_limit[joint] ?
          -std::numeric_limits<float>::infinity() : std::numeric_limits<float>::infinity();
        commands[leg].position_desired[joint] = std::nextafter(bound, outside);
        EXPECT_FALSE(mit.validateCommands(commands, 0.0, false));
      }
    }
  }
}

TEST(Dm1Contract, StandingJointBoundsClampKneeSupportAndVerticalCalf)
{
  const auto robot = robots::dm1::makeModel<double>();
  const auto & rear = robot.leg(LegId::RR);
  Vec3<double> home = rear.joints.home_position;
  ASSERT_TRUE(robots::dm1::clampStanceJointTarget(home));
  EXPECT_TRUE(home.isApprox(rear.joints.home_position, 1e-12));
  Vec3<double> unsafe(0.0, -1.23, 2.1);
  ASSERT_TRUE(robots::dm1::clampStanceJointTarget(unsafe));
  EXPECT_DOUBLE_EQ(unsafe[1], robots::dm1::kStanceThighLower);
  EXPECT_DOUBLE_EQ(unsafe[2], robots::dm1::kStanceCalfUpper);
  const double knee_z = 0.32 + rear.hip_location_body.z() +
    (Eigen::AngleAxisd(unsafe[1] + rear.joints.zero_offset[1],
      Vec3<double>::UnitY()) * rear.thigh_to_calf).z();
  EXPECT_GE(knee_z - robots::dm1::kKneeEnvelopeRadius,
    robots::dm1::kMinimumKneeClearance);
  Vec3<double> vertical_calf(0.0, 0.203, 1.338803755111750);
  ASSERT_TRUE(robots::dm1::clampStanceJointTarget(vertical_calf));
  EXPECT_LE(vertical_calf[1], robots::dm1::kStanceThighUpper);
  EXPECT_LE(vertical_calf[2], robots::dm1::kStanceCalfUpper);
  Vec3<double> invalid = rear.joints.home_position;
  invalid[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(robots::dm1::clampStanceJointTarget(invalid));

  // 0.32 m 工作窗口：机身俯仰 ±0.10 rad，横滚/HAA 各 ±0.03 rad。
  for (const auto & leg : robot.legs()) {
    for (const double pitch : {-0.10, 0.0, 0.10}) {
      for (const double roll : {-0.03, 0.0, 0.03}) {
        const Mat3<double> body = (Eigen::AngleAxisd(pitch, Vec3<double>::UnitY()) *
          Eigen::AngleAxisd(roll, Vec3<double>::UnitX())).toRotationMatrix();
        for (const double haa : {-0.03, 0.0, 0.03}) {
          const Mat3<double> hip = Eigen::AngleAxisd(haa, Vec3<double>::UnitX())
            .toRotationMatrix();
          for (const double thigh : {
              robots::dm1::kStanceThighLower, robots::dm1::kStanceThighUpper})
          {
            const Mat3<double> thigh_rotation =
              Eigen::AngleAxisd(thigh + leg.joints.zero_offset[1],
              Vec3<double>::UnitY()).toRotationMatrix();
            const Vec3<double> knee = body * (leg.hip_location_body + hip *
              (leg.hip_to_thigh + thigh_rotation * leg.thigh_to_calf));
            EXPECT_GE(0.32 + knee.z() - robots::dm1::kKneeEnvelopeRadius,
              robots::dm1::kMinimumKneeClearance - 1e-6);
            for (const double calf : {
                robots::dm1::kStanceCalfLower, robots::dm1::kStanceCalfUpper})
            {
              const Vec3<double> calf_vector = body * hip * thigh_rotation *
                Eigen::AngleAxisd(calf + leg.joints.zero_offset[2],
                Vec3<double>::UnitY()) * leg.calf_to_foot;
              EXPECT_LE(std::abs(calf_vector.z()) / leg.calf_to_foot.norm(),
                std::sin(75.0 * 3.141592653589793 / 180.0) + 1e-6);
            }
          }
        }
      }
    }
  }
}

TEST(Dm1Contract, StandingPosePreservesHeightAndEqualLinkInclinations)
{
  const auto robot = robots::dm1::makeModel<double>();
  for (const auto & leg : robot.legs()) {
    const Vec3<double> q = leg.joints.home_position;
    Vec3<double> foot;
    computeLegJacobianAndPosition(robot, q, static_cast<Mat3<double> *>(nullptr),
      &foot, leg.leg);
    const double height = leg.foot_radius - leg.hip_location_body.z() - foot.z();
    EXPECT_NEAR(height, 0.32, 2e-7);
    const double thigh_mechanical = q[1] + leg.joints.zero_offset[1];
    const Vec3<double> knee_from_hip = leg.hip_to_thigh +
      Eigen::AngleAxisd(thigh_mechanical, Vec3<double>::UnitY()) * leg.thigh_to_calf;
    EXPECT_GE(height + leg.hip_location_body.z() + knee_from_hip.z() -
      robots::dm1::kKneeEnvelopeRadius, 0.066);
    const Vec3<double> thigh = knee_from_hip - leg.hip_to_thigh;
    const Vec3<double> calf = foot - knee_from_hip;
    const double thigh_inclination = std::atan2(std::abs(thigh.z()), std::abs(thigh.x()));
    const double calf_inclination = std::atan2(std::abs(calf.z()), std::abs(calf.x()));
    EXPECT_NEAR(thigh_inclination, calf_inclination, 2e-6);
    EXPECT_NEAR(thigh_inclination, 0.5958042448882499, 2e-6);
    EXPECT_GT(q[1], -0.597);
  }
}

TEST(Dm1Contract, SharedOrientationGainPreservesHeightAndDynamicFeedback)
{
  test_support::FsmContext context(0.002F);
  for (auto & joint : context.joint_states) {joint.valid = true;}
  auto model = model::makeFloatingBaseModel(context.quadruped);
  auto state = model::makeFloatingBaseState(context.estimate, context.joint_states);
  state.q += model.getJointPositionOffsets();
  model.setState(state);
  BodyOriTask<float> orientation(model);
  orientation.setProportionalGain(Vec3<float>(69.0F, 69.0F, 34.0F));
  orientation.setDerivativeGain(Vec3<float>(11.8F, 11.8F, 5.8F));
  const Eigen::Quaternionf current(
    state.bodyOrientation[0], state.bodyOrientation[1],
    state.bodyOrientation[2], state.bodyOrientation[3]);
  for (int axis = 0; axis < 3; ++axis) {
    for (const float angle : {-0.08F, -0.02F, 0.02F, 0.08F}) {
      const Eigen::Quaternionf desired = current * Eigen::Quaternionf(
        Eigen::AngleAxisf(angle, Vec3<float>::Unit(axis)));
      ASSERT_TRUE(orientation.update(desired));
      Vec3<float> error = Vec3<float>::Zero();
      error[axis] = angle;
      EXPECT_TRUE(orientation.getPosError().isApprox(0.25F * error, 1.0e-5F));
      DVec<float> acceleration;
      orientation.getCommand(acceleration);
      EXPECT_TRUE(acceleration.isApprox(
          orientation.proportionalGain().cwiseProduct(error), 1.0e-5F));
    }
  }
  BodyPosTask<float> position(model);
  Vec3<float> desired_position = state.bodyPosition;
  desired_position.z() += 0.01F;
  ASSERT_TRUE(position.UpdateTask(
      &desired_position, DVec<float>::Zero(3), DVec<float>::Zero(3)));
  EXPECT_NEAR(position.getPosError()[2], 0.01F, 1.0e-6F);
}

TEST(Dm1Contract, EnablingWbcDiagnosticsDoesNotChangeControlCommands)
{
  test_support::FsmContext quiet(0.002F), logged(0.002F);
  for (auto * context : {&quiet, &logged}) {
    for (auto & joint : context->joint_states) {
      joint.valid = true;
      ASSERT_TRUE(context->leg_controller.updateData(joint));
    }
    context->estimate.position_world.z() = 0.32F;
    context->desired_state.body_position_world.z() = 0.32F;
    context->desired_state.mode = ControlMode::Locomotion;
    context->data().locomotion_gait = GaitType::STAND;
  }
  FSM_State_Locomotion<float> first(&quiet.data()), second(&logged.data());
  first.onEnter();
  second.onEnter();
  {DiagnosticEnvironment diagnostics("0"); first.run();}
  {DiagnosticEnvironment diagnostics("1"); second.run();}
  ASSERT_TRUE(quiet.leg_controller.legsEnabled());
  ASSERT_TRUE(logged.leg_controller.legsEnabled());
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & a = quiet.leg_controller.commands[leg];
    const auto & b = logged.leg_controller.commands[leg];
    EXPECT_TRUE(a.position_desired.isApprox(b.position_desired, 1.0e-6F));
    EXPECT_TRUE(a.velocity_desired.isApprox(b.velocity_desired, 1.0e-6F));
    EXPECT_TRUE(a.torque_feedforward.isApprox(b.torque_feedforward, 1.0e-6F));
    EXPECT_TRUE(a.kp_joint.isApprox(b.kp_joint));
    EXPECT_TRUE(a.kd_joint.isApprox(b.kd_joint));
  }
}

TEST(Dm1Contract, HasSingleModelAndCanonicalJointOrder)
{
  const auto model = makeQuadruped<float>(RobotType::DM1);
  const auto parameters = makeRobotControlParameters<float>(RobotType::DM1);
  EXPECT_EQ(model.robotType(), RobotType::DM1);
  EXPECT_FLOAT_EQ(model.nominalBodyHeight(), 0.32F);
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
    EXPECT_TRUE(
      model.leg(static_cast<LegId>(leg)).joints.home_position.isApprox(
      Vec3<float>(0.0F, -0.392804F, 0.959416F)));
    EXPECT_FLOAT_EQ(model.leg(static_cast<LegId>(leg)).joints.lower_limit[2], -0.03F);
    EXPECT_FLOAT_EQ(model.leg(static_cast<LegId>(leg)).joints.upper_limit[2], 2.72F);
  }
}

TEST(Dm1Contract, MotorFacingPdProfilesMatchTheDeployedContract)
{
  const auto parameters = makeRobotControlParameters<float>(RobotType::DM1);
  EXPECT_TRUE(
    parameters.initialization_kp.isApprox(
      Vec3<float>(99.9F, 99.9F, 99.9F)));
  EXPECT_TRUE(
    parameters.initialization_kd.isApprox(
      Vec3<float>(1.999F, 1.999F, 1.999F)));
  EXPECT_TRUE(
    parameters.prone_home_joint_kp.isApprox(
      Vec3<float>(70.0F, 70.0F, 70.0F)));
  EXPECT_TRUE(
    parameters.prone_home_joint_kd.isApprox(
      Vec3<float>(1.6F, 1.6F, 1.6F)));
  // 机身姿态环属于 BalanceStand 的 WBC 任务增益，与直接关节 PD 量纲不同。
  EXPECT_TRUE(
    parameters.balance_body_orientation_kp.isApprox(
      Vec3<float>(90.0F, 90.0F, 35.0F)));
  EXPECT_TRUE(
    parameters.balance_body_orientation_kd.isApprox(
      Vec3<float>(16.0F, 16.0F, 7.0F)));
  EXPECT_TRUE(
    parameters.balance_joint_kp.isApprox(
      Vec3<float>(90.0F, 85.0F, 100.0F)));
  EXPECT_TRUE(
    parameters.balance_joint_kd.isApprox(
      Vec3<float>(3.0F, 2.4F, 2.6F)));
  EXPECT_TRUE(
    parameters.stand_up_joint_kp.isApprox(
      Vec3<float>(80.0F, 80.0F, 90.0F)));
  EXPECT_TRUE(
    parameters.stand_up_joint_kd.isApprox(
      Vec3<float>(1.8F, 1.8F, 1.8F)));
  EXPECT_TRUE(
    parameters.locomotion_joint_kp.isApprox(
      Vec3<float>(55.0F, 25.0F, 90.0F)));
  EXPECT_TRUE(
    parameters.locomotion_joint_kd.isApprox(
      Vec3<float>(4.7F, 4.9F, 4.8F)));
}

// 所有会被写进 command.kp_joint/kd_joint、最终原样下发给 DM 电机的增益，
// 都必须落在 MIT 协议 0..500 / 0..5 范围内；越界会让 dm1_mit_interface
// 的 validateCommands() 直接 fail() 并 disableAll()，真机上等于整车失能。
TEST(Dm1Contract, MotorFacingJointGainsStayInsideMitProtocolRange)
{
  const auto parameters = makeRobotControlParameters<float>(RobotType::DM1);
  constexpr float kOperationalKpLimit = 100.0F;
  constexpr float kOperationalKdLimit = 4.95F;
  // 直接下发给电机的关节 PD：初始化/趴卧保持、BalanceStand、StandUp 收腿、
  // MPC/WBC 行走（LieDown 折叠复用）；RL 复用较温和的 initialization_kp/kd。
  const std::array<Vec3<float>, 5> kp_sets{
    parameters.initialization_kp,
    parameters.prone_home_joint_kp,
    parameters.balance_joint_kp,
    parameters.stand_up_joint_kp,
    parameters.locomotion_joint_kp};
  const std::array<Vec3<float>, 5> kd_sets{
    parameters.initialization_kd,
    parameters.prone_home_joint_kd,
    parameters.balance_joint_kd,
    parameters.stand_up_joint_kd,
    parameters.locomotion_joint_kd};
  for (const auto & set : kp_sets) {
    EXPECT_TRUE((set.array() >= 0.0F).all());
    EXPECT_TRUE((set.array() <= dm1_hardware::mit_protocol::kKpMax).all())
      << "Kp set exceeds the DM MIT protocol limit";
    EXPECT_LE(set.maxCoeff(), kOperationalKpLimit)
      << "Kp set exceeds the validated DM1 operating profile";
  }
  for (const auto & set : kd_sets) {
    EXPECT_TRUE((set.array() >= 0.0F).all());
    EXPECT_TRUE((set.array() <= dm1_hardware::mit_protocol::kKdMax).all())
      << "Kd set exceeds the DM MIT protocol limit";
    EXPECT_LE(set.maxCoeff(), kOperationalKdLimit)
      << "Kd set exceeds the validated DM1 operating profile";
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

TEST(Dm1Contract, UnsafeLocomotionLatchesBalanceStandWithoutRerunningController)
{
  const auto model = makeQuadruped<float>(RobotType::DM1);
  const auto parameters = makeRobotControlParameters<float>(RobotType::DM1);
  StateEstimate<float> estimate;
  estimate.valid = true;
  std::array<JointState<float>, kNumLegs> joints{};
  LegController<float> leg_controller(model);
  GaitScheduler<float> gait_scheduler(0.002F);
  DesiredState<float> desired;
  desired.valid = true;
  desired.mode = ControlMode::Locomotion;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    leg_controller.datas[leg].valid = true;
    leg_controller.datas[leg].p.z() = -0.30F;
  }
  // 足端越过髋部上方，模拟实机单腿折叠后触发的安全回站。
  leg_controller.datas[0].p.z() = 0.01F;
  leg_controller.commands[0].position_desired.setConstant(0.123F);

  ControlFSMData<float> data;
  data.quadruped = &model;
  data.control_parameters = &parameters;
  data.state_estimate = &estimate;
  data.joint_states = &joints;
  data.leg_controller = &leg_controller;
  data.gait_scheduler = &gait_scheduler;
  data.desired_state = &desired;
  data.control_time_step = 0.002F;
  FSM_State_Locomotion<float> locomotion(&data);
  locomotion.onEnter();

  EXPECT_EQ(locomotion.checkTransition(), FSM_StateName::BALANCE_STAND);
  EXPECT_EQ(desired.mode, ControlMode::BalanceStand);
  EXPECT_TRUE(locomotion.transition().done);
  EXPECT_TRUE(leg_controller.commands[0].position_desired.isConstant(0.123F));
}

TEST(Dm1Contract, BalanceStandUsesZeroVelocityStandingMpcAndRawWbcFeedforward)
{
  test_support::FsmContext context(0.002F);
  ASSERT_TRUE(context.initialized);
  context.parameters.start_in_prone_home = false;
  context.estimate.position_world.z() = 0.32F;
  context.estimate.velocity_world.x() = 0.08F;
  context.desired_state.body_position_world.z() = 0.32F;
  // 站立必须忽略遗留行走速度；实测漂移则由 MPC 反力制动。
  context.desired_state.body_velocity_world << 0.3F, 0.2F, 0.1F;
  context.desired_state.body_acceleration_world.setConstant(0.2F);
  context.desired_state.body_angular_velocity.setConstant(0.1F);
  for (auto & joint : context.joint_states) {
    joint.valid = true;
    ASSERT_TRUE(context.leg_controller.updateData(joint));
  }

  LocomotionCtrl<float> reference(model::makeFloatingBaseModel(context.quadruped));
  const auto & parameters = context.parameters;
  reference.setBodyPositionGains(
    parameters.balance_body_position_kp, parameters.balance_body_position_kd);
  reference.setBodyOrientationGains(
    parameters.balance_body_orientation_kp, parameters.balance_body_orientation_kd);
  reference.setFloatingBaseWeight(parameters.balance_floating_base_weight);
  reference.setReactionForceWeight(parameters.balance_reaction_force_weight);
  reference.setMaxNormalForce(parameters.maximum_normal_force);
  mpc::SolverSettings<float> settings;
  settings.time_step = 0.05F;
  mpc::ConvexMPCLocomotion<float> planner(context.quadruped, 0.002F, 20, settings, 25);
  planner.initialize();
  planner.setGait(GaitType::STAND);
  planner.setVelocityCommand(0.0F, 0.0F, 0.0F);
  auto desired = context.desired_state;
  desired.body_velocity_world.setZero();
  desired.body_acceleration_world.setZero();
  desired.body_angular_velocity.setZero();
  std::array<Vec3<float>, kNumLegs> feet_world{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    feet_world[leg] = context.estimate.position_world +
      context.estimate.rotation_world_from_body *
      (context.quadruped.hipLocation(static_cast<LegId>(leg)) +
      context.leg_controller.datas[leg].p);
  }
  FSM_State_BalanceStand<float> standing(&context.data());
  standing.onEnter();
  std::size_t updates = 0;
  for (int frame = 0; frame < 50; ++frame) {
    const auto result = planner.run(context.estimate, desired, feet_world);
    ASSERT_TRUE(result.valid);
    if (result.mpc_updated) {++updates;}
    EXPECT_TRUE(result.command.body_velocity_world.isZero());
    EXPECT_TRUE(result.command.body_angular_velocity.isZero());
    LocomotionCtrlData<float> input;
    input.pBody_des = result.command.body_position_world;
    input.pBody_RPY_des = result.command.body_rpy;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      ASSERT_TRUE(result.contact_state[leg]);
      EXPECT_FLOAT_EQ(result.swing_time[leg], 0.0F);
      input.contact_state[leg] = 1.0F;
      input.Fr_des[leg] = result.reaction_forces_world[leg];
    }
    if (frame == 0) {
      EXPECT_GT(std::abs(input.Fr_des[0].x()), 0.01F);
    }
    ASSERT_TRUE(reference.run(&input, context.estimate, context.joint_states));
    const auto expected = reference.result().joint_torque;
    standing.run();
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      ASSERT_TRUE(context.leg_controller.command(static_cast<LegId>(leg)).enabled);
      EXPECT_TRUE(context.leg_controller.commands[leg].torque_feedforward.isApprox(
          expected.segment<3>(static_cast<Eigen::Index>(3 * leg)), 1.0e-5F));
      EXPECT_TRUE(context.leg_controller.commands[leg].kp_joint.isApprox(
          parameters.balance_joint_kp));
      EXPECT_TRUE(context.leg_controller.commands[leg].kd_joint.isApprox(
          parameters.balance_joint_kd));
    }
  }
  EXPECT_EQ(updates, 3U);
}

TEST(Dm1Contract, LocomotionFirstCommandIgnoresDisabledPreviousCommands)
{
  const auto model = makeQuadruped<float>(RobotType::DM1);
  const auto parameters = makeRobotControlParameters<float>(RobotType::DM1);
  const auto first_command = [&](float previous_value) {
      StateEstimate<float> estimate;
      estimate.valid = true;
      estimate.position_world.z() = 0.325F;
      std::array<JointState<float>, kNumLegs> joints{};
      LegController<float> controller(model);
      GaitScheduler<float> gait(0.002F);
      DesiredState<float> desired;
      desired.valid = true;
      desired.mode = ControlMode::Locomotion;
      desired.body_position_world.z() = 0.32F;
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        joints[leg].leg = static_cast<LegId>(leg);
        joints[leg].valid = true;
        joints[leg].position = model.leg(joints[leg].leg).joints.home_position;
        EXPECT_TRUE(controller.updateData(joints[leg]));
        auto & command = controller.commands[leg];
        command.position_desired.setConstant(previous_value);
        command.velocity_desired.setConstant(previous_value);
        command.torque_feedforward.setConstant(previous_value);
        command.kp_joint.setConstant(previous_value);
        command.kd_joint.setConstant(previous_value);
      }
      ControlFSMData<float> data;
      data.quadruped = &model;
      data.control_parameters = &parameters;
      data.state_estimate = &estimate;
      data.joint_states = &joints;
      data.leg_controller = &controller;
      data.gait_scheduler = &gait;
      data.desired_state = &desired;
      data.control_time_step = 0.002F;
      FSM_State_Locomotion<float> locomotion(&data);
      locomotion.onEnter();
      locomotion.run();
      EXPECT_TRUE(controller.command(LegId::FR).enabled);
      return controller.commands;
    };
  const auto first = first_command(0.1F);
  const auto second = first_command(2.0F);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_TRUE(first[leg].position_desired.isApprox(second[leg].position_desired));
    EXPECT_TRUE(first[leg].velocity_desired.isApprox(second[leg].velocity_desired));
    EXPECT_TRUE(first[leg].torque_feedforward.isApprox(second[leg].torque_feedforward));
    EXPECT_TRUE(first[leg].kp_joint.isApprox(second[leg].kp_joint));
    EXPECT_TRUE(first[leg].kd_joint.isApprox(second[leg].kd_joint));
    EXPECT_GT(first[leg].torque_feedforward.norm(), 1.0F);
  }
}

TEST(Dm1Contract, ClassicEntryPreservesTotalTorqueAndReleasesStandingCommand)
{
  for (const auto gait : {GaitType::STAND, GaitType::TROT, GaitType::TROT_WALK}) {
    test_support::FsmContext actual(0.002F), reference(0.002F);
    for (auto * context : {&actual, &reference}) {
      context->parameters.start_in_prone_home = false;
      context->estimate.position_world.z() = 0.32F;
      context->estimate.rpy.y() = -0.035F;
      context->estimate.orientation_world_from_body =
        Eigen::AngleAxisf(-0.035F, Vec3<float>::UnitY());
      context->estimate.rotation_world_from_body =
        context->estimate.orientation_world_from_body.toRotationMatrix();
      context->desired_state.body_position_world.z() = 0.32F;
      for (auto & joint : context->joint_states) {
        joint.valid = true;
        joint.position[1] -= 0.015F;
        joint.position[2] -= 0.025F;
        joint.velocity << 0.01F, -0.02F, 0.03F;
        ASSERT_TRUE(context->leg_controller.updateData(joint));
      }
      FSM_State_BalanceStand<float> standing(&context->data());
      standing.onEnter();
      standing.run();
      ASSERT_TRUE(context->leg_controller.legsEnabled());
      context->desired_state.mode = ControlMode::Locomotion;
      context->data().locomotion_gait = gait;
    }
    std::array<JointCommand<float>, kNumLegs> entry;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      entry[leg] = actual.leg_controller.command(static_cast<LegId>(leg));
      ASSERT_TRUE(entry[leg].enabled);
    }
    reference.leg_controller.setEnabled(false);
    reference.data().locomotion_gait = GaitType::STAND;
    FSM_State_Locomotion<float> transitioning(&actual.data()), raw(&reference.data());
    transitioning.onEnter();
    raw.onEnter();
    const auto torque = [](const JointCommand<float> & command,
        const JointState<float> & feedback) -> Vec3<float> {
        return command.torque_feedforward +
               command.kp.cwiseProduct(command.position_desired - feedback.position) +
               command.kd.cwiseProduct(command.velocity_desired - feedback.velocity);
      };
    for (int frame = 0; frame <= 130; ++frame) {
      if (frame == 101) {reference.data().locomotion_gait = gait;}
      actual.gait_scheduler.step();
      reference.gait_scheduler.step();
      transitioning.run();
      raw.run();
      const float progress = std::min(frame * 0.002F / 0.2F, 1.0F);
      const float alpha = progress * progress * (3.0F - 2.0F * progress);
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        const auto command = actual.leg_controller.command(static_cast<LegId>(leg));
        const auto target = reference.leg_controller.command(static_cast<LegId>(leg));
        ASSERT_TRUE(command.enabled);
        ASSERT_TRUE(target.enabled);
        const auto & feedback = actual.joint_states[leg];
        const Vec3<float> expected = (1.0F - alpha) * torque(entry[leg], feedback) +
          alpha * torque(target, feedback);
        EXPECT_LT((torque(command, feedback) - expected).cwiseAbs().maxCoeff(), 2.0e-4F);
        if (frame == 0) {
          EXPECT_TRUE(command.position_desired.isApprox(entry[leg].position_desired));
          EXPECT_TRUE(command.kp.isApprox(entry[leg].kp));
          EXPECT_TRUE(command.kd.isApprox(entry[leg].kd));
          EXPECT_TRUE(command.torque_feedforward.isApprox(entry[leg].torque_feedforward));
        }
        if (frame == 130) {
          EXPECT_TRUE(command.position_desired.isApprox(target.position_desired));
          EXPECT_TRUE(command.kp.isApprox(target.kp));
          EXPECT_TRUE(command.kd.isApprox(target.kd));
          EXPECT_TRUE(command.torque_feedforward.isApprox(target.torque_feedforward));
        }
      }
    }
  }
}

TEST(Dm1Contract, CommonLegCommandOutputLimitsTotalTorqueToContinuousRating)
{
  const auto model = makeQuadruped<float>(RobotType::DM1);
  LegController<float> controller(model);
  JointState<float> feedback;
  feedback.leg = LegId::FR;
  feedback.valid = true;
  ASSERT_TRUE(controller.updateData(feedback));
  controller.setEnabled(true);
  controller.commands[0].position_desired.setConstant(1.0F);
  controller.commands[0].velocity_desired.setConstant(2.0F);
  controller.commands[0].torque_feedforward.setConstant(10.0F);
  controller.commands[0].kp_joint.setConstant(100.0F);
  controller.commands[0].kd_joint.setConstant(5.0F);

  const auto command = controller.command(LegId::FR);
  ASSERT_TRUE(command.enabled);
  const Vec3<float> total_torque = command.torque_feedforward +
    command.kp.cwiseProduct(command.position_desired - feedback.position) +
    command.kd.cwiseProduct(command.velocity_desired - feedback.velocity);
  EXPECT_LE(total_torque.cwiseAbs().maxCoeff(), 60.0F + 1.0e-4F);
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
  constexpr std::array<std::size_t, kNumLegs> kInterleavedLegOrder{0, 2, 1, 3};
  for (std::size_t index = 0; index < transport.frames.size(); ++index) {
    const std::size_t joint = index / kNumLegs;
    const std::size_t leg = kInterleavedLegOrder[index % kNumLegs];
    const std::size_t calibration_index = leg * kJointsPerLeg + joint;
    const auto bus = static_cast<std::uint8_t>(calibration_index / 6);
    const auto can_id = static_cast<std::uint16_t>(calibration_index % 6 + 1);
    EXPECT_EQ(transport.frames[index].bus, bus);
    EXPECT_EQ(transport.frames[index].can_id, can_id);
    EXPECT_EQ(transport.frames[index].master_id, can_id + 0x10);
  }

  // 力矩超限不再丢弃整帧：只把该关节的 kp/kd/前馈按比例缩小，其余 11 路保持
  // 原样，整批 12 帧照常发送。丢帧会让电机收不到新帧而自行超时失能。
  transport.frames.clear();
  commands[0].torque_feedforward[0] = 31.0F;
  EXPECT_TRUE(hardware.send(commands, 1.0));
  EXPECT_EQ(transport.frames.size(), kNumJoints);
  EXPECT_FALSE(transport.disabled);

  transport.frames.clear();
  commands[0].torque_feedforward[0] = 40.0F;
  EXPECT_TRUE(hardware.send(commands, 1.0));
  EXPECT_EQ(transport.frames.size(), kNumJoints);
  EXPECT_FALSE(transport.disabled);
}
