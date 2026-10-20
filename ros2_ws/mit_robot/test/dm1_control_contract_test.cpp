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

#include "controller/leg_controller.hpp"
#include "FSM/FSM_State_BalanceStand.h"
#include "FSM/FSM_State_Locomotion.h"
#include "hardware/dm1_mit_interface.hpp"
#include "MPC/SolverMPC.h"
#include "model/quadruped.hpp"
#include "model/robot_control_parameters.hpp"
#include "model/robots/dm1.hpp"
#include "fsm/fsm_test_support.hpp"

namespace
{
class DiagnosticEnvironment
{
public:
  explicit DiagnosticEnvironment(const char * value, const char * name = "DM1_CONTROL_DIAGNOSTICS")
  : name_(name)
  {
    if (const char * old = std::getenv(name_)) {old_ = old;}
    setenv(name_, value, 1);
  }
  ~DiagnosticEnvironment()
  {
    if (old_) {setenv(name_, old_->c_str(), 1);}
    else {unsetenv(name_);}
  }
private:
  const char * name_;
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
  context.estimate.position_world.z() = 0.28F;
  context.estimate.rpy.y() = -0.0583F;
  context.estimate.orientation_world_from_body = Eigen::AngleAxisf(-0.0583F, Vec3<float>::UnitY());
  context.estimate.rotation_world_from_body = context.estimate.orientation_world_from_body.toRotationMatrix();
  context.desired_state.body_position_world.z() = 0.28F;
  FSM_State_BalanceStand<float> standing(&context.data());
  standing.onEnter();
  ::testing::internal::CaptureStderr();
  standing.run();
  const std::string log = ::testing::internal::GetCapturedStderr();
  ASSERT_TRUE(context.leg_controller.legsEnabled()) << log;
  ASSERT_NE(log.find("[WBC][DIAGNOSTIC] t_s=12.25"), std::string::npos);
  EXPECT_NE(log.find("floating_weight=100 contacts=4"), std::string::npos);
  const auto raw = diagnosticVector(log, "q_wbc");
  const auto orientation_error = diagnosticVector(log, "task0_error");
  ASSERT_EQ(raw.size(), kNumJoints);
  ASSERT_EQ(orientation_error.size(), 3U);
  EXPECT_NEAR(orientation_error[1], 0.0583F, 1.0e-6F);
  EXPECT_LT(raw[7], context.joint_states[2].position[1] - 0.01F);
  // MIT BalanceStand 直接使用 KinWBC 给出的关节参考，不再用零空间姿态覆盖。
  EXPECT_FLOAT_EQ(
    context.leg_controller.commands[2].position_desired[1], raw[7]);
  context.desired_state.mode = ControlMode::Locomotion;
  context.data().locomotion_gait = GaitType::STAND;
  FSM_State_Locomotion<float> locomotion(&context.data());
  locomotion.onEnter();
  ::testing::internal::CaptureStderr();
  locomotion.run();
  const std::string locomotion_log = ::testing::internal::GetCapturedStderr();
  const auto walking_raw = diagnosticVector(locomotion_log, "q_wbc");
  EXPECT_TRUE(walking_raw.empty() || walking_raw.size() == kNumJoints);
  EXPECT_TRUE(context.leg_controller.command(LegId::RR).enabled || walking_raw.empty());
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
  const double knee_z = 0.28 + rear.hip_location_body.z() +
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

  // 0.28 m 工作窗口：机身俯仰 ±0.10 rad，横滚/HAA 各 ±0.03 rad。
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
            EXPECT_GE(0.28 + knee.z() - robots::dm1::kKneeEnvelopeRadius,
              -0.020 - 1e-6);
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
    EXPECT_NEAR(height, 0.28, 2e-7);
    const double thigh_mechanical = q[1] + leg.joints.zero_offset[1];
    const Vec3<double> knee_from_hip = leg.hip_to_thigh +
      Eigen::AngleAxisd(thigh_mechanical, Vec3<double>::UnitY()) * leg.thigh_to_calf;
    EXPECT_GE(height + leg.hip_location_body.z() + knee_from_hip.z() -
      robots::dm1::kKneeEnvelopeRadius, 0.044);
    const Vec3<double> thigh = knee_from_hip - leg.hip_to_thigh;
    const Vec3<double> calf = foot - knee_from_hip;
    const double thigh_inclination = std::atan2(std::abs(thigh.z()), std::abs(thigh.x()));
    const double calf_inclination = std::atan2(std::abs(calf.z()), std::abs(calf.x()));
    EXPECT_NEAR(thigh_inclination, calf_inclination, 2e-6);
    EXPECT_NEAR(thigh_inclination, 0.4859610000000000, 2e-6);
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
      EXPECT_TRUE(orientation.getPosError().isApprox(error, 1.0e-5F));
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
    context->estimate.position_world.z() = 0.28F;
    context->desired_state.body_position_world.z() = 0.28F;
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

TEST(Dm1Contract, WbcTraceObservesRepeatedTimeFailuresAndRecoveryWithoutChangingCommands)
{
  DiagnosticEnvironment full_trace("1", "DM1_CONTROL_DIAGNOSTICS_FULL");
  test_support::FsmContext context(0.002F);
  LocomotionCtrl<float> quiet(model::makeFloatingBaseModel(context.quadruped));
  LocomotionCtrl<float> logged(model::makeFloatingBaseModel(context.quadruped));
  LegController<float> quiet_legs(context.quadruped), logged_legs(context.quadruped);
  LocomotionCtrlData<float> input;
  input.pBody_des << 0.0F, 0.0F, 0.28F;
  input.contact_state.setOnes();
  context.estimate.position_world = input.pBody_des;
  for (auto & joint : context.joint_states) {joint.valid = true;}
  for (auto & force : input.Fr_des) {
    force << 0.0F, 0.0F, context.quadruped.totalMass() * 9.81F / 4.0F;
  }
  const std::array<float, 12> times{
    1.0F, 1.002F, 1.002F, 0.5F, 0.502F, 0.504F,
    0.506F, 0.508F, 0.510F, 0.512F, 0.514F, 0.516F};
  for (std::size_t frame = 0; frame < times.size(); ++frame) {
    SCOPED_TRACE(frame);
    context.estimate.timestamp = times[frame];
    for (auto & joint : context.joint_states) {joint.timestamp = times[frame];}
    context.joint_states[0].position.x() = frame == 4 || frame == 5 ?
      std::numeric_limits<float>::quiet_NaN() : 0.001F;
    input.contact_state.setOnes();
    if (frame == 7) {input.contact_state << 1.0F, 0.0F, 0.0F, 1.0F;}
    bool quiet_success, logged_success;
    ::testing::internal::CaptureStderr();
    {
      DiagnosticEnvironment diagnostics("0");
      quiet_success = quiet.runAndApply(&input, context.estimate, context.joint_states, quiet_legs);
    }
    (void)::testing::internal::GetCapturedStderr();
    ::testing::internal::CaptureStderr();
    {
      DiagnosticEnvironment diagnostics("1");
      logged_success = logged.runAndApply(&input, context.estimate, context.joint_states, logged_legs);
    }
    const auto log = ::testing::internal::GetCapturedStderr();
    ASSERT_EQ(logged_success, quiet_success);
    ASSERT_EQ(logged_success, frame != 4 && frame != 5);
    EXPECT_EQ(logged_legs.legsEnabled(), quiet_legs.legsEnabled());
    ASSERT_NE(log.find("[WBC][CONTINUITY]"), std::string::npos) << log;
    const bool comparable = frame > 0 && frame != 2 && frame != 3 &&
      frame != 4 && frame != 5 && frame != 6;
    EXPECT_NE(log.find(comparable ? "comparable=1" : "comparable=0"), std::string::npos);
    if (frame == 7 || frame == 8) {
      EXPECT_NE(log.find("contact_changed=1"), std::string::npos);
      EXPECT_EQ(log.find("force_step_norm="), std::string::npos);
    }
    if (logged_success) {
      const auto residual = diagnosticVector(log, "dynamics_residual");
      ASSERT_EQ(residual.size(), 6U + kNumJoints);
      for (const float value : residual) {EXPECT_NEAR(value, 0.0F, 2.0e-4F);}
      EXPECT_NE(log.find("[WBC][DIAGNOSTIC]"), std::string::npos) << log;
      EXPECT_EQ(diagnosticVector(log, "task0_vel_des").size(), 3U);
      EXPECT_EQ(diagnosticVector(log, "task1_acc_des").size(), 3U);
      EXPECT_EQ(diagnosticVector(log, "task1_command").size(), 3U);
      EXPECT_EQ(diagnosticVector(log, "qddot_wbc").size(), 18U);
      EXPECT_EQ(diagnosticVector(log, "gravity").size(), 18U);
      EXPECT_EQ(diagnosticVector(log, "coriolis").size(), 18U);
      const auto force_des = diagnosticVector(log, "contact0_force_des");
      ASSERT_EQ(force_des.size(), 3U);
      EXPECT_FLOAT_EQ(force_des[2], input.Fr_des[0].z());
      EXPECT_EQ(diagnosticVector(log, "reaction_force").size(), frame == 7 ? 6U : 12U);
      if (frame == 7) {
        EXPECT_EQ(diagnosticVector(log, "task2_command").size(), 3U);
        EXPECT_EQ(diagnosticVector(log, "task3_command").size(), 3U);
      }
    } else {
      EXPECT_EQ(log.find("[WBC][DIAGNOSTIC]"), std::string::npos);
    }
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      const auto & a = quiet_legs.commands[leg];
      const auto & b = logged_legs.commands[leg];
      EXPECT_TRUE(a.position_desired.isApprox(b.position_desired, 1.0e-7F));
      EXPECT_TRUE(a.velocity_desired.isApprox(b.velocity_desired, 1.0e-7F));
      EXPECT_TRUE(a.torque_feedforward.isApprox(b.torque_feedforward, 1.0e-7F));
      EXPECT_TRUE(a.kp_joint.isApprox(b.kp_joint));
      EXPECT_TRUE(a.kd_joint.isApprox(b.kd_joint));
    }
  }
}

TEST(Dm1Contract, MpcTracePreservesForceTrajectoryAndReportsTimeStep)
{
  const auto quadruped = robots::dm1::makeModel<double>();
  mpc::SolverSettings<double> settings;
  mpc::SolverMPC<double> solver(quadruped, settings);
  StateEstimate<double> estimate;
  estimate.valid = true;
  estimate.position_world << 0.0, 0.0, 0.27;
  estimate.timestamp = 2.5;
  const std::array<Vec3<double>, kNumLegs> feet{
    Vec3<double>(0.2, -0.13, 0.0), Vec3<double>(0.2, 0.13, 0.0),
    Vec3<double>(-0.2, -0.13, 0.0), Vec3<double>(-0.2, 0.13, 0.0)};
  const auto state = mpc::RobotState<double>::fromEstimate(estimate, feet);
  DesiredState<double> desired;
  desired.valid = true;
  desired.body_position_world << 0.0, 0.0, 0.27;
  const std::vector<DesiredState<double>> trajectory(settings.horizon, desired);
  for (int mode = 0; mode < 4; ++mode) {
    SCOPED_TRACE(mode);
    std::vector<int> contacts(settings.horizon * kNumLegs, mode == 3 ? 0 : 1);
    if (mode == 1 || mode == 2) {
      // 两种对角支撑先摆动再落地，验证完整预测窗而非仅第一步。
      for (std::size_t step = 0; step < settings.horizon / 2; ++step) {
        contacts[step * kNumLegs + (mode == 1 ? 1 : 0)] = 0;
        contacts[step * kNumLegs + (mode == 1 ? 2 : 3)] = 0;
      }
    }
    mpc::SolverResult<double> quiet, logged;
    {DiagnosticEnvironment diagnostics("0", "DM1_MPC_DIAGNOSTICS"); quiet = solver.solve(state, trajectory, contacts);}
    ::testing::internal::CaptureStderr();
    {DiagnosticEnvironment diagnostics("1", "DM1_MPC_DIAGNOSTICS"); logged = solver.solve(state, trajectory, contacts);}
    const auto log = ::testing::internal::GetCapturedStderr();
    ASSERT_TRUE(quiet.valid);
    ASSERT_TRUE(logged.valid);
    EXPECT_EQ(quiet.force_trajectory, logged.force_trajectory);
    EXPECT_EQ(quiet.cost, logged.cost);
    EXPECT_NE(log.find("t_s=2.5"), std::string::npos) << log;
    const auto dt_field = log.find("dt_mpc_s=");
    ASSERT_NE(dt_field, std::string::npos);
    EXPECT_DOUBLE_EQ(std::stod(log.substr(dt_field + 9)), settings.time_step);
    if (mode != 3) {EXPECT_NE(log.find("min_constraint_margin="), std::string::npos);}
    const auto contact_trace = diagnosticVector(log, "contact_table");
    ASSERT_EQ(contact_trace.size(), contacts.size());
    for (std::size_t i = 0; i < contacts.size(); ++i) {EXPECT_EQ(contact_trace[i], contacts[i]);}
    const auto forces = diagnosticVector(log, "force_trajectory");
    ASSERT_EQ(forces.size(), logged.force_trajectory.size());
    for (std::size_t i = 0; i < forces.size(); ++i) {
      EXPECT_FLOAT_EQ(forces[i], static_cast<float>(logged.force_trajectory[i]));
      if (!contacts[i / 3]) {EXPECT_FLOAT_EQ(forces[i], 0.0F);}
    }
    const auto initial = diagnosticVector(log, "x0");
    ASSERT_EQ(initial.size(), 13U);
    EXPECT_FLOAT_EQ(initial[5], 0.27F);
    EXPECT_FLOAT_EQ(initial[12], -9.81F);
    const auto reference = diagnosticVector(log, "x_ref");
    ASSERT_EQ(reference.size(), settings.horizon * 13);
    for (std::size_t step = 0; step < settings.horizon; ++step) {
      EXPECT_FLOAT_EQ(reference[step * 13 + 5], 0.27F);
      EXPECT_FLOAT_EQ(reference[step * 13 + 12], 0.0F);
    }
    EXPECT_EQ(diagnosticVector(log, "foot_world").size(), 12U);
    EXPECT_EQ(diagnosticVector(log, "state_weights").size(), 12U);
    EXPECT_EQ(diagnosticVector(log, "inertia_body_row_major").size(), 9U);
    EXPECT_EQ(diagnosticVector(log, "force_min").size(), 4U);
    EXPECT_EQ(diagnosticVector(log, "force_max").size(), 4U);
  }
}

TEST(Dm1Contract, MpcComOffsetDefaultsToZeroAndRejectsInvalidCalibration)
{
  const auto robot = robots::dm1::makeModel<double>();
  mpc::SolverSettings<double> settings;
  EXPECT_TRUE(settings.com_offset_body.isZero());
  EXPECT_TRUE(settings.isValid());
  settings.com_offset_body.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(settings.isValid());
  EXPECT_THROW((mpc::SolverMPC<double>(robot, settings)), std::invalid_argument);
  settings.com_offset_body << -0.051, 0.0, 0.0;
  EXPECT_FALSE(settings.isValid());
  EXPECT_THROW((mpc::SolverMPC<double>(robot, settings)), std::invalid_argument);
}

TEST(Dm1Contract, MpcRearwardComOffsetIncreasesRearSupportLoad)
{
  const auto robot = robots::dm1::makeModel<double>();
  mpc::SolverSettings<double> settings;
  StateEstimate<double> estimate;
  estimate.valid = true;
  estimate.position_world << 0.0, 0.0, 0.28;
  const std::array<Vec3<double>, kNumLegs> feet{
    Vec3<double>(0.2, -0.13, 0.0), Vec3<double>(0.2, 0.13, 0.0),
    Vec3<double>(-0.2, -0.13, 0.0), Vec3<double>(-0.2, 0.13, 0.0)};
  const auto state = mpc::RobotState<double>::fromEstimate(estimate, feet);
  DesiredState<double> desired;
  desired.valid = true;
  desired.body_position_world = estimate.position_world;
  const std::vector<DesiredState<double>> trajectory(settings.horizon, desired);
  const std::vector<int> contacts(settings.horizon * kNumLegs, 1);
  const auto baseline = mpc::SolverMPC<double>(robot, settings).solve(
    state, trajectory, contacts);
  settings.com_offset_body.x() = -0.010;
  const auto corrected = mpc::SolverMPC<double>(robot, settings).solve(
    state, trajectory, contacts);
  ASSERT_TRUE(baseline.valid);
  ASSERT_TRUE(corrected.valid);
  const auto & before = baseline.reaction_forces_world;
  const auto & after = corrected.reaction_forces_world;
  EXPECT_GT(after[2].z() + after[3].z(), before[2].z() + before[3].z() + 0.2);
  EXPECT_LT(after[0].z() + after[1].z(), before[0].z() + before[1].z() - 0.2);
  EXPECT_NEAR(after[0].z(), after[1].z(), 0.1);
  EXPECT_NEAR(after[2].z(), after[3].z(), 0.1);
  double total_before = 0.0;
  double total_after = 0.0;
  double weighted_support_x = 0.0;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    total_before += before[leg].z();
    total_after += after[leg].z();
    weighted_support_x += feet[leg].x() * after[leg].z();
  }
  // 首段预测力不是静态平衡解；校准应主要改变分配，而非总竖直力。
  EXPECT_NEAR(total_after, total_before, 0.1);
  EXPECT_NEAR(weighted_support_x / total_after, -0.010, 0.003);
}

TEST(Dm1Contract, MpcComOffsetMatchesExplicitComStateAtDifferentHeadings)
{
  const auto robot = robots::dm1::makeModel<double>();
  mpc::SolverSettings<double> settings;
  settings.com_offset_body.x() = -0.010;
  const auto zero_settings = mpc::SolverSettings<double>{};
  // 参考求解器显式接收计算重心状态；同时覆盖不同朝向、倾斜和角速度。
  for (const double yaw : {0.0, 1.5707963267948966}) {
    SCOPED_TRACE(yaw);
    StateEstimate<double> estimate;
    estimate.valid = true;
    estimate.position_world << 0.4, -0.3, 0.28;
    estimate.rpy << 0.08, -0.06, yaw;
    estimate.orientation_world_from_body =
      Eigen::AngleAxisd(yaw, Vec3<double>::UnitZ()) *
      Eigen::AngleAxisd(estimate.rpy.y(), Vec3<double>::UnitY()) *
      Eigen::AngleAxisd(estimate.rpy.x(), Vec3<double>::UnitX());
    estimate.rotation_world_from_body = estimate.orientation_world_from_body.toRotationMatrix();
    estimate.velocity_world << 0.08, -0.03, 0.01;
    estimate.angular_velocity_body << 0.12, -0.09, 0.21;
    const std::array<Vec3<double>, kNumLegs> feet{
      Vec3<double>(0.6, -0.43, 0.0), Vec3<double>(0.6, -0.17, 0.0),
      Vec3<double>(0.2, -0.43, 0.0), Vec3<double>(0.2, -0.17, 0.0)};
    const auto state = mpc::RobotState<double>::fromEstimate(estimate, feet);
    auto com_state = state;
    const Vec3<double> offset_world = estimate.rotation_world_from_body * settings.com_offset_body;
    com_state.position_world += offset_world;
    com_state.velocity_world += state.angular_velocity_world.cross(offset_world);
    DesiredState<double> desired;
    desired.valid = true;
    desired.body_position_world = estimate.position_world + Vec3<double>(0.01, -0.02, 0.0);
    desired.body_rpy = estimate.rpy + Vec3<double>(0.02, -0.015, 0.03);
    desired.body_velocity_world << 0.06, -0.04, 0.01;
    desired.body_angular_velocity << 0.11, -0.08, 0.19;
    std::vector<DesiredState<double>> trajectory(settings.horizon, desired);
    auto com_trajectory = trajectory;
    for (std::size_t step = 0; step < trajectory.size(); ++step) {
      trajectory[step].body_rpy.z() += 0.003 * step;
      trajectory[step].body_position_world.x() += 0.002 * step;
      com_trajectory[step] = trajectory[step];
      const auto & rpy = trajectory[step].body_rpy;
      const Mat3<double> rotation = (Eigen::AngleAxisd(rpy.z(), Vec3<double>::UnitZ()) *
        Eigen::AngleAxisd(rpy.y(), Vec3<double>::UnitY()) *
        Eigen::AngleAxisd(rpy.x(), Vec3<double>::UnitX())).toRotationMatrix();
      const Vec3<double> reference_offset = rotation * settings.com_offset_body;
      const Vec3<double> omega_world = rotation * trajectory[step].body_angular_velocity;
      com_trajectory[step].body_position_world += reference_offset;
      com_trajectory[step].body_velocity_world += omega_world.cross(reference_offset);
    }
    // 四脚支撑和按键 3/4 的两种对角支撑，摆动脚必须保持零规划反力。
    for (const int mask : {15, 9, 6}) {
      SCOPED_TRACE(mask);
      std::vector<int> contacts(settings.horizon * kNumLegs);
      for (std::size_t step = 0; step < settings.horizon; ++step) {
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
          contacts[step * kNumLegs + leg] = (mask >> leg) & 1;
        }
      }
      const auto actual = mpc::SolverMPC<double>(robot, settings).solve(state, trajectory, contacts);
      const auto expected = mpc::SolverMPC<double>(robot, zero_settings).solve(
        com_state, com_trajectory, contacts);
      ASSERT_TRUE(actual.valid);
      ASSERT_TRUE(expected.valid);
      ASSERT_EQ(actual.force_trajectory.size(), expected.force_trajectory.size());
      for (std::size_t i = 0; i < actual.force_trajectory.size(); ++i) {
        EXPECT_NEAR(actual.force_trajectory[i], expected.force_trajectory[i], 1.0e-7);
        const auto leg = (i % 12) / 3;
        if (!((mask >> leg) & 1)) {EXPECT_DOUBLE_EQ(actual.force_trajectory[i], 0.0);}
      }
    }
  }
}

TEST(Dm1Contract, HasSingleModelAndCanonicalJointOrder)
{
  const auto model = makeQuadruped<float>(RobotType::DM1);
  EXPECT_EQ(model.robotType(), RobotType::DM1);
  EXPECT_FLOAT_EQ(model.nominalBodyHeight(), 0.28F);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_EQ(static_cast<std::size_t>(model.leg(static_cast<LegId>(leg)).leg), leg);
    EXPECT_TRUE(
      model.leg(static_cast<LegId>(leg)).joints.home_position.isApprox(
      Vec3<float>(0.0F, -0.282961F, 0.739729F)));
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
      Vec3<float>(200.0F, 200.0F, 100.0F)));
  EXPECT_TRUE(
    parameters.balance_body_orientation_kd.isApprox(
      Vec3<float>(10.0F, 10.0F, 10.0F)));
  // BalanceStand 下发给电机的关节 PD 使用当前 DM1 站立参数。
  EXPECT_TRUE(
    parameters.balance_joint_kp.isApprox(
      Vec3<float>(8.0F, 8.0F, 15.0F)));
  EXPECT_TRUE(
    parameters.balance_joint_kd.isApprox(
      Vec3<float>(1.8F, 1.8F, 2.0F)));
  EXPECT_TRUE(
    parameters.joint_kp.isApprox(
      Vec3<float>(10.0F, 10.0F, 10.0F)));
  EXPECT_TRUE(
    parameters.joint_kd.isApprox(
      Vec3<float>(2.0F, 2.0F, 2.0F)));
  EXPECT_TRUE(
    parameters.stand_up_joint_kp.isApprox(
      Vec3<float>(80.0F, 80.0F, 90.0F)));
  EXPECT_TRUE(
    parameters.stand_up_joint_kd.isApprox(
      Vec3<float>(1.8F, 1.8F, 1.8F)));
  EXPECT_TRUE(
    parameters.joint_kp.isApprox(
      Vec3<float>(10.0F, 10.0F, 10.0F)));
  EXPECT_TRUE(
    parameters.joint_kd.isApprox(
      Vec3<float>(2.0F, 2.0F, 2.0F)));
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
  // MPC/WBC 行走（LieDown 折叠复用）。
  const std::array<Vec3<float>, 6> kp_sets{
    parameters.initialization_kp,
    parameters.prone_home_joint_kp,
    parameters.joint_kp,
    parameters.balance_joint_kp,
    parameters.stand_up_joint_kp,
    parameters.joint_kp};
  const std::array<Vec3<float>, 6> kd_sets{
    parameters.initialization_kd,
    parameters.prone_home_joint_kd,
    parameters.joint_kd,
    parameters.balance_joint_kd,
    parameters.stand_up_joint_kd,
    parameters.joint_kd};
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
  EXPECT_FALSE(data.stand_handoff.pending);
}

TEST(Dm1Contract, NormalLocomotionExitPreservesHeightTargetsAndGains)
{
  DiagnosticEnvironment full("1", "DM1_CONTROL_DIAGNOSTICS_FULL");
  test_support::FsmContext context(0.002F);
  for (auto & joint : context.joint_states) {
    joint.valid = true;
    ASSERT_TRUE(context.leg_controller.updateData(joint));
  }
  context.estimate.position_world << 0.02F, -0.01F, 0.254F;
  context.desired_state.body_position_world.z() = 0.28F;
  context.desired_state.mode = ControlMode::Locomotion;
  FSM_State_Locomotion<float> locomotion(&context.data());
  locomotion.onEnter();
  context.desired_state.mode = ControlMode::BalanceStand;
  ASSERT_EQ(locomotion.checkTransition(), FSM_StateName::BALANCE_STAND);
  ASSERT_TRUE(locomotion.transition().done);
  auto & handoff = context.data().stand_handoff;
  ASSERT_TRUE(handoff.pending);
  EXPECT_FLOAT_EQ(handoff.body.body_position_world.z(), 0.28F);
  const auto entry = handoff.commands;
  locomotion.onExit();
  FSM_State_BalanceStand<float> standing(&context.data());
  standing.onEnter();
  EXPECT_FALSE(handoff.pending);
  ::testing::internal::CaptureStderr();
  standing.run();
  const auto log = ::testing::internal::GetCapturedStderr();
  ASSERT_TRUE(context.leg_controller.legsEnabled()) << log;
  const auto p = diagnosticVector(log, "body_p");
  ASSERT_EQ(p.size(), 3U);
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(p[axis], handoff.body.body_position_world[axis], 1.e-7F);
  }
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & command = context.leg_controller.commands[leg];
    EXPECT_TRUE(command.position_desired.isApprox(entry[leg].position_desired));
    EXPECT_TRUE(command.velocity_desired.isApprox(entry[leg].velocity_desired));
    EXPECT_TRUE(command.kp_joint.isApprox(entry[leg].kp_joint));
    EXPECT_TRUE(command.kd_joint.isApprox(entry[leg].kd_joint));
    EXPECT_TRUE(command.kd_cartesian.isApprox(entry[leg].kd_cartesian));
  }
  for (int i = 1; i <= 50; ++i) {
    context.estimate.timestamp += 0.002F;
    standing.run();
  }
  EXPECT_FALSE(handoff.active);
  for (const auto & command : context.leg_controller.commands) {
    EXPECT_TRUE(command.kp_joint.isApprox(context.parameters.balance_joint_kp));
    EXPECT_TRUE(command.kd_joint.isApprox(context.parameters.balance_joint_kd));
    EXPECT_TRUE(command.kd_cartesian.isZero());
  }
  standing.onExit();
  standing.onEnter();
  EXPECT_FALSE(handoff.active);
}

TEST(Dm1Contract, StandHandoffPreservesMovingBodyReferenceAndStopsSmoothly)
{
  DiagnosticEnvironment full("1", "DM1_CONTROL_DIAGNOSTICS_FULL");
  DiagnosticEnvironment diagnostics("1");
  test_support::FsmContext context(0.002F);
  for (auto & joint : context.joint_states) {
    joint.valid = true;
    ASSERT_TRUE(context.leg_controller.updateData(joint));
  }
  context.estimate.position_world.z() = 0.254F;
  context.desired_state.body_position_world.z() = 0.28F;
  auto & h = context.data().stand_handoff;
  h.pending = true;
  h.body.body_position_world << 0.01F, -0.02F, 0.28F;
  h.body.body_velocity_world << 0.08F, -0.04F, 0.F;
  h.body.body_acceleration_world << 0.02F, -0.01F, 0.F;
  h.body.body_rpy << 0.01F, -0.02F, 0.3F;
  h.body.body_angular_velocity << 0.01F, -0.02F, 0.1F;
  // A new height command waits for the reference handoff, then resumes its rate limit.
  context.desired_state.body_position_world.z() = 0.30F;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    h.commands[leg].position_desired = context.joint_states[leg].position;
    h.commands[leg].velocity_desired.setConstant(0.1F);
  }
  FSM_State_BalanceStand<float> standing(&context.data());
  standing.onEnter();
  std::array<Vec3<float>, 52> positions{}, velocities{}, accelerations{};
  for (int i = 0; i <= 51; ++i) {
    context.estimate.timestamp = i * 0.002F;
    ::testing::internal::CaptureStderr();
    standing.run();
    const auto log = ::testing::internal::GetCapturedStderr();
    ASSERT_TRUE(context.leg_controller.legsEnabled()) << log;
    const auto p = diagnosticVector(log, "body_p");
    const auto v = diagnosticVector(log, "body_v");
    const auto a = diagnosticVector(log, "body_a");
    const auto rpy = diagnosticVector(log, "body_rpy");
    const auto omega = diagnosticVector(log, "body_omega");
    ASSERT_EQ(p.size(), 3U);
    ASSERT_EQ(v.size(), 3U);
    ASSERT_EQ(a.size(), 3U);
    ASSERT_EQ(rpy.size(), 3U);
    ASSERT_EQ(omega.size(), 3U);
    for (int axis = 0; axis < 3; ++axis) {
      positions[i][axis] = p[axis];
      velocities[i][axis] = v[axis];
      accelerations[i][axis] = a[axis];
    }
    if (i == 0 || i >= 50) {
      for (int axis = 0; axis < 3; ++axis) {
        EXPECT_NEAR(v[axis], i == 0 ? h.body.body_velocity_world[axis] : 0.F, 1.e-6F);
        EXPECT_NEAR(a[axis], i == 0 ? h.body.body_acceleration_world[axis] : 0.F, 1.e-5F);
        EXPECT_NEAR(omega[axis], i == 0 ? h.body.body_angular_velocity[axis] : 0.F, 1.e-6F);
        if (i == 0) {
          EXPECT_NEAR(p[axis], h.body.body_position_world[axis], 1.e-7F);
          EXPECT_NEAR(rpy[axis], h.body.body_rpy[axis], 1.e-7F);
        }
      }
    }
    if (i >= 50) {
      EXPECT_NEAR(p[0], 0.014F, 1.e-7F);
      EXPECT_NEAR(p[1], -0.022F, 1.e-7F);
      EXPECT_NEAR(p[2], 0.28F + (i == 51 ? context.parameters.balance_height_step : 0.F), 1.e-7F);
      EXPECT_NEAR(rpy[0], 0.F, 1.e-7F);
      EXPECT_NEAR(rpy[1], 0.F, 1.e-7F);
    }
    const auto raw_tau = diagnosticVector(log, "ff_wbc");
    const auto raw_q = diagnosticVector(log, "q_wbc");
    if (!raw_tau.empty()) {
      ASSERT_EQ(raw_tau.size(), kNumJoints);
      ASSERT_EQ(raw_q.size(), kNumJoints);
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        for (int joint = 0; joint < 3; ++joint) {
          EXPECT_NEAR(context.leg_controller.commands[leg].torque_feedforward[joint],
            raw_tau[leg*3+joint], 1.e-5F);
          if (i >= 50) {
            EXPECT_NEAR(context.leg_controller.commands[leg].position_desired[joint],
              raw_q[leg*3+joint], 1.e-6F);
          }
        }
      }
    }
  }
  EXPECT_TRUE(((positions[26]-positions[24])/0.004F).isApprox(velocities[25], 0.002F));
  EXPECT_TRUE(((velocities[26]-velocities[24])/0.004F).isApprox(accelerations[25], 0.002F));
}

TEST(Dm1Contract, StandWithoutHandoffRetainsMeasuredHeightRamp)
{
  DiagnosticEnvironment full("1", "DM1_CONTROL_DIAGNOSTICS_FULL");
  test_support::FsmContext context(0.002F);
  for (auto & joint : context.joint_states) {
    joint.valid = true;
    ASSERT_TRUE(context.leg_controller.updateData(joint));
  }
  context.estimate.position_world.z() = 0.12F;
  context.desired_state.body_position_world.z() = 0.28F;
  context.parameters.start_in_prone_home = true;
  FSM_State_BalanceStand<float> standing(&context.data());
  standing.onEnter();
  ::testing::internal::CaptureStderr();
  standing.run();
  const auto p = diagnosticVector(::testing::internal::GetCapturedStderr(), "body_p");
  ASSERT_EQ(p.size(), 3U);
  EXPECT_NEAR(p[2], 0.12F + context.parameters.balance_height_step, 1.e-7F);
}

TEST(Dm1Contract, StandHandoffCannotMaskWbcFailure)
{
  test_support::FsmContext context(0.002F);
  for (auto & joint : context.joint_states) {
    joint.valid = true;
    ASSERT_TRUE(context.leg_controller.updateData(joint));
  }
  context.data().stand_handoff.pending = true;
  context.data().stand_handoff.body.body_position_world.z() = 0.28F;
  context.desired_state.body_position_world.z() = 0.28F;
  context.joint_states[0].valid = false;
  FSM_State_BalanceStand<float> standing(&context.data());
  standing.onEnter();
  standing.run();
  EXPECT_FALSE(context.leg_controller.legsEnabled());
  for (const auto & command : context.leg_controller.commands) {
    EXPECT_TRUE(command.kp_joint.isZero());
    EXPECT_TRUE(command.kd_joint.isZero());
  }
}

TEST(Dm1Contract, BalanceStandUsesMitWbcOnlyAndEqualGravitySupport)
{
  test_support::FsmContext context(0.002F);
  context.parameters.start_in_prone_home = false;
  context.estimate.position_world.z() = 0.28F;
  context.desired_state.body_position_world.z() = 0.28F;
  for (auto & joint : context.joint_states) {
    joint.valid = true;
    ASSERT_TRUE(context.leg_controller.updateData(joint));
  }
  FSM_State_BalanceStand<float> standing(&context.data());
  standing.onEnter();
  standing.run();
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & command = context.leg_controller.commands[leg];
    EXPECT_TRUE(command.torque_feedforward.allFinite());
    EXPECT_TRUE(command.kp_joint.isApprox(context.parameters.balance_joint_kp));
    EXPECT_TRUE(command.kd_joint.isApprox(context.parameters.balance_joint_kd));
  }
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
      desired.body_position_world.z() = 0.28F;
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
  for (const auto gait : {GaitType::STAND, GaitType::TROT}) {
    test_support::FsmContext actual(0.002F), reference(0.002F);
    for (auto * context : {&actual, &reference}) {
      context->parameters.start_in_prone_home = false;
      context->estimate.position_world.z() = 0.28F;
      context->estimate.rpy.y() = -0.035F;
      context->estimate.orientation_world_from_body =
        Eigen::AngleAxisf(-0.035F, Vec3<float>::UnitY());
      context->estimate.rotation_world_from_body =
        context->estimate.orientation_world_from_body.toRotationMatrix();
      context->desired_state.body_position_world.z() = 0.28F;
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
    reference.data().locomotion_gait = gait;
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
      actual.gait_scheduler.step();
      reference.gait_scheduler.step();
      transitioning.run();
      raw.run();
      const float alpha = 1.0F;
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        const auto command = actual.leg_controller.command(static_cast<LegId>(leg));
        const auto target = reference.leg_controller.command(static_cast<LegId>(leg));
        ASSERT_TRUE(target.enabled);
        const auto & feedback = actual.joint_states[leg];
        const Vec3<float> expected = (1.0F - alpha) * torque(entry[leg], feedback) +
          alpha * torque(target, feedback);
        EXPECT_LT((torque(command, feedback) - expected).cwiseAbs().maxCoeff(), 2.0e-4F);
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
  const Vec3<float> total_torque = command.torque_feedforward +
    command.kp.cwiseProduct(command.position_desired - feedback.position) +
    command.kd.cwiseProduct(command.velocity_desired - feedback.velocity);
  EXPECT_LE(total_torque.cwiseAbs().maxCoeff(), 60.0F + 1.0e-4F);
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
