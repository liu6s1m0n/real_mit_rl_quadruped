/*! @file opendog_reference_test.cpp
 *  @brief OpenDogReference 目标轨迹和 OpenDogGaitCommand 步态插入测试。
 */
#include <pinocchio/fwd.hpp>

#include <cmath>
#include <string>

#include <gtest/gtest.h>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "control/opendog_reference.hpp"
#include "legged_interface/LeggedInterface.h"

namespace
{
std::string config(const char * file)
{
  return std::string(DM1_OPENDOG_TEST_CONFIG) + "/" + file;
}

ocs2::SystemObservation observation()
{
  ocs2::SystemObservation result;
  result.time = 2.0;
  result.state = ocs2::vector_t::Zero(24);
  result.state.segment<6>(6) << 1.0, 2.0, 0.28, 0.4, 0.0, 0.0;
  result.input = ocs2::vector_t::Zero(24);
  return result;
}
}  // namespace

// 运动目标轨迹的字段顺序与原 OpenDog 一致。
TEST(OpenDogReference, KeepsOriginalVelocityTrajectoryFieldOrder)
{
  double horizon = 0.0;
  ocs2::loadData::loadCppDataType(config("opendog_task.info"), "mpc.timeHorizon", horizon);
  OpenDogReference reference(config("opendog_reference.info"), config("opendog_task.info"));
  auto state = observation();
  DesiredState<float> desired;
  desired.valid = true;
  desired.mode = ControlMode::Locomotion;
  desired.body_position_world.z() = 0.28F;
  desired.body_velocity_world << 0.2F, -0.1F, 0.0F;
  desired.body_angular_velocity.z() = 0.3F;
  auto target = reference.update(desired, state);
  ASSERT_EQ(target.stateTrajectory.size(), 2U);
  EXPECT_DOUBLE_EQ(target.timeTrajectory.back(), state.time + horizon);
  EXPECT_NEAR(target.stateTrajectory.back()(6), 1.0 + 0.2 * horizon, 1e-7);
  EXPECT_NEAR(target.stateTrajectory.back()(7), 2.0 - 0.1 * horizon, 1e-7);
  // 高度前馈：目标高度 −= heightVelocityGain × 机体系前进命令速度（偏航 0.4 rad）
  boost::property_tree::ptree reference_settings;
  boost::property_tree::read_info(config("opendog_reference.info"), reference_settings);
  const double height_gain = reference_settings.get<double>("heightVelocityGain", 0.0);
  const double forward = std::cos(0.4) * 0.2 + std::sin(0.4) * -0.1;
  EXPECT_NEAR(target.stateTrajectory.back()(8), 0.28 - height_gain * forward, 1e-6);
  EXPECT_NEAR(target.stateTrajectory.back()(9), 0.4 + 0.3 * horizon, 1e-7);
  EXPECT_NEAR(target.stateTrajectory.back()(0), 0.2, 1e-7);
  EXPECT_NEAR(target.stateTrajectory.back()(1), -0.1, 1e-7);
  EXPECT_DOUBLE_EQ(target.stateTrajectory.back()(3), 0.0);
  for (int leg = 0; leg < 4; ++leg) {
    EXPECT_DOUBLE_EQ(target.stateTrajectory.back()(12 + 3 * leg), 0.0);
    EXPECT_NEAR(target.stateTrajectory.back()(13 + 3 * leg), -0.602113, 1e-9);
    EXPECT_NEAR(target.stateTrajectory.back()(14 + 3 * leg), -1.259860, 1e-9);
  }
}

// 运动模式下偏航目标按期望角速度积分锁定，不随当前偏航重置；领先量限幅 0.3 rad。
TEST(OpenDogReference, LocomotionYawAnchorIntegratesCommandedRate)
{
  double horizon = 0.0;
  ocs2::loadData::loadCppDataType(config("opendog_task.info"), "mpc.timeHorizon", horizon);
  OpenDogReference reference(config("opendog_reference.info"), config("opendog_task.info"));
  auto state = observation();
  DesiredState<float> desired;
  desired.valid = true;
  desired.mode = ControlMode::Locomotion;
  desired.body_position_world.z() = 0.28F;
  desired.body_angular_velocity.z() = 0.3F;
  reference.update(desired, state);
  // 机身没转（偏航仍为 0.4），0.5 s 后锚点应前进 0.15 rad。
  state.time += 0.5;
  auto target = reference.update(desired, state);
  EXPECT_NEAR(target.stateTrajectory.back()(9), 0.4 + 0.3 * 0.5 + 0.3 * horizon, 1e-6);
  // 再过 2 s 锚点被限幅在当前偏航 + 0.3 rad。
  state.time += 2.0;
  target = reference.update(desired, state);
  EXPECT_NEAR(target.stateTrajectory.back()(9), 0.4 + 0.3 + 0.3 * horizon, 1e-6);
  // 离开运动模式后锚点清除，重新进入时从当前偏航开始。
  desired.mode = ControlMode::BalanceStand;
  reference.update(desired, state);
  desired.mode = ControlMode::Locomotion;
  target = reference.update(desired, state);
  EXPECT_NEAR(target.stateTrajectory.back()(9), 0.4 + 0.3 * horizon, 1e-6);
}

// 速度外环积分：有平移命令且质心速度跟不上时，修正量在机体系累积并叠加到速度参考，受上限约束；
// 停止平移命令后衰减，离开运动模式清零。
TEST(OpenDogReference, LocomotionVelocityIntegralCompensatesPersistentLag)
{
  boost::property_tree::ptree settings;
  boost::property_tree::read_info(config("opendog_reference.info"), settings);
  const double gain = settings.get<double>("velocityIntegral.gain", 0.0);
  const double limit = settings.get<double>("velocityIntegral.limit", 0.06);
  OpenDogReference reference(config("opendog_reference.info"), config("opendog_task.info"));
  auto state = observation();
  state.state(9) = 0.0;  // 偏航 0，机体系与世界系一致
  DesiredState<float> desired;
  desired.valid = true;
  desired.mode = ControlMode::Locomotion;
  desired.body_position_world.z() = 0.28F;
  desired.body_velocity_world << 0.1F, 0.0F, 0.0F;
  reference.update(desired, state);
  state.time += 0.2;
  auto target = reference.update(desired, state);
  const double expected = std::min(gain * 0.1 * 0.2, limit);
  EXPECT_NEAR(reference.velocityCorrection().x(), expected, 1e-9);
  EXPECT_NEAR(reference.velocityCorrection().y(), 0.0, 1e-9);
  EXPECT_NEAR(target.stateTrajectory.back()(0), 0.1 + expected, 1e-6);
  // 长时间跟不上时修正量被限幅
  for (int i = 0; i < 100; ++i) {
    state.time += 0.1;
    reference.update(desired, state);
  }
  EXPECT_NEAR(reference.velocityCorrection().x(), gain > 0.0 ? limit : 0.0, 1e-9);
  // 停止平移命令后衰减
  desired.body_velocity_world.setZero();
  state.time += 2.0;
  reference.update(desired, state);
  EXPECT_LT(reference.velocityCorrection().x(), 0.1 * limit + 1e-12);
  // 离开运动模式清零
  desired.mode = ControlMode::BalanceStand;
  reference.update(desired, state);
  EXPECT_DOUBLE_EQ(reference.velocityCorrection().norm(), 0.0);
}

// 高度积分：机身低于站立高度时，修正量逐步增大并叠加到高度目标；踏步时俯仰目标随前进速度前馈。
TEST(OpenDogReference, HeightIntegralAndPitchFeedforward)
{
  boost::property_tree::ptree settings;
  boost::property_tree::read_info(config("opendog_reference.info"), settings);
  const double gain = settings.get<double>("heightIntegral.gain", 0.0);
  const double limit = settings.get<double>("heightIntegral.limit", 0.03);
  const double pitch_gain = settings.get<double>("pitchVelocityGain", 0.0);
  const double height_gain = settings.get<double>("heightVelocityGain", 0.0);
  double com_height = 0.0;
  ocs2::loadData::loadCppDataType(config("opendog_reference.info"), "comHeight", com_height);
  OpenDogReference reference(config("opendog_reference.info"), config("opendog_task.info"));
  auto state = observation();
  state.state(8) = com_height - 0.02;   // 机身比目标低 2 cm
  state.state(9) = 0.0;
  DesiredState<float> desired;
  desired.valid = true;
  desired.mode = ControlMode::Locomotion;
  desired.body_position_world.z() = static_cast<float>(com_height);
  desired.body_velocity_world << 0.2F, 0.0F, 0.0F;
  reference.update(desired, state);
  state.time += 0.1;
  auto target = reference.update(desired, state);
  const double expected = std::min(gain * 0.02 * 0.1, limit);
  EXPECT_NEAR(reference.heightCorrection(), expected, 1e-9);
  EXPECT_NEAR(target.stateTrajectory.back()(8), com_height + expected - height_gain * 0.2, 1e-6);
  EXPECT_NEAR(target.stateTrajectory.back()(10), -pitch_gain * 0.2, 1e-6);
  for (int i = 0; i < 200; ++i) {
    state.time += 0.1;
    reference.update(desired, state);
  }
  EXPECT_NEAR(reference.heightCorrection(), gain > 0.0 ? limit : 0.0, 1e-9);
  reference.reset();
  EXPECT_DOUBLE_EQ(reference.heightCorrection(), 0.0);
}

// 站立期间轨迹保持首次生成时的起始时间，不随每帧刷新。
TEST(OpenDogReference, StandUpTrajectoryRetainsItsInitialTime)
{
  OpenDogReference reference(config("opendog_reference.info"), config("opendog_task.info"));
  auto state = observation();
  state.state(8) = 0.12;
  DesiredState<float> desired;
  desired.valid = true;
  desired.mode = ControlMode::StandUp;
  auto target = reference.update(desired, state);
  EXPECT_DOUBLE_EQ(target.timeTrajectory.back(), 5.2);
  EXPECT_DOUBLE_EQ(target.stateTrajectory.front()(8), 0.12);
  double com_height = 0.0;
  ocs2::loadData::loadCppDataType(config("opendog_reference.info"), "comHeight", com_height);
  EXPECT_DOUBLE_EQ(target.stateTrajectory.back()(8), com_height);
  state.time = 2.5;
  EXPECT_DOUBLE_EQ(reference.update(desired, state).timeTrajectory.back(), 5.2);
}

// 支撑相只约束足端速度为 0，不把足端高度拉到 0。
TEST(OpenDogReference, StanceConstrainsVelocityWithoutPullingFeetToZeroHeight)
{
  legged::LeggedInterface interface(
    config("opendog_task.info"), config("dm1_opendog.urdf"), config("opendog_reference.info"));
  interface.setupOptimalControlProblem(
    config("opendog_task.info"), config("dm1_opendog.urdf"), config("opendog_reference.info"), false);
  auto manager = interface.getSwitchedModelReferenceManagerPtr();
  auto state = interface.getInitialState();
  state.head<6>().setZero();
  const auto input = ocs2::vector_t::Zero(interface.getCentroidalModelInfo().inputDim).eval();
  manager->preSolverRun(0.0, 1.0, state);
  const auto & problem = interface.getOptimalControlProblem();
  for (double height : {0.26, 0.28, 0.30}) {
    state(8) = height;
    for (const auto & foot : interface.modelSettings().contactNames3DoF) {
      auto & constraint = problem.equalityConstraintPtr->get(foot + "_zeroVelocity");
      ASSERT_TRUE(constraint.isActive(0.0));
      EXPECT_NEAR(constraint.getValue(0.0, state, input, *problem.preComputationPtr).norm(),
        0.0, 1e-9) << foot << " height=" << height;
    }
  }
}

// 两组对角摆动的离地/触地高度都使用估计器的足端半径。
TEST(OpenDogReference, BothDiagonalSwingsUseEstimatorFootCenterHeight)
{
  legged::LeggedInterface interface(
    config("opendog_task.info"), config("dm1_opendog.urdf"), config("opendog_reference.info"));
  interface.setupOptimalControlProblem(
    config("opendog_task.info"), config("dm1_opendog.urdf"), config("opendog_reference.info"), false);
  double radius = 0.0;
  double clearance = 0.0;
  ocs2::loadData::loadCppDataType(config("opendog_task.info"), "kalmanFilter.footRadius", radius);
  ocs2::loadData::loadCppDataType(config("opendog_task.info"), "swing_trajectory_config.swingHeight", clearance);
  // 摆腿最高点时刻比例（缺省 0.5 为原 OCS2 对称样条）
  boost::property_tree::ptree swing;
  boost::property_tree::read_info(config("opendog_task.info"), swing);
  const double apex_phase = swing.get<double>("swing_trajectory_config.swingApexPhase", 0.5);
  // 摆腿落地目标相对地面的偏移（缺省 0）
  const double touch_down_offset = swing.get<double>("swing_trajectory_config.touchDownOffset", 0.0);
  const double touch_down_offset_rear = swing.get<double>("swing_trajectory_config.touchDownOffsetRear", 0.0);
  auto manager = interface.getSwitchedModelReferenceManagerPtr();
  const ocs2::ModeSchedule modes({0.1, 0.55, 1.1, 1.55}, {15, 6, 15, 9, 15});
  manager->setModeSchedule(modes);
  manager->preSolverRun(0.0, 2.0, interface.getInitialState());
  auto planner = manager->getSwingTrajectoryPlanner();
  for (double start : {0.1, 1.1}) {
    const double middle = start + 0.225;
    const double apex = start + apex_phase * 0.45;
    EXPECT_EQ(manager->getModeSchedule().modeAtTime(middle), modes.modeAtTime(middle));
    const auto contacts = manager->getContactFlags(middle);
    for (std::size_t leg = 0; leg < contacts.size(); ++leg) {
      EXPECT_NEAR(planner->getZpositionConstraint(leg, start + 1e-8), radius, 1e-8);
      EXPECT_NEAR(planner->getZpositionConstraint(leg, start + 0.45 - 1e-8),
        radius + (contacts[leg] ? 0.0 : touch_down_offset + (leg >= 2 ? touch_down_offset_rear : 0.0)), 1e-8);
      EXPECT_NEAR(planner->getZpositionConstraint(leg, apex),
        radius + (contacts[leg] ? 0.0 : clearance), 1e-9);
      EXPECT_NEAR(planner->getZvelocityConstraint(leg, apex), 0.0, 1e-9);
      if (!contacts[leg]) {
        EXPECT_GT(planner->getZvelocityConstraint(leg, start + 1e-8), 0.0);
      }
    }
  }
}

// 落地高度偏移只改变摆腿落地目标，不改变抬脚高度。
TEST(OpenDogReference, TouchDownOffsetsShiftOnlyTouchDownHeight)
{
  legged::LeggedInterface interface(
    config("opendog_task.info"), config("dm1_opendog.urdf"), config("opendog_reference.info"));
  interface.setupOptimalControlProblem(
    config("opendog_task.info"), config("dm1_opendog.urdf"), config("opendog_reference.info"), false);
  boost::property_tree::ptree swing;
  boost::property_tree::read_info(config("opendog_task.info"), swing);
  const double offset = swing.get<double>("swing_trajectory_config.touchDownOffset", 0.0);
  const double offset_rear = swing.get<double>("swing_trajectory_config.touchDownOffsetRear", 0.0);
  auto manager = interface.getSwitchedModelReferenceManagerPtr();
  const ocs2::ModeSchedule modes({0.1, 0.55, 1.1, 1.55}, {15, 6, 15, 9, 15});
  manager->setModeSchedule(modes);
  manager->setFootTerrainHeights({{0.02, 0.02, 0.02, 0.02}});
  manager->setFootTouchDownOffsets({{0.008, -0.008, 0.004, -0.004}});
  manager->preSolverRun(0.0, 2.0, interface.getInitialState());
  auto planner = manager->getSwingTrajectoryPlanner();
  // 模式 6（RF_LH 支撑）时 LF、RH 摆动，模式 9（LF_RH 支撑）时 RF、LH 摆动
  const std::array<double, 4> extra{{0.008, -0.008, 0.004, -0.004}};
  for (double start : {0.1, 1.1}) {
    const auto contacts = manager->getContactFlags(start + 0.2);
    for (std::size_t leg = 0; leg < contacts.size(); ++leg) {
      if (contacts[leg]) {continue;}
      EXPECT_NEAR(planner->getZpositionConstraint(leg, start + 1e-8), 0.02, 1e-8);
      EXPECT_NEAR(planner->getZpositionConstraint(leg, start + 0.45 - 1e-8),
        0.02 + offset + (leg >= 2 ? offset_rear : 0.0) + extra[leg], 1e-8);
    }
  }
}

// trot 和 stance 请求插入原始步态模板。
TEST(OpenDogGaitCommand, InsertsOriginalTrotAndStanceTemplates)
{
  auto schedule = std::make_shared<ocs2::legged_robot::GaitSchedule>(
    ocs2::ModeSchedule({0.5}, {15, 15}),
    ocs2::legged_robot::loadModeSequenceTemplate(config("opendog_gait.info"), "stance", false), 0.1);
  OpenDogGaitCommand command(config("opendog_gait.info"), schedule);
  ocs2::ReferenceManager reference;
  command.setMode(ControlMode::Locomotion);
  command.preSolverRun(0.0, 1.0, ocs2::vector_t(), reference);
  auto modes = schedule->getModeSchedule(0.0, 2.0);
  EXPECT_EQ(modes.modeAtTime(1.15), 9U);
  EXPECT_EQ(modes.modeAtTime(1.45), 6U);
  command.setMode(ControlMode::BalanceStand);
  command.preSolverRun(1.0, 2.0, ocs2::vector_t(), reference);
  EXPECT_EQ(schedule->getModeSchedule(1.0, 3.0).modeAtTime(2.5), 15U);
}

// 对角单步不会重复执行，按键再次按下可再次触发。
TEST(OpenDogGaitCommand, SingleDiagonalStepsDoNotRepeatAndCanBeRequestedAgain)
{
  auto schedule = std::make_shared<ocs2::legged_robot::GaitSchedule>(
    ocs2::ModeSchedule({0.5}, {15, 15}),
    ocs2::legged_robot::loadModeSequenceTemplate(config("opendog_gait.info"), "stance", false), 0.1);
  OpenDogGaitCommand command(config("opendog_gait.info"), schedule);
  ocs2::ReferenceManager reference;
  command.setMode(ControlMode::Locomotion);
  command.setMotion(teleop::Motion::TwoLegStand, 1);
  command.preSolverRun(0.0, 1.0, {}, reference);
  auto modes = schedule->getModeSchedule(0.0, 3.0);
  EXPECT_EQ(modes.modeAtTime(1.05), 15U);
  EXPECT_EQ(modes.modeAtTime(1.25), 6U);  // RF/LH 支撑，LF/RH 摆动。
  EXPECT_EQ(modes.modeAtTime(1.45), 6U);  // 配置的摆动时长为 0.45 s。
  EXPECT_EQ(modes.modeAtTime(1.60), 15U);
  for (int i = 1; i <= 10; ++i) {
    command.preSolverRun(i, i + 1.0, {}, reference);
    EXPECT_EQ(schedule->getModeSchedule(i, i + 2.0).modeAtTime(i + 1.25), 15U);
  }
  command.setMotion(teleop::Motion::TwoLegStand, 2);
  command.preSolverRun(11.0, 12.0, {}, reference);
  EXPECT_EQ(schedule->getModeSchedule(11.0, 14.0).modeAtTime(12.25), 6U);
  command.setMotion(teleop::Motion::TwoLegStandReverse, 3);
  command.preSolverRun(12.0, 13.0, {}, reference);
  modes = schedule->getModeSchedule(12.0, 15.0);
  EXPECT_EQ(modes.modeAtTime(13.25), 9U);
  EXPECT_EQ(modes.modeAtTime(14.25), 15U);
  command.setMotion(teleop::Motion::MarchInPlace, 4);
  command.preSolverRun(14.0, 15.0, {}, reference);
  EXPECT_EQ(schedule->getModeSchedule(14.0, 17.0).modeAtTime(15.15), 9U);
  command.setMode(ControlMode::BalanceStand);
  command.preSolverRun(15.0, 16.0, {}, reference);
  EXPECT_EQ(schedule->getModeSchedule(15.0, 18.0).modeAtTime(17.25), 15U);
  command.reset();
  EXPECT_EQ(schedule->getModeSchedule(0.0, 2.0).modeAtTime(1.25), 15U);
  command.setMode(ControlMode::Locomotion);
  command.setMotion(teleop::Motion::TwoLegStandReverse, 3);
  command.preSolverRun(0.0, 1.0, {}, reference);
  EXPECT_EQ(schedule->getModeSchedule(0.0, 3.0).modeAtTime(1.25), 9U);
}

// 对角单步追加在当前预测窗口末尾之后。
TEST(OpenDogGaitCommand, SingleDiagonalStepIsAppendedAfterHorizon)
{
  auto schedule = std::make_shared<ocs2::legged_robot::GaitSchedule>(
    ocs2::ModeSchedule({0.5}, {15, 15}),
    ocs2::legged_robot::loadModeSequenceTemplate(config("opendog_gait.info"), "stance", false), 0.1);
  OpenDogGaitCommand command(config("opendog_gait.info"), schedule);
  ocs2::ReferenceManager reference;
  command.setMode(ControlMode::Locomotion);
  command.setMotion(teleop::Motion::TwoLegStand, 1);
  command.preSolverRun(1.0, 2.0, {}, reference);
  EXPECT_EQ(schedule->getModeSchedule(1.0, 4.0).modeAtTime(2.25), 6U);
}
