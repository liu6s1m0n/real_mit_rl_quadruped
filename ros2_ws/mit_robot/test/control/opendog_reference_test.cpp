#include <string>

#include <gtest/gtest.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "control/opendog_reference.hpp"

namespace
{
std::string config(const char * file)
{
  return std::string(MYMIT_OPENDOG_TEST_CONFIG) + "/" + file;
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

TEST(OpenDogReference, KeepsOriginalVelocityTrajectoryFieldOrder)
{
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
  EXPECT_DOUBLE_EQ(target.timeTrajectory.back(), 3.0);
  EXPECT_NEAR(target.stateTrajectory.back()(6), 1.2, 1e-7);
  EXPECT_NEAR(target.stateTrajectory.back()(7), 1.9, 1e-7);
  EXPECT_NEAR(target.stateTrajectory.back()(8), 0.28, 1e-7);
  EXPECT_NEAR(target.stateTrajectory.back()(9), 0.7, 1e-7);
  EXPECT_NEAR(target.stateTrajectory.back()(0), 0.2, 1e-7);
  EXPECT_NEAR(target.stateTrajectory.back()(1), -0.1, 1e-7);
  EXPECT_DOUBLE_EQ(target.stateTrajectory.back()(3), 0.0);
  for (int leg = 0; leg < 4; ++leg) {
    EXPECT_DOUBLE_EQ(target.stateTrajectory.back()(12 + 3 * leg), 0.0);
    EXPECT_NEAR(target.stateTrajectory.back()(13 + 3 * leg), -0.282961, 1e-9);
    EXPECT_NEAR(target.stateTrajectory.back()(14 + 3 * leg), 0.739729, 1e-9);
  }
}

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
  EXPECT_DOUBLE_EQ(target.stateTrajectory.back()(8), 0.28);
  state.time = 2.5;
  EXPECT_DOUBLE_EQ(reference.update(desired, state).timeTrajectory.back(), 5.2);
}

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
  EXPECT_EQ(modes.modeAtTime(1.25), 6U);  // RF/LH support, LF/RH swing.
  EXPECT_EQ(modes.modeAtTime(1.45), 15U);
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
