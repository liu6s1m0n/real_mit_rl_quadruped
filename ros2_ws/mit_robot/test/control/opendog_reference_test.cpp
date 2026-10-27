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
