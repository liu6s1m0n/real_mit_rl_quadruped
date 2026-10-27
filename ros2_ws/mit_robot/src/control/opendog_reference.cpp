// Target trajectory formulas ported from OpenDog TargetTrajectoriesAdapter.cpp
// (qiayuan, 2022/7/24); gait insertion follows the original OCS2 GaitReceiver.
#include "control/opendog_reference.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <ocs2_core/misc/LoadData.h>

OpenDogReference::OpenDogReference(
  const std::string & reference_file, const std::string & task_file)
{
  ocs2::loadData::loadCppDataType(reference_file, "targetDisplacementVelocity", displacement_velocity_);
  ocs2::loadData::loadCppDataType(reference_file, "targetRotationVelocity", rotation_velocity_);
  ocs2::loadData::loadCppDataType(reference_file, "comHeight", com_height_);
  ocs2::loadData::loadCppDataType(reference_file, "standUpDuration", stand_up_duration_);
  ocs2::loadData::loadEigenMatrix(reference_file, "defaultJointState", default_joints_);
  ocs2::loadData::loadCppDataType(task_file, "mpc.timeHorizon", horizon_);
}

ocs2::TargetTrajectories OpenDogReference::poseToTrajectory(
  const ocs2::vector_t & pose, const ocs2::SystemObservation & observation,
  double reaching_time) const
{
  ocs2::vector_t current_pose = observation.state.segment<6>(6);
  current_pose(2) = com_height_;
  current_pose(4) = 0.0;
  current_pose(5) = 0.0;
  ocs2::vector_array_t states(2, ocs2::vector_t::Zero(observation.state.size()));
  states[0] << ocs2::vector_t::Zero(6), current_pose, default_joints_;
  states[1] << ocs2::vector_t::Zero(6), pose, default_joints_;
  return {{observation.time, reaching_time}, states,
    ocs2::vector_array_t(2, ocs2::vector_t::Zero(observation.input.size()))};
}

ocs2::TargetTrajectories OpenDogReference::update(
  const DesiredState<float> & desired, const ocs2::SystemObservation & observation)
{
  if (!desired.valid || desired.mode == ControlMode::Passive ||
    desired.mode == ControlMode::JointPd)
  {
    standing_up_ = false;
    if (!idle_reference_set_) {
      idle_target_ = {{observation.time}, {observation.state}, {observation.input}};
      idle_reference_set_ = true;
    }
    return idle_target_;
  }
  idle_reference_set_ = false;
  if (desired.mode == ControlMode::StandUp) {
    if (!standing_up_) {
      auto current = observation.state;
      current.head<6>().setZero();
      auto target = current;
      target(8) = com_height_;
      target(10) = target(11) = 0.0;
      target.tail(12) = default_joints_;
      stand_up_target_ = {{observation.time, observation.time + stand_up_duration_},
        {current, target}, ocs2::vector_array_t(2, ocs2::vector_t::Zero(observation.input.size()))};
      standing_up_ = true;
    }
    return stand_up_target_;
  }
  standing_up_ = false;
  ocs2::vector_t pose(6);
  pose << desired.body_position_world.cast<double>(),
    double(desired.body_rpy.z()), double(desired.body_rpy.y()), double(desired.body_rpy.x());
  if (desired.mode == ControlMode::Locomotion || desired.mode == ControlMode::WalkClassic) {
    pose.head<2>() = observation.state.segment<2>(6) +
      horizon_ * desired.body_velocity_world.head<2>().cast<double>();
    pose(3) = observation.state(9) + horizon_ * desired.body_angular_velocity.z();
    auto target = poseToTrajectory(pose, observation, observation.time + horizon_);
    for (auto & state : target.stateTrajectory) {
      state.head<3>() = desired.body_velocity_world.cast<double>();
    }
    return target;
  }
  const auto delta = (pose - observation.state.segment<6>(6)).eval();
  const auto reaching_time = observation.time + std::max(
    std::abs(delta(3)) / rotation_velocity_, delta.head<2>().norm() / displacement_velocity_);
  return poseToTrajectory(pose, observation, reaching_time);
}

OpenDogGaitCommand::OpenDogGaitCommand(
  const std::string & gait_file,
  std::shared_ptr<ocs2::legged_robot::GaitSchedule> schedule)
: schedule_(std::move(schedule)),
  stance_(ocs2::legged_robot::loadModeSequenceTemplate(gait_file, "stance", false)),
  trot_(ocs2::legged_robot::loadModeSequenceTemplate(gait_file, "trot", false))
{}

void OpenDogGaitCommand::setMode(ControlMode mode)
{
  requested_trot_.store(mode == ControlMode::Locomotion || mode == ControlMode::WalkClassic);
}

void OpenDogGaitCommand::preSolverRun(
  double init_time, double final_time, const ocs2::vector_t &,
  const ocs2::ReferenceManagerInterface &)
{
  const bool trot = requested_trot_.load();
  if (trot != applied_trot_) {
    schedule_->insertModeSequenceTemplate(trot ? trot_ : stance_, final_time, final_time - init_time);
    applied_trot_ = trot;
  }
}
