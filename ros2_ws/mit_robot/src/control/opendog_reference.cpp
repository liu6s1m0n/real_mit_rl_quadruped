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
  if (!desired.valid || desired.mode == ControlMode::Passive)
  {
    standing_up_ = false;
    joint_pd_reference_set_ = false;
    if (!idle_reference_set_) {
      idle_target_ = {{observation.time}, {observation.state}, {observation.input}};
      idle_reference_set_ = true;
    }
    return idle_target_;
  }
  if (desired.mode == ControlMode::JointPd) {
    standing_up_ = false;
    idle_reference_set_ = false;
    if (!joint_pd_reference_set_) {
      // Match the DM1 hardware runner's startup contract: keep the measured
      // base pose and move all twelve model joints smoothly to motor zero.
      auto current = observation.state;
      current.head<6>().setZero();
      auto target = current;
      target.tail(12).setZero();
      joint_pd_target_ = {{observation.time, observation.time + stand_up_duration_},
        {current, target}, ocs2::vector_array_t(2, ocs2::vector_t::Zero(observation.input.size()))};
      joint_pd_reference_set_ = true;
    }
    return joint_pd_target_;
  }
  joint_pd_reference_set_ = false;
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
  trot_(ocs2::legged_robot::loadModeSequenceTemplate(gait_file, "trot", false)),
  single_step_{
    ocs2::legged_robot::loadModeSequenceTemplate(gait_file, "single_step_lf_rh", false),
    ocs2::legged_robot::loadModeSequenceTemplate(gait_file, "single_step_rf_lh", false)}
{}

void OpenDogGaitCommand::setMode(ControlMode mode)
{
  const bool trot = mode == ControlMode::Locomotion || mode == ControlMode::WalkClassic;
  requested_trot_.store(trot);
  if (!trot) {
    // Match the original trot command lifecycle: leaving Locomotion clears a
    // pending one-shot extension, so it cannot replay on the next entry.
    requested_motion_.store(0);
  }
}

void OpenDogGaitCommand::setMotion(teleop::Motion motion, std::uint64_t sequence)
{
  const std::uint64_t diagonal = motion == teleop::Motion::TwoLegStand ? 1 :
    (motion == teleop::Motion::TwoLegStandReverse ? 2 : 0);
  requested_motion_.store(diagonal == 0 ? 0 : (sequence << 2) | diagonal);
}

void OpenDogGaitCommand::reset() noexcept
{
  requested_trot_.store(false);
  requested_motion_.store(0);
  applied_trot_ = false;
  applied_motion_ = 0;
  // Called after the solver thread has joined. Restore its schedule and template.
  schedule_->insertModeSequenceTemplate(stance_, 0.0, stance_.switchingTimes.back());
  schedule_->setModeSchedule({{stance_.switchingTimes.back()}, {15, 15}});
}

void OpenDogGaitCommand::preSolverRun(
  double init_time, double final_time, const ocs2::vector_t &,
  const ocs2::ReferenceManagerInterface &)
{
  const bool trot = requested_trot_.load();
  const auto motion = trot ? requested_motion_.load() : 0;
  if (trot == applied_trot_ && motion == applied_motion_) {return;}
  const auto horizon = final_time - init_time;
  const auto diagonal = motion & 3U;
  if (diagonal == 0) {
    schedule_->insertModeSequenceTemplate(trot ? trot_ : stance_, final_time, horizon);
  } else {
    // Keep the already planned prefix, then append exactly one finite step.
    // Future getModeSchedule() extensions use STANCE, so a latched key cannot loop.
    schedule_->insertModeSequenceTemplate(stance_, final_time, final_time + horizon);
    auto modes = schedule_->getModeSchedule(init_time - horizon, final_time + horizon);
    const auto first = std::lower_bound(modes.eventTimes.begin(), modes.eventTimes.end(), final_time);
    const auto index = std::distance(modes.eventTimes.begin(), first);
    modes.eventTimes.erase(first, modes.eventTimes.end());
    modes.modeSequence.resize(index + 1);
    const auto & step = single_step_[diagonal - 1];
    for (std::size_t i = 0; i < step.modeSequence.size(); ++i) {
      modes.eventTimes.push_back(final_time + step.switchingTimes[i]);
      modes.modeSequence.push_back(step.modeSequence[i]);
    }
    modes.eventTimes.push_back(final_time + step.switchingTimes.back());
    modes.modeSequence.push_back(ocs2::legged_robot::ModeNumber::STANCE);
    schedule_->setModeSchedule(modes);
  }
  applied_trot_ = trot;
  applied_motion_ = motion;
}
