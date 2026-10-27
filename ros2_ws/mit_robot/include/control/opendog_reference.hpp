#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include "model/robot_types.hpp"
#include "teleop/operator_command.hpp"

// OpenDog's target-pose / velocity / stand-up trajectory generation, separated
// from the solver and the hardware bridge. Velocities at this boundary are world-frame.
class OpenDogReference
{
public:
  OpenDogReference(const std::string & reference_file, const std::string & task_file);
  ocs2::TargetTrajectories update(
    const DesiredState<float> & desired, const ocs2::SystemObservation & observation);
  void reset() noexcept
  {
    standing_up_ = false;
    idle_reference_set_ = false;
    joint_pd_reference_set_ = false;
  }

private:
  ocs2::TargetTrajectories poseToTrajectory(
    const ocs2::vector_t & pose, const ocs2::SystemObservation & observation,
    double reaching_time) const;
  double displacement_velocity_ = 0.0;
  double rotation_velocity_ = 0.0;
  double com_height_ = 0.0;
  double horizon_ = 0.0;
  double stand_up_duration_ = 0.0;
  ocs2::vector_t default_joints_ = ocs2::vector_t::Zero(12);
  bool standing_up_ = false;
  bool idle_reference_set_ = false;
  bool joint_pd_reference_set_ = false;
  ocs2::TargetTrajectories idle_target_;
  ocs2::TargetTrajectories joint_pd_target_;
  ocs2::TargetTrajectories stand_up_target_;
};

// Same synchronized insertion as the original GaitReceiver; the shared command
// snapshot supplies the request instead of an OCS2 ROS message subscription.
class OpenDogGaitCommand final : public ocs2::SolverSynchronizedModule
{
public:
  OpenDogGaitCommand(
    const std::string & gait_file,
    std::shared_ptr<ocs2::legged_robot::GaitSchedule> schedule);
  void setMode(ControlMode mode);
  void setMotion(teleop::Motion motion, std::uint64_t sequence);
  void reset() noexcept;
  void preSolverRun(
    double init_time, double final_time, const ocs2::vector_t & state,
    const ocs2::ReferenceManagerInterface & reference_manager) override;
  void postSolverRun(const ocs2::PrimalSolution &) override {}

private:
  std::shared_ptr<ocs2::legged_robot::GaitSchedule> schedule_;
  ocs2::legged_robot::ModeSequenceTemplate stance_;
  ocs2::legged_robot::ModeSequenceTemplate trot_;
  ocs2::legged_robot::ModeSequenceTemplate single_step_[2];
  // Low two bits select a diagonal; the remaining bits identify a key press.
  std::atomic<std::uint64_t> requested_motion_{0};
  std::uint64_t applied_motion_ = 0;
  std::atomic<bool> requested_trot_{false};
  bool applied_trot_ = false;
};
