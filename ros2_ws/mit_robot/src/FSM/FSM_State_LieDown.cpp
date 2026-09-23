// MPC/WBC supported descent; grounded contact is handed off to a smooth fold.
#include "FSM/FSM_State_LieDown.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "model/floating_base_model_factory.hpp"

template<typename T>
FSM_State_LieDown<T>::FSM_State_LieDown(ControlFSMData<T> * control_fsm_data)
: FSM_State<T>(control_fsm_data, FSM_StateName::LIE_DOWN, "LIE_DOWN")
{
  if (control_fsm_data == nullptr || !control_fsm_data->valid()) {
    throw std::invalid_argument("lie-down state requires valid FSM data");
  }
  const auto & parameters = *control_fsm_data->control_parameters;
  prone_body_height_ = parameters.prone_body_height;
  if (!std::isfinite(static_cast<double>(prone_body_height_)) ||
    prone_body_height_ <= T(0) ||
    !std::isfinite(static_cast<double>(parameters.prone_down_fold_duration)) ||
    parameters.prone_down_fold_duration <= T(0))
  {
    throw std::invalid_argument("lie-down parameters are invalid");
  }
  const std::size_t mpc_interval = static_cast<std::size_t>(std::max(
      T(1), std::round(T(0.04) / control_fsm_data->control_time_step)));
  const std::size_t gait_segment_interval = static_cast<std::size_t>(std::max(
      T(1), std::round(T(0.05) / control_fsm_data->control_time_step)));
  mpc_ = std::make_unique<mpc::ConvexMPCLocomotion<T>>(
    *control_fsm_data->quadruped, control_fsm_data->control_time_step,
    mpc_interval, mpc::SolverSettings<T>{}, gait_segment_interval);
  wbc_ctrl_ = std::make_unique<LocomotionCtrl<T>>(
    model::makeFloatingBaseModel(*control_fsm_data->quadruped));
  // Keep the existing BalanceStand task weighting for the descent. The
  // LocomotionCtrl defaults are intentionally generic (Kd_z=1), which is too
  // lightly damped for a body being brought within millimetres of the floor.
  wbc_ctrl_->setBodyPositionGains(
    parameters.balance_body_position_kp, parameters.balance_body_position_kd);
  wbc_ctrl_->setBodyOrientationGains(
    parameters.locomotion_body_orientation_kp,
    parameters.locomotion_body_orientation_kd);
  wbc_ctrl_->setJointGains(
    parameters.locomotion_joint_kp, parameters.locomotion_joint_kd);
  wbc_ctrl_->setFloatingBaseWeight(parameters.balance_floating_base_weight);
  wbc_ctrl_->setReactionForceWeight(parameters.balance_reaction_force_weight);
  wbc_ctrl_->setMaxNormalForce(parameters.maximum_normal_force);
  fold_iterations_ = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(
      parameters.prone_down_fold_duration / control_fsm_data->control_time_step)));
  // First move only the calf (the user's "claw" motor) to a preparation
  // point. The thigh and calf then move together for the final fold. Keep the
  // original full duration for the combined stage so the load-bearing thigh
  // does not move faster because of the sequencing change.
  calf_fold_iterations_ = std::max<std::size_t>(1, fold_iterations_ / 4);
  thigh_fold_iterations_ = fold_iterations_;
  this->turnOnAllSafetyChecks();
  this->checkPDesFoot = false;
}

template<typename T>
void FSM_State_LieDown<T>::onEnter()
{
  this->nextStateName = this->stateName;
  this->transitionData.zero();
  phase_ = Phase::Lowering;
  fold_iteration_ = 0;
  hold_body_position_world_ = Vec3<T>::Zero();
  mpc_->initialize();
  mpc_->setGait(GaitType::STAND);
  this->_data->gait_scheduler->requestGait(GaitType::STAND);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & feedback = this->_data->leg_controller->datas[leg];
    entry_joint_positions_[leg] = feedback.valid && feedback.q.allFinite() ?
      feedback.q : this->_data->control_parameters->motor_zero_position;
    fold_start_joint_positions_[leg] = entry_joint_positions_[leg];
  }
}

template<typename T>
std::array<Vec3<T>, kNumLegs> FSM_State_LieDown<T>::footPositionsWorld() const
{
  std::array<Vec3<T>, kNumLegs> positions{};
  const auto & estimate = *this->_data->state_estimate;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const LegId leg_id = static_cast<LegId>(leg);
    const Vec3<T> foot_body = this->_data->quadruped->hipLocation(leg_id) +
      this->_data->leg_controller->datas[leg].p;
    positions[leg] = estimate.position_world +
      estimate.rotation_world_from_body * foot_body;
  }
  return positions;
}

template<typename T>
void FSM_State_LieDown<T>::runMpcLowering()
{
  const auto feet_world = footPositionsWorld();
  const auto result = mpc_->run(
    *this->_data->state_estimate, *this->_data->desired_state, feet_world);
  if (!result.valid) {
    std::fprintf(stderr,
      "[FSM][LIE_DOWN] 失能前判断：MPC result.valid=false。\n");
    this->_data->leg_controller->zeroCommand();
    this->_data->leg_controller->setEnabled(false);
    return;
  }
  wbc_data_.pBody_des = result.command.body_position_world;
  wbc_data_.vBody_des = result.command.body_velocity_world;
  wbc_data_.aBody_des = result.command.body_acceleration_world;
  wbc_data_.pBody_RPY_des = result.command.body_rpy;
  wbc_data_.vBody_Ori_des = result.command.body_angular_velocity;
  if (phase_ == Phase::Hold) {
    // ConvexMPCLocomotion normally re-anchors x/y to the current estimate for
    // velocity teleoperation.  That is correct while walking, but it turns
    // the prone hold into an uncontrolled drift.  Hold the completion point
    // and let the existing WBC position/velocity task damp any residual slip.
    wbc_data_.pBody_des.x() = hold_body_position_world_.x();
    wbc_data_.pBody_des.y() = hold_body_position_world_.y();
    wbc_data_.vBody_des.x() = T(0);
    wbc_data_.vBody_des.y() = T(0);
    wbc_data_.aBody_des.x() = T(0);
    wbc_data_.aBody_des.y() = T(0);
    // Let the trunk settle onto the floor instead of allowing the four foot
    // contacts to hold it a few centimetres above the initialization pose.
    wbc_data_.pBody_des.z() = prone_body_height_ - T(0.015);
  }
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    wbc_data_.pFoot_des[leg] = feet_world[leg];
    wbc_data_.vFoot_des[leg].setZero();
    wbc_data_.aFoot_des[leg].setZero();
    wbc_data_.Fr_des[leg] = result.reaction_forces_world[leg];
    wbc_data_.contact_state[leg] = T(1);
  }
  if (!wbc_ctrl_->runAndApply(
      &wbc_data_, *this->_data->state_estimate, *this->_data->joint_states,
      *this->_data->leg_controller))
  {
    std::fprintf(stderr,
      "[FSM][LIE_DOWN] 失能前判断：WBC runAndApply 失败。\n");
    this->_data->leg_controller->zeroCommand();
    this->_data->leg_controller->setEnabled(false);
    return;
  }

}

template<typename T>
bool FSM_State_LieDown<T>::loweringPostureStable() const noexcept
{
  const auto & estimate = *this->_data->state_estimate;
  if (!estimate.valid || std::abs(estimate.position_world.z() - prone_body_height_) > T(0.015) ||
    std::abs(estimate.velocity_world.z()) > T(0.05) ||
    std::abs(estimate.rpy.x()) > T(0.08) || std::abs(estimate.rpy.y()) > T(0.08) ||
    !estimate.angular_velocity_body.allFinite() ||
    estimate.angular_velocity_body.norm() > T(0.15))
  {
    return false;
  }
  for (const auto & joint : *this->_data->joint_states) {
    if (!joint.valid || joint.velocity.norm() > T(0.20)) {return false;}
  }
  return true;
}

template<typename T>
void FSM_State_LieDown<T>::runJointFolding()
{
  auto & controller = *this->_data->leg_controller;
  controller.setEnabled(true);
  const bool folding_calf = phase_ == Phase::FoldingCalf;
  const std::size_t stage_iterations = folding_calf ? calf_fold_iterations_ :
    thigh_fold_iterations_;
  const T linear_progress = std::clamp(
    static_cast<T>(fold_iteration_) / static_cast<T>(stage_iterations), T(0), T(1));
  const T progress = linear_progress * linear_progress *
    (T(3) - T(2) * linear_progress);
  const T progress_rate = T(6) * linear_progress * (T(1) - linear_progress) /
    (static_cast<T>(stage_iterations) * this->_data->control_time_step);
  const auto & parameters = *this->_data->control_parameters;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    auto & command = controller.commands[leg];
    const Vec3<T> target = parameters.motor_zero_position;
    const Vec3<T> travel = target - fold_start_joint_positions_[leg];
    // Keep every non-active joint at the captured posture. The first explicit
    // fold stage changes only the calf; the thigh and hip are held.
    command.position_desired = fold_start_joint_positions_[leg];
    command.velocity_desired.setZero();
    if (folding_calf) {
      // Do not take the calf all the way to zero here. Leaving the remaining
      // travel for the second stage makes the thigh and calf move together.
      constexpr T kCalfPreparationProgress = T(0.5);
      command.position_desired[2] += progress * kCalfPreparationProgress * travel[2];
      command.velocity_desired[2] =
        progress_rate * kCalfPreparationProgress * travel[2];
    } else {
      constexpr T kCalfPreparationProgress = T(0.5);
      const T calf_preparation_position =
        fold_start_joint_positions_[leg][2] + kCalfPreparationProgress * travel[2];
      const T calf_remaining_travel = target[2] - calf_preparation_position;
      command.position_desired[0] += progress * travel[0];
      command.position_desired[2] = calf_preparation_position + progress * calf_remaining_travel;
      command.position_desired[1] += progress * travel[1];
      command.velocity_desired[0] = progress_rate * travel[0];
      command.velocity_desired[2] = progress_rate * calf_remaining_travel;
      command.velocity_desired[1] = progress_rate * travel[1];
    }
    // Use the already validated stand-up impedance during the moving fold.
    // The grounded Hold phase keeps the same moderate impedance to avoid
    // converting tiny joint errors into tangential contact forces.
    command.kp_joint = parameters.stand_up_joint_kp;
    command.kd_joint = parameters.stand_up_joint_kd;
  }
  if (fold_iteration_ >= stage_iterations) {
    if (folding_calf) {
      // The calf is now at the preparation point. Start the combined
      // thigh/calf stage from zero progress on the next control frame so no
      // position jump is introduced at the stage boundary.
      fold_iteration_ = 0;
      phase_ = Phase::FoldingThighAndCalf;
    } else {
      hold_body_position_world_ = this->_data->state_estimate->position_world;
      phase_ = Phase::Hold;
      // Keep the LieDown state alive after the fold. Its Hold phase continues
      // to run the existing MPC/WBC body task, providing horizontal damping;
      // switching to JointPd here would leave the floating base uncontrolled.
      this->_data->desired_state->mode = ControlMode::ProneDown;
    }
  } else {
    ++fold_iteration_;
  }
}

template<typename T>
void FSM_State_LieDown<T>::runProneHold()
{
  runMpcLowering();
  auto & controller = *this->_data->leg_controller;
  const auto & parameters = *this->_data->control_parameters;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    auto & command = controller.commands[leg];
    // The final pose must be identical to the MuJoCo initialization keyframe.
    command.position_desired = parameters.motor_zero_position;
    command.velocity_desired.setZero();
    command.kp_joint = parameters.stand_up_joint_kp;
    command.kd_joint = parameters.stand_up_joint_kd;
  }
}

template<typename T>
void FSM_State_LieDown<T>::run()
{
  bool started_folding_this_frame = false;
  if (phase_ == Phase::Lowering) {
    runMpcLowering();
    // Begin folding before the trunk reaches the floor.  This prevents the
    // standing leg posture from becoming a high-knee snapshot at the exact
    // moment the body reaches prone height.
    const T fold_start_height = prone_body_height_ + T(0.14);
    if (this->_data->state_estimate->position_world.z() <= fold_start_height ||
      loweringPostureStable())
    {
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        const auto & feedback = this->_data->leg_controller->datas[leg];
        fold_start_joint_positions_[leg] = feedback.valid && feedback.q.allFinite() ?
          feedback.q : entry_joint_positions_[leg];
      }
      fold_iteration_ = 0;
      phase_ = Phase::FoldingCalf;
      started_folding_this_frame = true;
    }
  }
  if ((phase_ == Phase::FoldingCalf || phase_ == Phase::FoldingThighAndCalf) &&
    !started_folding_this_frame)
  {
    // Keep lowering the trunk while the staged leg fold is in progress.
    runMpcLowering();
  }
  if (phase_ == Phase::FoldingCalf || phase_ == Phase::FoldingThighAndCalf) {
    runJointFolding();
  } else if (phase_ == Phase::Hold) {
    runProneHold();
  }
}

template<typename T>
FSM_StateName FSM_State_LieDown<T>::checkTransition()
{
  switch (this->_data->desired_state->mode) {
    case ControlMode::ProneDown:
      this->nextStateName = this->stateName;
      break;
    case ControlMode::JointPd:
      this->nextStateName = FSM_StateName::JOINT_PD;
      break;
    case ControlMode::BalanceStand:
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      break;
    case ControlMode::StandUp:
      this->nextStateName = FSM_StateName::STAND_UP;
      break;
    case ControlMode::RecoveryStand:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      break;
    case ControlMode::Passive:
      this->nextStateName = FSM_StateName::PASSIVE;
      break;
    case ControlMode::Locomotion:
    case ControlMode::WalkClassic:
    case ControlMode::WalkRl:
    case ControlMode::StairsRl:
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      this->_data->desired_state->mode = ControlMode::BalanceStand;
      break;
  }
  this->transitionDuration = T(0);
  return this->nextStateName;
}

template<typename T>
TransitionData<T> FSM_State_LieDown<T>::transition()
{
  if (this->nextStateName != FSM_StateName::PASSIVE) {run();}
  this->transitionData.done = true;
  return this->transitionData;
}

template<typename T>
void FSM_State_LieDown<T>::onExit()
{
  fold_iteration_ = 0;
}

template class FSM_State_LieDown<float>;
