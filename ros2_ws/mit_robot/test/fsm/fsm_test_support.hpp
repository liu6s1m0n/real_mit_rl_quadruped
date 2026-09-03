#ifndef MYMIT_ROBOT_TEST_FSM_TEST_SUPPORT_HPP_
#define MYMIT_ROBOT_TEST_FSM_TEST_SUPPORT_HPP_

#include <array>

#include "FSM/ControlFSMData.h"

namespace test_support
{

/** Minimal DM1-only fixture shared by focused FSM safety tests. */
struct FsmContext
{
  Quadruped<float> quadruped = makeQuadruped<float>(RobotType::DM1);
  RobotControlParameters<float> parameters =
    makeRobotControlParameters<float>(RobotType::DM1);
  StateEstimate<float> estimate;
  std::array<JointState<float>, kNumLegs> joint_states{};
  LegController<float> leg_controller;
  GaitScheduler<float> gait_scheduler;
  DesiredState<float> desired_state;
  ControlFSMData<float> fsm_data;
  bool initialized = false;

  explicit FsmContext(float time_step)
  : leg_controller(quadruped), gait_scheduler(time_step)
  {
    estimate.valid = true;
    estimate.orientation_world_from_body.setIdentity();
    estimate.rotation_world_from_body.setIdentity();
    desired_state.valid = true;
    desired_state.mode = ControlMode::BalanceStand;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      auto & state = joint_states[leg];
      state.leg = static_cast<LegId>(leg);
      state.position = quadruped.leg(state.leg).joints.home_position;
      state.velocity.setZero();
      state.torque_estimate.setZero();
      state.timestamp = 0.0F;
      state.valid = leg_controller.updateData(state);
    }
    fsm_data.quadruped = &quadruped;
    fsm_data.control_parameters = &parameters;
    fsm_data.state_estimate = &estimate;
    fsm_data.joint_states = &joint_states;
    fsm_data.leg_controller = &leg_controller;
    fsm_data.gait_scheduler = &gait_scheduler;
    fsm_data.desired_state = &desired_state;
    fsm_data.control_time_step = time_step;
    initialized = fsm_data.valid();
  }

  ControlFSMData<float> & data() noexcept {return fsm_data;}
};

}  // namespace test_support

#endif  // MYMIT_ROBOT_TEST_FSM_TEST_SUPPORT_HPP_
