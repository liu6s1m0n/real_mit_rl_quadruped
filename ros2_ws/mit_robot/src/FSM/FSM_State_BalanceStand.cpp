// 平衡站立状态：四足作为接触约束，机身位置与姿态作为高优先级任务。
#include "FSM/FSM_State_BalanceStand.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "model/floating_base_model_factory.hpp"
#include "MPC/RobotState.h"

template<typename T>
FSM_State_BalanceStand<T>::FSM_State_BalanceStand(
  ControlFSMData<T> * control_fsm_data)
: FSM_State<T>(
    control_fsm_data, FSM_StateName::BALANCE_STAND, "BALANCE_STAND")
{
  if (control_fsm_data == nullptr || !control_fsm_data->valid()) {
    throw std::invalid_argument("balance-stand state requires valid FSM data");
  }
  this->turnOnAllSafetyChecks();
  this->checkPDesFoot = false;
  // 与 MIT 一致：站立不运行 MPC，四足均分重力作为 WBIC 的期望反力。
  //创建WBC
  wbc_ctrl_ = std::make_unique<LocomotionCtrl<T>>(
    model::makeFloatingBaseModel(*control_fsm_data->quadruped));
  const auto & parameters = *control_fsm_data->control_parameters;
  wbc_ctrl_->setBodyPositionGains(
    parameters.balance_body_position_kp, parameters.balance_body_position_kd);
  // DM IMU 的偏航角当前不可稳定观测。保留滚转/俯仰姿态环，
  // 暂时关闭 BalanceStand 的偏航力矩，避免把航向漂移误当成机身扭转误差。
  auto balance_orientation_kp = parameters.balance_body_orientation_kp;
  auto balance_orientation_kd = parameters.balance_body_orientation_kd;
  balance_orientation_kp.z() = T(0);
  balance_orientation_kd.z() = T(0);
  wbc_ctrl_->setBodyOrientationGains(
    balance_orientation_kp, balance_orientation_kd);
  wbc_ctrl_->setFootPositionGains(
    parameters.wbc_foot_position_kp, parameters.wbc_foot_position_kd);
  wbc_ctrl_->setJointGains(
    parameters.balance_joint_kp, parameters.balance_joint_kd);
  wbc_ctrl_->setFloatingBaseWeight(parameters.balance_floating_base_weight);
  wbc_ctrl_->setReactionForceWeight(parameters.balance_reaction_force_weight);
  wbc_ctrl_->setMaxNormalForce(parameters.maximum_normal_force);
}

template<typename T>
void FSM_State_BalanceStand<T>::onEnter()
{
  this->nextStateName = this->stateName;
  this->transitionData.zero();
  //请求站立步态
  this->_data->gait_scheduler->requestGait(GaitType::STAND);

  // 普通入站从实测高度起身；正常回站继承最后一次 WBC 参考。
  auto & handoff = this->_data->stand_handoff;
  handoff.active = handoff.pending;
  handoff.pending = false;
  initial_body_position_ = this->_data->state_estimate->position_world;
  // 趴卧起身的高度限速从实测值开始。
  last_height_command_ = initial_body_position_.z();
  if (this->_data->control_parameters->start_in_prone_home &&
    initial_body_position_.z() <
    this->_data->control_parameters->minimum_standing_height)
  {
    initial_body_position_.z() = this->_data->quadruped->nominalBodyHeight();
  }
  // 世界航向沿用进入状态时的实测值，但站立目标始终保持机身水平。不能把
  // 切换瞬间的 roll/pitch 锁存为目标，否则已有倾斜会被 WBC 永久维持。
  initial_body_rpy_ << T(0), T(0), this->_data->state_estimate->rpy.z();
  if (handoff.active) {
    initial_body_position_ = handoff.body.body_position_world;
    initial_body_position_.template head<2>() +=
      T(0.5) * handoff.duration * handoff.body.body_velocity_world.template head<2>();
    last_height_command_ = handoff.body.body_position_world.z();
    initial_body_rpy_.z() = handoff.body.body_rpy.z() + T(0.5) * handoff.duration *
      mpc::RobotState<T>::rpyRateFromBodyAngularVelocity(
      handoff.body.body_rpy, handoff.body.body_angular_velocity).z();
  }
  posture_ramp_iteration_ = 0;
}

template<typename T>
void FSM_State_BalanceStand<T>::run()
{
  BalanceStandStep();
}

//检查是否切换状态
template<typename T>
FSM_StateName FSM_State_BalanceStand<T>::checkTransition()
{
  /*每次检查状态切换时，运行计数增加1。
    注意：它发生在 run() 之前还是之后，由 ControlFSM::runFSM() 的调用顺序决定。
    当前控制框架中，先检查切换，如果不切换才执行当前状态的 run()。*/
  ++iteration_;
  this->nextStateName = this->stateForMode(this->_data->desired_state->mode);
  this->transitionDuration = T(0);
  if (this->nextStateName == FSM_StateName::LOCOMOTION) {
    this->_data->gait_scheduler->requestGait(this->_data->locomotion_gait);
  }
  return this->nextStateName;
}

/*transition：执行状态切换 在真正离开站立状态前，
再输出一帧有效的站立控制，避免切换瞬间没有关节命令。*/
template<typename T>
TransitionData<T> FSM_State_BalanceStand<T>::transition()
{
  /*在真正离开站立状态前，再输出一帧有效的站立控制，避免切换瞬间没有关节命令。*/
  if (this->nextStateName == FSM_StateName::LOCOMOTION) {BalanceStandStep();}
  /*进入Passive时关闭当前状态的安全检查.原因是Passive
    本身就是一种停止主动控制的状态，继续执行普通状态的足端和力矩检查没有意义。*/
  if (this->nextStateName == FSM_StateName::PASSIVE) {
    this->turnOffAllSafetyChecks();
  }
  this->transitionData.done = true;
  return this->transitionData;
}

//onExit：退出状态
template<typename T>
void FSM_State_BalanceStand<T>::onExit()
{
  iteration_ = 0;
  this->_data->stand_handoff.active = false;
  this->_data->stand_handoff.pending = false;
}
/*BalanceStandStep：真正的站立控制
            这是整个文件最重要的函数。*/
template<typename T>
void FSM_State_BalanceStand<T>::BalanceStandStep()
{
  auto & handoff = this->_data->stand_handoff;
  // 完成交接后锁定水平位置与航向，保持水平；高度继续跟随限速目标。
  wbc_data_.pBody_des = initial_body_position_;
  wbc_data_.vBody_des.setZero();
  wbc_data_.aBody_des.setZero();
  wbc_data_.pBody_RPY_des = initial_body_rpy_;
  wbc_data_.vBody_Ori_des.setZero();

  //读取外部期望高度
  const DesiredState<T> & desired = *this->_data->desired_state;
  // Keep the Hermite endpoint fixed during handoff; resume height commands afterwards.
  if (desired.valid && !handoff.active) {
    wbc_data_.pBody_des.z() = desired.body_position_world.z();
  }
  // 额外限制单周期下降量，防止高度滑块快速下拉造成腿部瞬时折叠。
  // 参数2 高度变化率
  const T maximum_height_step =
    this->_data->control_parameters->balance_height_step;
  if (last_height_command_ - wbc_data_.pBody_des.z() > maximum_height_step) {
    wbc_data_.pBody_des.z() = last_height_command_ - maximum_height_step;
  }
  if (wbc_data_.pBody_des.z() - last_height_command_ > maximum_height_step) {
      wbc_data_.pBody_des.z() = last_height_command_ + maximum_height_step;
  }
  last_height_command_ = wbc_data_.pBody_des.z();

  const T u = std::min(T(1), T(posture_ramp_iteration_) *
    this->_data->control_time_step / handoff.duration);
  const T s = u * u * (T(3) - T(2) * u);
  const T ds = T(6) * u * (T(1) - u) / handoff.duration;
  if (handoff.active) {
    // Quintic Hermite: preserve entry p/v/a and finish at rest.
    const T u2 = u*u, u3 = u2*u, u4 = u3*u, u5 = u4*u;
    const T h0 = T(1)-T(10)*u3+T(15)*u4-T(6)*u5;
    const T h1 = u-T(6)*u3+T(8)*u4-T(3)*u5;
    const T h2 = (u2-T(3)*u3+T(3)*u4-u5)/T(2);
    const T d0 = -T(30)*u2+T(60)*u3-T(30)*u4;
    const T d1 = T(1)-T(18)*u2+T(32)*u3-T(15)*u4;
    const T d2 = (T(2)*u-T(9)*u2+T(12)*u3-T(5)*u4)/T(2);
    const auto interpolate = [&](const Vec3<T> & start, const Vec3<T> & velocity,
      const Vec3<T> & acceleration, Vec3<T> & position, Vec3<T> & speed, Vec3<T> & acc) {
        const Vec3<T> delta = start-position;
        position += h0*delta + handoff.duration*h1*velocity +
          handoff.duration*handoff.duration*h2*acceleration;
        speed = d0/handoff.duration*delta + d1*velocity + handoff.duration*d2*acceleration;
        acc = (-T(60)*u+T(180)*u2-T(120)*u3)/(handoff.duration*handoff.duration)*delta +
          (-T(36)*u+T(96)*u2-T(60)*u3)/handoff.duration*velocity +
          (T(1)-T(9)*u+T(18)*u2-T(10)*u3)*acceleration;
      };
    interpolate(handoff.body.body_position_world, handoff.body.body_velocity_world,
      handoff.body.body_acceleration_world, wbc_data_.pBody_des,
      wbc_data_.vBody_des, wbc_data_.aBody_des);
    Vec3<T> rpy_rate, unused_acceleration;
    interpolate(handoff.body.body_rpy, mpc::RobotState<T>::rpyRateFromBodyAngularVelocity(
      handoff.body.body_rpy, handoff.body.body_angular_velocity), Vec3<T>::Zero(),
      wbc_data_.pBody_RPY_des, rpy_rate, unused_acceleration);
    const T roll = wbc_data_.pBody_RPY_des.x(), pitch = wbc_data_.pBody_RPY_des.y();
    wbc_data_.vBody_Ori_des << rpy_rate.x()-std::sin(pitch)*rpy_rate.z(),
      std::cos(roll)*rpy_rate.y()+std::sin(roll)*std::cos(pitch)*rpy_rate.z(),
      -std::sin(roll)*rpy_rate.y()+std::cos(roll)*std::cos(pitch)*rpy_rate.z();
  }

  // 四腿接触；期望反力 mg/4，由 WBIC 在摩擦锥内修正。
  const T support_force =
    this->_data->quadruped->totalMass() * T(9.81) / static_cast<T>(kNumLegs);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    // MIT BalanceStand uses contact constraints for all four feet; the
    // support-foot position task is therefore unused in this state.
    wbc_data_.pFoot_des[leg].setZero();
    wbc_data_.vFoot_des[leg].setZero();
    wbc_data_.aFoot_des[leg].setZero();
    wbc_data_.Fr_des[leg] = Vec3<T>(T(0), T(0), support_force);
    wbc_data_.contact_state[leg] = T(1);
  }
  //调用WBC
  /*&wbc_data_ 机身想去哪里；机身想保持什么姿态；四只脚是否接触；期望反力是多少。*/
  /*&state_estimate，包括：当前机身位置；当前机身姿态；当前机身速度；当前角速度。*/
  /*&joint_states,  12个关机的位置和速度*/
  /*position_desired  velocity_desired ,torque_feedforward
                                       kp_joint kd_joint */
  const bool wbc_valid = wbc_ctrl_->runAndApply(
    &wbc_data_, *this->_data->state_estimate, *this->_data->joint_states,
    *this->_data->leg_controller);

  if (!wbc_valid) {
    std::fprintf(stderr,
      "[FSM][BALANCE_STAND] 失能前判断：WBC runAndApply 失败，iteration=%zu target_height=%.4f。\n",
      iteration_, static_cast<double>(wbc_data_.pBody_des.z()));
    return;
  }

  // 站立与行走统一直接使用 WBC 前馈，保持求解器给出的动力学比例。
  T maximum_feedforward_torque = T(0);
  for (const auto & command : this->_data->leg_controller->commands) {
    maximum_feedforward_torque = std::max(
      maximum_feedforward_torque,
      command.torque_feedforward.cwiseAbs().maxCoeff());
  }
  if (posture_ramp_iteration_ == 0) {
    std::fprintf(
      stderr,
      "[FSM][BALANCE_STAND] 直接使用 WBC 前馈：first_frame_max_tau=%.3f Nm。\n",
      static_cast<double>(maximum_feedforward_torque));
  }

  // Smooth only feedback references/gains; retain this frame's solved WBC torque.
  if (handoff.active) {
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      auto & command = this->_data->leg_controller->commands[leg];
      const auto & entry = handoff.commands[leg];
      command.velocity_desired = (T(1)-u)*(T(1)-T(3)*u)*entry.velocity_desired +
        s*command.velocity_desired + ds*(command.position_desired-entry.position_desired);
      command.position_desired = (T(1)-s)*entry.position_desired + s*command.position_desired +
        handoff.duration*u*(T(1)-u)*(T(1)-u)*entry.velocity_desired;
      const auto & bounds = this->_data->quadruped->leg(static_cast<LegId>(leg)).joints;
      command.position_desired = command.position_desired.cwiseMax(bounds.lower_limit).cwiseMin(bounds.upper_limit);
      command.velocity_desired = command.velocity_desired.cwiseMax(-bounds.velocity_limit).cwiseMin(bounds.velocity_limit);
      command.kp_joint = (T(1)-s)*entry.kp_joint + s*command.kp_joint;
      command.kd_joint = (T(1)-s)*entry.kd_joint + s*command.kd_joint;
      command.kd_cartesian = (T(1)-s)*entry.kd_cartesian;
      command.foot_velocity_desired = entry.foot_velocity_desired;
    }
    if (u >= T(1)) {handoff.active = false;}
  }
  ++posture_ramp_iteration_;
}

template class FSM_State_BalanceStand<float>;
