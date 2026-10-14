// 平衡站立状态：四足作为接触约束，机身位置与姿态作为高优先级任务。
#include "FSM/FSM_State_BalanceStand.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "model/floating_base_model_factory.hpp"

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
  wbc_ctrl_->setBodyOrientationGains(
    parameters.balance_body_orientation_kp,
    parameters.balance_body_orientation_kd);
  wbc_ctrl_->setFootPositionGains(
    Vec3<T>::Constant(T(500)), Vec3<T>::Constant(T(10)));
  wbc_ctrl_->setJointGains(
    parameters.joint_kp, parameters.joint_kd);
  wbc_ctrl_->setFloatingBaseWeight(parameters.balance_floating_base_weight);
  wbc_ctrl_->setReactionForceWeight(parameters.balance_reaction_force_weight);
  wbc_ctrl_->setMaxNormalForce(parameters.maximum_normal_force);
}

/*仅在第一次启动，后续不会启动*/
template<typename T>
void FSM_State_BalanceStand<T>::onEnter()
{
  this->nextStateName = this->stateName;
  this->transitionData.zero();
  //请求站立步态
  this->_data->gait_scheduler->requestGait(GaitType::STAND);

  // 进入状态时锁定当前水平位置和姿态，避免突然跳到世界原点。
  //这样子就到导致了步行时刻突然切换站立直接翻到
  initial_body_position_ = this->_data->state_estimate->position_world;
  // 高度限速必须从真实趴卧高度开始；否则下面的有效低高度会被替换成
  // 标称站立高度，导致首个 WBC 周期的高度目标发生跳变。
  last_height_command_ = initial_body_position_.z();
  //如果估计的机身高度低于20厘米，就认为高度估计不可靠，强行设置为30厘米。
  if (this->_data->control_parameters->start_in_prone_home &&
    initial_body_position_.z() <
    this->_data->control_parameters->minimum_standing_height)
  {
    initial_body_position_.z() = this->_data->quadruped->nominalBodyHeight();
  }
  // 世界航向沿用进入状态时的实测值，但站立目标始终保持机身水平。不能把
  // 切换瞬间的 roll/pitch 锁存为目标，否则已有倾斜会被 WBC 永久维持。
  initial_body_rpy_ << T(0), T(0), this->_data->state_estimate->rpy.z();
  posture_ramp_iteration_ = 0;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    entry_joint_positions_[leg] = this->_data->leg_controller->datas[leg].q;
  }
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
}
/*BalanceStandStep：真正的站立控制
            这是整个文件最重要的函数。*/
template<typename T>
void FSM_State_BalanceStand<T>::BalanceStandStep()
{
  // MIT BalanceStand：水平位置与航向锁定入场值，roll/pitch 保持水平，
  // 仅高度跟随外部目标；速度、加速度目标均为零。
  wbc_data_.pBody_des = initial_body_position_;
  wbc_data_.vBody_des.setZero();
  wbc_data_.aBody_des.setZero();
  wbc_data_.pBody_RPY_des = initial_body_rpy_;
  wbc_data_.vBody_Ori_des.setZero();

  //读取外部期望高度
  const DesiredState<T> & desired = *this->_data->desired_state;
  if (desired.valid) {
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

  // 四腿始终接触；期望反力取 mg/4 竖直分量，由 WBIC 在摩擦锥内修正。
  const auto & estimate = *this->_data->state_estimate;
  const T support_force =
    this->_data->quadruped->totalMass() * T(9.81) / static_cast<T>(kNumLegs);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    // 接触腿在 WBC 中只使用反力；足端目标取当前世界系落脚点，便于诊断。
    wbc_data_.pFoot_des[leg] = estimate.position_world +
      estimate.rotation_world_from_body *
      (this->_data->quadruped->hipLocation(static_cast<LegId>(leg)) +
      this->_data->leg_controller->datas[leg].p);
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

  const Mat3<T> rotation_body_from_world =
    estimate.rotation_world_from_body.transpose();
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    auto & command = this->_data->leg_controller->commands[leg];
    command.foot_position_desired = rotation_body_from_world *
      (wbc_data_.pFoot_des[leg] - estimate.position_world) -
      this->_data->quadruped->hipLocation(static_cast<LegId>(leg));
    command.foot_velocity_desired = rotation_body_from_world * wbc_data_.vFoot_des[leg];
    command.kp_cartesian = Mat3<T>::Zero();
    command.kd_cartesian = Mat3<T>::Identity() * T(7);
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

  // 四足接触（12 维约束）加机身 6 维任务已唯一确定构型，KinWBC 不存在
  // 零空间漂移，因此与 MIT 一致直接使用 WBC 的 q_des/qd_des 与前馈力矩。
  ++posture_ramp_iteration_;
  if (desired.mode != ControlMode::WalkRl &&
    !this->_data->rl_entry_posture_active)
  {
    return;
  }

  // RL 入场单独处理：RL 的第一帧历史必须建立在训练默认姿态上，因此把关节
  // 参考从入场姿态平滑过渡到 RL 默认姿态；WBIC 前馈与反力仍保留。
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const LegId leg_id = static_cast<LegId>(leg);
    auto & command = this->_data->leg_controller->commands[leg];
    const auto & leg_model = this->_data->quadruped->leg(leg_id);
    Vec3<T> home;
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      home[static_cast<Eigen::Index>(joint)] =
        static_cast<T>(kDm1RlDefaultJointPosition[
          leg * kJointsPerLeg + joint]);
    }
    const T posture_alpha = this->_data->control_parameters->start_in_prone_home ?
      std::clamp(static_cast<T>(posture_ramp_iteration_) / T(2000), T(0), T(1)) :
      T(1);
    const T smooth_alpha = posture_alpha * posture_alpha *
      (T(3) - T(2) * posture_alpha);
    command.position_desired = entry_joint_positions_[leg] +
      smooth_alpha * (home - entry_joint_positions_[leg]);
    command.position_desired = command.position_desired.cwiseMax(
      leg_model.joints.lower_limit).cwiseMin(leg_model.joints.upper_limit);
    command.velocity_desired.setZero();
    command.kp_joint = this->_data->control_parameters->joint_kp;
    command.kd_joint = this->_data->control_parameters->joint_kd;
  }
}

template class FSM_State_BalanceStand<float>;
