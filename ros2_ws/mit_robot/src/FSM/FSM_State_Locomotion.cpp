// 行走状态的数据流：状态估计 -> MPC 接触力规划 -> WBC 关节力矩。
#include "FSM/FSM_State_Locomotion.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>
#include <stdexcept>

#include "model/floating_base_model_factory.hpp"
#include "Utilities/orientation_tools.h"

template<typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(
  ControlFSMData<T> * control_fsm_data)
: FSM_State<T>(control_fsm_data, FSM_StateName::LOCOMOTION, "LOCOMOTION")
{
  //检查控制系统数据是否有效。
  if (control_fsm_data == nullptr || !control_fsm_data->valid()) {
    throw std::invalid_argument("locomotion state requires valid FSM data");
  }
  // MIT 时序：MPC 每段求解一次，预测接触表每段 dtMPC，10 段构成 TROT 周期；
  // GaitScheduler 的 TROT 周期由 ControlFSM 按同一参数设置。
  const std::size_t mpc_interval = mpcSegmentIterations(
    *control_fsm_data->control_parameters, control_fsm_data->control_time_step);
  mpc::SolverSettings<T> solver_settings;
  solver_settings.time_step =
    control_fsm_data->control_time_step * static_cast<T>(mpc_interval);
  // 重心校准恢复为零偏移，MPC 使用原机身参考位置。
  solver_settings.com_offset_body.setZero();
  //创建 MPC 控制器。传入:机器人模型；控制周期；MPC 步态时间间隔。  
  mpc_ = std::make_unique<mpc::ConvexMPCLocomotion<T>>(
    *control_fsm_data->quadruped, control_fsm_data->control_time_step,
    mpc_interval, solver_settings);
  //创建 WBC。buildFloatingBaseModel() 会建立一个带浮动基座的机器人动力学模型。
  //浮动基座表示机身不是固定在地面上，而是有：3 个平移自由度 + 3 个旋转自由度
  wbc_ctrl_ = std::make_unique<LocomotionCtrl<T>>(
    model::makeFloatingBaseModel(*control_fsm_data->quadruped));
  // 姿态环保留足够的 roll/pitch 阻尼，同时避免用过高增益放大实机估计噪声。
  const auto & parameters = *control_fsm_data->control_parameters;
  // 两种机型独立设置行走增益、抬脚高度和横向安全边界。DM1 使用较低摆幅
  // 以减小惯性滚转，并按更宽的机械足距放宽边界。
  swing_height_ = parameters.locomotion_swing_height;
  maximum_lateral_foot_offset_ =
    parameters.locomotion_max_lateral_foot_offset;
  auto initial_orientation_kp = parameters.locomotion_body_orientation_kp;
  auto initial_orientation_kd = parameters.locomotion_body_orientation_kd;
  // yaw 只作为转向/落足规划输入，不作为机身平衡反馈。
  initial_orientation_kp.z() = T(0);
  initial_orientation_kd.z() = T(0);
  wbc_ctrl_->setBodyOrientationGains(
    initial_orientation_kp, initial_orientation_kd);
  wbc_ctrl_->setFootPositionGains(
    parameters.wbc_foot_position_kp, parameters.wbc_foot_position_kd);
  wbc_ctrl_->setJointGains(
    parameters.joint_kp, parameters.joint_kd);
  wbc_ctrl_->setFloatingBaseWeight(parameters.locomotion_floating_base_weight);
  wbc_ctrl_->setReactionForceWeight(parameters.locomotion_reaction_force_weight);
  wbc_ctrl_->setMaxNormalForce(parameters.maximum_normal_force);
  //安全检查
  this->turnOnAllSafetyChecks();
  this->checkPDesFoot = false;
}

/*仅在第一次启动，后续不会启动*/
template<typename T>
void FSM_State_Locomotion<T>::onEnter()
{
  this->_data->stand_handoff.pending = false;
  this->nextStateName = this->stateName;
  this->transitionData.zero();
  /*初始化 MPC 内部状态。包括：MPC 迭代器；步态相位；
    目标位置；约束和缓存。*/
  mpc_->initialize();
  resetSwingTrajectories();
  safety_fallback_transition_ = false;
  entry_transition_time_ = T(0);
  gait_applied_ = false;
  // 保留 0.2 s 的步态启动保持，使按键动作从完整支撑相开始；不再插值
  // 关节目标、增益或前馈，进入 Locomotion 后直接使用 MIT WBC 输出。
  entry_transition_active_ = true;
  /*步态由 ControlFSMData::locomotion_gait 选择；普通行走统一使用 MIT TROT。*/
  applyRequestedGait(true);
}

/*把请求的步态同步给 GaitScheduler。*/
template<typename T>
void FSM_State_Locomotion<T>::applyRequestedGait(bool force)
{
  const GaitType requested_input = this->_data->locomotion_gait;
  const GaitType requested = entry_transition_active_ ?
    GaitType::STAND : requested_input;
  if (!force && gait_applied_ && requested == applied_gait_) {return;}
  const bool switching = gait_applied_ && requested != applied_gait_;
  if (switching) {mpc_->initialize();}
  if (force || switching) {
    mpc::NormalForceBounds<T> normal;
    normal.minimum.fill(mpc_->solverSettings().minimum_normal_force);
    normal.maximum.fill(mpc_->solverSettings().maximum_normal_force);
    mpc_->setNormalForceBounds(normal);
    std::array<T, kNumLegs> wbc_min{};
        std::array<T, kNumLegs> wbc_max{};
    wbc_max.fill(this->_data->control_parameters->maximum_normal_force);
    wbc_ctrl_->setNormalForceBounds(wbc_min, wbc_max);
  }
  applied_gait_ = requested;
  gait_applied_ = true;
  this->checkSafeOrientation = true;
  wbc_ctrl_->setFloatingBaseWeight(T(100));
  // yaw 只用于转向命令和落足几何，完全不通过 WBC 姿态误差施加平衡力矩。
  auto orientation_kp =
    this->_data->control_parameters->locomotion_body_orientation_kp;
  auto orientation_kd =
    this->_data->control_parameters->locomotion_body_orientation_kd;
  orientation_kp.z() = T(0);
  orientation_kd.z() = T(0);
  wbc_ctrl_->setBodyOrientationGains(orientation_kp, orientation_kd);
  wbc_ctrl_->setBodyPositionGains(
    this->_data->control_parameters->balance_body_position_kp,
    this->_data->control_parameters->balance_body_position_kd);
  this->_data->gait_scheduler->requestGait(requested);
  this->_data->gait_scheduler->modifyGait();
}

/*每个控制周期直接执行一次完整行走控制。*/
template<typename T>
void FSM_State_Locomotion<T>::run()
{
  LocomotionControlStep();
}
/*将用户要求的前进速度传给 MPC。
  MPC 会利用该速度计算足端落点。*/
template<typename T>
void FSM_State_Locomotion<T>::setForwardVelocity(T velocity)
{
  setVelocityCommand(velocity, T(0), T(0));
}

template<typename T>
void FSM_State_Locomotion<T>::setVelocityCommand(
  T forward_velocity, T lateral_velocity, T yaw_rate)
{
  mpc_->setVelocityCommand(forward_velocity, lateral_velocity, yaw_rate);
}

/*checkTransition()每次检查状态切换时，迭代计数加一。*/
template<typename T>
FSM_StateName FSM_State_Locomotion<T>::checkTransition()
{
  ++iteration_;
  safety_fallback_transition_ = false;
  if (!locomotionSafe()) {
    // 安全回退必须同步清除 Locomotion 期望，否则外层下一周期仍看到旧模式，
    // 会把 BalanceStand 立即拉回 Locomotion，形成 500 Hz 状态振荡。
    this->_data->desired_state->mode = ControlMode::BalanceStand;
    safety_fallback_transition_ = true;
    this->nextStateName = FSM_StateName::BALANCE_STAND;
    this->transitionData.done = false;
    this->transitionDuration = T(0);
    return this->nextStateName;
  }

  constexpr unsigned int kTouchdownWaitTimeoutFrames = 50;
  if (std::any_of(touchdown_wait_frames_.begin(), touchdown_wait_frames_.end(),
      [](unsigned int frames) {return frames >= kTouchdownWaitTimeoutFrames;}))
  {
    std::fprintf(stderr,
      "[FSM][LOCOMOTION][TOUCHDOWN] confirmation timeout; safe return\n");
    this->_data->desired_state->mode = ControlMode::BalanceStand;
    safety_fallback_transition_ = true;
    this->nextStateName = FSM_StateName::BALANCE_STAND;
    this->transitionData.done = false;
    this->transitionDuration = T(0);
    return this->nextStateName;
  }

  // 一次对角踏步落地后回站；外层桥接会清除锁存运动，等待下一次按键。
  if (!entry_transition_active_ &&
    (applied_gait_ == GaitType::TWO_LEG_STAND ||
    applied_gait_ == GaitType::TWO_LEG_STAND_REVERSE) &&
    this->_data->locomotion_gait == applied_gait_ &&
    this->_data->gait_scheduler->gait_data.initial_phase >= T(1) &&
    std::none_of(touchdown_waiting_.begin(), touchdown_waiting_.end(),
      [](bool waiting) {return waiting;}) &&
    this->_data->desired_state->mode == ControlMode::Locomotion)
  {
    this->_data->desired_state->mode = ControlMode::BalanceStand;
  }
  const ControlMode requested_mode = this->_data->desired_state->mode;
  this->nextStateName = this->stateForMode(requested_mode);
  this->transitionDuration = T(0);
  return this->nextStateName;
}
/*普通请求回到 BalanceStand 时再执行一次控制周期，保持切换帧命令连续；
  安全回退沿用上一帧有效命令，不重复执行已经判定不安全的行走控制。*/
template<typename T>
TransitionData<T> FSM_State_Locomotion<T>::transition()
{
  auto & handoff = this->_data->stand_handoff;
  handoff.pending = false;
  if (this->nextStateName == FSM_StateName::BALANCE_STAND &&
    !safety_fallback_transition_)
  {
    run();
    if (this->_data->use_wbc && this->_data->leg_controller->legsEnabled()) {
      handoff.height = wbc_data_.pBody_des.z();
      handoff.locomotion_commands = this->_data->leg_controller->commands;
      handoff.blend_frames = 2;
      handoff.blend_pending = true;
      handoff.pending = true;
    }
  }
  if (this->nextStateName == FSM_StateName::PASSIVE) {
    this->turnOffAllSafetyChecks();
  }
  this->transitionData.done = true;
  return this->transitionData;
}

/*  检查机器人是否适合继续走。读取状态估计器结果：
    机身位置；机身速度；姿态；角速度。*/
template<typename T>
bool FSM_State_Locomotion<T>::locomotionSafe() const
{
  // 行走安全条件比普通 FSM 姿态检查更严格，并同时限制足端位置与速度。
  const StateEstimate<T> & estimate = *this->_data->state_estimate;
  /* 参数四： 俯仰角 翻滚的安全上限了*/
  constexpr T max_roll_degrees = T(40);
  constexpr T max_pitch_degrees = T(40);
  if (!estimate.valid) {
    std::fprintf(stderr,
      "[FSM][LOCOMOTION] 安全回站：状态估计无效。\n");
    return false;
  }
  const T max_roll = ori::deg2rad(max_roll_degrees);
  const T max_pitch = ori::deg2rad(max_pitch_degrees);
  if (std::abs(estimate.rpy.x()) > max_roll ||
    std::abs(estimate.rpy.y()) > max_pitch)
  {
    std::fprintf(stderr,
      "[FSM][LOCOMOTION] 安全回站：姿态越界 roll=%.3f pitch=%.3f rad。\n",
      static_cast<double>(estimate.rpy.x()),
      static_cast<double>(estimate.rpy.y()));
    return false;
  }
  
  // DM1 的 HAA 到足端本身已有横向偏置，边界按模型参数配置。
  /*abs(data.p[1]) 超过机型横向安全边界时拒绝继续行走。
    data.v.norm() > 9  足端速度不能超过 9 m/s。如果超过，通常表示：*/
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & data = this->_data->leg_controller->datas[leg];
    if (!data.valid) {
      std::fprintf(stderr,
        "[FSM][LOCOMOTION] 安全回站：leg=%zu 反馈无效。\n", leg);
      return false;
    }
    const T foot_speed = data.v.norm();
    if (data.p.z() > T(0) ||
      std::abs(data.p.y()) > maximum_lateral_foot_offset_ + T(0.01) || foot_speed > T(9))
    {
      std::fprintf(stderr,
        "[FSM][LOCOMOTION] 安全回站：leg=%zu p=(%.4f,%.4f,%.4f)m "
        "v_norm=%.3fm/s q=(%.4f,%.4f,%.4f)rad；边界 p_z<=0, "
        "|p_y|<=%.3f, v<=9。\n",
        leg, static_cast<double>(data.p.x()),
        static_cast<double>(data.p.y()), static_cast<double>(data.p.z()),
        static_cast<double>(foot_speed), static_cast<double>(data.q.x()),
        static_cast<double>(data.q.y()), static_cast<double>(data.q.z()),
        static_cast<double>(maximum_lateral_foot_offset_));
      return false;
    }
  }
  return true;
}

template<typename T>
std::array<Vec3<T>, kNumLegs>
FSM_State_Locomotion<T>::footPositionsWorld() const
{
  std::array<Vec3<T>, kNumLegs> positions{};
  /*读取机身世界位置和姿态。*/
  const auto & estimate = *this->_data->state_estimate;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    /*遍历四条腿，并将整数转换成 LegId 枚举。*/
    const LegId leg_id = static_cast<LegId>(leg);
    // 腿传感器给出足端相对髋的位置：先平移到机身系，再旋转和平移到世界系。
    const Vec3<T> foot_body = this->_data->quadruped->hipLocation(leg_id) +
     this->_data->leg_controller->datas[leg].p;
     /*足端相对机身位置=髋关节相对机身位置+足端相对髋关节位置*/
    positions[leg] = estimate.position_world +
      estimate.rotation_world_from_body * foot_body;
  }
  return positions;
}

/* 将四条腿全部标记为没有开始摆动。清除每条腿轨迹内部的：
   起点；终点；高度；时间；相位。*/
template<typename T>
void FSM_State_Locomotion<T>::resetSwingTrajectories() noexcept
{
  swing_active_.fill(false);
  touchdown_waiting_.fill(false);
  touchdown_confirm_frames_.fill(0);
  touchdown_wait_frames_.fill(0);
  touchdown_reported_.fill(false);
  touchdown_confirmed_reported_.fill(false);
  for (auto & target : two_leg_hold_targets_) {target.setZero();}
  for (auto & trajectory : swing_trajectories_) {
    trajectory.reset();
  }
}

template<typename T>
void FSM_State_Locomotion<T>::startSwingTrajectory(
  std::size_t leg, const Vec3<T> & initial_position,
  const mpc::LocomotionResult<T> & locomotion_result)
{
  /*先清除旧轨迹，再锁存当前足端位置作为起点。关键点是：
    initial_position只在离地瞬间记录一次。
    不能每帧用新的实测足端位置覆盖它，否则起点会不断移动。*/
  auto & trajectory = swing_trajectories_.at(leg);
  trajectory.reset();
  trajectory.setInitialPosition(initial_position);
  if (applied_gait_ == GaitType::TWO_LEG_STAND ||
    applied_gait_ == GaitType::TWO_LEG_STAND_REVERSE)
  {
    // 原地抬落一次：复用 TROT 的 Bezier 轨迹，落点固定为本次离地位置。
    trajectory.setFinalPosition(initial_position);
    trajectory.setHeight(swing_height_);
    swing_active_[leg] = true;
    return;
  }
  // 落脚点必须相对当前机身/髋部规划，不能在上一次世界足点上反复累加步长；
  // 否则机身稍有打滑，脚就会越走越远并最终把腿拉直。
  const auto & estimate = *this->_data->state_estimate;
  const LegId leg_id = static_cast<LegId>(leg);
  const T yaw = locomotion_result.command.body_rpy.z();
  Eigen::Matrix<T, 2, 2> yaw_rotation;
  yaw_rotation << std::cos(yaw), -std::sin(yaw),
    std::sin(yaw), std::cos(yaw);
  // DM1 的 Hip 安装点并不是名义落脚点。q_abad=0 时，Hip 横向
  // 连杆仍会把左右足分别向机身外侧推出；若只使用 hipLocation，落脚宽度会
  // 从约 25.35 cm 缩到 9.35 cm，四条腿便会在每次摆动时明显向内收。
  Vec3<T> nominal_foot_from_hip = Vec3<T>::Zero();
  const auto & leg_model = this->_data->quadruped->leg(leg_id);
  computeLegJacobianAndPosition(
    *this->_data->quadruped, leg_model.joints.home_position,
    static_cast<Mat3<T> *>(nullptr), &nominal_foot_from_hip, leg_id);
  const Vec2<T> nominal_foot_body =
    (this->_data->quadruped->hipLocation(leg_id) + nominal_foot_from_hip)
    .template head<2>();

  // 用四条腿 home 姿态正运动学得到的名义足端均值作为支撑中心。
  // 这样保留相对足距，同时消除 DM1 几何造成的固定前后偏置。
  Vec2<T> nominal_foot_center = Vec2<T>::Zero();
  for (std::size_t nominal_leg = 0; nominal_leg < kNumLegs; ++nominal_leg) {
    const LegId nominal_leg_id = static_cast<LegId>(nominal_leg);
    const auto & nominal_leg_model = this->_data->quadruped->leg(nominal_leg_id);
    Vec3<T> nominal_leg_foot_from_hip = Vec3<T>::Zero();
    computeLegJacobianAndPosition(
      *this->_data->quadruped, nominal_leg_model.joints.home_position,
      static_cast<Mat3<T> *>(nullptr), &nominal_leg_foot_from_hip,
      nominal_leg_id);
    nominal_foot_center += (
      this->_data->quadruped->hipLocation(nominal_leg_id) +
      nominal_leg_foot_from_hip).template head<2>();
  }
  nominal_foot_center /= static_cast<T>(kNumLegs);
  const Vec2<T> centered_nominal_foot_body = nominal_foot_body - nominal_foot_center;
  const T stance_time = locomotion_result.gait_frame.stance_time[leg];
  const T swing_time = locomotion_result.gait_frame.swing_time[leg];
  const T swing_phase = std::clamp(
    locomotion_result.gait_frame.swing_phase[leg], T(0), T(1));
  const T swing_time_remaining = swing_time * (T(1) - swing_phase);
  const Vec2<T> measured_velocity = estimate.velocity_world.template head<2>();
  const Vec2<T> desired_velocity =
    locomotion_result.command.body_velocity_world.template head<2>();
  const T yaw_rate = locomotion_result.command.body_angular_velocity.z();
  const Vec2<T> desired_velocity_body = yaw_rotation.transpose() * desired_velocity;
  // MIT interleave correction reduces leg interference during forward trot.
  // DM1 nominal foot geometry remains the source of the absolute foot spacing.
  constexpr std::array<T, kNumLegs> interleave_y{
    T(-0.08), T(0.08), T(0.02), T(-0.02)};
  constexpr T interleave_gain = T(-0.2);
  Vec2<T> interleaved_foot_body = centered_nominal_foot_body;
  interleaved_foot_body.y() +=
    interleave_y[leg] * std::abs(desired_velocity_body.x()) * interleave_gain;
  const T yaw_step = -yaw_rate * stance_time * T(0.5);
  const T yaw_step_cos = std::cos(yaw_step);
  const T yaw_step_sin = std::sin(yaw_step);
  const Vec2<T> yaw_corrected_nominal_body(
    yaw_step_cos * interleaved_foot_body.x() -
    yaw_step_sin * interleaved_foot_body.y(),
    yaw_step_sin * interleaved_foot_body.x() +
    yaw_step_cos * interleaved_foot_body.y());
  const Vec2<T> predicted_foot_body = yaw_corrected_nominal_body +
    desired_velocity_body * swing_time_remaining;
  const Vec2<T> nominal_foot_world =
    estimate.position_world.template head<2>() +
    yaw_rotation * predicted_foot_body;
  // MIT Raibert 落足：半个支撑期速度前馈、0.03 捕获项和偏航离心项。
  // 目标按世界坐标计算，每次摆动开始时锁存一次，避免足点随反馈累加漂移。
  Vec2<T> step = Vec2<T>::Zero();
  step.x() = measured_velocity.x() * T(0.5) * stance_time +
    T(0.03) * (measured_velocity.x() - desired_velocity.x()) +
    T(0.5) * estimate.position_world.z() / T(9.81) *
    (measured_velocity.y() * yaw_rate);
  step.y() = measured_velocity.y() * T(0.5) * stance_time +
    T(0.03) * (measured_velocity.y() - desired_velocity.y()) -
    T(0.5) * estimate.position_world.z() / T(9.81) *
    (measured_velocity.x() * yaw_rate);
  step.x() = std::clamp(step.x(), -maximum_step_length_, maximum_step_length_);
  step.y() = std::clamp(step.y(), -maximum_step_length_, maximum_step_length_);
  Vec3<T> landing_position = initial_position;
  landing_position.template head<2>() = nominal_foot_world + step;
  landing_position.z() = initial_position.z();
  trajectory.setFinalPosition(landing_position);
  trajectory.setHeight(swing_height_);
  swing_active_[leg] = true;
}

template<typename T>
void FSM_State_Locomotion<T>::constrainJointTargets()
{
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto leg_id = static_cast<LegId>(leg);
    const auto & joints = this->_data->quadruped->leg(leg_id).joints;
    auto & command = this->_data->leg_controller->commands[leg];

    command.velocity_desired = command.velocity_desired.cwiseMin(
      joints.velocity_limit).cwiseMax(-joints.velocity_limit);
    command.position_desired = command.position_desired.cwiseMax(
      joints.lower_limit).cwiseMin(joints.upper_limit);

  }
}

template<typename T>
void FSM_State_Locomotion<T>::LocomotionControlStep()
{
  // 每帧求解经典 MPC/WBC；入口输出平滑交接到本周期的新控制结果。
  applyRequestedGait(false);
  if (applied_gait_ == GaitType::STAND) {
    mpc_->setVelocityCommand(T(0), T(0), T(0));
  }
  // MPC 输入必须使用统一的世界坐标系，否则反作用力方向会与 WBC 不一致。
  const auto feet_world = footPositionsWorld();
  mpc_->setPositionOffsetWorld(Vec2<T>::Zero());
  /*MPC 输入：当前状态估计；用户期望状态；当前四个足端的世界坐标。
    MPC 输出 LocomotionResult，包括：command:机身期望状态；
    reaction_forces_world：各腿期望地面反作用力；
    contact_state：当前是否接触；,swing_phase：摆动进度，通常范围为 [0, 1]
    ,swing_time：摆动总时间,stance_time,valid：支撑总时间。*/
  if (!this->_data->gait_frame.valid(mpc_->solverSettings().horizon)) {
    this->_data->gait_frame = this->_data->gait_scheduler->gaitFrame(
      mpc_->solverSettings().horizon);
  }
  const auto & gait_frame = this->_data->gait_frame;
  const auto result = mpc_->run(
    *this->_data->state_estimate, *this->_data->desired_state, feet_world,
    gait_frame);
  if (!result.valid) {
    std::fprintf(stderr,
      "[FSM][LOCOMOTION] 失能前判断：MPC result.valid=false。\n");
    this->_data->leg_controller->zeroCommand();
    this->_data->leg_controller->setEnabled(false);
    return;
  }
  //设置 WBC 身体目标
  const auto & desired = result.command;
  wbc_data_.pBody_des = desired.body_position_world;
  wbc_data_.vBody_des = desired.body_velocity_world;
  wbc_data_.aBody_des = desired.body_acceleration_world;
  wbc_data_.pBody_RPY_des = desired.body_rpy;
  wbc_data_.vBody_Ori_des = desired.body_angular_velocity;
  //逐腿处理支撑或摆动逻辑。
  auto contact_state = result.gait_frame.contact_state;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    //将 MPC 计算出的世界坐标系反作用力传给 WBC。
    wbc_data_.Fr_des[leg] = result.reaction_forces_world[leg];
    //将布尔接触状态转成浮点数：
    wbc_data_.contact_state[leg] = contact_state[leg] ? T(1) : T(0);

    const bool single_step_gait =
      applied_gait_ == GaitType::TWO_LEG_STAND ||
      applied_gait_ == GaitType::TWO_LEG_STAND_REVERSE;
    const bool landing_candidate =
      single_step_gait && contact_state[leg] && swing_active_[leg];
    const bool touchdown_ready =
      !landing_candidate ||
      std::abs(this->_data->leg_controller->datas[leg].v.z()) <= T(0.20);
    const T nominal_support = this->_data->quadruped->totalMass() * T(9.81) /
      static_cast<T>(kNumLegs);
    const bool force_confirmed =
      !this->_data->measured_contact_valid[leg] ||
      (this->_data->measured_contact_probability[leg] >= T(0.5) &&
      this->_data->measured_normal_force[leg] >= T(0.15) * nominal_support);
    if (landing_candidate && touchdown_ready && force_confirmed) {
      ++touchdown_confirm_frames_[leg];
    } else if (landing_candidate) {
      touchdown_confirm_frames_[leg] = 0;
    } else {
      touchdown_confirm_frames_[leg] = 0;
    }
    const bool touchdown_confirmed =
      touchdown_confirm_frames_[leg] >= 3;
    if (landing_candidate && !touchdown_confirmed) {
      ++touchdown_wait_frames_[leg];
    } else {
      touchdown_wait_frames_[leg] = 0;
    }
    if (landing_candidate && !touchdown_reported_[leg]) {
      std::fprintf(stderr,
        "[FSM][LOCOMOTION][TOUCHDOWN] leg=%zu planned=1 waiting=%d "
        "p_world=[%.3f %.3f %.3f] v_body_norm=%.3f vz=%.3f "
        "prob=%.3f normal_force=%.2fN\n", leg, !touchdown_confirmed,
        static_cast<double>(feet_world[leg].x()),
        static_cast<double>(feet_world[leg].y()),
        static_cast<double>(feet_world[leg].z()),
        static_cast<double>(this->_data->leg_controller->datas[leg].v.norm()),
        static_cast<double>(this->_data->leg_controller->datas[leg].v.z()),
        static_cast<double>(this->_data->measured_contact_probability[leg]),
        static_cast<double>(this->_data->measured_normal_force[leg]));
      touchdown_reported_[leg] = true;
    }
    if (landing_candidate && touchdown_confirmed &&
      !touchdown_confirmed_reported_[leg]) {
      std::fprintf(stderr,
        "[FSM][LOCOMOTION][TOUCHDOWN_CONFIRMED] leg=%zu frames=%u "
        "p_world=[%.3f %.3f %.3f] prob=%.3f normal_force=%.2fN\n", leg,
        touchdown_confirm_frames_[leg], static_cast<double>(feet_world[leg].x()),
        static_cast<double>(feet_world[leg].y()),
        static_cast<double>(feet_world[leg].z()),
        static_cast<double>(this->_data->measured_contact_probability[leg]),
        static_cast<double>(this->_data->measured_normal_force[leg]));
      touchdown_confirmed_reported_[leg] = true;
    }
    if (!landing_candidate) {
      touchdown_reported_[leg] = false;
      touchdown_confirmed_reported_[leg] = false;
    }
    touchdown_waiting_[leg] = landing_candidate &&
      (!touchdown_ready || !touchdown_confirmed);
    if (!touchdown_ready) {
      wbc_data_.contact_state[leg] = T(0);
    }
    if (contact_state[leg] && (!landing_candidate || touchdown_confirmed)) {
      // 支撑时足端位置任务不会加入 WBC；同步当前值仅用于诊断，并清除上一摆动态。
      swing_active_[leg] = false;
      wbc_data_.pFoot_des[leg] = feet_world[leg];
      wbc_data_.vFoot_des[leg].setZero();
      wbc_data_.aFoot_des[leg].setZero();
      continue;
    }

    // 只在离地沿锁存起点和落脚点。后续周期即使实测足端受到扰动，目标轨迹
    // 也只由锁存数据和 swing_phase 推进，不会把扰动累积成新的目标高度。
    if (!swing_active_[leg]) {
      startSwingTrajectory(leg, feet_world[leg], result);
    }
    auto & trajectory = swing_trajectories_[leg];
    /*Bezier 轨迹会生成平滑的：足端位置；足端速度；足端加速度。*/
    trajectory.computeSwingTrajectoryBezier(
      result.gait_frame.swing_phase[leg], result.gait_frame.swing_time[leg]);
    wbc_data_.pFoot_des[leg] = trajectory.getPosition();
    wbc_data_.vFoot_des[leg] = trajectory.getVelocity();
    wbc_data_.aFoot_des[leg] = trajectory.getAcceleration();
  }

  if (this->_data->use_wbc) {
    // WBC和最终关节PD均保持500 Hz。测试确认WBC降频会使支撑力和足端约束
    // 滞后并造成小腿擦地，因此计算削减只放在25 Hz的MPC内部。
    const bool wbc_valid = wbc_ctrl_->runAndApply(
      &wbc_data_, *this->_data->state_estimate, *this->_data->joint_states,
      *this->_data->leg_controller);
    if (!wbc_valid) {
      std::fprintf(stderr,
        "[FSM][LOCOMOTION] 失能前判断：WBC runAndApply 失败。\n");
    }
    if (wbc_valid) {
      // MIT LegController 保留一层足端速度阻尼：支撑腿 Kp=0，摆动腿使用
      // 足端任务的刚度，Kd=7 直接作用在实际足端速度上。
      const auto & estimate = *this->_data->state_estimate;
      const Mat3<T> rotation_body_from_world =
        estimate.rotation_world_from_body.transpose();
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        auto & command = this->_data->leg_controller->commands[leg];
        // MIT lets WBC own the foot position task.  Keep only the outer
        // velocity damping layer after the joint torque result is applied.
        command.foot_position_desired.setZero();
        command.foot_velocity_desired = rotation_body_from_world * wbc_data_.vFoot_des[leg];
        command.kp_cartesian = Mat3<T>::Zero();
        command.kd_cartesian = Mat3<T>::Identity() *
          this->_data->control_parameters->wbc_foot_velocity_damping;
      }
      // 行走层只负责自身的目标范围；所有状态共用的连续力矩约束由
      // LegController 最终命令出口统一执行。
      constrainJointTargets();
    }
  } else {
    // 关闭 WBC 时的降级路径只发送足端前馈力，主要用于调试算法分层。
    auto & controller = *this->_data->leg_controller;
    controller.zeroCommand();
    controller.setEnabled(true);
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      controller.commands[leg].force_feedforward =
        this->_data->state_estimate->rotation_world_from_body.transpose() *
        result.reaction_forces_world[leg];
    }
  }
  if (entry_transition_active_) {
    entry_transition_time_ += this->_data->control_time_step;
    if (entry_transition_time_ >= T(0.2)) {entry_transition_active_ = false;}
  }
}

template<typename T>
void FSM_State_Locomotion<T>::onExit()
{
  iteration_ = 0;
  resetSwingTrajectories();
}

template class FSM_State_Locomotion<float>;
