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

namespace
{
constexpr std::size_t kStableFrameCount = 50;
}

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
  const std::size_t gait_segment_interval = mpc_interval;
  mpc::SolverSettings<T> solver_settings;
  solver_settings.time_step =
    control_fsm_data->control_time_step * static_cast<T>(gait_segment_interval);
  //创建 MPC 控制器。传入:机器人模型；控制周期；MPC 步态时间间隔。  
  mpc_ = std::make_unique<mpc::ConvexMPCLocomotion<T>>(
    *control_fsm_data->quadruped, control_fsm_data->control_time_step,
    mpc_interval, solver_settings, gait_segment_interval);
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
  wbc_ctrl_->setBodyOrientationGains(
    parameters.locomotion_body_orientation_kp,
    parameters.locomotion_body_orientation_kd);
  wbc_ctrl_->setFootPositionGains(
    Vec3<T>::Constant(T(500)), Vec3<T>::Constant(T(10)));
  wbc_ctrl_->setJointGains(
    parameters.joint_kp, parameters.joint_kd);
  wbc_ctrl_->setFloatingBaseWeight(T(0.1));
  wbc_ctrl_->setReactionForceWeight(T(1));
  wbc_ctrl_->setMaxNormalForce(parameters.maximum_normal_force);
  com_model_ = std::make_unique<FloatingBaseModel<T>>(
    model::makeFloatingBaseModel(*control_fsm_data->quadruped));
  //安全检查
  this->turnOnAllSafetyChecks();
  this->checkPDesFoot = false;
}

/*仅在第一次启动，后续不会启动*/
template<typename T>
void FSM_State_Locomotion<T>::onEnter()
{
  this->nextStateName = this->stateName;
  this->transitionData.zero();
  /*初始化 MPC 内部状态。包括：MPC 迭代器；步态相位；
    目标位置；约束和缓存。*/
  mpc_->initialize();
  resetSwingTrajectories();
  active_mode_ = this->_data->desired_state->mode;
  rl_history_.fill(0.0F);
  rl_history_initialized_ = false;
  rl_previous_action_.fill(0.0F);
  rl_policy_counter_ = 0;
  safety_fallback_transition_ = false;
  entry_transition_time_ = T(0);
  load_transfer_elapsed_ = T(0);
  load_transfer_bucket_ = static_cast<std::size_t>(-1);
  stage1_stable_frames_ = 0;
  load_transfer_ready_frames_ = 0;
  load_transfer_gate_open_ = false;
  cycle_stall_frames_ = 0;
  cycle_unload_logged_ = false;
  cycle_shift_ready_ = false;
  gait_applied_ = false;
  single_leg_cycle_index_ = 0;
  // 保留 0.2 s 的步态启动保持，使按键动作从完整支撑相开始；不再插值
  // 关节目标、增益或前馈，进入 Locomotion 后直接使用 MIT WBC 输出。
  entry_transition_active_ = !isRlMode();
  /*步态由 ControlFSMData::locomotion_gait 选择；3 号入口请求 STAND
    做四脚接触诊断，踏步和方向运动统一使用 MIT TROT（50% 支撑，0.26 s 周期）。
    MPC 与 GaitScheduler 必须同时设置：只改一个就会出现"MPC 认为后腿支撑、
    GaitScheduler 认为后腿摆动"的错相，导致 WBC 约束错误、足端轨迹漂移。*/
  applyRequestedGait(true);
}

/*把请求的步态同步给 MPC 接触表和 GaitScheduler，并把两者的相位对齐。*/
template<typename T>
void FSM_State_Locomotion<T>::applyRequestedGait(bool force)
{
  // 承重交接时四脚保持支撑，避免旧站立命令作用于已被规划为摆动的腿。
  const GaitType requested_input = this->_data->locomotion_gait;
  const bool cycle_input = requested_input == GaitType::STAND_CYCLE ||
    requested_input == GaitType::THREE_FOOT;
  if (requested_input != GaitType::STAND && !cycle_input)
  {
    stage1_stable_frames_ = 0;
  }
  const bool transfer_ready = stage1_stable_frames_ >= kStableFrameCount;
  const GaitType requested = entry_transition_active_ ||
    (cycle_input && !transfer_ready) ?
    GaitType::STAND : requested_input;
  if (!force && gait_applied_ && requested == applied_gait_) {return;}
  if (requested == GaitType::THREE_FOOT &&
    (!gait_applied_ || applied_gait_ != GaitType::THREE_FOOT))
  {
    single_leg_cycle_index_ = 0;
  }
  // 中途换步态：MPC 的相位由内部周期计数换算，而 GaitScheduler 换步态时会把
  // 相位重置到各自的 phase_offset。这里把 MPC 一起归零，两者才不会错相。
  const bool switching = gait_applied_ && requested != applied_gait_;
  // 强制开始下一次 4/5 号抬落时也要重置 MPC 的位置锚点；否则上一条腿
  // 的机身微小漂移会叠加到下一周期，最终让后腿动作越过支撑三角形。
  if (switching || (force && cycle_input)) {mpc_->initialize();}
  if (force || switching) {
    load_transfer_elapsed_ = T(0);
    load_transfer_bucket_ = static_cast<std::size_t>(-1);
    load_transfer_ready_frames_ = 0;
    load_transfer_gate_open_ = false;
    cycle_stall_frames_ = 0;
    cycle_unload_logged_ = false;
    cycle_shift_ready_ = false;
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
  // STAND 是 3 号四脚诊断入口；STAND_CYCLE 是 4 号换载；THREE_FOOT 是 5 号单腿循环。
  // 三者都直接传给 MPC 和 GaitScheduler，不映射到 TROT。
  const GaitType effective = (requested == GaitType::STAND ||
    requested == GaitType::STAND_CYCLE ||
    requested == GaitType::THREE_FOOT) ? requested : GaitType::TROT;
  const bool is_stand_like = requested == GaitType::STAND_CYCLE ||
    requested == GaitType::THREE_FOOT;
  this->checkSafeOrientation = !is_stand_like;
  wbc_ctrl_->setFloatingBaseWeight(is_stand_like ? T(100) : T(100));
  // 零速度指令：STAND / STAND_CYCLE / THREE_FOOT 都不需要平移速度。
  if (effective == GaitType::STAND || effective == GaitType::STAND_CYCLE ||
    effective == GaitType::THREE_FOOT) {
    mpc_->setVelocityCommand(T(0), T(0), T(0));
  }
  wbc_ctrl_->setBodyOrientationGains(
    this->_data->control_parameters->locomotion_body_orientation_kp,
    this->_data->control_parameters->locomotion_body_orientation_kd);
  wbc_ctrl_->setBodyPositionGains(
    this->_data->control_parameters->balance_body_position_kp,
    this->_data->control_parameters->balance_body_position_kd);
  // 5 号单腿抬落复用 4 号的 STAND_CYCLE 时序，每次只摆动一条腿。
  if (requested == GaitType::STAND_CYCLE || requested == GaitType::THREE_FOOT) {
    std::array<bool, kNumLegs> swing{false, false, false, false};
    if (requested == GaitType::THREE_FOOT) {
      swing[single_leg_cycle_index_] = true;
    } else {
      swing = {false, true, true, false};
    }
    if (swing != cycle_swing_) {cycle_learned_correction_.setZero();}
    cycle_swing_ = swing;
    mpc_->setStandCycleSwingLegs(swing);
    this->_data->gait_scheduler->setStandCycleSwingLegs(swing);
  }
  const GaitType planner_gait =
    effective == GaitType::THREE_FOOT ? GaitType::STAND_CYCLE : effective;
  mpc_->setGait(planner_gait);
  this->_data->gait_scheduler->requestGait(planner_gait);
  this->_data->gait_scheduler->modifyGait();
  if (requested_input == GaitType::STAND) {
    std::fprintf(stderr,
      "[FSM][MARCH-DIAGNOSTIC] stage=1 zero_velocity=true planned_contacts=4 swing=false；"
      "即按键 4 换载前的四脚站稳阶段，不自动换载。\n");
  }
}

/*每个控制周期直接执行一次完整行走控制。*/
template<typename T>
void FSM_State_Locomotion<T>::run()
{
  if (isRlMode()) {RlControlStep();} else {LocomotionControlStep();}
}
/*将用户要求的前进速度传给 MPC。
  MPC 会利用该速度计算足端落点。*/
template<typename T>
void FSM_State_Locomotion<T>::setForwardVelocity(T velocity)
{
  setVelocityCommand(velocity, rl_velocity_command_.y(), rl_velocity_command_.z());
}

template<typename T>
void FSM_State_Locomotion<T>::setVelocityCommand(
  T forward_velocity, T lateral_velocity, T yaw_rate)
{
  rl_velocity_command_ << forward_velocity, lateral_velocity, yaw_rate;
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

  const ControlMode requested_mode = this->_data->desired_state->mode;
  this->nextStateName = this->stateForMode(requested_mode);
  this->transitionDuration = T(0);
  if (this->nextStateName == FSM_StateName::LOCOMOTION && requested_mode != active_mode_) {
    // 控制器之间禁止热切换：先回到 BalanceStand，下一次进入本状态
    // 时 onEnter() 才会建立新的策略历史或 MPC 相位。
    this->_data->desired_state->mode = ControlMode::BalanceStand;
    this->nextStateName = FSM_StateName::BALANCE_STAND;
  }
  return this->nextStateName;
}
/*普通请求回到 BalanceStand 时再执行一次控制周期，保持切换帧命令连续；
  安全回退沿用上一帧有效命令，不重复执行已经判定不安全的行走控制。*/
template<typename T>
TransitionData<T> FSM_State_Locomotion<T>::transition()
{
  if (this->nextStateName == FSM_StateName::BALANCE_STAND &&
    !safety_fallback_transition_)
  {
    run();
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
  // 4 号一次抬落关闭了通用姿态检查，这里保留 0.3 rad 的宽松上限：
  // 质心已移到支撑线上，正常抬落远小于该值，超过即说明正在绕对角线倾倒。
  const bool stand_cycle = isStandCycle();
  const T max_roll = stand_cycle ? T(0.3) : ori::deg2rad(max_roll_degrees);
  const T max_pitch = stand_cycle ? T(0.3) : ori::deg2rad(max_pitch_degrees);
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
    if (isStandCycle()) {continue;}
    const T foot_speed = data.v.norm();
    if (data.p.z() > T(0) ||
      std::abs(data.p.y()) > maximum_lateral_foot_offset_ || foot_speed > T(9))
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

/* MPC 把质心视为机身原点；DM1 四条腿约占整机一半质量，真实质心与机身原点
   有明显水平偏差，对角两脚支撑时必须按全身质心判断是否落在支撑线上。*/
template<typename T>
Vec3<T> FSM_State_Locomotion<T>::wholeBodyComWorld()
{
  auto state = model::makeFloatingBaseState(
    *this->_data->state_estimate, *this->_data->joint_states);
  state.q += com_model_->getJointPositionOffsets();
  com_model_->setState(state);
  const auto & inertias = com_model_->getBodyInertiaVector();
  T mass = T(0);
  Vec3<T> moment = Vec3<T>::Zero();
  for (std::size_t body = 5; body < com_model_->getNumDof(); ++body) {
    const T body_mass = inertias[body].getMass();
    if (body_mass <= T(0)) {continue;}
    moment += body_mass * com_model_->getPosition(
      static_cast<int>(body), inertias[body].getCOM());
    mass += body_mass;
  }
  return moment / mass;
}

/* 4 号一次抬落只用 FR+RL 两脚支撑，质心不在两脚连线上就会绕对角线倾倒，
   表现为机身掉高并向抬起腿一侧滚转/俯仰。起始时锁存一次：把质心垂直投影到
   FR-RL 连线所需的水平位移；之后由 integrateStandCycleShift() 按实测偏差修正，
   整个抬落期间保持，恢复承重后再平滑撤回。*/
template<typename T>
void FSM_State_Locomotion<T>::prepareStandCycleShift(
  const std::array<Vec3<T>, kNumLegs> & feet_world)
{
  cycle_shift_ready_ = true;
  cycle_support_shift_world_.setZero();
  // 4 号开始时四脚都着地，记录足端高度作为 FL/RR 的落脚高度。
  load_transfer_foot_position_ = feet_world;
  cycle_touched_.fill(false);
  cycle_reaching_.fill(false);
  cycle_reach_frames_ = 0;
  cycle_initial_shift_world_.setZero();
  cycle_hold_latched_ = false;
  cycle_gate_wait_frames_ = 0;
  cycle_tipping_ = false;
  cycle_had_tip_ = false;
  cycle_aborted_ = false;
  cycle_tip_count_ = 0;
  cycle_start_rpy_ = this->_data->state_estimate->rpy;
  const auto & estimate = *this->_data->state_estimate;
  Vec3<T> com_world = Vec3<T>::Zero();
  try {
    com_world = wholeBodyComWorld();
  } catch (const std::exception & error) {
    std::fprintf(stderr,
      "[FSM][STAND_CYCLE] 质心计算失败，不平移机身：%s\n", error.what());
    return;
  }
  // 质心随机身刚性平移：先换算到机身处于零速锚点时的质心位置。
  const Vec2<T> anchor = mpc_->positionAnchorWorld(estimate).template head<2>();
  cycle_com_shift_world_ =
    com_world.template head<2>() - estimate.position_world.template head<2>();
  const Vec2<T> com_at_anchor = anchor + cycle_com_shift_world_;
  const Vec2<T> target = cycleSupportTarget(com_at_anchor, feet_world);
  if (!com_at_anchor.allFinite() || !target.allFinite()) {return;}
  Vec2<T> shift = target - com_at_anchor;
  constexpr T kMaximumShift = T(0.04);
  if (shift.norm() > kMaximumShift) {shift *= kMaximumShift / shift.norm();}
  cycle_initial_shift_world_ = shift;
  cycle_support_shift_world_ = shift + cycle_learned_correction_;
  if (cycle_support_shift_world_.norm() > T(0.045)) {
    cycle_support_shift_world_ *= T(0.045) / cycle_support_shift_world_.norm();
  }
  std::fprintf(stderr,
    "[FSM][STAND_CYCLE] gait=%d leg=%zu com-body=(%.4f,%.4f)m 支撑线平移=(%.4f,%.4f)m（含已学修正 (%.4f,%.4f)m）\n",
    static_cast<int>(applied_gait_), cycleSwingLegs().empty() ? kNumLegs : cycleSwingLegs().front(),
    static_cast<double>(cycle_com_shift_world_.x()),
    static_cast<double>(cycle_com_shift_world_.y()),
    static_cast<double>(cycle_support_shift_world_.x()),
    static_cast<double>(cycle_support_shift_world_.y()),
    static_cast<double>(cycle_learned_correction_.x()),
    static_cast<double>(cycle_learned_correction_.y()));
}

/* 将四条腿全部标记为没有开始摆动。清除每条腿轨迹内部的：
   起点；终点；高度；时间；相位。*/
template<typename T>
void FSM_State_Locomotion<T>::resetSwingTrajectories() noexcept
{
  swing_active_.fill(false);
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
  if (isStandCycle()) {
    // 落点高度用 4 号开始时四脚着地的足端高度，而不是离地瞬间的高度：
    // 卸载等待期间轻载脚可能已悬空约 1 cm，按离地高度落脚会落在空中。
    Vec3<T> landing = initial_position;
    landing.z() = load_transfer_foot_position_[leg].z();
    trajectory.setFinalPosition(landing);
    // 按键 4/5 均使用 3 cm 抬脚高度，不改变动作时序。
    trajectory.setHeight(T(0.03));
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
  const Vec2<T> nominal_foot_world =
    estimate.position_world.template head<2>() +
    yaw_rotation * centered_nominal_foot_body;
  const T stance_time = locomotion_result.stance_time[leg];
  const Vec2<T> measured_velocity = estimate.velocity_world.template head<2>();
  const Vec2<T> desired_velocity =
    locomotion_result.command.body_velocity_world.template head<2>();
  const T yaw_rate = locomotion_result.command.body_angular_velocity.z();
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
  step.x() = std::clamp(step.x(), T(-0.3), T(0.3));
  step.y() = std::clamp(step.y(), T(-0.3), T(0.3));
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
  const bool cycle_complete = load_transfer_elapsed_ >=
    T(1.34) - T(0.5) * this->_data->control_time_step;
  if (applied_gait_ == GaitType::THREE_FOOT &&
    cycle_complete && !cycle_aborted_)
  {
    single_leg_cycle_index_ = (single_leg_cycle_index_ + 1) % kNumLegs;
    applyRequestedGait(true);
    std::fprintf(stderr,
      "[FSM][THREE_FOOT] 当前单腿抬落完成，切换到第 %zu 条腿并继续循环。\n",
      single_leg_cycle_index_);
  }
  if (this->_data->stand_cycle_restart_requested) {
    this->_data->stand_cycle_restart_requested = false;
    if (isStandCycle() && (cycle_complete || cycle_aborted_))
    {
      // 上一次抬落已完整结束（含平移撤回），从头重新执行一次；保留上一次
      // 由实测力差学到的平移修正，下一次从修正后的位置开始。
      if (applied_gait_ == GaitType::THREE_FOOT && !cycle_aborted_) {
        single_leg_cycle_index_ = (single_leg_cycle_index_ + 1) % kNumLegs;
      }
      cycle_learned_correction_ = cycle_support_shift_world_ - cycle_initial_shift_world_;
      applyRequestedGait(true);
      std::fprintf(stderr, "[FSM][STAND_CYCLE] 再次按键：重新执行一次抬落。\n");
    } else if (isStandCycle()) {
      std::fprintf(stderr,
        "[FSM][STAND_CYCLE] 再次按键被忽略：上一次抬落尚未结束（t=%.3f s）。\n",
        static_cast<double>(load_transfer_elapsed_));
    }
  }
  if (applied_gait_ == GaitType::STAND ||
    applied_gait_ == GaitType::STAND_CYCLE ||
    applied_gait_ == GaitType::THREE_FOOT) {
    mpc_->setVelocityCommand(T(0), T(0), T(0));
  } else {
    mpc_->setVelocityCommand(
      rl_velocity_command_.x(), rl_velocity_command_.y(), rl_velocity_command_.z());
  }
  // MPC 输入必须使用统一的世界坐标系，否则反作用力方向会与 WBC 不一致。
  const auto feet_world = footPositionsWorld();
  if (applied_gait_ == GaitType::STAND) {
    updateStage1Stability();
  }
  if (isStandCycle() && !entry_transition_active_) {
    // 一次抬落：0.3 s 保持（同时把质心移到 FR-RL 支撑线）、0.3 s 卸载、
    // 0.24 s 摆腿、0.2 s 恢复承重，再用 0.3 s 把机身移回原锚点。
    constexpr T kHold = T(0.3);
    constexpr T kRamp = T(0.3);
    constexpr T kTouchdown = T(0.84);
    constexpr T kReload = T(0.2);
    constexpr T kEnd = T(1.04);
    constexpr T kShiftReturn = T(0.3);
    constexpr T kShiftEnd = kEnd + kShiftReturn;
    // 卸载斜坡按 10 ms 分段刷新上下限，既平滑又不让 MPC 每帧重解。
    constexpr T kForceStep = T(0.01);
    // 卸载到 95% 时暂停整条时间轴等待承重确认；确认后从同一时刻继续，
    // 不重置 MPC，也不先回载，保证卸载、摆腿和平移撤回始终同相。
    constexpr T kGatePhase = T(0.95);
    constexpr T kGateTime = kHold + kGatePhase * kRamp;
    const T time = std::round(load_transfer_elapsed_ / this->_data->control_time_step) *
      this->_data->control_time_step;
    if (!cycle_shift_ready_) {prepareStandCycleShift(feet_world);}
    // 实机机身位置环有约 1 cm 稳态误差，只靠平移目标无法把质心送到支撑线上。
    // 保持结束、卸载开始前（动作时钟在此暂停），按实测全身质心到 FR-RL
    // 连线的偏差积分修正平移量；卸载开始后冻结，避免轻载脚被拖动。
    const bool unload_start = time >= kHold &&
      time < kHold + T(0.5) * this->_data->control_time_step;
    if (unload_start) {integrateStandCycleShift(feet_world);}
    if (!unload_start && time >= kHold && !cycle_hold_latched_) {
      // 卸载开始时锁存 FL/RR 实测关节角（四脚着地），卸载期间保持。
      cycle_hold_latched_ = true;
      // 平移本身会让机身侧倾约 2°，倾倒基准取卸载开始时的姿态。
      cycle_start_rpy_ = this->_data->state_estimate->rpy;
      for (const std::size_t leg : cycleSwingLegs()) {
        cycle_hold_q_[leg] = this->_data->leg_controller->datas[leg].q;
      }
    }
    if (!unload_start && time >= kHold && !cycle_unload_logged_) {
      cycle_unload_logged_ = true;
      const Vec2<T> com_error = standCycleComLineError(feet_world);
      std::fprintf(stderr,
        "[FSM][STAND_CYCLE] 开始卸载：支撑线平移=(%.4f,%.4f)m 质心距支撑线=(%.4f,%.4f)m\n",
        static_cast<double>(cycle_support_shift_world_.x()),
        static_cast<double>(cycle_support_shift_world_.y()),
        static_cast<double>(com_error.x()), static_cast<double>(com_error.y()));
    }
    const std::size_t bucket = static_cast<std::size_t>(time >= kEnd ?
      std::ceil(kEnd / kForceStep) : std::floor(time / kForceStep + T(1e-4)));
    if (!load_transfer_gate_open_ && time > kHold && load_transfer_elapsed_ < kGateTime &&
      applied_gait_ == GaitType::STAND_CYCLE)
    {
      // 卸载途中（含倾倒回载）同样按实测 RR-FL 承重差修正平移：倾倒时 RR 承重
      // 明显多于 FL，平移持续推向 FL 一侧，回载后再卸载时质心更靠近支撑线。
      standCycleDiagonalImbalance(feet_world, true);
    }
    if (!load_transfer_gate_open_ && load_transfer_elapsed_ >= kGateTime) {
      // 卸到 95% 时 FL/RR 的残余承重应近似相等；RR 明显多于 FL 说明真实
      // 质心仍偏在 RR 一侧（模型质心与实物不符），抬腿后会朝 RR 倾倒。
      // 等待期间按实测残余力差把机身朝 FL 一侧积分平移，直到两者相等。
      // 5 号三腿支撑静定，没有对角力差的概念，只看承重确认。
      const T imbalance = applied_gait_ == GaitType::STAND_CYCLE ?
        standCycleDiagonalImbalance(feet_world, true) : T(0);
      // 力差判据只作辅助：等待超过 2 s 仍不满足时放行，仅依赖承重确认。
      constexpr std::size_t kImbalanceTimeoutFrames = 1000;
      if (++cycle_gate_wait_frames_ == kImbalanceTimeoutFrames) {
        std::fprintf(stderr,
          "[FSM][STAND_CYCLE] RR-FL 承重差等待 2 s 仍为 %.1f%% 体重，忽略该判据。\n",
          static_cast<double>(imbalance) * 100.0);
      }
      const bool balanced = cycle_gate_wait_frames_ >= kImbalanceTimeoutFrames ||
        (std::isfinite(static_cast<double>(imbalance)) && std::abs(imbalance) < T(0.03));
      if (loadTransferReady() && balanced) {
        ++load_transfer_ready_frames_;
      } else {
        load_transfer_ready_frames_ = 0;
      }
      if (load_transfer_ready_frames_ >= kStableFrameCount) {
        load_transfer_gate_open_ = true;
        std::fprintf(stderr,
          "[FSM][STAND_CYCLE] load-transfer gate open after stable support confirmation. "
          "RR-FL 承重差=%.1f%% 体重 支撑线平移=(%.4f,%.4f)m\n",
          static_cast<double>(imbalance) * 100.0,
          static_cast<double>(cycle_support_shift_world_.x()),
          static_cast<double>(cycle_support_shift_world_.y()));
      }
    }
    const T phase = std::clamp(time < kTouchdown ?
      (time - kHold) / kRamp : T(1) - (time - kTouchdown) / kReload, T(0), T(1));
    // 承重反馈确认前停在 95% 卸载，FL/RR 仍保持接触。
    const T guarded_phase = load_transfer_gate_open_ ? phase :
      std::min(phase, kGatePhase);
    const T blend = guarded_phase * guarded_phase * (T(3) - T(2) * guarded_phase);
    // 平移必须在卸载前完成、在抬落腿重新承重后才撤回，否则两脚支撑期间
    // 质心不在支撑线上，机身会绕对角线倾倒并掉高。
    const T shift_phase = std::clamp(time < kEnd ?
      time / kHold : T(1) - (time - kEnd) / kShiftReturn, T(0), T(1));
    const T shift_blend = shift_phase * shift_phase * (T(3) - T(2) * shift_phase);
    mpc_->setPositionOffsetWorld(shift_blend * cycle_support_shift_world_);
    // MPC 接触表与本状态共用同一动作时钟，等待期间两者一起暂停。
    mpc_->setStandCycleTime(time);
    if (bucket != load_transfer_bucket_) {
      load_transfer_bucket_ = bucket;
      mpc::NormalForceBounds<T> bounds;
      bounds.minimum.fill(mpc_->solverSettings().minimum_normal_force);
      bounds.maximum.fill(mpc_->solverSettings().maximum_normal_force);
      for (const std::size_t leg : cycleSwingLegs()) {
        bounds.minimum[leg] = T(0);
        bounds.maximum[leg] *= T(1) - blend;
      }
      mpc_->setNormalForceBounds(bounds);
      wbc_ctrl_->setNormalForceBounds(bounds.minimum, bounds.maximum);
    }
    // 动作时钟只在当前段达标时推进：质心未到位不卸载、支撑腿未接住不继续
    // 卸载、摆动脚跟踪不上不继续抬、未触地不回载。时间常数只决定达标时的
    // 推进快慢。摆动段悬空等待有风险，超过 0.3 s 强制推进到收腿。
    const char * blocker = standCycleStepBlocker(time, blend, shift_blend, feet_world);
    if (blocker == nullptr && !load_transfer_gate_open_ &&
      load_transfer_elapsed_ >= kGateTime)
    {
      blocker = "等待承重确认（含 FL/RR 残余承重差 < 3% 体重）";
    }
    // 触地确认：到触地时刻摆动脚若仍高于起始地面 8 mm，则保持摆动并继续下探，
    // 动作时钟停在触地点，回载不开始；否则 WBC 接触约束会把脚固定在空中，
    // 回载力打空，机身朝另一侧塌陷。最多下探 0.2 s。
    cycle_reaching_.fill(false);
    if (time >= kTouchdown && time < kEnd) {
      for (const std::size_t leg : cycleSwingLegs()) {
        if (!cycle_touched_[leg] &&
          feet_world[leg].z() - load_transfer_foot_position_[leg].z() <= T(0.008))
        {
          cycle_touched_[leg] = true;
        }
      }
      bool all_touched = true;
      for (const std::size_t leg : cycleSwingLegs()) {all_touched = all_touched && cycle_touched_[leg];}
      if (!all_touched) {
        if (++cycle_reach_frames_ > 100) {
          std::fprintf(stderr,
            "[FSM][STAND_CYCLE] 下探 0.2 s 仍未触地 FR=%d FL=%d RR=%d RL=%d（1=已触地），按触地处理。\n",
            static_cast<int>(cycle_touched_[0] || !cycle_swing_[0]),
            static_cast<int>(cycle_touched_[1] || !cycle_swing_[1]),
            static_cast<int>(cycle_touched_[2] || !cycle_swing_[2]),
            static_cast<int>(cycle_touched_[3] || !cycle_swing_[3]));
          for (const std::size_t leg : cycleSwingLegs()) {cycle_touched_[leg] = true;}
        } else {
          for (const std::size_t leg : cycleSwingLegs()) {cycle_reaching_[leg] = !cycle_touched_[leg];}
          if (blocker == nullptr) {blocker = "摆动脚尚未触地，继续下探";}
        }
      }
    }
    // 摆动期两脚支撑绕 FR-RL 连线不稳定，悬空越久倒得越多：摆动段绝不暂停，
    // 不达标时以 3 倍速走完剩余轨迹尽快落脚（MIT Cheetah 3 早触地思路）。
    const bool swinging = time >= kGateTime + (T(1) - kGatePhase) * kRamp &&
      time < kTouchdown;
    T step = this->_data->control_time_step;
    if (blocker != nullptr) {
      ++cycle_stall_frames_;
      if (swinging) {
        if (cycle_stall_frames_ == 1) {
          std::fprintf(stderr,
            "[FSM][STAND_CYCLE] 摆动段 t=%.3f s 未达标（%s），加速落脚。\n",
            static_cast<double>(time), blocker);
        }
        step *= T(3);
        blocker = nullptr;
      } else if (cycle_stall_frames_ % 250 == 1 && cycle_stall_frames_ > 1) {
        std::fprintf(stderr,
          "[FSM][STAND_CYCLE] 动作暂停 t=%.3f s：%s（已等待 %.2f s）。\n",
          static_cast<double>(time), blocker,
          static_cast<double>(cycle_stall_frames_) *
          static_cast<double>(this->_data->control_time_step));
      }
    }
    // 卸载中（抬腿前）若机身开始绕支撑线倾倒（FL/RR 足端离地 > 1 cm 或
    // roll/pitch 相对动作起点变化 > 0.04 rad），动作时钟倒退回载，同时力差
    // 积分继续把机身推向 FL 一侧，恢复后再重新卸载，而不是等到倒地。
    bool tipping = false;
    if (!load_transfer_gate_open_ && time > kHold && !cycle_aborted_) {
      const auto & estimate = *this->_data->state_estimate;
      tipping =
        std::abs(estimate.rpy.x() - cycle_start_rpy_.x()) > T(0.04) ||
        std::abs(estimate.rpy.y() - cycle_start_rpy_.y()) > T(0.04);
      for (const std::size_t leg : cycleSwingLegs()) {
        tipping = tipping ||
          feet_world[leg].z() - load_transfer_foot_position_[leg].z() > T(0.01);
      }
    }
    if (tipping || cycle_tipping_) {
      // 一旦触发就一直回载到满载点再重新卸载；不能在倾倒刚消失时停在半卸载处。
      if (!cycle_tipping_) {
        ++cycle_tip_count_;
        std::fprintf(stderr,
          "[FSM][STAND_CYCLE] 卸载中检测到倾倒（第 %zu 次） t=%.3f s，回载重试；支撑线平移=(%.4f,%.4f)m\n",
          cycle_tip_count_,
          static_cast<double>(time),
          static_cast<double>(cycle_support_shift_world_.x()),
          static_cast<double>(cycle_support_shift_world_.y()));
      }
      cycle_tipping_ = true;
      cycle_had_tip_ = true;
      load_transfer_ready_frames_ = 0;
      load_transfer_elapsed_ = std::max(kHold,
        load_transfer_elapsed_ - this->_data->control_time_step);
      blocker = "卸载中倾倒，回载中";
      // 用未取整的动作时钟判断回到满载点；取整后的 time 因浮点误差可能略大于 kHold。
      if (load_transfer_elapsed_ <= kHold + T(0.5) * this->_data->control_time_step) {
        cycle_tipping_ = false;
        cycle_start_rpy_ = this->_data->state_estimate->rpy;
        // 连续倾倒 3 次说明平移无法把质心送上支撑线，反复卸载/回载会让轻载腿
        // 触底弹起、机身抖动；停在四脚满载，等待再次按 4。
        constexpr std::size_t kMaximumTipRetries = 3;
        if (cycle_tip_count_ >= kMaximumTipRetries) {
          cycle_aborted_ = true;
          std::fprintf(stderr,
            "[FSM][STAND_CYCLE] 连续 %zu 次卸载倾倒，本次抬落中止，保持四脚站立；"
            "可再次按 4 重试。支撑线平移=(%.4f,%.4f)m\n",
            cycle_tip_count_,
            static_cast<double>(cycle_support_shift_world_.x()),
            static_cast<double>(cycle_support_shift_world_.y()));
        }
      }
    }
    if (blocker == nullptr && !cycle_tipping_ && !cycle_aborted_) {
      if (!swinging) {cycle_stall_frames_ = 0;}
      // 加速时不越过触地点，保证回载从触地时刻开始。
      const T next = load_transfer_elapsed_ + step;
      load_transfer_elapsed_ = std::min(kShiftEnd,
        swinging && step > this->_data->control_time_step ? std::min(next, kTouchdown) : next);
    }
    // 下一周期 GaitScheduler::step() 先于本状态运行，提前同步估计器接触先验。
    this->_data->gait_scheduler->setStandCycleTime(load_transfer_elapsed_);
  } else {
    mpc_->setPositionOffsetWorld(Vec2<T>::Zero());
  }
  /*MPC 输入：当前状态估计；用户期望状态；当前四个足端的世界坐标。
    MPC 输出 LocomotionResult，包括：command:机身期望状态；
    reaction_forces_world：各腿期望地面反作用力；
    contact_state：当前是否接触；,swing_phase：摆动进度，通常范围为 [0, 1]
    ,swing_time：摆动总时间,stance_time,valid：支撑总时间。*/
  const auto result = mpc_->run(
    *this->_data->state_estimate, *this->_data->desired_state, feet_world);
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
  auto contact_state = result.contact_state;
  if (isStandCycle() && !load_transfer_gate_open_) {
    // MPC 的时间表可以先进入低载荷段，但 WBC 在反馈确认前仍保持四脚接触，
    // 防止计划时刻直接把未承重的腿交给摆动任务。
    for (const std::size_t leg : cycleSwingLegs()) {contact_state[leg] = true;}
  }
  if (isStandCycle()) {
    for (const std::size_t leg : cycleSwingLegs()) {
      if (cycle_reaching_[leg]) {contact_state[leg] = false;}
    }
  }
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    //将 MPC 计算出的世界坐标系反作用力传给 WBC。
    wbc_data_.Fr_des[leg] = result.reaction_forces_world[leg];
    //将布尔接触状态转成浮点数：
    wbc_data_.contact_state[leg] = contact_state[leg] ? T(1) : T(0);

    if (contact_state[leg]) {
      // 支撑时足端位置任务不会加入 WBC；同步当前值仅用于诊断，并清除上一摆动态。
      swing_active_[leg] = false;
      wbc_data_.pFoot_des[leg] = feet_world[leg];
      wbc_data_.vFoot_des[leg].setZero();
      wbc_data_.aFoot_des[leg].setZero();
      continue;
    }

    // 只在离地沿锁存起点和落脚点。后续周期即使实测足端受到扰动，目标轨迹
    // 也只由锁存数据和 swing_phase 推进，不会把扰动累积成新的目标高度。
    if (isStandCycle() && cycle_reaching_[leg]) {
      // 未触地：以 0.1 m/s 向起始地面以下 2 cm 下探，直到触地确认。
      wbc_data_.pFoot_des[leg] = feet_world[leg];
      wbc_data_.pFoot_des[leg].z() = load_transfer_foot_position_[leg].z() - T(0.02);
      wbc_data_.vFoot_des[leg] = Vec3<T>(T(0), T(0), T(-0.1));
      wbc_data_.aFoot_des[leg].setZero();
      swing_active_[leg] = true;
      continue;
    }
    if (!swing_active_[leg]) {
      startSwingTrajectory(leg, feet_world[leg], result);
    }
    auto & trajectory = swing_trajectories_[leg];
    /*Bezier 轨迹会生成平滑的：足端位置；足端速度；足端加速度。*/
    trajectory.computeSwingTrajectoryBezier(
      result.swing_phase[leg], result.swing_time[leg]);
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
        const LegId leg_id = static_cast<LegId>(leg);
        const Vec3<T> desired_foot_body = rotation_body_from_world *
          (wbc_data_.pFoot_des[leg] - estimate.position_world) -
          this->_data->quadruped->hipLocation(leg_id);
        command.foot_position_desired = desired_foot_body;
        command.foot_velocity_desired = rotation_body_from_world * wbc_data_.vFoot_des[leg];
        command.kp_cartesian = Mat3<T>::Zero();
        if (!contact_state[leg]) {
          command.kp_cartesian = Vec3<T>(T(500), T(500), T(150)).asDiagonal();
        }
        command.kd_cartesian = Mat3<T>::Identity() * T(7);
      }
      // 行走层只负责自身的目标范围；所有状态共用的连续力矩约束由
      // LegController 最终命令出口统一执行。
      constrainJointTargets();
      if (isStandCycle() && cycle_hold_latched_ &&
        load_transfer_elapsed_ < T(0.6))
      {
        // 卸载期间 FL/RR 力上限接近零，关节目标若跟随 WBC（按机身到位计算）
        // 漂移，轻载脚会被提离地面；保持卸载开始时的关节角直到摆动开始。
        for (const std::size_t leg : cycleSwingLegs()) {
          if (!contact_state[leg]) {continue;}
          auto & command = this->_data->leg_controller->commands[leg];
          command.position_desired = cycle_hold_q_[leg];
          command.velocity_desired.setZero();
        }
      }
      if (isStandCycle() && load_transfer_elapsed_ > T(1.04)) {
        // 单次抬落结束后把零空间姿态平滑收回 DM1 home，避免支撑脚的
        // 微小位移永久留在关节目标中；普通 TROT 不走这条动作专用路径。
        const T alpha = std::clamp(
          (load_transfer_elapsed_ - T(1.04)) / T(0.3), T(0), T(1));
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
          // 摆动腿由足端轨迹控制，不能被 home 关节目标拉住。
          if (!contact_state[leg]) {continue;}
          auto & command = this->_data->leg_controller->commands[leg];
          const auto & home = this->_data->quadruped->leg(
            static_cast<LegId>(leg)).joints.home_position;
          command.position_desired =
            (T(1) - alpha) * command.position_desired + alpha * home;
        }
      }
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
T FSM_State_Locomotion<T>::estimatedVerticalSupport(std::size_t leg) const
{
  if (leg >= kNumLegs || this->_data->leg_controller == nullptr ||
    this->_data->state_estimate == nullptr)
  {
    return T(0);
  }
  const auto & data = this->_data->leg_controller->datas[leg];
  if (!data.valid || !data.J.allFinite() || !data.torque_estimate.allFinite()) {
    return T(0);
  }
  Vec3<T> torque = data.torque_estimate;
  // MuJoCo 的当前执行层通过 qfrc_applied 施加 MIT-PD 总力矩，
  // qfrc_actuator 不会回读这一路；反馈为近零时，用已下发命令作门控代理。
  // 真机有有效力矩回读时始终优先使用测量值，不改变任何 MPC/WBIC 输出。
  if (torque.norm() < T(1e-3)) {
    const auto & command = this->_data->leg_controller->commands[leg];
    if (!command.torque_feedforward.allFinite() ||
      !command.kp_joint.allFinite() || !command.kd_joint.allFinite() ||
      !command.position_desired.allFinite() || !command.velocity_desired.allFinite())
    {
      return T(0);
    }
    torque = command.torque_feedforward +
      command.kp_joint.cwiseProduct(command.position_desired - data.q) +
      command.kd_joint.cwiseProduct(command.velocity_desired - data.qd);
  }
  const Mat3<T> normal = data.J * data.J.transpose() +
    T(1e-6) * Mat3<T>::Identity();
  Eigen::LDLT<Mat3<T>> solver(normal);
  if (solver.info() != Eigen::Success) {return T(0);}
  const Vec3<T> force_body = solver.solve(data.J * torque);
  const Vec3<T> force_world =
    this->_data->state_estimate->rotation_world_from_body * force_body;
  if (!force_world.allFinite()) {return T(0);}
  return std::abs(force_world.z());
}

template<typename T>
void FSM_State_Locomotion<T>::updateStage1Stability()
{
  const auto & estimate = *this->_data->state_estimate;
  const auto & parameters = *this->_data->control_parameters;
  const T target_height = std::clamp(
    this->_data->desired_state->body_position_world.z(),
    parameters.minimum_standing_height, parameters.maximum_standing_height);
  bool stable = estimate.valid && estimate.position_world.z() > target_height - T(0.025) &&
    estimate.position_world.z() < target_height + T(0.025) &&
    std::abs(estimate.rpy.x()) < T(0.10) &&
    std::abs(estimate.rpy.y()) < T(0.10);
  for (std::size_t leg = 0; leg < kNumLegs && stable; ++leg) {
    const auto & data = this->_data->leg_controller->datas[leg];
    stable = data.valid && data.p.allFinite() && data.v.allFinite() &&
      data.v.norm() < T(0.20);
  }
  const std::size_t previous_frames = stage1_stable_frames_;
  stage1_stable_frames_ = stable ?
    std::min(stage1_stable_frames_ + 1, kStableFrameCount) : 0;
  if (this->_data->locomotion_gait == GaitType::STAND &&
    previous_frames < kStableFrameCount && stage1_stable_frames_ >= kStableFrameCount)
  {
    // 3 号与 4 号共用同一四脚站稳判据；达标即 4 号可立即进入换载。
    std::fprintf(stderr,
      "[FSM][MARCH-DIAGNOSTIC] stage=1 四脚站稳达标（与按键 4 换载前判据相同）。\n");
  }
}

template<typename T>
std::vector<std::size_t> FSM_State_Locomotion<T>::cycleSwingLegs() const
{
  std::vector<std::size_t> legs;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    if (cycle_swing_[leg]) {legs.push_back(leg);}
  }
  return legs;
}

template<typename T>
Vec2<T> FSM_State_Locomotion<T>::standCycleComLineError(
  const std::array<Vec3<T>, kNumLegs> & feet_world)
{
  Vec3<T> com_world = Vec3<T>::Zero();
  try {
    com_world = wholeBodyComWorld();
  } catch (const std::exception &) {
    return Vec2<T>::Constant(std::numeric_limits<T>::quiet_NaN());
  }
  const Vec2<T> com = com_world.template head<2>();
  // 返回质心到目标点的水平位移，即还需要平移的量。
  return cycleSupportTarget(com, feet_world) - com;
}

template<typename T>
Vec2<T> FSM_State_Locomotion<T>::cycleSupportTarget(
  const Vec2<T> & com, const std::array<Vec3<T>, kNumLegs> & feet_world) const
{
  // 4 号：质心投影到 FR(0)-RL(3) 对角支撑线上（保持原有行为）。
  // 5 号（THREE_FOOT）：目标 = 三条支撑腿足端的质心（支撑三角形的中心），
  //   保证结果始终在三角形内部，与腿的顺序、坐标系方向无关。
  if (applied_gait_ != GaitType::THREE_FOOT) {
    const Vec2<T> a = feet_world[0].template head<2>();
    const Vec2<T> line = feet_world[3].template head<2>() - a;
    if (!com.allFinite() || !line.allFinite() || line.squaredNorm() < T(1e-4)) {
      return Vec2<T>::Constant(std::numeric_limits<T>::quiet_NaN());
    }
    const T along = (com - a).dot(line) / line.squaredNorm();
    return a + along * line;
  }
  // THREE_FOOT：把质心投影到摆动腿对侧对角线上，再朝第三条支撑腿内缩 2 cm。
  // 对角线对（FR↔RL，FL↔RR）：
  //   FR(0) 抬 → 对侧对角线 FL(1)-RR(2)，朝 RL(3) 内缩
  //   FL(1) 抬 → 对侧对角线 FR(0)-RL(3)，朝 RR(2) 内缩
  //   RR(2) 抬 → 对侧对角线 FR(0)-RL(3)，朝 FL(1) 内缩
  //   RL(3) 抬 → 对侧对角线 FL(1)-RR(2)，朝 FR(0) 内缩
  const std::size_t swing = cycleSwingLegs().empty() ? 0 : cycleSwingLegs().front();
  std::size_t ea = 0, eb = 3, inward = 1;
  switch (swing) {
    case 0: ea = 1; eb = 2; inward = 3; break;
    case 1: ea = 0; eb = 3; inward = 2; break;
    case 2: ea = 0; eb = 3; inward = 1; break;
    case 3: ea = 1; eb = 2; inward = 0; break;
    default: break;
  }
  const Vec2<T> a2 = feet_world[ea].template head<2>();
  const Vec2<T> line2 = feet_world[eb].template head<2>() - a2;
  if (!line2.allFinite() || line2.squaredNorm() < T(1e-4)) {
    return Vec2<T>::Constant(std::numeric_limits<T>::quiet_NaN());
  }
  Vec2<T> normal2(-line2.y(), line2.x());
  normal2.normalize();
  if (normal2.dot(feet_world[inward].template head<2>() - a2) < T(0)) {normal2 = -normal2;}
  const T along2 = (com - a2).dot(line2) / line2.squaredNorm();
  constexpr T kInset = T(0.02);
  return a2 + along2 * line2 + kInset * normal2;
}

template<typename T>
T FSM_State_Locomotion<T>::standCycleDiagonalImbalance(
  const std::array<Vec3<T>, kNumLegs> & feet_world, bool integrate)
{
  const T weight = this->_data->quadruped->totalMass() * T(9.81);
  const T imbalance =
    (estimatedVerticalSupport(2) - estimatedVerticalSupport(1)) / weight;
  const Vec2<T> front_right = feet_world[0].template head<2>();
  const Vec2<T> line = feet_world[3].template head<2>() - front_right;
  if (!std::isfinite(static_cast<double>(imbalance)) || !line.allFinite() ||
    line.squaredNorm() < T(1e-4))
  {
    return std::numeric_limits<T>::quiet_NaN();
  }
  if (integrate) {
    // 支撑线法向，指向 FL 一侧。
    Vec2<T> normal(-line.y(), line.x());
    normal.normalize();
    if (normal.dot(feet_world[1].template head<2>() - front_right) < T(0)) {normal = -normal;}
    // 10% 体重的力差每秒平移 1 cm。
    constexpr T kGain = T(0.1);
    // 实测力差不依赖质心模型；实机模型质心与实物相差可超过 1 cm，允许修正 3 cm。
    applyStandCycleShiftCorrection(
      kGain * imbalance * this->_data->control_time_step * normal, T(0.03));
  }
  return imbalance;
}

template<typename T>
void FSM_State_Locomotion<T>::integrateStandCycleShift(
  const std::array<Vec3<T>, kNumLegs> & feet_world)
{
  const Vec2<T> error = standCycleComLineError(feet_world);
  if (!error.allFinite()) {return;}
  // 机身仍在移动时偏差会自行减小，此时积分会超调；只在停稳后修正稳态误差。
  if (this->_data->state_estimate->velocity_world.template head<2>().norm() > T(0.02)) {
    return;
  }
  constexpr T kIntegralGain = T(0.5);  // 1/s：1 cm 偏差每秒修正 5 mm。
  applyStandCycleShiftCorrection(
    kIntegralGain * this->_data->control_time_step * error, T(0.01));
}

template<typename T>
void FSM_State_Locomotion<T>::applyStandCycleShiftCorrection(
  const Vec2<T> & delta, T maximum_correction)
{
  // 防积分饱和：机身实际位置未跟上当前平移目标（误差 >= 1.5 cm，实机位置环
  // 稳态误差约 1 cm）时不再加大平移量，否则 WBC 关节目标会按“机身已到位”
  // 持续偏离实际，卸载腿被提起。
  const auto & estimate = *this->_data->state_estimate;
  const Vec2<T> target = mpc_->positionAnchorWorld(estimate).template head<2>() +
    cycle_support_shift_world_;
  if ((estimate.position_world.template head<2>() - target).norm() >= T(0.015)) {return;}
  // 修正量相对起始锁存平移不超过 maximum_correction。
  Vec2<T> correction = cycle_support_shift_world_ + delta - cycle_initial_shift_world_;
  if (correction.norm() > maximum_correction) {
    correction *= maximum_correction / correction.norm();
  }
  cycle_support_shift_world_ = cycle_initial_shift_world_ + correction;
  // 总平移不得超过 MPC 位置偏移上限 5 cm（超过会抛异常），留 5 mm 余量。
  constexpr T kMaximumTotalShift = T(0.045);
  if (cycle_support_shift_world_.norm() > kMaximumTotalShift) {
    cycle_support_shift_world_ *= kMaximumTotalShift / cycle_support_shift_world_.norm();
  }
}

template<typename T>
const char * FSM_State_Locomotion<T>::standCycleStepBlocker(
  T time, T blend, T /*shift_blend*/,
  const std::array<Vec3<T>, kNumLegs> & feet_world)
{
  constexpr T kHold = T(0.3);
  constexpr T kLiftoff = T(0.6);
  constexpr T kTouchdown = T(0.84);
  constexpr T kEnd = T(1.04);
  const auto & estimate = *this->_data->state_estimate;
  if (!estimate.valid) {return "状态估计无效";}
  if (std::abs(estimate.rpy.x()) > T(0.12) || std::abs(estimate.rpy.y()) > T(0.12)) {
    return "机身姿态超过 0.12 rad";
  }
  const T target_height = this->_data->desired_state->body_position_world.z();
  if (estimate.position_world.z() < target_height - T(0.03)) {return "机身高度下降超过 3 cm";}
  const T weight = this->_data->quadruped->totalMass() * T(9.81);
  T support = T(0);
  T moving = T(0);
  T swing_count = T(0);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    if (cycle_swing_[leg]) {
      moving += estimatedVerticalSupport(leg);
      swing_count += T(1);
    } else {
      support += estimatedVerticalSupport(leg);
    }
  }
  // 摆动腿当前单腿法向力上限，与下发给 MPC/WBC 的上限一致。
  const T cap = mpc_->solverSettings().maximum_normal_force * (T(1) - blend);
  if (time >= kEnd) {
    // 撤回质心平移前，摆动腿必须已重新接回载荷（每条腿约 17.5% 体重）。
    if (moving < T(0.175) * swing_count * weight) {return "摆动腿尚未接回载荷";}
    // 四脚均已承重，撤回平移只需姿态正常，不再要求位置跟踪精度。
    return nullptr;
  }
  if (time < kHold) {
    // 平移斜坡期间目标在移动，机身天然滞后；到位判据留到卸载开始前检查。
    return nullptr;
  }
  if (time < kLiftoff) {
    // 卸载段：FL+RR 实测承重不得超过当前法向力上限（留 10% 体重裕量），
    // 即载荷确实转给了 FR+RL，才继续压低上限。
    // 卸载开始前（仅在该时刻检查）要求实测全身质心距 FR-RL 支撑线 < 6 mm
    // 且机身已停稳；卸载途中再暂停会让轻载脚反复离地。
    // 发生过倾倒回载后，平移已由实测力差修正，模型质心判据不再适用（实机
    // 模型质心偏差超过其 ±1 cm 修正范围，会卡死在此处）。
    if (time < kHold + T(0.5) * this->_data->control_time_step && !cycle_had_tip_) {
      // 4 号只有 FR/RL 两脚支撑，必须先把质心压到支撑线附近；
      // 5 号始终保留三条支撑腿，不能复用这条对角线判据，否则当
      // 支撑三角形的目标点超出机身平移限幅时会永久停在 0.3 s。
      const bool require_diagonal_alignment = applied_gait_ == GaitType::STAND_CYCLE;
      const Vec2<T> com_error = standCycleComLineError(feet_world);
      const bool position_ok = !require_diagonal_alignment ||
        (com_error.allFinite() && com_error.norm() <= T(0.006));
      if (!position_ok || estimate.velocity_world.template head<2>().norm() > T(0.05)) {
        return require_diagonal_alignment ?
          "质心距支撑目标超过 6 mm 或未停稳" : "机身尚未停稳";
      }
    }
    return moving <= swing_count * cap + T(0.1) * weight ?
      nullptr : "摆动腿实测承重高于卸载上限";
  }
  if (time < kTouchdown) {
    // 足端跟踪滞后时加速只会让脚更早被切成支撑而悬在空中，故不作为判据；
    // 落脚由触地确认（未触地继续下探）保证。
    if (support < T(0.85) * weight) {return "摆动期支撑腿承重不足 85%";}
    return nullptr;
  }
  // 回载过程中实测力天然滞后于上限，不逐帧卡住，留到撤回平移前统一检查。
  return nullptr;
}

template<typename T>
bool FSM_State_Locomotion<T>::loadTransferReady() const
{
  const auto & estimate = *this->_data->state_estimate;
  const T weight = this->_data->quadruped->totalMass() * T(9.81);
  T support = T(0);
  bool legs_ok = true;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    if (cycle_swing_[leg]) {continue;}
    const auto & data = this->_data->leg_controller->datas[leg];
    const T force = estimatedVerticalSupport(leg);
    support += force;
    legs_ok = legs_ok && data.valid && data.v.norm() < T(0.30) &&
      force > T(0.15) * weight / T(4);
  }
  return stage1_stable_frames_ >= kStableFrameCount && estimate.valid &&
    estimate.position_world.z() > T(0.295) &&
    std::abs(estimate.rpy.x()) < T(0.12) &&
    std::abs(estimate.rpy.y()) < T(0.12) &&
    estimate.velocity_world.norm() < T(0.30) &&
    estimate.angular_velocity_body.norm() < T(0.40) &&
    legs_ok && support > T(0.60) * weight;
}

template<typename T>
bool FSM_State_Locomotion<T>::isRlMode() const noexcept
{
  return active_mode_ == ControlMode::WalkRl ||
         active_mode_ == ControlMode::StairsRl;
}

template<typename T>
bool FSM_State_Locomotion<T>::buildRlObservation(
  std::array<float, kRlObservationSize> & observation) const
{
  const auto & estimate = *this->_data->state_estimate;
  if (!estimate.valid || !estimate.rotation_world_from_body.allFinite()) {
    return false;
  }
  observation.fill(0.0F);
  observation[0] = static_cast<float>(rl_velocity_command_.x()) * 2.0F;
  observation[1] = static_cast<float>(rl_velocity_command_.y()) * 2.0F;
  observation[2] = static_cast<float>(rl_velocity_command_.z()) * 0.25F;
  for (std::size_t i = 0; i < 3; ++i) {
    observation[3 + i] = static_cast<float>(estimate.angular_velocity_body(
      static_cast<Eigen::Index>(i))) * 0.25F;
  }
  const Vec3<T> projected_gravity =
    estimate.rotation_world_from_body.transpose() * Vec3<T>(T(0), T(0), T(-1));
  for (std::size_t i = 0; i < 3; ++i) {
    observation[6 + i] = static_cast<float>(projected_gravity(
      static_cast<Eigen::Index>(i)));
  }
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & data = this->_data->leg_controller->datas[leg];
    if (!data.valid) {return false;}
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const std::size_t index = leg * kJointsPerLeg + joint;
      observation[9 + index] = static_cast<float>(data.q(
        static_cast<Eigen::Index>(joint))) - kDm1RlDefaultJointPosition[index];
      observation[21 + index] = static_cast<float>(data.qd(
        static_cast<Eigen::Index>(joint))) * 0.05F;
      observation[33 + index] = rl_previous_action_[index];
    }
  }
  for (const float value : observation) {
    if (!std::isfinite(value)) {return false;}
  }
  return true;
}

template<typename T>
void FSM_State_Locomotion<T>::RlControlStep()
{
  auto & controller = *this->_data->leg_controller;
  controller.zeroCommand();
  const auto policy = this->_data->rl_policy;
  if (!policy) {
    // 没有注入已冻结的推理器时必须拒绝输出，禁止“保持上一帧动作”。
    std::fprintf(stderr,
      "[FSM][LOCOMOTION] 失能前判断：RL policy 为空。\n");
    controller.setEnabled(false);
    return;
  }
  const RlPolicyMetadata metadata = policy->metadata();
  const bool expected_checkpoint =
    (active_mode_ == ControlMode::StairsRl && metadata.name == "model_4700" &&
    metadata.checkpoint_sha256 == kDm1StairsCheckpointSha256) ||
    (active_mode_ == ControlMode::WalkRl &&
    ((metadata.name == "model_4210" &&
      metadata.checkpoint_sha256 == kDm1FlatCheckpointSha256) ||
    (metadata.name == "model_4245" &&
      metadata.checkpoint_sha256 == kDm1YawRecoveryCheckpointSha256)));
  if (!expected_checkpoint || !metadata.frozen ||
    !metadata.uses_vae_posterior_mean ||
    (active_mode_ == ControlMode::StairsRl && !metadata.supports_stairs))
  {
    std::fprintf(stderr,
      "[FSM][LOCOMOTION] 失能前判断：RL policy 元数据校验失败，name=%s frozen=%d vae_mean=%d stairs=%d。\n",
      metadata.name.c_str(), metadata.frozen, metadata.uses_vae_posterior_mean,
      metadata.supports_stairs);
    controller.setEnabled(false);
    return;
  }

  const std::size_t interval = std::max<std::size_t>(
    1, static_cast<std::size_t>(std::lround(
      this->_data->control_parameters->rl_policy_period /
      this->_data->control_time_step)));
  if (rl_policy_counter_ == 0) {
    std::array<float, kRlObservationSize> observation{};
    if (!buildRlObservation(observation)) {
      std::fprintf(stderr,
        "[FSM][LOCOMOTION] 失能前判断：RL observation 构造失败。\n");
      controller.setEnabled(false);
      return;
    }
    // The VAE receives the six observations available before this policy
    // step. On first entry there is no prior window, so training-compatible
    // startup repeats the current observation across all six frames.
    if (!rl_history_initialized_) {
      for (std::size_t frame = 0; frame < kRlHistoryLength; ++frame) {
        std::copy(
          observation.begin(), observation.end(),
          rl_history_.begin() + frame * kRlObservationSize);
      }
      rl_history_initialized_ = true;
    }
    // Copy the exact pre-inference window.  Do not expose the post-inference
    // appended history here: Python training gives the VAE the history ending
    // at the previous observation, while the actor receives this observation.
    std::array<float, kRlActionSize> action{};
    if (!policy->infer(observation, rl_history_, action)) {
      std::fprintf(stderr,
        "[FSM][LOCOMOTION] 失能前判断：RL policy infer 失败。\n");
      controller.setEnabled(false);
      return;
    }
    for (const float value : action) {
      if (!std::isfinite(value)) {
        std::fprintf(stderr,
          "[FSM][LOCOMOTION] 失能前判断：RL action 出现非有限值。\n");
        controller.setEnabled(false);
        return;
      }
    }
    // Append only after inference. This makes the next policy step's history
    // end at the current frame, matching the training environment ordering.
    std::copy(
      rl_history_.begin() + kRlObservationSize, rl_history_.end(),
      rl_history_.begin());
    std::copy(observation.begin(), observation.end(),
      rl_history_.end() - kRlObservationSize);
    const float policy_dt = static_cast<float>(
      this->_data->control_parameters->rl_policy_period);
    const float alpha = policy_dt / (
      this->_data->control_parameters->rl_action_filter_time_constant + policy_dt);
    const float max_delta = this->_data->control_parameters->rl_max_action_delta;
    for (std::size_t i = 0; i < kRlActionSize; ++i) {
      // Isaac 的 DM1 配置为 clip_actions=100；部署端也只执行同一数值
      // 边界，随后由低通、目标变化限速和力矩包络负责物理安全。
      const float raw = std::clamp(action[i], -100.0F, 100.0F);
      const float low_passed = rl_previous_action_[i] +
        alpha * (raw - rl_previous_action_[i]);
      rl_previous_action_[i] += std::clamp(
        low_passed - rl_previous_action_[i], -max_delta, max_delta);
    }
    rl_policy_counter_ = interval;
  }
  --rl_policy_counter_;

  if (!rl_target_initialized_) {
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
        // Isaac resets joint_pos_target to the RL default posture. Starting
        // from the measured BalanceStand angle would create a deployment-only
        // target trajectory during the first policy frames.
        rl_target_position_[leg * kJointsPerLeg + joint] =
          kDm1RlDefaultJointPosition[leg * kJointsPerLeg + joint];
      }
    }
    rl_target_initialized_ = true;
  }
  const float max_target_step = static_cast<float>(
    this->_data->control_parameters->rl_max_target_velocity *
    this->_data->control_time_step);
  controller.setEnabled(true);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    auto & command = controller.commands[leg];
    const auto & joints = this->_data->quadruped->leg(
      static_cast<LegId>(leg)).joints;
    // RL 直接位置跟踪复用较温和的初始化阻抗；MPC 需要更高阻尼来抑制
    // 接触切换振荡，不能把它的 Kd 原样用于 RL 策略输出。
    command.kp_joint = this->_data->control_parameters->initialization_kp;
    command.kd_joint = this->_data->control_parameters->initialization_kd;
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const std::size_t index = leg * kJointsPerLeg + joint;
      const float raw_target = kDm1RlDefaultJointPosition[index] +
        this->_data->control_parameters->rl_action_scale * rl_previous_action_[index];
      const float bounded_target = std::clamp(
        raw_target, static_cast<float>(joints.lower_limit(
          static_cast<Eigen::Index>(joint))), static_cast<float>(joints.upper_limit(
          static_cast<Eigen::Index>(joint))));
      rl_target_position_[index] += std::clamp(
        bounded_target - rl_target_position_[index], -max_target_step, max_target_step);
      command.position_desired(static_cast<Eigen::Index>(joint)) =
        static_cast<T>(rl_target_position_[index]);
    }
  }
  if (rl_policy_counter_ == interval - 1) {
  }
}

template<typename T>
void FSM_State_Locomotion<T>::onExit()
{
  iteration_ = 0;
  resetSwingTrajectories();
  rl_target_initialized_ = false;
}

template class FSM_State_Locomotion<float>;
