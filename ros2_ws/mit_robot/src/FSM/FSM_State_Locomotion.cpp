// 行走状态的数据流：状态估计 -> MPC 接触力规划 -> WBC 关节力矩。
#include "FSM/FSM_State_Locomotion.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
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
  // MPC 优化仍以约25 Hz刷新；经典步态的10段接触周期在 setGait() 中同步，
  // 每段50 ms。
  const std::size_t mpc_interval = static_cast<std::size_t>(std::max(
      T(1), std::round(T(0.04) / control_fsm_data->control_time_step)));
  const std::size_t gait_segment_interval = static_cast<std::size_t>(std::max(
      T(1), std::round(T(0.05) / control_fsm_data->control_time_step)));
  //创建 MPC 控制器。传入:机器人模型；控制周期；MPC 步态时间间隔。  
  mpc_ = std::make_unique<mpc::ConvexMPCLocomotion<T>>(
    *control_fsm_data->quadruped, control_fsm_data->control_time_step,
    mpc_interval, mpc::SolverSettings<T>{}, gait_segment_interval);
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
  wbc_ctrl_->setJointGains(
    parameters.locomotion_joint_kp, parameters.locomotion_joint_kd);
  wbc_ctrl_->setMaxNormalForce(parameters.maximum_normal_force);
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
  startup_step_blend_.fill(T(0.4));
  active_mode_ = this->_data->desired_state->mode;
  rl_history_.fill(0.0F);
  rl_history_initialized_ = false;
  rl_previous_action_.fill(0.0F);
  rl_policy_counter_ = 0;
  safety_fallback_transition_ = false;
  prev_joint_target_initialized_ = false;
  /*步态由 ControlFSMData::locomotion_gait 选择；经典运动统一落到
    TROT_WALK（60% 支撑，0.5 s 周期），保留四足重叠换载区。
    MPC 与 GaitScheduler 必须同时设置：只改一个就会出现"MPC 认为后腿支撑、
    GaitScheduler 认为后腿摆动"的错相，导致 WBC 约束错误、足端轨迹漂移。*/
  applyRequestedGait(true);
}

/*把请求的步态同步给 MPC 接触表和 GaitScheduler，并把两者的相位对齐。*/
template<typename T>
void FSM_State_Locomotion<T>::applyRequestedGait(bool force)
{
  const GaitType requested = this->_data->locomotion_gait;
  if (!force && gait_applied_ && requested == applied_gait_) {return;}
  // 中途换步态：MPC 的相位由内部周期计数换算，而 GaitScheduler 换步态时会把
  // 相位重置到各自的 phase_offset。这里把 MPC 一起归零，两者才不会错相。
  const bool switching = gait_applied_ && requested != applied_gait_;
  if (switching) {mpc_->initialize();}
  applied_gait_ = requested;
  gait_applied_ = true;
  // 实机经典步态统一使用带四足重叠区的 60% 支撑率。外部保留 TROT 请求
  // 兼容现有按键/API，但不再进入无承重余量的 50% 支撑分支。
  mpc_->setGait(GaitType::TROT_WALK);
  this->_data->gait_scheduler->requestGait(GaitType::TROT_WALK);
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
  mpc_->setForwardVelocity(velocity);
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
  if (std::abs(estimate.rpy.x()) > ori::deg2rad(max_roll_degrees) ||
    std::abs(estimate.rpy.y()) > ori::deg2rad(max_pitch_degrees))
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
  const Vec2<T> desired_velocity =
    locomotion_result.command.body_velocity_world.template head<2>();
  const Vec2<T> measured_velocity =
    estimate.velocity_world.template head<2>();
  const Vec2<T> velocity_error =
    measured_velocity - desired_velocity;
  const T swing_time = locomotion_result.swing_time[leg];
  const T half_stance_time = T(0.5) * locomotion_result.stance_time[leg];
  // 摆动落足点分成两条互不影响的路径，用期望速度连续过渡：
  //   1) 行进（期望速度非零，含前进/后退/横移/原地旋转）：使用实测速度主导、
  //      目标速度参与半个支撑期的落足公式；
  //   2) 原地踏步（期望速度为零）：不能再用实测速度前馈，因为它等于把当前速度
  //      原样维持住，零命令下任何微小扰动都会被保留，四条腿乱抖、机身匀速漂移。
  //      这一路改用速度误差捕获项，把足端收敛回机身正下方。
  // 方向运动保留实测速度主导的落足反馈，只在半个支撑期内把实测/目标速度
  // 各取一半，再加小幅速度误差捕获。
  constexpr T kTravelCaptureGain = T(0.08);
  // DM1 零速原地踏步的纵向/横向滑移不同，分别调节已有速度捕获项；
  // 有明确行走速度时两轴仍连续过渡到原有 0.08 参数。
  const Vec2<T> kInPlaceCaptureGain = Vec2<T>::Constant(T(0.25));
  constexpr T kInPlaceVelocityThreshold = T(0.05);
  const T desired_speed = desired_velocity.norm();
  const T in_place_blend = std::clamp(
    desired_speed / kInPlaceVelocityThreshold, T(0), T(1));
  const Vec2<T> in_place_step =
    kInPlaceCaptureGain.cwiseProduct(velocity_error);
  const Vec2<T> stance_velocity =
    T(0.5) * (measured_velocity + desired_velocity);
  const Vec2<T> travel_step =
    swing_time * measured_velocity + half_stance_time * stance_velocity +
    kTravelCaptureGain * velocity_error;
  Vec2<T> step = (T(1) - in_place_blend) * in_place_step +
    in_place_blend * travel_step;
  // 首两次摆动逐级接入实测反馈，第三次恢复完整反馈。
  const T startup_blend = startup_step_blend_[leg];
  const Vec2<T> commanded_step =
    (swing_time + half_stance_time) * desired_velocity;
  step = (T(1) - startup_blend) * commanded_step + startup_blend * step;
  if (step.norm() > maximum_step_length_) {
    step *= maximum_step_length_ / step.norm();
  }
  // 落脚点始终相对当前机身规划：nominal_foot_world 跟随状态估计的机身位置，
  // step 提供速度前馈与落点捕获项。零速度时前馈为零，靠捕获项把足端收敛回
  // 机身正下方，即为原地踏步。
  // 注意：曾把零速度下的落点"锚定"到离地瞬间的绝对世界坐标，等于丢掉捕获项，
  // 实测机身会一路乱摆并摔倒（复测确认），因此落点必须继续跟随机身。
  Vec3<T> landing_position = initial_position;
  landing_position.template head<2>() = nominal_foot_world + step;
  // z 始终使用离地瞬间锁存的地面高度，不能跟随摆动中的实测足高漂移。
  landing_position.z() = initial_position.z();
  trajectory.setFinalPosition(landing_position);
  T trajectory_swing_height = swing_height_;
  if (startup_blend < T(1)) {
    trajectory_swing_height *= T(0.375) + T(0.625) * startup_blend;
  }
  trajectory.setHeight(trajectory_swing_height);
  startup_step_blend_[leg] = std::min(T(1), startup_blend + T(0.3));
  swing_active_[leg] = true;
}

template<typename T>
void FSM_State_Locomotion<T>::constrainJointTargets(
  const std::array<bool, kNumLegs> & contact_state)
{
  // WBC 最大单周期关节目标步长：0.15 rad/step @ 500 Hz = 75 rad/s 的等效速率。
  // 该值远高于任何物理关节速度，只阻止 WBC 在接触状态跳变或状态估计突变时
  // 产生的瞬间大角度目标跳变（如接触融合退出瞬间 hip q_des 跳到 -1.57 rad）。
  constexpr T kMaxPositionStep = T(0.15);

  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto leg_id = static_cast<LegId>(leg);
    const auto & joints = this->_data->quadruped->leg(leg_id).joints;
    auto & command = this->_data->leg_controller->commands[leg];

    if (!contact_state[leg]) {
      command.velocity_desired =
        command.velocity_desired.cwiseProduct(swing_joint_velocity_scale_);
    }
    command.velocity_desired = command.velocity_desired.cwiseMin(
      joints.velocity_limit).cwiseMax(-joints.velocity_limit);
    command.position_desired = command.position_desired.cwiseMax(
      joints.lower_limit).cwiseMin(joints.upper_limit);

    // 对 WBC 输出的关节目标施加逐帧变化率限制，防止接触融合退出等事件
    // 导致目标瞬间跳变到关节极限、产生无法跟踪的巨大 PD 误差。
    if (prev_joint_target_initialized_) {
      for (Eigen::Index joint = 0;
        joint < static_cast<Eigen::Index>(kJointsPerLeg); ++joint)
      {
        const T delta =
          command.position_desired[joint] - prev_joint_position_desired_[leg][joint];
        if (std::abs(delta) > kMaxPositionStep) {
          command.position_desired[joint] =
            prev_joint_position_desired_[leg][joint] +
            std::copysign(kMaxPositionStep, delta);
        }
      }
    }
    prev_joint_position_desired_[leg] = command.position_desired;
  }
  prev_joint_target_initialized_ = true;
}

template<typename T>
void FSM_State_Locomotion<T>::LocomotionControlStep()
{
  // 同步当前步态请求；未变化时是空操作。
  applyRequestedGait(false);
  // MPC 输入必须使用统一的世界坐标系，否则反作用力方向会与 WBC 不一致。
  const auto feet_world = footPositionsWorld();
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
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    //将 MPC 计算出的世界坐标系反作用力传给 WBC。
    wbc_data_.Fr_des[leg] = result.reaction_forces_world[leg];
    //将布尔接触状态转成浮点数：
    wbc_data_.contact_state[leg] = result.contact_state[leg] ? T(1) : T(0);

    if (result.contact_state[leg]) {
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
      // 行走层只负责自身的目标范围；所有状态共用的连续力矩约束由
      // LegController 最终命令出口统一执行。
      constrainJointTargets(result.contact_state);
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
  prev_joint_target_initialized_ = false;
}

template class FSM_State_Locomotion<float>;
