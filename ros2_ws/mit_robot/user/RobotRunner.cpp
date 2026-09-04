// 本文件按“估计 -> 安全初始化 -> 期望状态 -> FSM/WBC -> 关节命令”的顺序执行。
#include "RobotRunner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace
{
constexpr std::array<LegId, kNumLegs> kLegOrder{
  LegId::FR, LegId::FL, LegId::RR, LegId::RL};

bool isLocomotionMode(ControlMode mode) noexcept
{
  return mode == ControlMode::Locomotion || mode == ControlMode::WalkClassic ||
         mode == ControlMode::WalkRl || mode == ControlMode::StairsRl;
}
}

/*使用仿真器数据创建指定腿的传感器。
  这里传入：数据源：SIMULATOR；
  腿编号；MuJoCo 模型；MuJoCo 数据。
  以后如果切换真实硬件，只需要替换传感器创建方式，上层控制器不用修改。*/
RobotRunner::LegSensorOwners RobotRunner::makeLegSensors(
  const mjModel * model, const mjData * data)
{
  LegSensorOwners sensors;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    sensors[leg] = makeLeg(LegSource::SIMULATOR, kLegOrder[leg], model, data);
  }
  return sensors;
}

RobotRunner::LegSensorOwners RobotRunner::makeHardwareLegSensors()
{
  LegSensorOwners sensors;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    sensors[leg] = makeLeg(LegSource::HARDWARE, kLegOrder[leg]);
  }
  return sensors;
}

/*将裸指针数组传给状态估计器。
  传感器的生命周期仍然由 leg_sensors_ 管理。*/
RobotRunner::LegSensorPointers RobotRunner::sensorPointers(
  const LegSensorOwners & sensors)
{
  LegSensorPointers pointers{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    pointers[leg] = sensors[leg].get();
  }
  return pointers;
}

RobotRunner::RobotRunner(
  mjModel * model, const mjData * data, RobotType robot_type)
: model_(model), data_(data),
  control_time_step_(static_cast<float>(model == nullptr ? 0.0 : model->opt.timestep)),
  control_parameters_(makeRobotControlParameters<float>(robot_type)),
  /*由 main/SimulationBridge 明确选择的机器人模型。*/
  quadruped_(makeQuadruped<float>(robot_type)),
  /*腿部控制器*/
  leg_controller_(quadruped_),
  /*IMU 传感器*/
  imu_(makeImu(ImuSource::SIMULATOR, model, data)),
  /*四条腿传感器*/
  leg_sensors_(makeLegSensors(model, data)),
  /*腿部传感器指针*/
  leg_sensor_pointers_(sensorPointers(leg_sensors_)),
  /*步态调度器*/
  gait_scheduler_(control_time_step_)
{
  if (model_ == nullptr || data_ == nullptr) {
    throw std::invalid_argument("RobotRunner requires a MuJoCo model and data");
  }
  initializeController(OrientationEstimatorMode::SIMULATION_TRUTH);
}

RobotRunner::RobotRunner(float control_time_step, RobotType robot_type)
: control_time_step_(control_time_step), hardware_mode_(true),
  control_parameters_(makeRobotControlParameters<float>(robot_type)),
  quadruped_(makeQuadruped<float>(robot_type)), leg_controller_(quadruped_),
  imu_(makeImu(ImuSource::HARDWARE)),
  leg_sensors_(makeHardwareLegSensors()),
  leg_sensor_pointers_(sensorPointers(leg_sensors_)),
  gait_scheduler_(control_time_step_)
{
  if (!std::isfinite(control_time_step_) || control_time_step_ <= 0.0F ||
    control_time_step_ > 0.02F)
  {
    throw std::invalid_argument("hardware control time step must be in (0, 0.02]");
  }
  initializeController(OrientationEstimatorMode::IMU_FUSION);
}

void RobotRunner::initializeController(OrientationEstimatorMode orientation_mode)
{
  setHomeCalfContactsEnabled(control_parameters_.start_in_prone_home);
  joint_initialization_duration_ = control_parameters_.joint_initialization_duration;
  // 换站立速度要修改的地方：硬件和仿真共用的抬升速率限制。
  standing_height_rate_limit_ = control_parameters_.standing_height_rate;

  PositionVelocityEstimatorParameters<float> estimator_parameters;
  estimator_parameters.nominal_time_step = control_time_step_;
  estimator_parameters.maximum_time_step = std::max(
    estimator_parameters.maximum_time_step, estimator_parameters.nominal_time_step);
  state_estimator_ = std::make_unique<PositionVelocityEstimator<float>>(
    quadruped_, *imu_, leg_sensor_pointers_, orientation_mode,
    estimator_parameters, std::max(0.02F, 2.0F * control_time_step_));

  desired_state_.mode = control_parameters_.start_in_prone_home ?
    ControlMode::JointPd : ControlMode::BalanceStand;
  desired_state_.body_position_world.z() = quadruped_.nominalBodyHeight();
  desired_state_.valid = true;
  standing_height_target_ = quadruped_.nominalBodyHeight();
  standing_height_command_ = standing_height_target_;
  control_fsm_ = std::make_unique<ControlFSM<float>>(
    quadruped_, state_estimate_, joint_states_, leg_controller_, gait_scheduler_,
    desired_state_, control_time_step_);
  control_fsm_->setUseWbc(true);
  setWalkingForwardSpeed(defaultWalkingForwardSpeed());
  disableCommands();
}

bool RobotRunner::updateHardwareFeedback(
  const ImuData<float> & imu,
  const std::array<JointState<float>, kNumLegs> & joints,
  float now_s) noexcept
{
  if (!hardware_mode_ || !std::isfinite(now_s) || now_s < hardware_time_) {
    return false;
  }
  auto * hardware_imu = dynamic_cast<HardwareImu *>(imu_.get());
  if (hardware_imu == nullptr || !hardware_imu->update(imu)) {return false;}
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    auto * hardware_leg = dynamic_cast<HardwareLeg *>(leg_sensors_[leg].get());
    if (hardware_leg == nullptr || !hardware_leg->update(joints[leg])) {return false;}
  }
  hardware_time_ = now_s;
  return true;
}

float RobotRunner::currentTime() const noexcept
{
  return hardware_mode_ ? hardware_time_ :
    static_cast<float>(data_ == nullptr ? 0.0 : data_->time);
}

/*设置目标控制模式*/
void RobotRunner::setControlMode(ControlMode mode) noexcept
{
  // DM1 未执行站起前拒绝从趴卧保持状态直接跳入 WBC/步态控制。
  if (control_parameters_.start_in_prone_home &&
    (mode == ControlMode::BalanceStand || isLocomotionMode(mode)) &&
    !standingReady())
  {
    return;
  }
  // MPC、平地 RL 和楼梯 RL 必须经由 BalanceStand 完成独立初始化；不允许
  // 在 LOCOMOTION 内热切换策略、历史和执行器语义。
  if (isLocomotionMode(desired_state_.mode) && isLocomotionMode(mode) &&
    desired_state_.mode != mode)
  {
    return;
  }
  if (desired_state_.mode == mode) {return;}
  if (state_estimate_.valid) {
    // 切换站立/行走时只继承不可观测的水平原点和航向。roll/pitch 必须保持
    // 水平目标；若把切换瞬间的倾斜锁存下来，WBC 会主动维持后仰姿态。
    desired_state_.body_position_world.x() = state_estimate_.position_world.x();
    desired_state_.body_position_world.y() = state_estimate_.position_world.y();
    desired_state_.body_rpy << 0.0F, 0.0F, state_estimate_.rpy.z();
  }
  desired_state_.body_velocity_world.setZero();
  desired_state_.body_acceleration_world.setZero();
  desired_state_.body_angular_velocity.setZero();
  desired_state_.mode = mode;
}

bool RobotRunner::requestStandUp() noexcept
{
  if (!jointInitializationComplete() || !state_estimate_.valid) {return false;}
  const FSM_StateName state = control_fsm_->currentStateName();
  if (state == FSM_StateName::BALANCE_STAND || state == FSM_StateName::LOCOMOTION) {
    // “Stand up” 按钮也作为行走中的回站按钮，始终回到当前水平位置，避免
    // 点击后把机身瞬移回世界原点。
    desired_state_.body_position_world.x() = state_estimate_.position_world.x();
    desired_state_.body_position_world.y() = state_estimate_.position_world.y();
    desired_state_.body_position_world.z() = quadruped_.nominalBodyHeight();
    desired_state_.body_rpy << 0.0F, 0.0F, state_estimate_.rpy.z();
    desired_state_.body_velocity_world.setZero();
    desired_state_.body_acceleration_world.setZero();
    desired_state_.body_angular_velocity.setZero();
    desired_state_.mode = ControlMode::BalanceStand;
    setHomeCalfContactsEnabled(false);
    return true;
  }
  if (state != FSM_StateName::JOINT_PD && state != FSM_StateName::PASSIVE) {
    return false;
  }
  if (std::abs(state_estimate_.rpy.x()) > 0.20F ||
    std::abs(state_estimate_.rpy.y()) > 0.20F)
  {
    return false;
  }
  desired_state_.body_position_world.x() = state_estimate_.position_world.x();
  desired_state_.body_position_world.y() = state_estimate_.position_world.y();
  desired_state_.body_position_world.z() = quadruped_.nominalBodyHeight();
  desired_state_.body_rpy << 0.0F, 0.0F, state_estimate_.rpy.z();
  desired_state_.body_velocity_world.setZero();
  desired_state_.body_acceleration_world.setZero();
  desired_state_.body_angular_velocity.setZero();
  // Home 的加粗小腿接触代理只用于静止贴地；站起前关闭，运动阶段继续使用
  // 原始细碰撞体，避免正常摆腿被误判为小腿擦地。
  setHomeCalfContactsEnabled(false);
  // DM1 先贴地收腿到四足支撑区，再自动交给
  // BalanceStand/WBC 抬升机身，避免展开的趴卧腿直接发力导致前翻。
  desired_state_.mode = ControlMode::StandUp;
  return true;
}

bool RobotRunner::standingReady() const noexcept
{
  const FSM_StateName state = control_fsm_->currentStateName();
  return state == FSM_StateName::BALANCE_STAND ||
         state == FSM_StateName::LOCOMOTION;
}

void RobotRunner::setRlPolicy(RlPolicyPtr policy) noexcept
{
  control_fsm_->setRlPolicy(std::move(policy));
}

/*把速度传递给 FSM 内部的 Locomotion*/
void RobotRunner::setWalkingForwardSpeed(float speed)
{
  control_fsm_->setLocomotionForwardVelocity(speed);
}

void RobotRunner::setLocomotionVelocityCommand(
  float forward_velocity, float lateral_velocity, float yaw_rate)
{
  control_fsm_->setLocomotionVelocityCommand(
    forward_velocity, lateral_velocity, yaw_rate);
}

/*设置站立高度*/
void RobotRunner::setStandingHeight(float height)
{
  /*不能是 NaN；不能是 Inf；
    不能低于 0.18 m；不能高于 0.34 m。*/
  if (!std::isfinite(height) || height < minimumStandingHeight() ||
    height > maximumStandingHeight())
  {
    throw std::invalid_argument("standing height is outside the selected robot profile");
  }
  standing_height_target_ = height;
}


void RobotRunner::reset()
{
  // 只清除算法内部状态。MuJoCo 已经处理了 qpos/qvel，本控制器不“扶正”机器人。
  state_estimator_->reset();
  gait_scheduler_.initialize();
  state_estimate_ = StateEstimate<float>{};
  joint_states_ = std::array<JointState<float>, kNumLegs>{};
  desired_state_ = DesiredState<float>{};
  desired_state_.mode = control_parameters_.start_in_prone_home ?
    ControlMode::JointPd : ControlMode::BalanceStand;
  desired_state_.body_position_world.z() = quadruped_.nominalBodyHeight();
  desired_state_.valid = true;
  joint_initialization_started_ = false;
  joint_initialization_start_time_ = 0.0F;
  standing_height_command_initialized_ = false;
  desired_state_initialized_ = false;
  control_fsm_->initialize();
  setHomeCalfContactsEnabled(control_parameters_.start_in_prone_home);
  disableCommands();
}

void RobotRunner::setHomeCalfContactsEnabled(bool enabled) noexcept
{
  if (!control_parameters_.start_in_prone_home || model_ == nullptr) {return;}
  constexpr std::array<const char *, kNumLegs> names{
    "FR_home_calf_contact", "FL_home_calf_contact",
    "RR_home_calf_contact", "RL_home_calf_contact"};
  for (const char * name : names) {
    const int geom = mj_name2id(model_, mjOBJ_GEOM, name);
    if (geom < 0) {continue;}
    model_->geom_contype[geom] = enabled ? 1 : 0;
    model_->geom_conaffinity[geom] = enabled ? 1 : 0;
  }
}

void RobotRunner::updateStandingHeightCommand()
{
  // 首帧从当前实测高度起步，避免 Reset 后目标高度突跳。
  if (!standing_height_command_initialized_) {
    standing_height_command_ = std::clamp(
      state_estimate_.position_world.z(), minimumStandingHeight(),
      maximumStandingHeight());
    standing_height_command_initialized_ = true;
  }

  // 速率限制换算为“每个控制周期允许变化的最大高度”。
  // 如果时间步为 0.001 s，则每个控制周期最多改变0. × 0.001 = 0.00008 m：
  const float maximum_step =
    standing_height_rate_limit_ * control_time_step_;
  //计算目标高度和当前高度之间的误差
  const float error = standing_height_target_ - standing_height_command_;
  standing_height_command_ += std::clamp(error, -maximum_step, maximum_step);
  //将平滑后的高度写回 FSM 使用的期望状态
  desired_state_.body_position_world.z() = standing_height_command_;
}

/*关节初始化完成必须满足：
  初始化已经开始；当前仿真时间减去开始时间大于等于 0.4 秒。
  如果还没有开始，即使时间足够长，也返回 false。*/
bool RobotRunner::jointInitializationComplete() const noexcept
{
  return joint_initialization_started_ &&
         currentTime() - joint_initialization_start_time_ >=
         joint_initialization_duration_;
}

/*这个函数负责启动时平滑进入默认姿态。*/
void RobotRunner::prepareJointInitialization()
{
  if (!joint_initialization_started_) {
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      //锁存四条腿当前实际关节角，作为轨迹起点
      initial_joint_positions_[leg] = leg_controller_.datas[leg].q;
    }
    //记录初始化起始时间
    joint_initialization_start_time_ = currentTime();
    joint_initialization_started_ = true;
  }
  //计算已经经过的时间。
  const float elapsed =
    currentTime() - joint_initialization_start_time_;
  //将准备时间化为[0, 1]区间的相位，超过 1.0 的部分会被 clamp 掉。
  const float phase = std::clamp(
    elapsed / joint_initialization_duration_, 0.0F, 1.0F);
  // 三次 smoothstep 的起止速度均为零，比线性插值更不容易产生冲击。
  /*它的特点是：起点速度为零；终点速度为零；比线性插值冲击更小。*/
  const float blend = phase * phase * (3.0F - 2.0F * phase);
  /*计算 smoothstep 的时间导数，作为期望关节速度。上一步的导数形式*/
  const float blend_rate =
    6.0F * phase * (1.0F - phase) / joint_initialization_duration_;
  /*先清除所有旧控制模式命令，再开启腿部输出。*/
  leg_controller_.zeroCommand();
  leg_controller_.setEnabled(true);

  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    /*target,travel这两个是啥玩意*/
    const Vec3<float> target = control_parameters_.start_in_prone_home ?
      control_parameters_.motor_zero_position :
      quadruped_.leg(kLegOrder[leg]).joints.home_position;
    const Vec3<float> travel = target - initial_joint_positions_[leg];
    auto & command = leg_controller_.commands[leg];
    command.position_desired = initial_joint_positions_[leg] + blend * travel;
    command.velocity_desired = blend_rate * travel;
    // 初始化阶段恢复经过验证的 PD；行走提速只在 Locomotion 摆动腿生效，
    // 避免把更快的启动过程误认为电机行走速度提升。
    // DM1 Home 有四段小腿同时接地，必须从第一帧使用专用零位刚度抵抗
    // 接触反力；该增益仅在趴卧锁定阶段使用，不会进入 WBC/步态控制。
    command.kp_joint = control_parameters_.prone_home_joint_kp;
    command.kd_joint = control_parameters_.prone_home_joint_kd;
  }
}

//从 LegController 生成四条腿最终输出命令。
bool RobotRunner::collectJointCommands()
{
  bool commands_valid = true;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    /*调用 LegController::command()：
      读取当前腿反馈；计算笛卡尔足端力；使用 JᵀF 转换为关节力矩；
      限制前馈力矩；生成 JointCommand。*/
    joint_commands_[leg] = leg_controller_.command(
      kLegOrder[leg], currentTime());
    commands_valid = joint_commands_[leg].enabled && commands_valid;
  }
  if (!commands_valid) {disableCommands();}
  return commands_valid;
}

/*设置完整的目标状态。*/
void RobotRunner::setDesiredState(const DesiredState<float> & desired)
{
  /*检查所有主要目标量是否有限*/
  if (!desired.body_position_world.allFinite() ||
    !desired.body_velocity_world.allFinite() ||
    !desired.body_acceleration_world.allFinite() || !desired.body_rpy.allFinite() ||
    !desired.body_angular_velocity.allFinite())
  {/*发现 NaN 或 Inf 时拒绝整个目标状态*/
    throw std::invalid_argument("desired robot state contains a non-finite value");
  }
  /*如果目标模式是 BalanceStand，则同步更新目标站立高度。
                     这会触发 0.18～0.34 m 范围检查*/
  if (desired.mode == ControlMode::BalanceStand) {
    setStandingHeight(desired.body_position_world.z());
  }
  desired_state_ = desired;
}

/*统一关闭全部腿部命令。*/
void RobotRunner::disableCommands() noexcept
{
  leg_controller_.zeroCommand();
  leg_controller_.setEnabled(false);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    joint_commands_[leg] = JointCommand<float>{};
    joint_commands_[leg].leg = kLegOrder[leg];
  }
}

bool RobotRunner::run()
{
  // 1. 推进一步态并把“预计接触概率”交给状态估计器。
  //让步态调度器前进一步。它会更新：当前 gait phase；
  //每条腿的接触概率；摆动/支撑状态。
  gait_scheduler_.step();
  state_estimator_->setContactProbabilities(
    gait_scheduler_.gait_data.estimatorContactProbabilities());
  /*运行状态估计器。如果失败：*/
  if (!state_estimator_->run()) {
    disableCommands();
    return false;
  }

  // 2. 将估计状态同步给腿控制器；任一腿反馈无效就关闭全部输出。
  state_estimate_ = state_estimator_->result();
  joint_states_ = state_estimator_->orientationEstimator().jointStates();
  bool feedback_valid = true;
  for (const auto & state : joint_states_) {
    feedback_valid = leg_controller_.updateData(state) && feedback_valid;
  }
  if (!feedback_valid) {
    disableCommands();
    return false;
  }

  // 3. 刚启动或 Reset 后先把关节平滑带到名义姿态，再启用全身控制。
  if (!jointInitializationComplete()) {
    prepareJointInitialization();
    return collectJointCommands();
  }

  // 4. 首次进入闭环时从当前水平位置和航向建立参考。初始化期间产生的
  // roll/pitch 是需要消除的扰动，不能锁存成后续站立和行走的目标姿态。
  if (!desired_state_initialized_) {
    desired_state_.body_position_world = state_estimate_.position_world;
    desired_state_.body_rpy << 0.0F, 0.0F, state_estimate_.rpy.z();
    desired_state_.body_velocity_world.setZero();
    desired_state_.body_acceleration_world.setZero();
    desired_state_.body_angular_velocity.setZero();
    desired_state_.timestamp = state_estimate_.timestamp;
    desired_state_.valid = true;
    desired_state_initialized_ = true;
  }

  // 5. 更新高度目标，最后由 FSM 选择站立或行走控制器并生成命令。
  updateStandingHeightCommand();

  control_fsm_->runFSM();
  return collectJointCommands();
}
