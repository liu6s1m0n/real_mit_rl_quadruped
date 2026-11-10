/*! @file opendog_reference.cpp
 *  @brief OpenDogReference / OpenDogGaitCommand 实现。
 *
 *  目标轨迹公式移植自 OpenDog TargetTrajectoriesAdapter.cpp（qiayuan, 2022/7/24）；
 *  步态插入沿用原 OCS2 GaitReceiver 的做法。
 */
#include "control/opendog_reference.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/LoadData.h>

namespace
{
/// 运动模式水平锚点最多领先当前位置的距离，m。
constexpr double kMaximumAnchorLead = 0.15;
/// 运动模式偏航锚点最多领先当前偏航的角度，rad。
constexpr double kMaximumYawAnchorLead = 0.3;
}  // namespace

OpenDogReference::OpenDogReference(
  const std::string & reference_file, const std::string & task_file)
{
  ocs2::loadData::loadCppDataType(reference_file, "targetDisplacementVelocity", displacement_velocity_);
  ocs2::loadData::loadCppDataType(reference_file, "targetRotationVelocity", rotation_velocity_);
  ocs2::loadData::loadCppDataType(reference_file, "comHeight", com_height_);
  ocs2::loadData::loadCppDataType(reference_file, "standUpDuration", stand_up_duration_);
  ocs2::loadData::loadEigenMatrix(reference_file, "defaultJointState", default_joints_);
  ocs2::loadData::loadCppDataType(task_file, "mpc.timeHorizon", horizon_);
  boost::property_tree::ptree settings;
  boost::property_tree::read_info(reference_file, settings);
  velocity_gain_ = std::max(0.0, settings.get<double>("velocityIntegral.gain", velocity_gain_));
  velocity_limit_ = std::max(0.0, settings.get<double>("velocityIntegral.limit", velocity_limit_));
  velocity_leak_time_ = std::max(1e-3, settings.get<double>("velocityIntegral.leakTime", velocity_leak_time_));
  height_gain_ = std::max(0.0, settings.get<double>("heightIntegral.gain", height_gain_));
  height_limit_ = std::max(0.0, settings.get<double>("heightIntegral.limit", height_limit_));
  pitch_velocity_gain_ = settings.get<double>("pitchVelocityGain", pitch_velocity_gain_);
  height_velocity_gain_ = settings.get<double>("heightVelocityGain", height_velocity_gain_);
}

ocs2::TargetTrajectories OpenDogReference::poseToTrajectory(
  const ocs2::vector_t & pose, const ocs2::SystemObservation & observation,
  double reaching_time, double pitch_offset, double height_offset) const
{
  // pose 为 (x, y, z, yaw, pitch, roll)；起点高度取站立高度（含高度积分修正），pitch 取 pitch_offset，roll 取 0。
  ocs2::vector_t current_pose = observation.state.segment<6>(6);
  current_pose(2) = com_height_ + height_integral_ + height_offset;
  current_pose(4) = pitch_offset;
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
  // 趴卧初始化和趴下由 runner 的关节插值完成，不经过这里；算法只接收
  // StandUp、BalanceStand 和 Locomotion。
  // 高度积分（2026-11-10）：实机站起结束只有约 0.30 m、踏步时 0.33–0.34 m（目标 0.32），
  // 各模式稳态高度不同。站起爬升段之外都积分 (comHeight − 机身高度)，叠加到高度目标上。
  const double dt = last_update_time_ < 0.0 ? 0.0 :
    std::clamp(observation.time - last_update_time_, 0.0, 0.1);
  last_update_time_ = observation.time;
  const bool climbing = desired.mode == ControlMode::StandUp &&
    (!standing_up_ || observation.time < stand_up_target_.timeTrajectory.back());
  if (height_gain_ > 0.0 && !climbing) {
    height_integral_ = std::clamp(
      height_integral_ + height_gain_ * (com_height_ - observation.state(8)) * dt,
      -height_limit_, height_limit_);
  }
  if (desired.mode == ControlMode::StandUp) {
    // 首次进入时以当前状态为起点生成站立轨迹，之后保持该轨迹直到离开 StandUp。
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
    auto target = stand_up_target_;
    target.stateTrajectory.back()(8) += height_integral_;
    return target;
  }
  standing_up_ = false;
  if (desired.mode != ControlMode::Locomotion) {
    anchor_valid_ = false;
    velocity_integral_.setZero();
  }
  ocs2::vector_t pose(6);
  pose << desired.body_position_world.cast<double>(),
    double(desired.body_rpy.z()), double(desired.body_rpy.y()), double(desired.body_rpy.x());
  pose(2) += height_integral_;
  if (desired.mode == ControlMode::Locomotion) {
    // 运动模式：水平目标取按期望速度积分的锚点再外推一个预测时长，轨迹各点带期望线速度。
    // 锚点不随当前位置重置，速度跟不上或原地漂移时误差会累积，MPC 据此追回；
    // 锚点与当前位置的距离限幅，被卡住时不无限累积。
    // 偏航同样锁定（2026-11-06 换 HFI IMU 后 yaw 静止漂移约 0.1°/20 s）：
    // 原先每拍跟随当前值，原地踏步会慢慢转身，转向只跟到命令的约 22%。
    const Eigen::Vector2d current_xy = observation.state.segment<2>(6);
    const Eigen::Vector2d velocity_xy = desired.body_velocity_world.head<2>().cast<double>();
    const double current_yaw = observation.state(9);
    const double yaw_rate = desired.body_angular_velocity.z();
    double dt = 0.0;
    if (!anchor_valid_) {
      anchor_xy_ = current_xy;
      anchor_yaw_ = current_yaw;
      anchor_valid_ = true;
      velocity_integral_.setZero();
    } else {
      dt = std::max(0.0, observation.time - anchor_time_);
      anchor_xy_ += velocity_xy * dt;
      anchor_yaw_ += yaw_rate * dt;
    }
    // 速度外环积分（2026-11-09）：实机同样的速度命令，后退和右移偏快、前进和左移偏慢（有约 8 N 的持续后向扰动，
    // MPC 只按位置误差比例纠正，总留有方向性误差）。有平移命令时在机体系积分命令与质心速度之差，
    // 叠加到给 MPC 的速度参考；锚点仍按原命令积分。
    const double cy = std::cos(current_yaw);
    const double sy = std::sin(current_yaw);
    const auto to_body = [cy, sy](const Eigen::Vector2d & v) {
        return Eigen::Vector2d(cy * v.x() + sy * v.y(), -sy * v.x() + cy * v.y());
      };
    if (velocity_gain_ > 0.0 && dt > 0.0) {
      const Eigen::Vector2d command_body = to_body(velocity_xy);
      if (command_body.norm() > 0.02) {
        const Eigen::Vector2d error = command_body - to_body(observation.state.head<2>());
        velocity_integral_ = (velocity_integral_ + velocity_gain_ * error * dt).cwiseMax(-velocity_limit_)
          .cwiseMin(velocity_limit_);
      } else {
        velocity_integral_ *= std::exp(-dt / velocity_leak_time_);
      }
    }
    const Eigen::Vector2d reference_xy = velocity_xy + Eigen::Vector2d(
      cy * velocity_integral_.x() - sy * velocity_integral_.y(),
      sy * velocity_integral_.x() + cy * velocity_integral_.y());
    anchor_time_ = observation.time;
    const Eigen::Vector2d lead = anchor_xy_ - current_xy;
    if (lead.norm() > kMaximumAnchorLead) {
      anchor_xy_ = current_xy + lead * (kMaximumAnchorLead / lead.norm());
    }
    // 偏航误差按 [-π, π] 取最短方向，与状态里的偏航是否连续展开无关。
    const double yaw_lead = std::remainder(anchor_yaw_ - current_yaw, 2.0 * M_PI);
    anchor_yaw_ = current_yaw + std::clamp(yaw_lead, -kMaximumYawAnchorLead, kMaximumYawAnchorLead);
    pose.head<2>() = anchor_xy_ + horizon_ * reference_xy;
    pose(3) = anchor_yaw_ + horizon_ * yaw_rate;
    // 前进/后退的俯仰前馈（2026-11-10）：实机前进低头约 0.3°、后退抬头约 0.5°（相对原地），
    // 目标俯仰取 −增益 × 机体系前进命令速度。
    const double pitch_offset = -pitch_velocity_gain_ * to_body(velocity_xy).x();
    pose(4) += pitch_offset;
    // 高度前馈（2026-11-10）：实机前进高约 1 cm、后退低约 1.3 cm（相对原地），高度积分太慢跟不上短时运动。
    const double height_offset = -height_velocity_gain_ * to_body(velocity_xy).x();
    pose(2) += height_offset;
    auto target = poseToTrajectory(
      pose, observation, observation.time + horizon_, pitch_offset, height_offset);
    for (auto & state : target.stateTrajectory) {
      state.head<3>() = desired.body_velocity_world.cast<double>();
      state.head<2>() = reference_xy;
    }
    return target;
  }
  // 站立平衡：按平移/转向速度上限估算到达目标位姿所需时间。
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
  // 两组模板分别对应左前/右后和右前/左后的单次对角支撑切换。
  single_step_{
    ocs2::legged_robot::loadModeSequenceTemplate(gait_file, "single_step_lf_rh", false),
    ocs2::legged_robot::loadModeSequenceTemplate(gait_file, "single_step_rf_lh", false)}
{}

void OpenDogGaitCommand::setMode(ControlMode mode)
{
  requested_trot_.store(mode == ControlMode::Locomotion);
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
  // 在求解线程 join 之后调用，恢复支撑步态表和模板。
  schedule_->insertModeSequenceTemplate(stance_, 0.0, stance_.switchingTimes.back());
  schedule_->setModeSchedule({{stance_.switchingTimes.back()}, {15, 15}});
}

void OpenDogGaitCommand::preSolverRun(
  double init_time, double final_time, const ocs2::vector_t &,
  const ocs2::ReferenceManagerInterface &)
{
  // 请求与上次插入的一致时不改动步态表。
  const bool trot = requested_trot_.load();
  const auto motion = trot ? requested_motion_.load() : 0;
  if (trot == applied_trot_ && motion == applied_motion_) {return;}
  const auto horizon = final_time - init_time;
  const auto diagonal = motion & 3U;
  if (diagonal == 0) {
    schedule_->insertModeSequenceTemplate(trot ? trot_ : stance_, final_time, horizon);
  } else {
    // 保留当前预测窗口的前缀，只追加一次有限时长的对角踏步。
    // 踏步结束后明确回到支撑态，避免按键保持时重复循环。
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
