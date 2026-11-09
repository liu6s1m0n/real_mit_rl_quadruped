/*! @file opendog_reference.hpp
 *  @brief OpenDog 目标轨迹生成和步态切换模块。
 */

#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include "common/robot_types.hpp"
#include "teleop/operator_command.hpp"

/**
 * @brief OpenDog 的目标位姿 / 速度 / 站立轨迹生成。
 *
 * 与求解器和硬件桥分离；这个边界上的速度都在世界坐标系中。
 */
class OpenDogReference
{
public:
  /// 从 reference 文件读取速度上限、站立高度和默认关节角，从 task 文件读取 MPC 预测时长。
  OpenDogReference(const std::string & reference_file, const std::string & task_file);
  /// 根据期望状态和当前观测生成 MPC 目标轨迹。
  ocs2::TargetTrajectories update(
    const DesiredState<float> & desired, const ocs2::SystemObservation & observation);
  /// 清除站立轨迹缓存，下次 StandUp 从当前状态重新生成。
  void reset() noexcept {standing_up_ = false; anchor_valid_ = false;}

private:
  /// 生成从当前位姿到 pose 的两点轨迹，在 reaching_time 到达。
  ocs2::TargetTrajectories poseToTrajectory(
    const ocs2::vector_t & pose, const ocs2::SystemObservation & observation,
    double reaching_time) const;
  double displacement_velocity_ = 0.0;  ///< 平移目标速度，m/s。
  double rotation_velocity_ = 0.0;      ///< 转向目标角速度，rad/s。
  double com_height_ = 0.0;             ///< 站立质心高度，m。
  double horizon_ = 0.0;                ///< MPC 预测时长，s。
  double stand_up_duration_ = 0.0;      ///< 站立轨迹时长，s。
  ocs2::vector_t default_joints_ = ocs2::vector_t::Zero(12);  ///< 站立默认关节角（OpenDog 腿序）。
  bool standing_up_ = false;            ///< 已生成并缓存站立轨迹。
  bool anchor_valid_ = false;           ///< 运动模式的水平位置和偏航锚点是否有效。
  double anchor_time_ = 0.0;            ///< 锚点上次更新的时间，s。
  Eigen::Vector2d anchor_xy_ = Eigen::Vector2d::Zero();  ///< 按期望速度积分的水平位置锚点，m。
  double anchor_yaw_ = 0.0;             ///< 按期望角速度积分的偏航锚点，rad。
  ocs2::TargetTrajectories stand_up_target_;  ///< 缓存的站立轨迹，站立期间保持不变。
};

/**
 * @brief 在每次 MPC 求解前同步插入步态的模块。
 *
 * 插入方式与原 OCS2 GaitReceiver 相同，但请求来自共享的命令快照，
 * 而不是订阅 OCS2 ROS 消息。set* 由控制线程调用，preSolverRun 在 MPC 线程执行。
 */
class OpenDogGaitCommand final : public ocs2::SolverSynchronizedModule
{
public:
  /// 从 gait 文件加载 stance、trot 和两种对角单步模板。
  OpenDogGaitCommand(
    const std::string & gait_file,
    std::shared_ptr<ocs2::legged_robot::GaitSchedule> schedule);
  /// Locomotion 模式请求 trot，其余模式请求 stance。
  void setMode(ControlMode mode);
  /// 请求一次对角踏步；sequence 区分不同的按键事件。
  void setMotion(teleop::Motion motion, std::uint64_t sequence);
  /// 恢复为支撑步态；必须在 MPC 线程停止后调用。
  void reset() noexcept;
  /// 请求有变化时把对应步态模板插入到预测窗口末尾。
  void preSolverRun(
    double init_time, double final_time, const ocs2::vector_t & state,
    const ocs2::ReferenceManagerInterface & reference_manager) override;
  void postSolverRun(const ocs2::PrimalSolution &) override {}

private:
  std::shared_ptr<ocs2::legged_robot::GaitSchedule> schedule_;  ///< MPC 使用的步态表。
  ocs2::legged_robot::ModeSequenceTemplate stance_;            ///< 四足支撑模板。
  ocs2::legged_robot::ModeSequenceTemplate trot_;              ///< 对角小跑模板。
  ocs2::legged_robot::ModeSequenceTemplate single_step_[2];    ///< LF/RH 和 RF/LH 单次踏步模板。
  /// 低两位选择对角（0 无、1 LF/RH、2 RF/LH），其余位是按键序号。
  std::atomic<std::uint64_t> requested_motion_{0};
  std::uint64_t applied_motion_ = 0;          ///< 已插入的动作请求，仅 MPC 线程访问。
  std::atomic<bool> requested_trot_{false};   ///< 控制线程请求的步态。
  bool applied_trot_ = false;                 ///< 已插入的步态，仅 MPC 线程访问。
};
