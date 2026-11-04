/*! @file opendog_robot_runner.hpp
 *  @brief 硬件桥/仿真桥调用的 OpenDog 运行器：命令处理、趴卧初始化和趴下流程。
 */

#pragma once

#include <string>
#include <vector>

#include "control/opendog_algorithm.hpp"
#include "teleop/operator_command.hpp"

/**
 * @brief 硬件桥看到的唯一算法边界。
 *
 * 持有 OpenDog 命令状态、趴卧初始化/趴下流程，以及独立的估计器 / SQP-MPC / WBC。
 * 趴卧初始化和趴下只做关节插值，不经过优化器；其余模式交给 OpenDogAlgorithm。
 */
class OpenDogRobotRunner final
{
public:
  /// @param config_directory OpenDog 配置目录；@param period 控制周期，单位 s。
  OpenDogRobotRunner(const std::string & config_directory, float period);
  /// 配置算法，并把期望机身高度初始化为站立高度。
  bool configure();
  /// 执行一个控制周期；初始化/趴下期间输出关节插值命令。
  bool compute(const ControlInputSnapshot & input, ControlOutput & output);
  /// 停止 MPC 线程。
  void stop() noexcept {algorithm_.stop();}
  /// 复位算法和命令状态，下一周期从当前姿态重新执行趴卧初始化。
  void reset() noexcept;
  /// 取消尚未执行的站立请求。
  void cancelPendingRequests() noexcept {stand_up_pending_ = false;}
  /// 记录本周期新到达的运动按键，同一运动的重复按下会递增序号。
  void acceptCommands(
    const teleop::OperatorCommandArbiter & arbiter,
    const std::vector<teleop::OperatorCommand> & commands);
  /**
   * @brief 处理仲裁器中的停止、站立、趴下和运动请求，更新期望状态。
   * @param imu 用于把机身系速度命令旋转到世界系。
   * @param active 原样返回，供调用方链式使用。
   */
  bool handleCommands(
    teleop::OperatorCommandArbiter & arbiter,
    const ImuData<float> & imu, bool active);
  /// 请求站立；初始化未完成时先挂起，完成后自动执行。成功切换时返回 true。
  bool requestStandUp();
  /// 返回当前模式的日志标签。
  const char * stateLabel() const noexcept;
  /// 每秒打印一次快照、电机反馈和策略的新鲜度，仅用于监测。
  void reportPublishTiming(double snapshot_age_ms, double motor_age_ms, bool ok) const;
  /// 当前期望机身状态。
  const DesiredState<float> & desiredState() const noexcept {return desired_;}

private:
  /// 切换模式并把期望位置/航向锁定为当前估计值，速度清零。
  void holdCurrentPose(ControlMode mode);
  /// 开始 P 趴下流程。
  void startProneDown();
  /// 从记录的初始关节角平滑插值到趴卧姿态。
  bool computeJointInitialization(const ControlInputSnapshot & input, ControlOutput & output);
  OpenDogAlgorithm algorithm_;                                   ///< 估计器 + MPC + WBC。
  DesiredState<float> desired_;                                  ///< 当前期望机身状态。
  StateEstimate<float> estimate_;                                ///< 最近一次算法状态估计。
  std::uint64_t motion_sequence_ = 0;                            ///< 运动按键序号。
  std::uint64_t output_sequence_ = 0;                            ///< 输出命令序号。
  std::array<Vec3<float>, kNumLegs> initial_joint_positions_{};  ///< 插值起点关节角。
  float initialization_start_time_ = 0.0F;                       ///< 插值开始时间，s。
  bool initialization_active_ = false;    ///< 正在执行趴卧初始化或趴下插值。
  bool initialization_started_ = false;   ///< 已记录插值起点。
  bool initialization_complete_ = false;  ///< 插值已到达趴卧姿态。
  bool stand_up_pending_ = false;         ///< 初始化期间收到的站立请求。
  // P 趴下：复用回零插值，前馈从站立时最后一帧 WBC 力矩逐步卸载到 0。
  // 注意：`{}` 不会把 Eigen 向量清零，构造函数和 reset() 里显式 setZero。
  std::array<Vec3<float>, kNumLegs> prone_feedforward_;
  float initialization_duration_ = 1.2F;  ///< 本次插值时长，s。
  bool prone_down_active_ = false;        ///< 当前插值是 P 趴下而不是启动初始化。
};
