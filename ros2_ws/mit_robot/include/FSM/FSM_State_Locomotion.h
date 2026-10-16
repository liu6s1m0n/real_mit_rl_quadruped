/**
 * @file FSM_State_Locomotion.h
 * @brief 行走状态：MPC 规划接触力和落脚运动，WBC 将结果转换为关节命令。
 */
#ifndef MYMIT_ROBOT_FSM_STATE_LOCOMOTION_H_
#define MYMIT_ROBOT_FSM_STATE_LOCOMOTION_H_

#include <array>
#include <vector>
#include <memory>

#include "FSM/FSM_State.h"
#include "MPC/ConvexMPCLocomotion.h"
#include "WBC/LocomotionCtrl/LocomotionCtrl.hpp"
#include "controller/FootSwingTrajectory.hpp"
#include "controller/RlPolicy.hpp"

template < typename T >
class FSM_State_Locomotion: public FSM_State < T >
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //ControlFSMData<T>  状态估计器；腿部控制器；四足机器人模型；
  //期望状态；电机命令；用户控制输入；FSM 数据
  explicit FSM_State_Locomotion(ControlFSMData < T > * control_fsm_data);
  ~FSM_State_Locomotion() override = default;

  void onEnter() override;
  void run() override;
  FSM_StateName checkTransition() override;
  TransitionData < T > transition() override;
  void onExit() override;
  /** 将用户层速度命令传递给本状态持有的 MPC。 */
  void setForwardVelocity(T velocity);
  /** 设置机身系前后、左右和自转速度。 */
  void setVelocityCommand(T forward_velocity, T lateral_velocity, T yaw_rate);

  /** 返回最近一次交给 WBC 的单腿世界系足端目标，便于运行时诊断。 */
  const Vec3 < T > & footPositionTargetWorld(LegId leg) const noexcept
  {
    return wbc_data_.pFoot_des[static_cast < std::size_t > (leg)];
  }

  /** 返回指定腿是否正沿已锁存的摆动轨迹运动。 */
  bool swingLegActive(LegId leg) const noexcept
  {
    return swing_active_[static_cast < std::size_t > (leg)];
  }

private:
  //核心函数
  void LocomotionControlStep();
  void RlControlStep();
  bool isRlMode() const noexcept;
  bool buildRlObservation(
    std::array<float, kRlObservationSize> & observation) const;
  //检查当前是否适合继续行走。
  bool locomotionSafe() const;
  std::array < Vec3 < T >, kNumLegs > footPositionsWorld() const;
  void resetSwingTrajectories() noexcept;
  /**
   * @brief 把 ControlFSMData 请求的步态同步到 MPC 接触表和 GaitScheduler。
   *
   * 两个系统的相位必须一致，否则会出现"MPC 认为支撑、调度器认为摆动"的错相。
   * 中途换步态时同时把 MPC 的相位计数归零，与调度器重置后的相位对齐。
   * @param force 为 true 时无论是否变化都重新下发（进入 Locomotion 时使用）。
   */
  void applyRequestedGait(bool force);
  /** 约束行走关节目标范围和摆动腿速度。 */
  void constrainJointTargets();
  void startSwingTrajectory(
    std::size_t leg, const Vec3 < T > & initial_position,
    const mpc::LocomotionResult < T > & locomotion_result);
  /*MPC 控制器。
    负责输出：身体期望状态；每条腿的地面反作用力；接触状态；
    摆动相位；摆动时间；支撑时间。*/
  std::unique_ptr < mpc::ConvexMPCLocomotion < T >> mpc_;
  /*行走专用 WBC 控制器。
    它会把：身体任务；足端摆动任务；支撑腿接触约束；MPC 反作用力；
    转换成关节控制命令。*/
  std::unique_ptr < LocomotionCtrl < T >> wbc_ctrl_;
  /*这是 MPC 到 WBC 的数据中间层。
    包含：pBody_des,vBody_des,aBody_des,pBody_RPY_des,vBody_Ori_des
    以及每条腿的：pFoot_des,vFoot_des,aFoot_des,Fr_des,contact_state*/
  LocomotionCtrlData < T > wbc_data_;
  /*四条腿各自拥有一条摆动轨迹对象。每条腿单独保存：
  起点；终点；摆动高度；当前相位；位置；速度；加速度。*/
  std::array < FootSwingTrajectory < T >, kNumLegs > swing_trajectories_ {};
  /*记录每条腿是否已经开始当前摆动周期。*/
  std::array < bool, kNumLegs > swing_active_ {};
  /*各腿从期望速度落点过渡到完整实测速度反馈的进度。*/
  std::array<float, kRlObservationSize * kRlHistoryLength> rl_history_{};
  bool rl_history_initialized_ = false;
  std::array<float, kRlActionSize> rl_previous_action_{};
  std::array<float, kRlActionSize> rl_target_position_{};
  bool rl_target_initialized_ = false;
  Vec3<T> rl_velocity_command_ = Vec3<T>::Zero();
  std::size_t rl_policy_counter_ = 0;
  ControlMode active_mode_ = ControlMode::WalkClassic;
  /*摆动腿抬脚高度，由 DM1 控制参数覆盖。*/
  T swing_height_ = T(0.10);
  /*足端相对髋的横向安全边界，必须容纳该机型名义足宽和横移步长。*/
  T maximum_lateral_foot_offset_ = T(0.18);
  /*单个步周期内最大水平步长为 15 cm。*/
  T maximum_step_length_ = T(0.30);
  /*安全条件触发的回站不再重复执行已经不安全的 Locomotion 控制帧。*/
  bool safety_fallback_transition_ = false;
  std::size_t iteration_ = 0;
  T entry_transition_time_ = T(0);
  bool entry_transition_active_ = false;
  /*当前已下发给 MPC/GaitScheduler 的步态，用于检测中途换步态。*/
  GaitType applied_gait_ = GaitType::TROT;
  bool gait_applied_ = false;
  T load_transfer_elapsed_ = T(0);
  std::size_t load_transfer_bucket_ = static_cast<std::size_t>(-1);
  std::size_t stage1_stable_frames_ = 0;
  std::size_t load_transfer_ready_frames_ = 0;
  bool load_transfer_gate_open_ = false;
  /*4 号动作时钟连续未达标的帧数，用于超时告警和摆动段强制收腿。*/
  std::size_t cycle_stall_frames_ = 0;
  bool cycle_unload_logged_ = false;
  /*4 号开始时（四脚着地）记录的足端世界位置，摆动腿按其高度落脚。*/
  std::array<Vec3<T>, kNumLegs> load_transfer_foot_position_{};
  /*4 号触地确认：已触地标志、本周期是否仍在下探，以及下探帧数。*/
  std::array<bool, kNumLegs> cycle_touched_{};
  std::array<bool, kNumLegs> cycle_reaching_{};
  std::size_t cycle_reach_frames_ = 0;
  /*4 号起始锁存的平移量（积分修正以它为基准，限幅 ±1 cm）、卸载开始时锁存的
    FL/RR 关节角，以及承重确认等待帧数。*/
  Vec2<T> cycle_initial_shift_world_ = Vec2<T>::Zero();
  std::array<Vec3<T>, kNumLegs> cycle_hold_q_{};
  bool cycle_hold_latched_ = false;
  std::size_t cycle_gate_wait_frames_ = 0;
  /*卸载中检测到倾倒后回退到满载、重新卸载；动作起点姿态作为倾倒基准。*/
  bool cycle_tipping_ = false;
  bool cycle_had_tip_ = false;
  /*当前抬落的摆动腿：4 号 FL+RR，5 号按 FR→FL→RR→RL 依次循环。*/
  std::array<bool, kNumLegs> cycle_swing_{false, true, true, false};
  std::size_t single_leg_cycle_index_ = 0;
  /*卸载倾倒重试次数；达到上限后中止本次抬落并保持四脚站立。*/
  std::size_t cycle_tip_count_ = 0;
  bool cycle_aborted_ = false;
  /*上一次 4 号由实测力差学到的平移修正，再次按 4 时沿用（本次进入 Locomotion 内有效）。*/
  Vec2<T> cycle_learned_correction_ = Vec2<T>::Zero();
  Vec3<T> cycle_start_rpy_ = Vec3<T>::Zero();
  /*4 号一次抬落：整机质心计算模型、起始时机身原点到整机质心的世界系水平
    偏移，以及把质心移到 FR-RL 支撑线上所需的水平位移。*/
  std::unique_ptr<FloatingBaseModel<T>> com_model_;
  Vec2<T> cycle_com_shift_world_ = Vec2<T>::Zero();
  Vec2<T> cycle_support_shift_world_ = Vec2<T>::Zero();
  bool cycle_shift_ready_ = false;
  /** 用全身模型计算当前整机质心世界坐标。 */
  Vec3<T> wholeBodyComWorld();
  /** 4 号起始时锁存质心偏移和对角支撑线位移。 */
  void prepareStandCycleShift(const std::array<Vec3<T>, kNumLegs> & feet_world);
  /** 用已有关节力矩和雅可比估算单腿世界竖直支撑力。 */
  T estimatedVerticalSupport(std::size_t leg) const;
  /** 更新阶段①四脚稳定计数。 */
  void updateStage1Stability();
  /** 判断承重对角腿是否已接管足够载荷且机身仍稳定。 */
  bool loadTransferReady() const;
  /** 4 号（STAND_CYCLE）或 5 号（THREE_FOOT，逐腿复用 4 号时序）正在执行。 */
  bool isStandCycle() const noexcept
  {
    return applied_gait_ == GaitType::STAND_CYCLE || applied_gait_ == GaitType::THREE_FOOT;
  }
  /** 当前抬落动作的摆动腿编号。 */
  std::vector<std::size_t> cycleSwingLegs() const;
  /** 质心应到达的水平目标点：4 号为 FR-RL 线上的垂足，5 号为当前三腿支撑边内侧目标。 */
  Vec2<T> cycleSupportTarget(
    const Vec2<T> & com, const std::array<Vec3<T>, kNumLegs> & feet_world) const;
  /** 实测全身质心到支撑目标点的水平位移；无法计算时返回 NaN。 */
  Vec2<T> standCycleComLineError(const std::array<Vec3<T>, kNumLegs> & feet_world);
  /**
   * (RR - FL) 实测竖直承重差占体重比例；integrate 为真时按该差值把机身
   * 沿支撑线法向朝 FL 一侧积分平移。无法计算时返回 NaN。
   */
  T standCycleDiagonalImbalance(
    const std::array<Vec3<T>, kNumLegs> & feet_world, bool integrate);
  /** 带防饱和地把修正量加到 4 号平移量上：机身未跟上目标时不累加。 */
  void applyStandCycleShiftCorrection(const Vec2<T> & delta, T maximum_correction);
  /** 按质心到支撑线的实测偏差积分修正 4 号机身平移量。 */
  void integrateStandCycleShift(const std::array<Vec3<T>, kNumLegs> & feet_world);
  /**
   * 判断 4 号当前动作段是否达标，达标才推进动作时钟。
   * @return 达标返回 nullptr，否则返回未达标原因。
   */
  const char * standCycleStepBlocker(
    T time, T blend, T shift_blend,
    const std::array<Vec3<T>, kNumLegs> & feet_world);
};

extern template class FSM_State_Locomotion < float >;

#endif  // MYMIT_ROBOT_FSM_STATE_LOCOMOTION_H_
