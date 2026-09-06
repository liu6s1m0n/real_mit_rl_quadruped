/**
 * @file SimulationDiagnostics.hpp
 * @brief 正式仿真运行时诊断：比较控制器估计和MuJoCo真值并监测小腿碰地。
 *
 * 这些能力最初只用于端到端测试，现在同时供正式SimulationBridge和测试复用。
 * 诊断只读取仿真状态，不修改控制命令或物理状态。
 */
#ifndef MYMIT_ROBOT_USER_SIMULATION_DIAGNOSTICS_HPP_
#define MYMIT_ROBOT_USER_SIMULATION_DIAGNOSTICS_HPP_

#include <array>
#include <cstddef>
#include <limits>

#include <mujoco/mujoco.h>

#include "controller/OrientationEstimator.hpp"
#include "controller/RlPolicy.hpp"

struct SimulationDirectionWindow
{
  double command_sum[3]{};
  double actual_sum[3]{};
  std::size_t samples = 0;

  std::array<float, 3> meanCommand() const noexcept;
  std::array<float, 3> meanActual() const noexcept;
};

struct SimulationFootContactStats
{
  std::size_t contact_samples = 0;
  std::size_t contact_transitions = 0;
  double contact_force_sum = 0.0;
  float maximum_contact_force = 0.0F;
  double slip_speed_sum = 0.0;
  float maximum_slip_speed = 0.0F;

  float meanContactForce() const noexcept;
  float meanSlipSpeed() const noexcept;
};

struct SimulationDiagnosticReport
{
  float maximum_position_error = 0.0F;      ///< 估计位置与 MuJoCo 真值的最大距离，m。
  float maximum_velocity_error = 0.0F;      ///< 估计线速度与真值的最大差值，m/s。
  float maximum_orientation_error = 0.0F;   ///< 估计姿态与真值的最大旋转角误差，rad。
  float maximum_absolute_pitch = 0.0F;      ///< 机身俯仰角绝对值的历史最大值，rad。
  float minimum_height = std::numeric_limits<float>::infinity();  ///< 机身最低离地高度，m。
  std::size_t calf_collision_frames = 0;    ///< 检测到任一小腿碰地的仿真帧数。
  std::size_t rejected_control_frames = 0;  ///< 控制器输出无效而被安全层拒绝的帧数。
  std::size_t observed_frames = 0;          ///< 已纳入统计的有效仿真帧总数。
  double squared_position_error_sum = 0.0;  ///< 位置误差平方累计值，用于计算 RMS。
  double squared_velocity_error_sum = 0.0;  ///< 速度误差平方累计值，用于计算 RMS。
  float command_vx = 0.0F;                  ///< 当前机身系期望前向速度，m/s。
  float command_vy = 0.0F;                  ///< 当前机身系期望横向速度，m/s。
  float command_wz = 0.0F;                  ///< 当前机身系期望偏航速度，rad/s。
  float actual_vx = 0.0F;                   ///< 当前 MuJoCo 机身系前向速度，m/s。
  float actual_vy = 0.0F;                   ///< 当前 MuJoCo 机身系横向速度，m/s。
  float actual_wz = 0.0F;                   ///< 当前 MuJoCo 机身系偏航速度，rad/s。
  // [0,1) s 为预热窗口；有效结论窗口为 [1,2)、[2,5)、[5,10) s。
  std::array<SimulationDirectionWindow, 4> direction_windows{};
  float raw_action_max_abs = 0.0F;          ///< 策略原始动作的历史最大绝对值。
  std::size_t raw_action_samples = 0;       ///< 已记录原始动作的策略帧数。
  std::size_t raw_action_over_one = 0;      ///< 原始动作分量超过 +/-1 的次数。
  std::size_t raw_action_components = 0;    ///< 原始动作分量总数。
  std::size_t target_joint_limit_hits = 0;  ///< 目标关节触及模型限位的次数。
  std::size_t torque_speed_saturation_count = 0;  ///< 力矩-转速包络饱和次数。
  std::array<std::size_t, kNumLegs> torque_speed_saturation_by_leg{};
  float maximum_foot_slip_speed = 0.0F;     ///< 接触地面时足端最大水平滑移速度。
  double foot_slip_speed_sum = 0.0;         ///< 接触地面时足端水平滑移速度累计值。
  std::size_t foot_contact_samples = 0;     ///< 足端接触样本数。
  float fall_time_s = 0.0F;                 ///< 低高度或大姿态角判定的累计时间。
  float net_displacement_body_x = 0.0F;     ///< 活动窗口内机身系净前向位移，m。
  float net_displacement_body_y = 0.0F;     ///< 活动窗口内机身系净横向位移，m。
  std::array<SimulationFootContactStats, kNumLegs> foot_contact_by_leg{};

  float rmsPositionError() const noexcept;
  float rmsVelocityError() const noexcept;
  float calfContactRatio() const noexcept;
  float meanFootSlipSpeed() const noexcept;
  float rawActionOverOneRatio() const noexcept;
  float netHorizontalDisplacement() const noexcept;
};

class SimulationDiagnostics
{
public:
  explicit SimulationDiagnostics(const mjModel * model);

  void reset() noexcept;
  void observe(
    const mjData * data, const StateEstimate<float> & estimate,
    bool control_valid, bool direction_active, const Vec3<float> & command_body,
    const std::array<float, kRlActionSize> * raw_action = nullptr,
    std::size_t target_joint_limit_hits = 0,
    const std::array<std::size_t, kNumLegs> * torque_speed_saturation_by_leg = nullptr);

  Vec3<float> bodyPosition(const mjData * data) const;
  Vec3<float> bodyLinearVelocityWorld(const mjData * data) const;
  Vec3<float> bodyLinearVelocity(const mjData * data) const;
  Vec3<float> bodyAngularVelocity(const mjData * data) const;
  float bodyYaw(const mjData * data) const;
  float orientationError(
    const mjData * data, const Eigen::Quaternionf & estimate) const;
  bool hasCalfCollision(const mjData * data) const;
  const SimulationDiagnosticReport & report() const noexcept {return report_;}

private:
  static int requireObject(const mjModel * model, int type, const char * name);

  const mjModel * model_ = nullptr;  ///< 非拥有型 MuJoCo 模型指针，用于查询对象和状态布局。
  int trunk_body_ = -1;             ///< trunk 刚体在 model->body_* 数组中的编号。
  int floor_geom_ = -1;             ///< 地面几何体在 model->geom_* 数组中的编号。
  std::array<int, kNumLegs> foot_geoms_{};   ///< FR/FL/RR/RL 足端几何体编号。
  std::array<int, kNumLegs> foot_bodies_{};  ///< FR/FL/RR/RL 足端刚体编号。
  std::array<int, kNumLegs> calf_bodies_{};  ///< FR/FL/RR/RL 小腿刚体编号。
  std::array<bool, kNumLegs> previous_foot_contact_{};
  Vec3<float> position_offset_ = Vec3<float>::Zero();  ///< 首帧对齐估计坐标系和世界坐标系的平移量，m。
  bool position_offset_initialized_ = false;  ///< position_offset_ 是否已由首帧建立。
  double active_direction_time_s_ = 0.0;  ///< 从真正进入 Locomotion 后开始计时。
  Vec3<float> active_start_position_world_ = Vec3<float>::Zero();
  float active_start_yaw_ = 0.0F;
  bool active_start_initialized_ = false;
  SimulationDiagnosticReport report_{};       ///< 从 reset() 后累计到当前帧的诊断结果。
};

#endif  // MYMIT_ROBOT_USER_SIMULATION_DIAGNOSTICS_HPP_
