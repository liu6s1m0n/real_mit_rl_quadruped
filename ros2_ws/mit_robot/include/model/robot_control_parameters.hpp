/**
 * @file robot_control_parameters.hpp
 * @brief DM1 控制参数；所有运行模式共享同一份模型语义。
 */
#ifndef MYMIT_ROBOT_MODEL_ROBOT_CONTROL_PARAMETERS_HPP_
#define MYMIT_ROBOT_MODEL_ROBOT_CONTROL_PARAMETERS_HPP_

#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "model/robot_types.hpp"

template<typename T>
struct RobotControlParameters
{
  T minimum_standing_height;
  T maximum_standing_height;
  T standing_height_rate;
  T prone_body_height;
  T prone_down_height_rate;
  T prone_down_fold_duration;
  T balance_height_step;
  T joint_initialization_duration;
  Vec3<T> initialization_kp;
  Vec3<T> initialization_kd;
  bool start_in_prone_home;
  Vec3<T> motor_zero_position;
  Vec3<T> prone_home_joint_kp;
  Vec3<T> prone_home_joint_kd;

  Vec3<T> balance_body_position_kp;
  Vec3<T> balance_body_position_kd;
  Vec3<T> balance_body_orientation_kp;
  Vec3<T> balance_body_orientation_kd;
  // BalanceStand 下发给电机的关节 PD；按 MIT 取低阻抗，支撑靠 WBIC 前馈。
  Vec3<T> balance_joint_kp;
  Vec3<T> balance_joint_kd;
  Vec3<T> joint_kp;
  Vec3<T> joint_kd;
  T balance_floating_base_weight;
  T balance_reaction_force_weight;
  T maximum_normal_force;

  T stand_up_duration;
  T stand_up_thigh_speed_scale;
  Vec3<T> stand_up_prepare_position;
  Vec3<T> stand_up_joint_kp;
  Vec3<T> stand_up_joint_kd;
  Vec3<T> stand_up_cartesian_kp;
  Vec3<T> stand_up_cartesian_kd;

  Vec3<T> locomotion_body_orientation_kp;
  Vec3<T> locomotion_body_orientation_kd;
  T locomotion_swing_height;
  T locomotion_max_lateral_foot_offset;
  // MPC 预测/接触表段长 dtMPC，s。实际段长按控制周期
  // 取整为整数帧；TROT 周期 = 10 段 x dtMPC，摆动期为其一半。
  T mpc_time_step;

  // RL 部署契约：45 维单帧观测、6 帧历史、50 Hz 策略。
  T rl_policy_period;
  std::size_t rl_observation_size;
  std::size_t rl_history_length;
  T rl_action_scale;
  T rl_action_filter_time_constant;
  T rl_max_action_delta;
  T rl_max_target_velocity;
  T rl_continuous_torque_limit;
  T rl_peak_torque_limit;
};

template<typename T>
RobotControlParameters<T> makeRobotControlParameters(RobotType robot_type)
{
  if (robot_type != RobotType::DM1) {
    throw std::invalid_argument("only the DM1 control profile is supported");
  }

  // 机械模型与 dm1_model_contract.yaml 保持一致；下列控制增益按实机连续力矩预算整定。
  // 换站立速度要修改的地方：站立速度由这两个共享参数控制。
  // standing_height_rate 限制支撑后抬升速度，stand_up_duration 限制趴地展开速度。
  return {
    // 站立高度范围、抬升速度、趴卧高度、趴下速度/折叠时间、单周期高度步长、初始化时长。
    T(0.30), T(0.42), T(0.04), T(0.12), T(0.08), T(2.0), T(0.00016), T(1.2),
    // 电机刚使能后的关节初始化 PD：[hip, thigh, calf] 的 Kp、Kd。
    Vec3<T>(T(99.9), T(99.9), T(99.9)), Vec3<T>(T(1.999), T(1.999), T(1.999)),
    // 从机械趴卧零位启动；模型零位对应三个电机角全零。
    true, Vec3<T>::Zero(),
    // 趴卧保持关节 PD：[hip, thigh, calf] 的 Kp、Kd。
    Vec3<T>(T(70), T(70), T(70)), Vec3<T>(T(1.6), T(1.6), T(1.6)),
    // BalanceStand 机身位置 WBC 增益：[x, y, z] 的 Kp、Kd；不是电机 MIT-PD。
    Vec3<T>(T(100), T(100), T(100)), Vec3<T>(T(10), T(10), T(10)),
    // BalanceStand 机身姿态 WBC 增益：[roll, pitch, yaw] 的 Kp、Kd；不是电机 MIT-PD。
    Vec3<T>(T(100), T(100), T(100)), Vec3<T>(T(10), T(10), T(10)),
    // BalanceStand 直接下发给电机的关节 PD：[hip, thigh, calf] 的 Kp、Kd。
    // 取 MIT WBC_Ctrl 的默认关节阻抗（Kp=5、Kd=1.5），即 MIT 的低 PD 站立：
    // 支撑力主要来自 WBIC 前馈，关节 PD 只做参考跟踪，不靠硬弹簧顶住自重。
    Vec3<T>(T(5), T(5), T(5)), Vec3<T>(T(1.5), T(1.5), T(1.5)),
    // 经典 WBC（行走/趴下）共用的关节 PD：[hip, thigh, calf] 的 Kp、Kd。
    Vec3<T>(T(10), T(10), T(10)), Vec3<T>(T(2), T(2), T(2)),
    // WBC浮动基权重、反力权重、单脚最大法向力（MIT BalanceStand 取 1000）。
    T(1000), T(1), T(120),
    // StandUp 趴卧展开时长、大腿速度比例、机械关节目标姿态。
    T(3.2), T(0.75), Vec3<T>(T(0), T(0.1), T(-2.15)),
    // StandUp 第一阶段直接下发给电机的关节 PD：[hip, thigh, calf] 的 Kp、Kd。
    Vec3<T>(T(80), T(80), T(90)), Vec3<T>(T(1.8), T(1.8), T(1.8)),
    // StandUp 非趴卧路径的足端笛卡尔 PD：[x, y, z] 的 Kp、Kd。
    Vec3<T>(T(190), T(190), T(260)), Vec3<T>(T(10), T(10), T(14)),
    // Locomotion 机身姿态 WBC 增益：[roll, pitch, yaw] 的 Kp、Kd。
    Vec3<T>(T(100), T(100), T(100)), Vec3<T>(T(10), T(10), T(10)),
    // 摆腿高度、足端最大横向偏移（扫参选定摆高 0.06 m）。
    T(0.06), T(0.30),
    // MPC 预测段长 dtMPC（500 Hz 下 15 帧，扫参选定 0.030 s）。
    T(0.030),
    // RL周期、观测维数、历史长度、动作比例、滤波常数、单步变化、速度和力矩限制。
    T(0.02), std::size_t(45), std::size_t(6), T(0.25), T(0.05),
    T(0.15), T(3.0), T(30.0), T(97.0)};
}

/** @brief dtMPC 换算成整数控制帧（MIT iterationsBetweenMPC），至少 1 帧。 */
template<typename T>
std::size_t mpcSegmentIterations(
  const RobotControlParameters<T> & parameters, T control_time_step)
{
  if (!(control_time_step > T(0)) || !(parameters.mpc_time_step > T(0))) {
    throw std::invalid_argument("MPC timing requires positive time steps");
  }
  const T frames = std::round(parameters.mpc_time_step / control_time_step);
  return frames < T(1) ? std::size_t(1) : static_cast<std::size_t>(frames);
}

#endif  // MYMIT_ROBOT_MODEL_ROBOT_CONTROL_PARAMETERS_HPP_
