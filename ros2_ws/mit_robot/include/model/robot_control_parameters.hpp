/**
 * @file robot_control_parameters.hpp
 * @brief DM1 控制参数；所有运行模式共享同一份模型语义。
 */
#ifndef MYMIT_ROBOT_MODEL_ROBOT_CONTROL_PARAMETERS_HPP_
#define MYMIT_ROBOT_MODEL_ROBOT_CONTROL_PARAMETERS_HPP_

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
  Vec3<T> balance_joint_kp;
  Vec3<T> balance_joint_kd;
  T balance_floating_base_weight;
  T balance_reaction_force_weight;
  T maximum_normal_force;
  T standing_supported_mass;

  T stand_up_duration;
  T stand_up_thigh_speed_scale;
  Vec3<T> stand_up_prepare_position;
  Vec3<T> stand_up_joint_kp;
  Vec3<T> stand_up_joint_kd;
  Vec3<T> stand_up_cartesian_kp;
  Vec3<T> stand_up_cartesian_kd;

  Vec3<T> locomotion_body_orientation_kp;
  Vec3<T> locomotion_body_orientation_kd;
  Vec3<T> locomotion_joint_kp;
  Vec3<T> locomotion_joint_kd;
  T locomotion_swing_height;
  T locomotion_max_lateral_foot_offset;

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

  // 数值与 dm1_model_contract.yaml、RL_Robot DM1 训练配置保持一致。
  // 换站立速度要修改的地方：站立速度由这两个共享参数控制。
  // standing_height_rate 限制支撑后抬升速度，stand_up_duration 限制趴地展开速度。
  return {
    T(0.30), T(0.42), T(0.04), T(0.12), T(0.08), T(2.0), T(0.00016), T(1.2),
    Vec3<T>(T(100), T(100), T(100)), Vec3<T>(T(2), T(2), T(2)),
    true, Vec3<T>::Zero(),
    // 初始化、趴卧保持和 RL 直接关节 PD 使用 RL 训练配置：
    // Kp={100,100,100}、Kd={2,2,2}。
    Vec3<T>(T(100), T(100), T(100)), Vec3<T>(T(2), T(2), T(2)),
    Vec3<T>(T(45), T(45), T(90)), Vec3<T>(T(8), T(8), T(14)),
    Vec3<T>(T(100), T(100), T(40)), Vec3<T>(T(18), T(18), T(8)),
    Vec3<T>(T(100), T(100), T(100)), Vec3<T>(T(2), T(2), T(2)),
    T(300), T(1), T(150), T(14.705035),
    // 趴卧展开阶段：适当放慢大腿/小腿轨迹，仿真与实机共用。
    T(3.2), T(0.75), Vec3<T>(T(0), T(0.1), T(-2.15)),
    Vec3<T>(T(100), T(100), T(100)), Vec3<T>(T(2), T(2), T(2)),
    Vec3<T>(T(220), T(220), T(300)), Vec3<T>(T(12), T(12), T(16)),
    Vec3<T>(T(70), T(70), T(35)), Vec3<T>(T(12), T(12), T(6)),
    // MPC/WBC 行走的位置增益与 RL、BalanceStand 保持一致（Kp=100），
    // 微分增益取达妙 MIT 协议上限 Kd=5：接触切换和横向扰动下需要这份
    // 阻尼才不会起振（Kd=2 时一次 120 N 横向扰动就足以让机身被弹起）。
    Vec3<T>(T(100), T(100), T(100)), Vec3<T>(T(5), T(5), T(5)),
    T(0.075), T(0.24),
    T(0.02), std::size_t(45), std::size_t(6), T(0.25), T(0.05),
    T(0.15), T(3.0), T(30.0), T(97.0)};
}

#endif  // MYMIT_ROBOT_MODEL_ROBOT_CONTROL_PARAMETERS_HPP_
