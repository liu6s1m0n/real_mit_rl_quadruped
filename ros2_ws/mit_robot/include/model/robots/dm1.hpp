/*! @file dm1.hpp
 *  @brief 达妙 DM1 的唯一机器人参数基线。
 *
 * 尺寸、质量、惯量、关节范围和电机限制来自
 * legged_damiao_description/urdf/dmgo/const.xacro。控制框架继续使用统一的
 * FR/FL/RR/RL 与 Hip/thigh/calf 顺序，分别映射源模型的 RF/LF/RH/LH 和
 * HAA/HFE/KFE。
 */
#ifndef MYMIT_ROBOT_MODEL_ROBOTS_DM1_HPP_
#define MYMIT_ROBOT_MODEL_ROBOTS_DM1_HPP_

#include "config/control_config.h"
#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Geometry>

#include "model/quadruped.hpp"

namespace robots
{
namespace dm1
{

template<typename T>
JointModelParameters<T> makeJointParameters(bool /*is_rear*/)
{
  JointModelParameters<T> result;
  // q 是电机/策略坐标；机械动力学模型通过 zero_offset 还原安装零位。
  // DM1 机械趴姿为 12 个电机角全零；控制器通过 zero_offset 还原机械角。
  // 这些零偏和执行边界与 Isaac 训练 DM1 配置保持一致；固定安装旋转
  // 由 MJCF body quaternion 表达，解析运动学只保留训练侧的关节偏置。
  const T thigh_zero_offset = T(-0.203);
  result.zero_offset << T(0), thigh_zero_offset, T(-2.22);
  // 电机坐标边界使用与 MIT 校验相同的规范值，避免零偏浮点运算造成整帧拒绝。
  result.lower_limit << T(-1.57), T(-1.367), T(-0.03);
  result.upper_limit << T(1.57), T(2.603), T(2.72);
  result.velocity_limit << T(6.28318530718), T(6.28318530718), T(6.28318530718);
  result.torque_limit << T(97), T(97), T(97);
  result.damping.setConstant(T(1));
  result.friction_loss.setConstant(T(0.1));
  result.armature.setConstant(T(0.01));
  // 0.28 m 调试站姿：侧视大小腿中心线对水平的倾角均约 27.85 度。
  // 安装零偏和小腿连杆方向不同，因此电机读数不相等。
  result.home_position <<
    T(0), T(-0.282961), T(0.739729);
  result.joint_axes.col(0) = Vec3<T>::UnitX();
  result.joint_axes.col(1) = Vec3<T>::UnitY();
  result.joint_axes.col(2) = Vec3<T>::UnitY();
  return result;
}

template<typename T>
RigidBodyInertia<T> makeInertia(
  T mass, const Vec3<T> & center, T ixx, T iyy, T izz,
  T ixy = T(0), T ixz = T(0), T iyz = T(0))
{
  RigidBodyInertia<T> result;
  result.mass = mass;
  result.center_of_mass = center;
  result.inertia_com << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz;
  return result;
}

template<typename T>
LegModelParameters<T> makeLeg(LegId leg_id, bool is_left, bool is_rear)
{
  const T side = is_left ? T(1) : T(-1);
  const T longitudinal = is_rear ? T(-1) : T(1);
  LegModelParameters<T> result;
  result.leg = leg_id;
  result.hip_location_body = Vec3<T>(
    longitudinal * T(0.16175), side * T(0.059997), T(-0.061498));
  // 通用三连杆模型使用源 URDF 的关节间有效距离。
  result.hip_link_length = T(0.097753);
  result.thigh_link_length = T(0.2);
  result.calf_link_length = T(0.225);
  result.hip_to_thigh = Vec3<T>(
    longitudinal * T(0.049998), side * T(0.019503), T(0));
  result.thigh_to_calf = Vec3<T>(T(-0.2), side * T(0.07825), T(0));
  result.calf_to_foot = Vec3<T>(T(-0.17362), T(0), T(-0.14312));
  result.foot_radius = T(0.02);
  result.foot_friction << T(0.8), T(0.02), T(0.01);
  result.joints = makeJointParameters<T>(is_rear);

  result.hip_inertia = makeInertia(
    T(0.566780616299),
    Vec3<T>(longitudinal * T(0.04756), -side * T(0.010434), T(-0.000046)),
    T(0.000323385833), T(0.000429308737), T(0.000357891022));
  result.thigh_inertia = makeInertia(
    T(0.784107130653), Vec3<T>(T(-0.023554), side * T(0.050542), T(0.001956)),
    T(0.000711769812), T(0.003013988066), T(0.003143984356));
  // 足部通过固定关节连接到小腿，动力学参数中将其质量并入小腿。
  result.calf_inertia = makeInertia(
    T(0.217078558467), Vec3<T>(T(-0.010235), T(0.000248), T(-0.054204)),
    T(0.000783989974), T(0.004269816427), T(0.003504282717));
  return result;
}

// 名义 0.28 m 站姿的固定软件角度范围；机械硬限位与标定保持原值。
// 大腿下界阻止膝部继续下压，上界配合小腿上界避开小腿竖直姿态。
constexpr double kKneeEnvelopeRadius = 0.080;
// 该常数目前未被控制或测试代码引用。2026-10-19 的 A/B（+0.020 / -0.020）
// 在 hardware-sim 中得到相同的踏步最低高度 0.167 m，说明它不是下沉原因；
// 保留数值待后续接入真实的膝间隙检查，避免把它误当成已生效的保护。
constexpr double kMinimumKneeClearance = -0.020;
constexpr double kStanceThighLower = control_config::safety::stance_thigh_lower;
constexpr double kStanceThighUpper = control_config::safety::stance_thigh_upper;
constexpr double kStanceCalfLower = control_config::safety::stance_calf_lower;
constexpr double kStanceCalfUpper = control_config::safety::stance_calf_upper;

template<typename T>
bool clampStanceJointTarget(Vec3<T> & q)
{
  if (!q.allFinite()) {return false;}
  q[1] = std::clamp(q[1], T(kStanceThighLower), T(kStanceThighUpper));
  q[2] = std::clamp(q[2], T(kStanceCalfLower), T(kStanceCalfUpper));
  return true;
}

/** @brief 创建完整 DM1 参数模型。 */
template<typename T>
Quadruped<T> makeModel()
{
  const auto body = makeInertia(
    T(5.528134778326), Vec3<T>(T(-0.001955), T(0.002743), T(-0.068608)),
    T(0.028868805821), T(0.065085611833), T(0.082712948320),
    T(0.000061788360), T(-0.000118762043), T(0.000218265376));

  typename Quadruped<T>::LegArray legs;
  legs[static_cast<std::size_t>(LegId::FR)] = makeLeg<T>(LegId::FR, false, false);
  legs[static_cast<std::size_t>(LegId::FL)] = makeLeg<T>(LegId::FL, true, false);
  legs[static_cast<std::size_t>(LegId::RR)] = makeLeg<T>(LegId::RR, false, true);
  legs[static_cast<std::size_t>(LegId::RL)] = makeLeg<T>(LegId::RL, true, true);
  return Quadruped<T>(RobotType::DM1, body, legs, T(0.28));
}

}  // namespace dm1
}  // namespace robots

#endif  // MYMIT_ROBOT_MODEL_ROBOTS_DM1_HPP_
