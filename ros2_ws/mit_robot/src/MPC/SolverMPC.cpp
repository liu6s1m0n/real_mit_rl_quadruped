/**
 * @file SolverMPC.cpp
 * @brief MIT Cheetah 凸 MPC：13 维线性模型、矩阵指数离散、稠密 QP 和 QuadProg++ 求解。
 */

#include "MPC/SolverMPC.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <unsupported/Eigen/MatrixFunctions>

#include "Utilities/QuadProg++.hh"

// 与 MIT convexMPC/SolverMPC.cpp 的 ct_ss_mats / c2qp / solve_mpc 一一对应。
// 内部统一用 double 计算，避免 float 下 B'SB 条件数过大。
namespace mpc
{
namespace
{

using MatD = Eigen::MatrixXd;
using VecD = Eigen::VectorXd;

constexpr Eigen::Index kStates = 13;
constexpr Eigen::Index kInputs = 12;
constexpr double kGravity = 9.81;

Eigen::Matrix3d skew(const Eigen::Vector3d & r)
{
  Eigen::Matrix3d result;
  result << 0.0, -r.z(), r.y(), r.z(), 0.0, -r.x(), -r.y(), r.x(), 0.0;
  return result;
}

/** ZYX 欧拉角对应的 body->world 旋转矩阵。 */
Eigen::Matrix3d rotationFromRpy(const Eigen::Vector3d & rpy)
{
  return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

}  // namespace

template<typename T>
bool SolverSettings<T>::isValid() const noexcept
{
  bool weights_valid = true;
  for (const T weight : state_weights) {
    weights_valid = weights_valid && std::isfinite(static_cast<double>(weight)) &&
      weight >= T(0);
  }
  return horizon > 0 && horizon < 20 && std::isfinite(static_cast<double>(time_step)) &&
    time_step > T(0) && std::isfinite(static_cast<double>(friction_coefficient)) &&
    friction_coefficient > T(0) && minimum_normal_force >= T(0) &&
    maximum_normal_force >= minimum_normal_force &&
    std::isfinite(static_cast<double>(alpha)) && alpha > T(0) &&
    std::isfinite(static_cast<double>(x_drag)) && x_drag >= T(0) && weights_valid;
}

template<typename T>
SolverMPC<T>::SolverMPC(
  const Quadruped<T> & quadruped, const SolverSettings<T> & settings)
: quadruped_(&quadruped), settings_(settings)
{
  if (!settings_.isValid()) {throw std::invalid_argument("invalid MPC settings");}
}

template<typename T>
SolverResult<T> SolverMPC<T>::solve(
  const RobotState<T> & state, const std::vector<DesiredState<T>> & trajectory,
  const std::vector<int> & contact_table,
  const NormalForceBounds<T> * force_bounds) const
{
  SolverResult<T> result;
  const std::size_t horizon = settings_.horizon;
  if (!state.isValid() || trajectory.size() != horizon ||
    contact_table.size() != horizon * kNumLegs)
  {
    return result;
  }
  for (const auto & desired : trajectory) {
    if (!desired.valid || !desired.body_position_world.allFinite() ||
      !desired.body_velocity_world.allFinite() || !desired.body_rpy.allFinite() ||
      !desired.body_angular_velocity.allFinite()) {return result;}
  }
  std::array<double, kNumLegs> force_min{};
  std::array<double, kNumLegs> force_max{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    force_min[leg] = static_cast<double>(force_bounds == nullptr ?
      settings_.minimum_normal_force : force_bounds->minimum[leg]);
    force_max[leg] = static_cast<double>(force_bounds == nullptr ?
      settings_.maximum_normal_force : force_bounds->maximum[leg]);
    if (!std::isfinite(force_min[leg]) || !std::isfinite(force_max[leg]) ||
      force_min[leg] < static_cast<double>(settings_.minimum_normal_force) ||
      force_max[leg] > static_cast<double>(settings_.maximum_normal_force) ||
      force_min[leg] > force_max[leg]) {return result;}
  }

  const double mass = static_cast<double>(quadruped_->totalMass());
  const Eigen::Matrix3d inertia_body =
    quadruped_->bodyInertia().inertia_com.template cast<double>();
  if (!std::isfinite(mass) || mass <= 0.0 || !inertia_body.allFinite() ||
    std::abs(inertia_body.determinant()) <= 1e-12) {return result;}

  // ---------- ct_ss_mats：连续时间模型，只用偏航旋转近似 ----------
  const double yaw = static_cast<double>(state.rpy.z());
  const Eigen::Matrix3d rotation_yaw =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d inertia_world =
    rotation_yaw * inertia_body * rotation_yaw.transpose();
  const Eigen::Matrix3d inertia_inverse = inertia_world.inverse();

  Eigen::Matrix<double, kStates, kStates> a_ct =
    Eigen::Matrix<double, kStates, kStates>::Zero();
  a_ct.block<3, 3>(0, 6) = rotation_yaw.transpose();
  a_ct.block<3, 3>(3, 9).setIdentity();
  a_ct(11, 9) = static_cast<double>(settings_.x_drag);
  a_ct(11, 12) = 1.0;

  Eigen::Matrix<double, kStates, kInputs> b_ct =
    Eigen::Matrix<double, kStates, kInputs>::Zero();
  const Eigen::Vector3d body_position = state.position_world.template cast<double>();
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const Eigen::Vector3d lever =
      state.foot_position_world[leg].template cast<double>() - body_position;
    const Eigen::Index column = static_cast<Eigen::Index>(leg * 3);
    b_ct.block<3, 3>(6, column) = inertia_inverse * skew(lever);
    b_ct.block<3, 3>(9, column) = Eigen::Matrix3d::Identity() / mass;
  }

  // ---------- c2qp：expm([A B; 0 0] dt) 精确离散 ----------
  Eigen::Matrix<double, 25, 25> ab_ct = Eigen::Matrix<double, 25, 25>::Zero();
  ab_ct.block<kStates, kStates>(0, 0) = a_ct;
  ab_ct.block<kStates, kInputs>(0, kStates) = b_ct;
  const Eigen::Matrix<double, 25, 25> ab_dt =
    (static_cast<double>(settings_.time_step) * ab_ct).exp();
  const Eigen::Matrix<double, kStates, kStates> a_dt = ab_dt.block<kStates, kStates>(0, 0);
  const Eigen::Matrix<double, kStates, kInputs> b_dt =
    ab_dt.block<kStates, kInputs>(0, kStates);
  if (!a_dt.allFinite() || !b_dt.allFinite()) {return result;}

  const Eigen::Index steps = static_cast<Eigen::Index>(horizon);
  std::vector<Eigen::Matrix<double, kStates, kStates>> powers(horizon + 1);
  powers[0].setIdentity();
  for (std::size_t k = 1; k <= horizon; ++k) {powers[k] = a_dt * powers[k - 1];}
  // A_qp 第 r 块 = Adt^(r+1)；B_qp(r, c) = Adt^(r-c) Bdt，r >= c。
  std::vector<Eigen::Matrix<double, kStates, kInputs>> input_response(horizon);
  for (std::size_t k = 0; k < horizon; ++k) {input_response[k] = powers[k] * b_dt;}

  // 初始状态与 MIT 一致：[rpy, p, omega_world, v, -g]。
  Eigen::Matrix<double, kStates, 1> x0;
  x0 << state.rpy.template cast<double>(), body_position,
    state.angular_velocity_world.template cast<double>(),
    state.velocity_world.template cast<double>(), -kGravity;

  // ---------- 权重 S 与参考轨迹 X_d ----------
  VecD weights(kStates * steps);
  VecD free_error(kStates * steps);
  for (Eigen::Index r = 0; r < steps; ++r) {
    const auto & desired = trajectory[static_cast<std::size_t>(r)];
    const Eigen::Vector3d desired_rpy = desired.body_rpy.template cast<double>();
    Eigen::Matrix<double, kStates, 1> x_desired;
    x_desired << desired_rpy, desired.body_position_world.template cast<double>(),
      rotationFromRpy(desired_rpy) * desired.body_angular_velocity.template cast<double>(),
      desired.body_velocity_world.template cast<double>(), 0.0;
    for (Eigen::Index i = 0; i < kInputs; ++i) {
      weights(kStates * r + i) =
        static_cast<double>(settings_.state_weights[static_cast<std::size_t>(i)]);
    }
    weights(kStates * r + 12) = 0.0;
    free_error.segment<kStates>(kStates * r) =
      powers[static_cast<std::size_t>(r) + 1] * x0 - x_desired;
  }

  // ---------- 消去摆动腿变量（MIT 的 var_elim/con_elim）----------
  std::vector<std::pair<std::size_t, std::size_t>> variables;
  for (std::size_t step = 0; step < horizon; ++step) {
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      if (contact_table[step * kNumLegs + leg] != 0) {variables.emplace_back(step, leg);}
    }
  }
  result.force_trajectory.assign(horizon * 12, T(0));
  if (variables.empty()) {
    result.converged = true;
    result.valid = true;
    return result;
  }

  const Eigen::Index n = static_cast<Eigen::Index>(variables.size() * 3);
  MatD b_red = MatD::Zero(kStates * steps, n);
  for (std::size_t v = 0; v < variables.size(); ++v) {
    const auto [column_step, leg] = variables[v];
    for (std::size_t r = column_step; r < horizon; ++r) {
      b_red.block(kStates * static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(v * 3),
        kStates, 3) = input_response[r - column_step].block(
        0, static_cast<Eigen::Index>(leg * 3), kStates, 3);
    }
  }

  // qH = 2(B'SB + alpha I)，qg = 2B'S(A_qp x0 - X_d)。
  const MatD weighted_b = weights.asDiagonal() * b_red;
  MatD hessian = 2.0 * (b_red.transpose() * weighted_b);
  hessian.diagonal().array() += 2.0 * static_cast<double>(settings_.alpha);
  VecD gradient = 2.0 * (weighted_b.transpose() * free_error);

  // 四棱锥摩擦：mu fz ± fx >= 0、mu fz ± fy >= 0；法向力 fmin <= fz <= fmax。
  // QuadProg++ 约定 CI' x + ci0 >= 0。
  const double mu = static_cast<double>(settings_.friction_coefficient);
  const Eigen::Index constraints = static_cast<Eigen::Index>(variables.size() * 6);
  MatD ci = MatD::Zero(n, constraints);
  VecD ci0 = VecD::Zero(constraints);
  for (std::size_t v = 0; v < variables.size(); ++v) {
    const Eigen::Index x = static_cast<Eigen::Index>(v * 3);
    const Eigen::Index c = static_cast<Eigen::Index>(v * 6);
    const std::size_t leg = variables[v].second;
    ci(x + 2, c) = mu; ci(x, c) = -1.0;
    ci(x + 2, c + 1) = mu; ci(x, c + 1) = 1.0;
    ci(x + 2, c + 2) = mu; ci(x + 1, c + 2) = -1.0;
    ci(x + 2, c + 3) = mu; ci(x + 1, c + 3) = 1.0;
    ci(x + 2, c + 4) = 1.0; ci0(c + 4) = -force_min[leg];
    ci(x + 2, c + 5) = -1.0; ci0(c + 5) = force_max[leg];
  }
  const MatD ce = MatD::Zero(n, 0);
  const VecD ce0 = VecD::Zero(0);
  VecD solution = VecD::Zero(n);
  const double cost = solve_quadprog(hessian, gradient, ce, ce0, ci, ci0, solution);
  if (!std::isfinite(cost) || !solution.allFinite()) {return SolverResult<T>{};}

  for (std::size_t v = 0; v < variables.size(); ++v) {
    const auto [step, leg] = variables[v];
    for (std::size_t axis = 0; axis < 3; ++axis) {
      result.force_trajectory[step * 12 + leg * 3 + axis] =
        static_cast<T>(solution(static_cast<Eigen::Index>(v * 3 + axis)));
    }
  }
  // 滚动时域：只执行第一段。
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      result.reaction_forces_world[leg](static_cast<Eigen::Index>(axis)) =
        result.force_trajectory[leg * 3 + axis];
    }
  }
  result.cost = static_cast<T>(cost);
  result.converged = true;
  result.valid = true;
  return result;
}

template struct SolverSettings<float>;
template struct SolverSettings<double>;
template struct SolverResult<float>;
template struct SolverResult<double>;
template class SolverMPC<float>;
template class SolverMPC<double>;

}  // namespace mpc
