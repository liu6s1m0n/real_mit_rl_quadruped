/**
 * @file SolverMPC.h
 * @brief MIT Cheetah 凸 MPC 求解器及其参数、结果结构。
 *
 * 构造与 MIT convexMPC/SolverMPC.cpp 一致：13 维状态 [rpy, p, omega_world, v, -g]，
 * 连续模型经矩阵指数离散，代价 2(B'SB + alpha I)，约束为四棱锥摩擦和法向力上下界。
 * 摆动腿变量在建模时直接消去，剩余 QP 用 qpOASES 求解。
 */
#ifndef MYMIT_ROBOT_MPC_SOLVER_MPC_H_
#define MYMIT_ROBOT_MPC_SOLVER_MPC_H_

#include <array>
#include <cstddef>
#include <vector>

#include "MPC/RobotState.h"

namespace mpc
{

/** MIT convex-MPC uses a ten-segment prediction window for the trot gait. */
inline constexpr std::size_t kDefaultMpcHorizon = 10;

template<typename T>
struct SolverSettings
{
  std::size_t horizon = kDefaultMpcHorizon;   ///< 预测窗包含的离散时刻数（MIT 为 10）。
  T time_step = T(0.030);     ///< 预测步长 dtMPC，s（500 Hz 下 15 个控制周期）。

  T friction_coefficient = T(0.4);  ///< 四棱锥摩擦约束的摩擦系数 mu。
  T minimum_normal_force = T(0);    ///< 支撑脚允许的最小法向力，N。
  T maximum_normal_force = T(120);  ///< 单只支撑脚允许的最大法向力，N（MIT f_max）。
  T alpha = T(4e-5);                ///< 反力二次正则权重（MIT alpha）。
  T x_drag = T(0);                  ///< MIT 机身高度/前向速度阻尼项。
  /** 状态权重 Q：[yaw, pitch, roll, x, y, z, wx, wy, wz, vx, vy, vz]；yaw 和 wz 反馈关闭。 */
  std::array<T, 12> state_weights{
    T(0), T(0.25), T(10), T(2), T(2), T(50),
    T(0), T(0), T(0), T(0.2), T(0.2), T(0.1)};

  /** @brief 检查预测时域、力约束和权重是否有效。 */
  bool isValid() const noexcept;
};

template<typename T>
struct SolverResult
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::array<Vec3<T>, kNumLegs> reaction_forces_world{};  ///< 当前时刻四脚世界系地面反力，N。
  std::vector<T> force_trajectory;  ///< 整个预测窗的最优接触力序列，按时刻和腿拼接。
  T cost = T(0);               ///< QP 最优目标值（不含常数项）。
  bool converged = false;      ///< qpOASES 返回成功且解有限时为 true。
  bool valid = false;          ///< 输出维度、数值和约束是否均可用于控制。
};

template<typename T>
struct NormalForceBounds
{
  std::array<T, kNumLegs> minimum{};
  std::array<T, kNumLegs> maximum{};
};

template<typename T>
class SolverMPC
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief 创建凸 MPC 求解器。
   * @param quadruped 四足机器人模型，提供质量和机身惯量。
   * @param settings 预测时域、摩擦和权重参数。
   */
  explicit SolverMPC(
    const Quadruped<T> & quadruped,
    const SolverSettings<T> & settings = SolverSettings<T>{});

  /**
   * @brief 求解预测窗内的接触反力。
   *
   * 当前控制周期只使用最优序列的第一段，完整序列保存在 force_trajectory 中。
   *
   * @param state 当前 MPC 状态。
   * @param trajectory 预测窗内的期望机身状态；角速度为机身系，内部转为世界系。
   * @param contact_table 预测窗接触表，按 [step * 4 + leg] 排列。
   * @param force_bounds 可选的逐腿法向力上下界。
   */
  SolverResult<T> solve(
    const RobotState<T> & state, const std::vector<DesiredState<T>> & trajectory,
    const std::vector<int> & contact_table,
    const NormalForceBounds<T> * force_bounds = nullptr) const;

  const SolverSettings<T> & settings() const noexcept {return settings_;}

private:
  const Quadruped<T> * quadruped_;  ///< 非拥有指针，提供质量和机身惯量。
  SolverSettings<T> settings_;      ///< 本求解器固定使用的预测和约束参数副本。
};

extern template struct SolverSettings<float>;
extern template struct SolverSettings<double>;
extern template struct SolverResult<float>;
extern template struct SolverResult<double>;
extern template class SolverMPC<float>;
extern template class SolverMPC<double>;

}  // namespace mpc

#endif  // MYMIT_ROBOT_MPC_SOLVER_MPC_H_
