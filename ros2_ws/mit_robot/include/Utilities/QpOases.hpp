/**
 * @file QpOases.hpp
 * @brief Eigen-to-qpOASES adapter used by MPC and WBIC.
 *
 * qpOASES stores dense matrices in row-major order and represents every
 * linear constraint as a lower/upper bounded row.  Keeping this conversion
 * here leaves the controller code in Eigen form and makes the constraint
 * convention explicit at one boundary.
 */
#ifndef MYMIT_ROBOT_UTILITIES_QP_OASES_HPP_
#define MYMIT_ROBOT_UTILITIES_QP_OASES_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <qpOASES.hpp>

namespace qp_adapter
{

inline constexpr double kInfinity = 1.0e20;

inline std::vector<qpOASES::real_t> rowMajor(
  const Eigen::MatrixXd & matrix)
{
  std::vector<qpOASES::real_t> result(
    static_cast<std::size_t>(matrix.rows() * matrix.cols()));
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      result[static_cast<std::size_t>(row * matrix.cols() + column)] =
        static_cast<qpOASES::real_t>(matrix(row, column));
    }
  }
  return result;
}

inline std::vector<qpOASES::real_t> columnVector(
  const Eigen::VectorXd & vector)
{
  std::vector<qpOASES::real_t> result(static_cast<std::size_t>(vector.size()));
  for (Eigen::Index index = 0; index < vector.size(); ++index) {
    result[static_cast<std::size_t>(index)] =
      static_cast<qpOASES::real_t>(vector(index));
  }
  return result;
}

/**
 * @brief Solve 1/2 x'Hx + g'x with variable and row bounds.
 *
 * A zero-row constraint matrix is accepted.  Equalities are represented by
 * equal lower and upper row bounds, so MPC and WBIC use one interface.
 */
inline bool solve(
  const Eigen::MatrixXd & hessian,
  const Eigen::VectorXd & gradient,
  const Eigen::MatrixXd & constraints,
  const Eigen::VectorXd & variable_lower,
  const Eigen::VectorXd & variable_upper,
  const Eigen::VectorXd & constraint_lower,
  const Eigen::VectorXd & constraint_upper,
  Eigen::VectorXd & solution,
  double * objective = nullptr,
  int max_working_set_recalculations = 100)
{
  const Eigen::Index variables = hessian.rows();
  const Eigen::Index rows = constraints.rows();
  if (variables <= 0 || hessian.cols() != variables || gradient.size() != variables ||
    constraints.cols() != variables || variable_lower.size() != variables ||
    variable_upper.size() != variables || constraint_lower.size() != rows ||
    constraint_upper.size() != rows || !hessian.allFinite() || !gradient.allFinite() ||
    !constraints.allFinite() || !variable_lower.allFinite() ||
    !variable_upper.allFinite() || !constraint_lower.allFinite() ||
    !constraint_upper.allFinite())
  {
    return false;
  }

  auto h = rowMajor(hessian);
  auto g = columnVector(gradient);
  auto lb = columnVector(variable_lower);
  auto ub = columnVector(variable_upper);
  auto a = rowMajor(constraints);
  auto lb_a = columnVector(constraint_lower);
  auto ub_a = columnVector(constraint_upper);
  std::vector<qpOASES::real_t> primal(static_cast<std::size_t>(variables), 0.0);

  qpOASES::QProblem problem(
    static_cast<qpOASES::int_t>(variables), static_cast<qpOASES::int_t>(rows),
    qpOASES::HST_POSDEF);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_NONE;
  options.enableRegularisation = qpOASES::BT_TRUE;
  problem.setOptions(options);

  qpOASES::int_t nwsr = static_cast<qpOASES::int_t>(
    std::max(1, max_working_set_recalculations));
  const qpOASES::returnValue status = problem.init(
    h.data(), g.data(), rows > 0 ? a.data() : nullptr,
    lb.data(), ub.data(), rows > 0 ? lb_a.data() : nullptr,
    rows > 0 ? ub_a.data() : nullptr, nwsr);
  if (status != qpOASES::SUCCESSFUL_RETURN ||
    problem.getPrimalSolution(primal.data()) != qpOASES::SUCCESSFUL_RETURN)
  {
    return false;
  }

  solution.resize(variables);
  for (Eigen::Index index = 0; index < variables; ++index) {
    solution(index) = static_cast<double>(primal[static_cast<std::size_t>(index)]);
  }
  if (!solution.allFinite()) {return false;}
  if (objective != nullptr) {*objective = problem.getObjVal();}
  return std::isfinite(problem.getObjVal());
}

}  // namespace qp_adapter

#endif  // MYMIT_ROBOT_UTILITIES_QP_OASES_HPP_
