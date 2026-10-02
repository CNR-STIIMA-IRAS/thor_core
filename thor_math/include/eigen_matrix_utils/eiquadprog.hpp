#ifndef THOR_MATH__EIQUADPROG_COMPAT_HPP_
#define THOR_MATH__EIQUADPROG_COMPAT_HPP_

#include <qpmad/solver.h>

#include <Eigen/Core>

#include <limits>
#include <stdexcept>

namespace Eigen {

// Compatibility adapter for thor_math. It preserves the
// eiquadprog constraint convention while delegating to Apache-2.0 qpmad.
inline double solve_quadprog(MatrixXd &hessian,
                             VectorXd &gradient,
                             const MatrixXd &equality_matrix,
                             const VectorXd &equality_offset,
                             const MatrixXd &inequality_matrix,
                             const VectorXd &inequality_offset,
                             VectorXd &solution)
{
  const Index variables = hessian.rows();
  const Index equalities = equality_matrix.cols();
  const Index inequalities = inequality_matrix.cols();

  if (hessian.cols() != variables || gradient.size() != variables ||
      equality_matrix.rows() != variables ||
      equality_offset.size() != equalities ||
      inequality_matrix.rows() != variables ||
      inequality_offset.size() != inequalities) {
    throw std::invalid_argument("Inconsistent quadratic-program dimensions");
  }

  MatrixXd constraints(equalities + inequalities, variables);
  VectorXd lower(equalities + inequalities);
  VectorXd upper(equalities + inequalities);

  if (equalities > 0) {
    constraints.topRows(equalities) = equality_matrix.transpose();
    lower.head(equalities) = -equality_offset;
    upper.head(equalities) = -equality_offset;
  }
  if (inequalities > 0) {
    constraints.bottomRows(inequalities) = inequality_matrix.transpose();
    lower.tail(inequalities) = -inequality_offset;
    upper.tail(inequalities).setConstant(
      std::numeric_limits<double>::infinity());
  }

  const MatrixXd original_hessian = hessian;
  qpmad::Solver solver;
  try {
    const qpmad::Solver::ReturnStatus status =
      solver.solve(solution, hessian, gradient, constraints, lower, upper);
    if (status != qpmad::Solver::OK || !solution.allFinite()) {
      return std::numeric_limits<double>::infinity();
    }
  } catch (const std::runtime_error &) {
    return std::numeric_limits<double>::infinity();
  }

  return 0.5 * solution.dot(original_hessian * solution) +
         gradient.dot(solution);
}

}  // namespace Eigen

#endif  // THOR_MATH__EIQUADPROG_COMPAT_HPP_
