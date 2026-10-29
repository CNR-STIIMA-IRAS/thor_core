/**
 * @file eigen_matrix_utils.h
 * @brief Small statistics helpers on Eigen vectors.
 */

#ifndef THOR_MATH__THOR_MATH__EIGEN_MATRIX_UTILS
#define THOR_MATH__THOR_MATH__EIGEN_MATRIX_UTILS

#include <cmath>
#include <limits>
#include <sstream>
#include <exception>

#include <Eigen/Core>
#include <Eigen/Dense>

/// Statistics helpers on Eigen expressions.
namespace eigen_utils
{

/**
 * @brief Population standard deviation of the coefficients of @p vet.
 * @return `sqrt(sum((v - mean)^2) / n)`.
 */
template <typename Derived>
inline double standard_deviation(const Eigen::MatrixBase<Derived>& vet)
{
  const auto vet_no_mean = vet.array() - vet.mean();
  return std::sqrt(vet_no_mean.square().sum() / vet.size());
}


/**
 * @brief Pearson correlation coefficient of two vectors of the same size.
 * @return a value in `[-1, 1]`, or 0 if either vector is constant (the
 *         correlation is undefined).
 */
template <typename Derived1, typename Derived2>
inline double correlation(const Eigen::MatrixBase<Derived1>& vet1, const Eigen::MatrixBase<Derived2>& vet2)
{
  const auto vet1_no_mean = vet1.array() - vet1.mean();
  const auto vet2_no_mean = vet2.array() - vet2.mean();
  const double var1 = vet1_no_mean.square().sum();
  const double var2 = vet2_no_mean.square().sum();

  // A constant vector has no variance: the correlation is undefined, return 0.
  // The tolerance absorbs the rounding error of mean() on constant vectors.
  constexpr double eps = std::numeric_limits<double>::epsilon();
  if (var1 <= eps * vet1.squaredNorm() || var2 <= eps * vet2.squaredNorm())
    return 0.0;

  return (vet1_no_mean * vet2_no_mean).sum() / std::sqrt(var1 * var2);
}

/// @brief `n!`; overflows `unsigned int` for `n > 12`.
inline unsigned int factorial(unsigned int n)
{
    return (n == 0) ? 1 : n * factorial(n - 1);
}



}

#endif  // THOR_MATH__THOR_MATH__EIGEN_MATRIX_UTILS