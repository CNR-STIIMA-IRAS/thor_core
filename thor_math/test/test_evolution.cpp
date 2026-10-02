#include <thor_math/thor_math.h>

#include <cmath>
#include <iostream>

namespace {

bool approximately_equal(double lhs, double rhs, double tolerance = 1e-12)
{
  return std::abs(lhs - rhs) <= tolerance;
}

}  // namespace

int main()
{
  constexpr unsigned int axes = 2;
  const Eigen::MatrixXd free = thor::math::freeResponse(0.5, axes);
  const Eigen::MatrixXd forced = thor::math::forcedResponse(0.5, axes);

  if (free.rows() != 4 || free.cols() != 4 ||
      forced.rows() != 4 || forced.cols() != 2) {
    std::cerr << "Unexpected evolution matrix dimensions\n";
    return 1;
  }

  if (!approximately_equal(free(0, 0), 1.0) ||
      !approximately_equal(free(0, 2), 0.5) ||
      !approximately_equal(forced(0, 0), 0.125) ||
      !approximately_equal(forced(2, 0), 0.5)) {
    std::cerr << "Unexpected evolution matrix values\n";
    return 1;
  }

  Eigen::VectorXd intervals;
  Eigen::VectorXd prediction;
  if (!thor::math::constantControlIntervals(1.0, 4, 0.05,
                                            intervals, prediction) ||
      intervals.size() != 4 || prediction.size() != 4 ||
      !approximately_equal(prediction(3), 1.0)) {
    std::cerr << "Constant control interval generation failed\n";
    return 1;
  }

  return 0;
}
