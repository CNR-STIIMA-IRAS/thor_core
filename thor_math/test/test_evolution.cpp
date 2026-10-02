#include <gtest/gtest.h>

#include <thor_math/thor_math.h>

#include <cmath>

namespace thor::math {
namespace {

TEST(EvolutionMatrix, FreeAndForcedResponsesHaveExpectedShapeAndValues)
{
  constexpr unsigned int axes = 2;
  const Eigen::MatrixXd free = freeResponse(0.5, axes);
  const Eigen::MatrixXd forced = forcedResponse(0.5, axes);

  ASSERT_EQ(free.rows(), 4);
  ASSERT_EQ(free.cols(), 4);
  ASSERT_EQ(forced.rows(), 4);
  ASSERT_EQ(forced.cols(), 2);

  EXPECT_DOUBLE_EQ(free(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(free(0, 2), 0.5);
  EXPECT_DOUBLE_EQ(forced(0, 0), 0.125);
  EXPECT_DOUBLE_EQ(forced(2, 0), 0.5);
}

TEST(EvolutionMatrix, ConstantIntervalsCoverTheRequestedHorizon)
{
  Eigen::VectorXd intervals;
  Eigen::VectorXd prediction;

  ASSERT_TRUE(constantControlIntervals(1.0, 4, 0.05,
                                       intervals, prediction));
  ASSERT_EQ(intervals.size(), 4);
  ASSERT_EQ(prediction.size(), 4);

  for (Eigen::Index index = 0; index < intervals.size(); ++index) {
    EXPECT_DOUBLE_EQ(intervals(index), 0.25);
    EXPECT_DOUBLE_EQ(prediction(index), 0.25 * (index + 1));
  }
}

TEST(EvolutionMatrix, RejectsEmptyPredictionOrControlVectors)
{
  Eigen::VectorXd prediction;
  Eigen::VectorXd intervals;
  Eigen::MatrixXd free;
  Eigen::MatrixXd forced;

  EXPECT_FALSE(computeEvolutionMatrix(prediction, intervals, 2, free, forced));

  prediction = Eigen::VectorXd::Constant(1, 0.1);
  EXPECT_FALSE(computeEvolutionMatrix(prediction, intervals, 2, free, forced));
}

TEST(QuadraticProgram, SolvesEqualityAndInequalityConstraints)
{
  Eigen::MatrixXd hessian = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd gradient(2);
  gradient << -2.0, -5.0;

  Eigen::MatrixXd equality(2, 1);
  equality << 1.0, -1.0;
  Eigen::VectorXd equality_offset = Eigen::VectorXd::Zero(1);

  Eigen::MatrixXd inequality(2, 1);
  inequality << -1.0, -1.0;
  Eigen::VectorXd inequality_offset(1);
  inequality_offset << 2.0;

  Eigen::VectorXd solution;
  const double objective = Eigen::solve_quadprog(
    hessian, gradient, equality, equality_offset,
    inequality, inequality_offset, solution);

  ASSERT_TRUE(std::isfinite(objective));
  ASSERT_EQ(solution.size(), 2);
  EXPECT_NEAR(solution(0), 1.0, 1e-9);
  EXPECT_NEAR(solution(1), 1.0, 1e-9);
  EXPECT_NEAR((equality.transpose() * solution)(0), 0.0, 1e-9);
  EXPECT_GE((inequality.transpose() * solution + inequality_offset)(0),
            -1e-9);
}

TEST(QuadraticProgram, ReportsInfeasibleConstraints)
{
  Eigen::MatrixXd hessian = Eigen::MatrixXd::Identity(1, 1);
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(1);
  Eigen::MatrixXd equality(1, 0);
  Eigen::VectorXd equality_offset(0);

  Eigen::MatrixXd inequality(1, 2);
  inequality << 1.0, -1.0;
  Eigen::VectorXd inequality_offset(2);
  inequality_offset << -1.0, 0.0;

  Eigen::VectorXd solution;
  const double objective = Eigen::solve_quadprog(
    hessian, gradient, equality, equality_offset,
    inequality, inequality_offset, solution);

  EXPECT_TRUE(std::isinf(objective));
}

}  // namespace
}  // namespace thor::math
