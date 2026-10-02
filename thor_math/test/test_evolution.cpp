#include <gtest/gtest.h>

#include <thor_math/thor_math.h>

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

}  // namespace
}  // namespace thor::math
