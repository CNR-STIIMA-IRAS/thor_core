#include <gtest/gtest.h>

#include <thor_math/thor_math.h>

#include <cmath>
#include <limits>
#include <vector>

namespace thor::math {
namespace {

class InspectableThorQP : public ThorQP
{
public:
  using ThorQP::compute_h;
  using ThorQP::compute_theta;

  bool torqueBoundsActive() const { return m_are_torque_bounds_active; }
  bool cbfBoundsActive() const { return m_use_cbf; }
  bool cbfMoveAwayActive() const { return m_use_cbf_move_away; }
  bool inputBlockingActive() const { return m_use_input_blocking; }
  Eigen::Index inequalityCount() const { return m_CI.cols(); }
};

void configureQp(InspectableThorQP& qp, bool input_blocking = true)
{
  constexpr unsigned int axes = 2;
  constexpr unsigned int intervals = 3;
  qp.setIntervals(intervals, axes, 0.3, 0.05, input_blocking);
  qp.setConstraints(Eigen::VectorXd::Constant(axes, 2.0),
                    Eigen::VectorXd::Constant(axes, -2.0),
                    Eigen::VectorXd::Constant(axes, 3.0),
                    Eigen::VectorXd::Constant(axes, 10.0),
                    Eigen::VectorXd::Constant(axes, 20.0));
  qp.setWeigthFunction(1e-3, 0.0, 1e-4, 10.0, 1.0);
  qp.setCBFParameters(2.5, 0.15, 0.5, 3.0);
  qp.setCbfIds({}, 0);
}

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

TEST(EvolutionMatrix, BuildsPiecewiseConstantAccelerationResponse)
{
  Eigen::VectorXd intervals(2);
  intervals << 0.5, 0.5;
  Eigen::VectorXd prediction(3);
  prediction << 0.25, 0.5, 1.0;
  Eigen::MatrixXd free;
  Eigen::MatrixXd forced;

  ASSERT_TRUE(computeEvolutionMatrix(prediction, intervals, 1, free, forced));
  ASSERT_EQ(free.rows(), 6);
  ASSERT_EQ(forced.rows(), 6);
  ASSERT_EQ(forced.cols(), 2);

  EXPECT_NEAR(forced(0, 0), 0.5 * 0.25 * 0.25, 1e-12);
  EXPECT_NEAR(forced(2, 1), 0.0, 1e-12);
  EXPECT_NEAR(forced(4, 0), 0.375, 1e-12);
  EXPECT_NEAR(forced(4, 1), 0.125, 1e-12);
  EXPECT_NEAR(forced(5, 0), 0.5, 1e-12);
  EXPECT_NEAR(forced(5, 1), 0.5, 1e-12);
}

TEST(EvolutionMatrix, SplitsPositionAndVelocityResponses)
{
  Eigen::VectorXd intervals = Eigen::VectorXd::Constant(2, 0.5);
  Eigen::VectorXd prediction(2);
  prediction << 0.5, 1.0;
  Eigen::MatrixXd free;
  Eigen::MatrixXd forced;
  ASSERT_TRUE(computeEvolutionMatrix(prediction, intervals, 1, free, forced));

  Eigen::MatrixXd velocity_free;
  Eigen::MatrixXd position_free;
  Eigen::MatrixXd velocity_forced;
  Eigen::MatrixXd position_forced;
  splitResponses(free, velocity_free, position_free, forced,
                 velocity_forced, position_forced, 1);

  ASSERT_EQ(position_free.rows(), 2);
  ASSERT_EQ(position_free.cols(), 2);
  ASSERT_EQ(velocity_free.rows(), 2);
  ASSERT_EQ(velocity_free.cols(), 1);
  EXPECT_DOUBLE_EQ(position_free(1, 1), 1.0);
  EXPECT_DOUBLE_EQ(velocity_free(1, 0), 1.0);
  EXPECT_DOUBLE_EQ(position_forced(1, 0), 0.375);
  EXPECT_DOUBLE_EQ(velocity_forced(1, 1), 0.5);
}

TEST(EvolutionMatrix, BuildsJerkDifferenceOperator)
{
  Eigen::VectorXd intervals(3);
  intervals << 0.1, 0.2, 0.4;
  Eigen::VectorXd prediction(3);
  prediction << 0.1, 0.3, 0.7;
  Eigen::MatrixXd free;
  Eigen::MatrixXd forced;

  ASSERT_TRUE(computeJerkEvolutionMatrix(prediction, intervals, 1,
                                         free, forced));
  ASSERT_EQ(free.rows(), 3);
  ASSERT_EQ(forced.rows(), 3);
  EXPECT_DOUBLE_EQ(free(0, 0), -10.0);
  EXPECT_DOUBLE_EQ(forced(0, 0), 10.0);
  EXPECT_DOUBLE_EQ(forced(1, 0), -5.0);
  EXPECT_DOUBLE_EQ(forced(1, 1), 5.0);
  EXPECT_DOUBLE_EQ(forced(2, 1), -2.5);
  EXPECT_DOUBLE_EQ(forced(2, 2), 2.5);

  Eigen::VectorXd empty;
  EXPECT_FALSE(computeJerkEvolutionMatrix(empty, intervals, 1, free, forced));
  EXPECT_FALSE(computeJerkEvolutionMatrix(prediction, empty, 1, free, forced));
}

TEST(ControlIntervals, BuildsQuadraticAndRoundedConstantSchedules)
{
  Eigen::VectorXd intervals;
  Eigen::VectorXd prediction;
  ASSERT_TRUE(quadraticControlIntervals(1.0, 4, 0.1,
                                        intervals, prediction));
  const Eigen::Vector4d expected_prediction(0.1, 0.2, 0.5, 1.0);
  const Eigen::Vector4d expected_intervals(0.1, 0.1, 0.3, 0.5);
  EXPECT_TRUE(prediction.isApprox(expected_prediction));
  EXPECT_TRUE(intervals.isApprox(expected_intervals));

  ASSERT_TRUE(constantControlIntervals(1.0, 3, 0.1,
                                       intervals, prediction));
  EXPECT_TRUE(intervals.isApprox(Eigen::Vector3d::Constant(0.3)));
  EXPECT_DOUBLE_EQ(prediction(2), 0.9);
}

TEST(ThorQp, SettersUpdateFlagsAndMatrixLayout)
{
  InspectableThorQP qp;
  configureQp(qp);
  EXPECT_TRUE(qp.needUpdate());
  EXPECT_TRUE(qp.inputBlockingActive());
  EXPECT_DOUBLE_EQ(qp.getDt(), 0.05);
  EXPECT_DOUBLE_EQ(qp.getNumPh(), 0.0);

  qp.activatePositionBounds(true);
  qp.activateTorqueBounds(true);
  qp.activateCbfBounds(true);
  qp.activateCbfMoveAway(true);
  EXPECT_TRUE(qp.arePositionBoundsActive());
  EXPECT_TRUE(qp.torqueBoundsActive());
  EXPECT_TRUE(qp.cbfBoundsActive());
  EXPECT_TRUE(qp.cbfMoveAwayActive());

  qp.setCbfIds({}, 0);
  ASSERT_NO_THROW(qp.updateMatrices());
  EXPECT_FALSE(qp.needUpdate());
  EXPECT_EQ(qp.getPredictionTimeInstant().size(), 3);
  EXPECT_EQ(qp.inequalityCount(), 8 * 3 * 2 + 2 * 3);

  qp.activatePositionBounds(false);
  qp.activateTorqueBounds(false);
  qp.activateCbfBounds(false);
  qp.activateCbfMoveAway(false);
  EXPECT_FALSE(qp.arePositionBoundsActive());
  EXPECT_FALSE(qp.torqueBoundsActive());
  EXPECT_FALSE(qp.cbfBoundsActive());
  EXPECT_FALSE(qp.cbfMoveAwayActive());
}

TEST(ThorQp, InitializesAndIntegratesState)
{
  InspectableThorQP qp;
  EXPECT_EQ(qp.getFirstPredictionPos().size(), 0);
  EXPECT_EQ(qp.getFirstPredictionVel().size(), 0);
  configureQp(qp, false);
  qp.updateMatrices();
  EXPECT_FALSE(qp.inputBlockingActive());

  Eigen::Vector4d state;
  state << 1.0, -1.0, 0.5, -0.5;
  qp.setInitialState(state);
  EXPECT_TRUE(qp.getState().isApprox(state));
  EXPECT_TRUE(qp.getFirstPredictionPos().isApprox(state.head<2>()));
  EXPECT_TRUE(qp.getFirstPredictionVel().isZero());

  Eigen::Vector2d acceleration(2.0, -2.0);
  qp.updateState(acceleration);
  Eigen::Vector4d expected;
  expected << 1.0275, -1.0275, 0.6, -0.6;
  EXPECT_TRUE(qp.getState().isApprox(expected, 1e-12));
}

TEST(ThorQp, ComputesFiniteUnconstrainedSolution)
{
  InspectableThorQP qp;
  configureQp(qp, false);
  qp.updateMatrices();
  const Eigen::Vector4d state = Eigen::Vector4d::Zero();
  qp.setInitialState(state);

  Eigen::VectorXd acceleration;
  double scaling = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(qp.computedUncostrainedSolution(
    Eigen::VectorXd::Zero(6), Eigen::Vector2d::Zero(), 0.75, state,
    acceleration, scaling));
  ASSERT_EQ(acceleration.size(), 2);
  EXPECT_TRUE(acceleration.allFinite());
  EXPECT_TRUE(std::isfinite(scaling));
}

TEST(ThorQp, ComputesBarrierValuesAndDerivativesForAllModes)
{
  InspectableThorQP qp;
  configureQp(qp);
  std::vector<double> theta(2);
  double h = 0.0;

  qp.compute_h(0.2, -0.1, 1.0, h);
  qp.compute_theta(0.2, -0.1, 1.0, theta);
  EXPECT_TRUE(std::isfinite(h));
  EXPECT_DOUBLE_EQ(theta[0], 1.0);
  EXPECT_TRUE(std::isfinite(theta[1]));

  qp.compute_h(0.0, 0.2, 1.0, h);
  qp.compute_theta(0.0, 0.2, 1.0, theta);
  EXPECT_TRUE(std::isfinite(h));
  EXPECT_DOUBLE_EQ(theta[1], 0.15);

  qp.activateCbfMoveAway(true);
  qp.compute_h(0.1, 0.2, 1.0, h);
  qp.compute_theta(0.1, 0.2, 1.0, theta);
  EXPECT_TRUE(std::isfinite(h));
  EXPECT_DOUBLE_EQ(theta[0], 1.0);
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
