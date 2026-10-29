#include <gtest/gtest.h>

#include <thor_math/thor_math.h>
#include <qpmad/solver.h>

#include <cmath>
#include <limits>
#include <vector>

namespace thor::math {
namespace {

class InspectableThorQP : public ThorQP
{
public:
  bool torqueBoundsActive() const { return m_are_torque_bounds_active; }
  bool cbfBoundsActive() const { return m_use_cbf; }
  bool inputBlockingActive() const { return m_use_input_blocking; }
  Eigen::Index inequalityCount() const { return m_CI.cols(); }
  const Eigen::MatrixXd& hessian() const { return m_H; }
};

ThorConfig makeConfig(bool input_blocking = true)
{
  ThorConfig config;
  config.n_intervals = 3;
  config.n_joints = 2;
  config.horizon_time = 0.3;
  config.sampling_period = 0.05;
  config.input_blocking = input_blocking;
  config.lambda_acc = 1e-3;
  config.lambda_jerk = 1e-4;
  config.lambda_scaling = 10.0;
  config.lambda_clik = 1.0;
  config.qmax = Eigen::VectorXd::Constant(2, 2.0);
  config.qmin = Eigen::VectorXd::Constant(2, -2.0);
  config.Dqmax = Eigen::VectorXd::Constant(2, 3.0);
  config.DDqmax = Eigen::VectorXd::Constant(2, 10.0);
  config.tau_max = Eigen::VectorXd::Constant(2, 20.0);
  config.a_s = 2.5;
  config.T_r = 0.15;
  config.C = 0.5;
  config.gamma = 3.0;
  return config;
}

ThorInput makeInput()
{
  ThorInput in;
  in.target_dq = Eigen::VectorXd::Constant(6, 0.5);
  in.next_target_q = Eigen::Vector2d(0.1, -0.1);
  in.target_scaling = 1.0;
  in.x0 = Eigen::Vector4d(0.0, 0.0, 0.2, -0.2);
  return in;
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

TEST(ThorQp, ConfigureSetsFlagsAndMatrixLayout)
{
  InspectableThorQP qp;
  EXPECT_TRUE(qp.needUpdate());

  ThorConfig config = makeConfig();
  config.position_bounds = true;
  ASSERT_NO_THROW(qp.configure(config));
  EXPECT_FALSE(qp.needUpdate());
  EXPECT_TRUE(qp.inputBlockingActive());
  EXPECT_TRUE(qp.arePositionBoundsActive());
  EXPECT_FALSE(qp.cbfBoundsActive());
  EXPECT_DOUBLE_EQ(qp.getDt(), 0.05);
  EXPECT_EQ(qp.getNumObstaclePoints(), 0U);
  EXPECT_EQ(qp.getPredictionTimeInstant().size(), 3);
  EXPECT_EQ(qp.inequalityCount(), 8 * 3 * 2 + 2 * 3);

  config.position_bounds = false;
  ASSERT_NO_THROW(qp.configure(config));
  EXPECT_FALSE(qp.arePositionBoundsActive());
  EXPECT_EQ(qp.inequalityCount(), 4 * 3 * 2 + 2 * 3);
}

TEST(ThorQp, InitializesAndIntegratesState)
{
  InspectableThorQP qp;
  EXPECT_EQ(qp.getFirstPredictionPos().size(), 0);
  EXPECT_EQ(qp.getFirstPredictionVel().size(), 0);
  qp.configure(makeConfig(false));
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

// Regression: the jerk weight was added to the Hessian at every cycle, so the
// cost changed from one solve to the next with the same inputs.
TEST(ThorQp, CostDoesNotAccumulateAcrossCycles)
{
  InspectableThorQP qp;
  ThorConfig config = makeConfig(false);
  config.lambda_jerk = 1e-2;
  qp.configure(config);
  const Eigen::Vector4d state(0.1, -0.1, 0.2, 0.0);
  qp.setInitialState(state);

  Eigen::VectorXd acceleration;
  double scaling = 0.0;
  ASSERT_TRUE(qp.computedUncostrainedSolution(
    Eigen::VectorXd::Constant(6, 0.3), Eigen::Vector2d::Zero(), 1.0, state, acceleration, scaling));
  const Eigen::MatrixXd first = qp.hessian();
  for (int cycle = 0; cycle < 5; ++cycle) {
    ASSERT_TRUE(qp.computedUncostrainedSolution(
      Eigen::VectorXd::Constant(6, 0.3), Eigen::Vector2d::Zero(), 1.0, state, acceleration, scaling));
  }
  EXPECT_TRUE(qp.hessian().isApprox(first, 1e-12)) << (qp.hessian() - first).norm();
}

TEST(ThorQp, ComputesFiniteUnconstrainedSolution)
{
  InspectableThorQP qp;
  qp.configure(makeConfig(false));
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

TEST(QuadraticProgram, SolvesEqualityAndInequalityConstraints)
{
  Eigen::MatrixXd hessian = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd original_hessian = hessian;
  Eigen::VectorXd gradient(2);
  gradient << -2.0, -5.0;
  Eigen::MatrixXd constraints(2, 2);
  constraints << 1.0, -1.0,
                 1.0,  1.0;
  Eigen::VectorXd lower(2), upper(2);
  lower << 0.0, -std::numeric_limits<double>::infinity();
  upper << 0.0, 2.0;
  Eigen::VectorXd solution;
  qpmad::Solver solver;
  ASSERT_EQ(solver.solve(solution, hessian, gradient, constraints, lower, upper),
            qpmad::Solver::OK);
  ASSERT_EQ(solution.size(), 2);
  EXPECT_NEAR(solution(0), 1.0, 1e-9);
  EXPECT_NEAR(solution(1), 1.0, 1e-9);
  EXPECT_NEAR(0.5 * solution.dot(original_hessian * solution) +
              gradient.dot(solution), -6.0, 1e-9);
  EXPECT_NEAR((constraints * solution)(0), 0.0, 1e-9);
  EXPECT_LE((constraints * solution)(1), 2.0 + 1e-9);
}

// Analytic optimum: x - 2y = 1 and x <= 3 force (3, 1).
TEST(QuadraticProgram, PreservesNonzeroOffsetsAndAsymmetricBounds)
{
  Eigen::MatrixXd original_hessian(2, 2);
  original_hessian << 4.0, 1.0, 1.0, 2.0;
  Eigen::VectorXd gradient(2);
  gradient << -20.0, -8.0;
  Eigen::MatrixXd constraints(3, 2);
  constraints << 1.0, -2.0,
                 1.0,  0.0,
                 0.0,  1.0;
  Eigen::VectorXd lower(3), upper(3);
  lower << 1.0, 0.5, -0.25;
  upper << 1.0, 3.0, std::numeric_limits<double>::infinity();
  qpmad::Solver solver;
  for (int iteration = 0; iteration < 3; ++iteration) {
    SCOPED_TRACE(iteration);
    Eigen::MatrixXd hessian = original_hessian;
    Eigen::VectorXd solution;
    ASSERT_EQ(solver.solve(solution, hessian, gradient, constraints, lower, upper),
              qpmad::Solver::OK);
    ASSERT_EQ(solution.size(), 2);
    EXPECT_NEAR(solution(0), 3.0, 1e-9);
    EXPECT_NEAR(solution(1), 1.0, 1e-9);
    EXPECT_NEAR(0.5 * solution.dot(original_hessian * solution) +
                gradient.dot(solution), -46.0, 1e-9);
    const Eigen::VectorXd values = constraints * solution;
    EXPECT_GE((values - lower).minCoeff(), -1e-9);
    EXPECT_GE((upper - values).minCoeff(), -1e-9);
  }
}

TEST(QuadraticProgram, ReportsInfeasibleConstraints)
{
  Eigen::MatrixXd hessian = Eigen::MatrixXd::Identity(1, 1);
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(1);
  Eigen::MatrixXd constraints(2, 1);
  constraints << 1.0, -1.0;
  Eigen::VectorXd lower(2);
  lower << 1.0, 0.0;
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(
    2, std::numeric_limits<double>::infinity());
  Eigen::VectorXd solution;
  qpmad::Solver solver;
  // qpmad reports inconsistent constraints through a runtime_error.
  EXPECT_THROW(solver.solve(solution, hessian, gradient, constraints, lower, upper),
               std::runtime_error);
}

TEST(ThorQpConfig, ConstructorConfigures)
{
  ThorQP qp(makeConfig());
  EXPECT_FALSE(qp.needUpdate());
  EXPECT_DOUBLE_EQ(qp.getDt(), 0.05);
}

TEST(ThorQpConfig, RejectsInvalidSettingsWithoutChangingState)
{
  InspectableThorQP qp;
  ASSERT_NO_THROW(qp.configure(makeConfig()));
  const Eigen::Index inequalities = qp.inequalityCount();

  const auto expectRejected = [&](const char* what, auto&& modify) {
    SCOPED_TRACE(what);
    ThorConfig config = makeConfig();
    modify(config);
    EXPECT_THROW(qp.configure(config), std::invalid_argument);
    EXPECT_FALSE(qp.needUpdate());
    EXPECT_EQ(qp.inequalityCount(), inequalities);
    EXPECT_DOUBLE_EQ(qp.getDt(), 0.05);
  };

  expectRejected("no intervals", [](ThorConfig& c) { c.n_intervals = 0; });
  expectRejected("no joints", [](ThorConfig& c) { c.n_joints = 0; });
  expectRejected("zero sampling period", [](ThorConfig& c) { c.sampling_period = 0.0; });
  expectRejected("horizon shorter than one sample",
                 [](ThorConfig& c) { c.horizon_time = 0.01; });
  expectRejected("zero-length quadratic intervals", [](ThorConfig& c) {
    c.horizon_time = 0.1;
    c.sampling_period = 0.01;
    c.n_intervals = 15;
  });
  expectRejected("single quadratic interval", [](ThorConfig& c) { c.n_intervals = 1; });
  expectRejected("wrong qmax size", [](ThorConfig& c) { c.qmax = Eigen::VectorXd::Ones(3); });
  expectRejected("missing DDqmax", [](ThorConfig& c) { c.DDqmax.resize(0); });
  expectRejected("wrong tau_max size", [](ThorConfig& c) { c.tau_max = Eigen::VectorXd::Ones(1); });
  expectRejected("qmin above qmax", [](ThorConfig& c) { c.qmin(0) = 3.0; });
  expectRejected("zero velocity limit", [](ThorConfig& c) { c.Dqmax(1) = 0.0; });
  expectRejected("negative acceleration limit", [](ThorConfig& c) { c.DDqmax(1) = -1.0; });
  expectRejected("inverted scaling bounds", [](ThorConfig& c) {
    c.min_scaling = 1.0;
    c.max_scaling = 0.5;
  });
  expectRejected("CBF without a model", [](ThorConfig& c) { c.cbf = true; });

  ThorConfig empty_tau = makeConfig();
  empty_tau.tau_max.resize(0);
  EXPECT_NO_THROW(qp.configure(empty_tau));
}

TEST(ThorQpConfig, SolveValidatesInputs)
{
  ThorQP unconfigured;
  EXPECT_THROW(unconfigured.solve(makeInput()), std::logic_error);

  ThorQP qp(makeConfig());
  qp.setInitialState(makeInput().x0);
  const auto expectRejected = [&](const char* what, auto&& modify) {
    SCOPED_TRACE(what);
    ThorInput in = makeInput();
    modify(in);
    EXPECT_THROW(qp.solve(in), std::invalid_argument);
  };
  expectRejected("target_dq for one interval",
                 [](ThorInput& in) { in.target_dq = Eigen::Vector2d::Zero(); });
  expectRejected("next_target_q too long",
                 [](ThorInput& in) { in.next_target_q = Eigen::Vector3d::Zero(); });
  expectRejected("x0 without velocities",
                 [](ThorInput& in) { in.x0 = Eigen::Vector2d::Zero(); });
}

TEST(ThorQpConfig, ScalingBoundsAreConfigurable)
{
  ThorConfig config = makeConfig(false);
  config.min_scaling = 0.2;
  config.max_scaling = 0.5;
  ThorQP qp(config);

  ThorInput in = makeInput();
  in.target_scaling = 1.0;
  qp.setInitialState(in.x0);
  ThorOutput out;
  ASSERT_NO_THROW(out = qp.solve(in));
  EXPECT_GE(out.next_scaling, 0.2 - 1e-8);
  EXPECT_LE(out.next_scaling, 0.5 + 1e-8);
  EXPECT_GT(out.next_scaling, 0.5 - 1e-3);  // the target pulls it to the upper bound
}

// ---------------------------------------------------------------------------
// Deprecated API: these tests keep the legacy setters and
// computedCostrainedSolution wrappers working until they are removed.
// ---------------------------------------------------------------------------
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

// Same settings as makeConfig.
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

TEST(LegacyApi, SettersUpdateFlagsAndMatrixLayout)
{
  InspectableThorQP qp;
  configureQp(qp);
  EXPECT_TRUE(qp.needUpdate());
  EXPECT_TRUE(qp.inputBlockingActive());
  EXPECT_DOUBLE_EQ(qp.getDt(), 0.05);
  EXPECT_EQ(qp.getNumObstaclePoints(), 0U);

  qp.activatePositionBounds(true);
  qp.activateTorqueBounds(true);
  qp.activateCbfBounds(true);
  qp.activateCbfMoveAway(true);
  EXPECT_TRUE(qp.arePositionBoundsActive());
  EXPECT_TRUE(qp.torqueBoundsActive());
  EXPECT_TRUE(qp.cbfBoundsActive());

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
}

TEST(LegacyApi, MatchesConfigureAndSolve)
{
  for (const bool input_blocking : {true, false}) {
    SCOPED_TRACE(input_blocking);
    InspectableThorQP legacy;
    configureQp(legacy, input_blocking);
    legacy.updateMatrices();

    InspectableThorQP configured;
    ASSERT_NO_THROW(configured.configure(makeConfig(input_blocking)));
    EXPECT_EQ(configured.inequalityCount(), legacy.inequalityCount());

    const ThorInput in = makeInput();
    legacy.setInitialState(in.x0);
    configured.setInitialState(in.x0);

    // Two cycles, so the jerk term sees a nonzero previous solution.
    for (int cycle = 0; cycle < 2; ++cycle) {
      Eigen::VectorXd acceleration;
      double scaling = 0.0;
      std::vector<double> diagnostics;
      ASSERT_NO_THROW(diagnostics = legacy.computedCostrainedSolution(
        in.target_dq, in.next_target_q, in.target_scaling, in.x0,
        acceleration, scaling));
      ThorOutput out;
      ASSERT_NO_THROW(out = configured.solve(in));
      EXPECT_TRUE(out.next_acc.isApprox(acceleration, 1e-12));
      EXPECT_DOUBLE_EQ(out.next_scaling, scaling);
      EXPECT_EQ(diagnostics, std::vector<double>(6, 0.0));
      EXPECT_DOUBLE_EQ(out.cbf.h_min, 0.0);
    }
  }
}

TEST(LegacyApi, SolveRejectsOutdatedMatrices)
{
  ThorQP qp(makeConfig());
  qp.setInitialState(makeInput().x0);
  qp.setWeigthFunction(1e-3, 0.0, 1e-4, 10.0, 2.0);
  EXPECT_THROW(qp.solve(makeInput()), std::logic_error);
}

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

}  // namespace
}  // namespace thor::math
