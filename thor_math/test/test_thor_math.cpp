#include <gtest/gtest.h>

#include <thor_math/thor_math.h>

#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

namespace thor::math {
namespace {

// GCC 9 on Ubuntu 20.04 does not provide the C++20 <numbers> header.
constexpr double kPi = 3.141592653589793238462643383279502884;

class InspectableOptimizer : public ThorQP
{
public:
  const Eigen::VectorXd& solution() const { return m_sol; }
  const Eigen::MatrixXd& inequalities() const { return m_CI; }
  const Eigen::VectorXd& offsets() const { return m_ci0; }
  const Eigen::VectorXd& positions() const { return m_prediction_pos; }
  const Eigen::VectorXd& velocities() const { return m_prediction_vel; }
  Eigen::VectorXd stateOffsets(const Eigen::VectorXd& state) const
  {
    Eigen::VectorXd result = m_ci0;
    const Eigen::Index block = m_nc * m_nax;
    const Eigen::Index start = 2 * m_nc * (m_nax + 1);
    result.segment(start, block) += m_velocity_free_resp * state.tail(m_nax);
    result.segment(start + block, block) -= m_velocity_free_resp * state.tail(m_nax);
    if (m_are_position_bounds_active) {
      result.segment(start + 2 * block, block) += m_position_free_resp * state;
      result.segment(start + 3 * block, block) -= m_position_free_resp * state;
      result.segment(start + 4 * block, block) += m_invariance_free_resp * state;
      result.segment(start + 5 * block, block) -= m_invariance_free_resp * state;
    }
    return result;
  }

  const Eigen::VectorXd& solvedOffsets() const { return m_ci0_solved; }
  void setSolution(const Eigen::VectorXd& solution) { m_sol = solution; }
  const Eigen::MatrixXd& hessian() const { return m_H; }
  const Eigen::VectorXd& gradient() const { return m_f; }
  const Eigen::VectorXd& controlIntervals() const { return m_control_intervals; }

  // Inverted Cholesky factor built from the constant acceleration block.
  bool blockInvertedFactor(Eigen::MatrixXd& factor)
  {
    if (!invertedCholeskyFactor())
      return false;
    factor = m_ws.qp_H.triangularView<Eigen::Upper>();
    return true;
  }

  // Inequalities after the variable bounds that hold for every x within them,
  // with the offsets of the last solve.
  Eigen::Index redundantInequalities() const
  {
    Eigen::Index count = 0;
    for (Eigen::Index col = 2 * m_H.rows(); col < m_CI.cols(); ++col)
      if (m_CI.col(col).cwiseMax(0.0).dot(m_lb) + m_CI.col(col).cwiseMin(0.0).dot(m_ub) + m_ci0_solved(col) >= 0.0)
        ++count;
    return count;
  }

  // Predicted [q; dq] at node m (0: x0, m: end of interval m-1) for inputs u.
  Eigen::VectorXd node(const Eigen::VectorXd& x0, const Eigen::VectorXd& u, unsigned int m) const
  {
    if (m == 0)
      return x0;
    Eigen::VectorXd x(2 * m_nax);
    x.head(m_nax) = m_position_forced_resp.middleRows(m_nax * (m - 1), m_nax) * u +
                    m_position_free_resp.middleRows(m_nax * (m - 1), m_nax) * x0;
    x.tail(m_nax) = m_velocity_forced_resp.middleRows(m_nax * (m - 1), m_nax) * u +
                    m_velocity_free_resp.middleRows(m_nax * (m - 1), m_nax) * x0.tail(m_nax);
    return x;
  }
};

class ThorQpIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const std::filesystem::path urdf =
      std::filesystem::path(THOR_MATH_TEST_DATA_DIR) /
      "data" / "ur10" / "ur10_with_intermediates.urdf";

    ASSERT_TRUE(std::filesystem::is_regular_file(urdf))
      << "Missing test URDF: " << urdf;
    ASSERT_NO_THROW(pinocchio::urdf::buildModel(urdf.string(), model_));
    ASSERT_EQ(model_.nv, 6);
  }

  Eigen::VectorXd constant(double value) const
  {
    return Eigen::VectorXd::Constant(model_.nv, value);
  }

  // Horizon and CBF gains shared by all tests; limits and weights are per test.
  ThorConfig baseConfig(unsigned int intervals, double horizon,
                        double sampling_period, bool input_blocking) const
  {
    ThorConfig config;
    config.n_intervals = intervals;
    config.n_joints = static_cast<unsigned int>(model_.nv);
    config.horizon_time = horizon;
    config.sampling_period = sampling_period;
    config.input_blocking = input_blocking;
    config.a_s = 2.5;
    config.T_r = 0.15;
    config.C = 0.5;
    config.gamma = 3.0;
    return config;
  }

  // Joint frames and the intermediate frames added along the links.
  std::vector<unsigned int> cbfFrameIds() const
  {
    std::vector<unsigned int> frame_ids;
    for (std::size_t index = 0; index < model_.frames.size(); ++index) {
      const pinocchio::Frame &frame = model_.frames[index];
      if (frame.type == pinocchio::JOINT ||
          frame.name.find("intermediate") != std::string::npos) {
        frame_ids.push_back(static_cast<unsigned int>(index));
      }
    }
    return frame_ids;
  }

  pinocchio::Model model_;
};

TEST_F(ThorQpIntegrationTest, LoadsUr10ModelAndIntermediateFrames)
{
  EXPECT_EQ(model_.nq, 6);
  EXPECT_GT(model_.frames.size(), 6U);

  bool found_intermediate_frame = false;
  for (const pinocchio::Frame &frame : model_.frames) {
    found_intermediate_frame =
      found_intermediate_frame ||
      frame.name.find("intermediate") != std::string::npos;
  }
  EXPECT_TRUE(found_intermediate_frame);
}

TEST_F(ThorQpIntegrationTest, ComputesOneFiniteConstrainedStep)
{
  constexpr unsigned int intervals = 5;
  const auto axes = static_cast<unsigned int>(model_.nv);

  ThorConfig config = baseConfig(intervals, 0.2, 0.002, true);
  config.qmax = constant(kPi);
  config.qmin = constant(-kPi);
  config.Dqmax = constant(30.0);
  config.DDqmax = constant(500.0);
  config.tau_max = constant(10.0);
  config.lambda_acc = 1.0e-6;
  config.lambda_tau = 1.0e-9;
  config.lambda_scaling = 5e+1;
  config.lambda_clik = 1e+4;
  config.position_bounds = true;
  config.cbf = true;
  config.cbf_formulation = CbfFormulation::continuous;  // rows checked below
  config.cbf_frame_ids = cbfFrameIds();
  config.n_obstacle_points = 1;
  const std::vector<unsigned int>& frame_ids = config.cbf_frame_ids;
  ASSERT_FALSE(frame_ids.empty());

  InspectableOptimizer qp;
  qp.setPinocchioModel(model_);
  ASSERT_NO_THROW(qp.configure(config));

  Eigen::VectorXd initial_position(axes);
  initial_position << 0.0, -kPi / 2.0,
    kPi / 2.0, 0.0, 0.0, 0.0;
  Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(2 * axes);
  initial_state.head(axes) = initial_position;
  qp.setInitialState(initial_state);

  ThorInput in;
  in.target_dq = Eigen::VectorXd::Zero(axes * intervals);
  in.next_target_q = initial_position;
  in.x0 = qp.getState();
  in.obstacle_pos = {Eigen::Vector3d(0.8, 0.3, 0.65)};
  in.obstacle_vel = {Eigen::Vector3d::Zero()};

  ThorOutput out;
  ASSERT_NO_THROW(out = qp.solve(in));

  ASSERT_EQ(out.next_acc.size(), axes);
  EXPECT_TRUE(out.next_acc.allFinite());
  EXPECT_TRUE(std::isfinite(out.next_scaling));

  // Robot and obstacle are still, so every interval is linearized at the initial
  // state. Independently rebuild each CBF column from cbf::barrierTerms:
  // column = J^T L_g h on the interval's accelerations, offset = L_f h + gamma h.
  pinocchio::Data data(model_);
  pinocchio::computeJointJacobians(model_, data, initial_position);
  pinocchio::updateFramePlacements(model_, data);
  const auto cbf_count = intervals * frame_ids.size();
  const Eigen::Index first_cbf = qp.inequalities().cols() - cbf_count;
  ASSERT_LT(out.cbf.h_min, config.cbf_activation_distance);  // full-constraint solver branch
  double h_min = std::numeric_limits<double>::infinity();
  for (std::size_t frame = 0; frame < frame_ids.size(); ++frame) {
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian =
      Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, model_.nv);
    pinocchio::getFrameJacobian(model_, data, frame_ids[frame],
                                pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
    const cbf::BarrierTerms barrier = cbf::barrierTerms(
      data.oMf[frame_ids[frame]].translation(), Eigen::Vector3d::Zero(),
      in.obstacle_pos[0], Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      config.T_r, config.a_s, config.C);
    h_min = std::min(h_min, barrier.h);
    const double offset = barrier.lie_f + config.gamma * barrier.h;
    const Eigen::VectorXd row = jacobian.topRows<3>().transpose() * barrier.lie_g;
    for (unsigned int interval = 0; interval < intervals; ++interval) {
      SCOPED_TRACE(::testing::Message() << "frame " << frame << " interval " << interval);
      const Eigen::Index column = qp.inequalities().cols() -
        intervals * (frame + 1) + interval;
      Eigen::VectorXd expected = Eigen::VectorXd::Zero(qp.inequalities().rows());
      expected.segment(axes * interval, axes) = row;
      EXPECT_TRUE(qp.inequalities().col(column).isApprox(expected, 1e-9) ||
                  (expected.isZero() && qp.inequalities().col(column).isZero()));
      EXPECT_GE(qp.inequalities().col(column).dot(qp.solution()) + offset, -1e-7);
    }
  }
  EXPECT_NEAR(out.cbf.h_min, h_min, 1e-12);
  const Eigen::VectorXd offsets = qp.stateOffsets(initial_state).head(first_cbf);
  EXPECT_GE((qp.inequalities().leftCols(first_cbf).transpose() * qp.solution()
    + offsets).minCoeff(), -1e-7);
  EXPECT_LE(qp.positions().cwiseAbs().maxCoeff(), kPi + 1e-7);
  EXPECT_LE(qp.velocities().cwiseAbs().maxCoeff(), 30.0 + 1e-7);

  ASSERT_NO_THROW(qp.updateState(out.next_acc));
  EXPECT_TRUE(qp.getState().allFinite());

  // Far-away obstacles select the reduced-constraint branch. Its result must
  // match the same configured optimizer with CBF disabled, including offsets
  // from a nonzero initial velocity.
  initial_state.tail(axes).setConstant(0.01);
  qp.setInitialState(initial_state);
  InspectableOptimizer without_cbf;
  without_cbf = qp;
  ThorConfig no_cbf = config;
  no_cbf.cbf = false;
  ASSERT_NO_THROW(without_cbf.configure(no_cbf));
  without_cbf.setInitialState(initial_state);
  in.x0 = initial_state;
  in.obstacle_pos = {Eigen::Vector3d(100.0, 100.0, 100.0)};
  ASSERT_NO_THROW(out = qp.solve(in));
  ASSERT_GT(out.cbf.h_min, config.cbf_activation_distance);
  ASSERT_NO_THROW(without_cbf.solve(in));
  EXPECT_TRUE(qp.solution().isApprox(without_cbf.solution(), 1e-8));
  EXPECT_GE((without_cbf.inequalities().transpose() * qp.solution() +
    without_cbf.stateOffsets(initial_state)).minCoeff(), -1e-7);
}

// With one interval the CBF is the one-step constraint of cbf_python, written
// at the current state, with the barrier at the worst-case distance d - delta_H:
// L_g h (J u + dJ dq) + L_f h + gamma h >= 0.
TEST_F(ThorQpIntegrationTest, SingleIntervalCbfUsesCurrentStateAndObstacleAcceleration)
{
  const auto axes = static_cast<unsigned int>(model_.nv);
  ThorConfig config = baseConfig(1, 0.002, 0.002, false);
  config.C = 0.1;
  config.delta_H = 0.05;
  config.qmax = constant(2.0 * kPi);
  config.qmin = constant(-2.0 * kPi);
  config.Dqmax = constant(3.0);
  config.DDqmax = constant(50.0);
  config.lambda_acc = 1e-6;
  config.lambda_scaling = 1.0;
  config.lambda_clik = 1e+2;
  config.cbf = true;
  config.cbf_formulation = CbfFormulation::continuous;  // rows checked below
  config.cbf_frame_ids = {static_cast<unsigned int>(model_.frames.size() - 1)};
  config.n_obstacle_points = 1;

  InspectableOptimizer qp;
  qp.setPinocchioModel(model_);
  ASSERT_NO_THROW(qp.configure(config));

  Eigen::VectorXd state(2 * axes);
  state << 0.0, -kPi / 2.0, kPi / 2.0, 0.0, 0.0, 0.0,
           0.3, -0.2, 0.4, 0.1, 0.0, -0.1;
  qp.setInitialState(state);

  ThorInput in;
  in.target_dq = Eigen::VectorXd::Zero(axes);
  in.next_target_q = state.head(axes);
  in.x0 = state;
  in.obstacle_pos = {Eigen::Vector3d(0.6, 0.4, 0.6)};
  in.obstacle_vel = {Eigen::Vector3d(-0.5, 0.0, 0.1)};
  in.obstacle_acc = {Eigen::Vector3d(-1.0, 0.5, 0.0)};

  ThorOutput out;
  ASSERT_NO_THROW(out = qp.solve(in));

  const unsigned int frame = config.cbf_frame_ids[0];
  const Eigen::VectorXd q = state.head(axes), dq = state.tail(axes);
  pinocchio::Data data(model_);
  pinocchio::computeJointJacobiansTimeVariation(model_, data, q, dq);
  pinocchio::updateFramePlacements(model_, data);
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian =
    Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, model_.nv);
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_dot = jacobian;
  pinocchio::getFrameJacobian(model_, data, frame, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
  pinocchio::getFrameJacobianTimeVariation(model_, data, frame, pinocchio::LOCAL_WORLD_ALIGNED,
                                           jacobian_dot);
  const Eigen::Vector3d robot_vel = jacobian.topRows<3>() * dq;
  const cbf::BarrierTerms barrier = cbf::barrierTerms(
    data.oMf[frame].translation(), robot_vel, in.obstacle_pos[0], in.obstacle_vel[0],
    in.obstacle_acc[0], config.T_r, config.a_s, config.C, config.delta_H);

  EXPECT_NEAR(out.cbf.h_min, barrier.h, 1e-12);
  EXPECT_NEAR(out.cbf.v_robot, barrier.v_r, 1e-12);
  EXPECT_NEAR(out.cbf.v_obstacle, barrier.v_o, 1e-12);

  const Eigen::VectorXd column = qp.inequalities().col(qp.inequalities().cols() - 1);
  EXPECT_TRUE(column.head(axes).isApprox(jacobian.topRows<3>().transpose() * barrier.lie_g, 1e-9));
  EXPECT_TRUE(column.tail(1).isZero());
  const double offset = barrier.lie_g.dot(jacobian_dot.topRows<3>() * dq) + barrier.lie_f +
                        config.gamma * barrier.h;
  EXPECT_GE(column.dot(qp.solution()) + offset, -1e-7);

  // Obstacle accelerations, when given, must match the obstacle positions.
  in.obstacle_acc.push_back(Eigen::Vector3d::Zero());
  EXPECT_THROW(qp.solve(in), std::invalid_argument);
}

// Robot and obstacle spheres: every (frame, obstacle) row uses the sum of the
// two radii, inflated by the obstacle's own delta_H (worst-case distance).
TEST_F(ThorQpIntegrationTest, CbfUsesSphereRadiiAndPerObstacleDeltaH)
{
  const auto axes = static_cast<unsigned int>(model_.nv);
  ThorConfig config = baseConfig(1, 0.002, 0.002, false);
  config.C = 0.05;
  config.delta_H = 0.5;  // overridden by ThorInput::obstacle_delta_H
  config.qmax = constant(2.0 * kPi);
  config.qmin = constant(-2.0 * kPi);
  config.Dqmax = constant(3.0);
  config.DDqmax = constant(50.0);
  config.lambda_acc = 1e-6;
  config.lambda_scaling = 1.0;
  config.lambda_clik = 1e+2;
  config.cbf = true;
  config.cbf_formulation = CbfFormulation::continuous;  // rows checked below
  config.cbf_frame_ids = {static_cast<unsigned int>(model_.getFrameId("wrist_1_link")),
                          static_cast<unsigned int>(model_.getFrameId("tool0"))};
  config.cbf_frame_radii = {0.08, 0.04};
  config.n_obstacle_points = 2;

  InspectableOptimizer qp;
  qp.setPinocchioModel(model_);
  ASSERT_NO_THROW(qp.configure(config));

  Eigen::VectorXd state(2 * axes);
  state << 0.0, -kPi / 2.0, kPi / 2.0, -kPi / 2.0, -kPi / 2.0, 0.0,
           0.4, 0.0, 0.0, 0.0, 0.0, 0.0;
  qp.setInitialState(state);

  ThorInput in;
  in.target_dq = Eigen::VectorXd::Zero(axes);
  in.next_target_q = state.head(axes);
  in.x0 = state;
  in.obstacle_pos = {Eigen::Vector3d(0.4, 0.6, 0.7), Eigen::Vector3d(0.9, 0.3, 0.9)};
  in.obstacle_vel = {Eigen::Vector3d(0.0, -0.3, 0.0), Eigen::Vector3d::Zero()};
  in.obstacle_radius = {0.10, 0.25};
  in.obstacle_delta_H = {0.02, 0.06};

  ThorOutput out;
  ASSERT_NO_THROW(out = qp.solve(in));

  const Eigen::VectorXd dq = state.tail(axes);
  pinocchio::Data data(model_);
  pinocchio::computeJointJacobiansTimeVariation(model_, data, state.head(axes), dq);
  pinocchio::updateFramePlacements(model_, data);

  double h_min = std::numeric_limits<double>::infinity();
  const Eigen::Index n_cols = qp.inequalities().cols();
  for (std::size_t j = 0; j < config.cbf_frame_ids.size(); ++j) {
    const unsigned int frame = config.cbf_frame_ids[j];
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian =
      Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, model_.nv);
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_dot = jacobian;
    pinocchio::getFrameJacobian(model_, data, frame, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
    pinocchio::getFrameJacobianTimeVariation(model_, data, frame, pinocchio::LOCAL_WORLD_ALIGNED,
                                             jacobian_dot);
    for (std::size_t k = 0; k < in.obstacle_pos.size(); ++k) {
      SCOPED_TRACE(::testing::Message() << "frame " << j << " obstacle " << k);
      const cbf::BarrierTerms barrier = cbf::barrierTerms(
        data.oMf[frame].translation(), jacobian.topRows<3>() * dq, in.obstacle_pos[k],
        in.obstacle_vel[k], Eigen::Vector3d::Zero(), config.T_r, config.a_s, config.C,
        config.cbf_frame_radii[j] + in.obstacle_radius[k] + in.obstacle_delta_H[k]);
      h_min = std::min(h_min, barrier.h);

      const Eigen::Index column = n_cols - static_cast<Eigen::Index>(in.obstacle_pos.size() * (j + 1) - k);
      const Eigen::VectorXd row = qp.inequalities().col(column);
      EXPECT_TRUE(row.head(axes).isApprox(jacobian.topRows<3>().transpose() * barrier.lie_g, 1e-9));
      const double offset = barrier.lie_g.dot(jacobian_dot.topRows<3>() * dq) + barrier.lie_f +
                            config.gamma * barrier.h;
      EXPECT_GE(row.dot(qp.solution()) + offset, -1e-7);
    }
  }
  EXPECT_NEAR(out.cbf.h_min, h_min, 1e-12);

  // Radii and delta_H must match the obstacle points and must not be negative.
  ThorInput wrong = in;
  wrong.obstacle_radius = {0.1};
  EXPECT_THROW(qp.solve(wrong), std::invalid_argument);
  wrong = in;
  wrong.obstacle_radius = {0.1, -0.01};
  EXPECT_THROW(qp.solve(wrong), std::invalid_argument);
  wrong = in;
  wrong.obstacle_delta_H = {0.1, -0.01};
  EXPECT_THROW(qp.solve(wrong), std::invalid_argument);

  // Robot sphere radii: one per frame, not negative.
  ThorConfig bad = config;
  bad.cbf_frame_radii = {0.1};
  EXPECT_THROW(qp.configure(bad), std::invalid_argument);
  bad.cbf_frame_radii = {0.1, -0.1};
  EXPECT_THROW(qp.configure(bad), std::invalid_argument);
}

// Regression: a robot at rest just inside the robust band of a static obstacle
// must be able to move away. With the first-order robust term
// gamma (h - h_chi delta_H) the CBF row had no control authority there
// (L_g h = 0 for a robot at rest with d_min >= C) and the QP was infeasible
// forever; at the worst-case distance the escape term of the barrier is active.
TEST_F(ThorQpIntegrationTest, RobotAtRestInsideTheRobustBandCanMoveAway)
{
  const auto axes = static_cast<unsigned int>(model_.nv);
  ThorConfig config = baseConfig(1, 0.002, 0.002, false);
  config.C = 0.05;
  config.delta_H = 0.15;
  config.qmax = constant(2.0 * kPi);
  config.qmin = constant(-2.0 * kPi);
  config.Dqmax = constant(3.0);
  config.DDqmax = constant(50.0);
  config.lambda_acc = 1e-6;
  config.lambda_scaling = 1.0;
  config.lambda_clik = 1e+2;
  config.cbf = true;
  const auto frame = static_cast<unsigned int>(model_.getFrameId("tool0"));
  config.cbf_frame_ids = {frame};
  config.n_obstacle_points = 1;

  // Both formulations: the discrete one has authority through the position too.
  for (const CbfFormulation formulation : {CbfFormulation::discrete, CbfFormulation::continuous}) {
    SCOPED_TRACE(formulation == CbfFormulation::discrete ? "discrete" : "continuous");
    InspectableOptimizer qp;
    qp.setPinocchioModel(model_);
    config.cbf_formulation = formulation;
    ASSERT_NO_THROW(qp.configure(config));

    Eigen::VectorXd state = Eigen::VectorXd::Zero(2 * axes);
    state.head(axes) << 0.0, -kPi / 2.0, kPi / 2.0, -kPi / 2.0, -kPi / 2.0, 0.0;
    qp.setInitialState(state);

    pinocchio::Data data(model_);
    pinocchio::framesForwardKinematics(model_, data, state.head(axes));
    const Eigen::Vector3d tool = data.oMf[frame].translation();
    const Eigen::Vector3d direction = Eigen::Vector3d::UnitX();

    ThorInput in;
    in.target_dq = Eigen::VectorXd::Zero(axes);
    in.next_target_q = state.head(axes);
    in.x0 = state;
    in.obstacle_pos = {tool + direction * (config.C + config.delta_H - 0.002)};
    in.obstacle_vel = {Eigen::Vector3d::Zero()};

    ThorOutput out;
    ASSERT_NO_THROW(out = qp.solve(in));
    EXPECT_NEAR(out.cbf.h_now, -0.002, 1e-9);
    EXPECT_NEAR(out.cbf.distance, config.C + config.delta_H - 0.002, 1e-9);

    // The commanded acceleration moves the tool away from the obstacle.
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian =
      Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, model_.nv);
    pinocchio::computeJointJacobians(model_, data, state.head(axes));
    pinocchio::getFrameJacobian(model_, data, frame, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
    EXPECT_LT(direction.dot(jacobian.topRows<3>() * out.next_acc), 0.0);
  }
}

// The discrete-time CBF needs d(v_frame)/dq of the frame origin velocity in
// world coordinates; the LOCAL_WORLD_ALIGNED derivatives of Pinocchio are not
// that quantity, so frameVelocityDerivatives() builds it from the LOCAL ones.
TEST_F(ThorQpIntegrationTest, FrameVelocityDerivativesMatchFiniteDifferences)
{
  const auto axes = static_cast<Eigen::Index>(model_.nv);
  Eigen::VectorXd q(axes), dq(axes);
  q << 0.3, -1.2, 1.4, -0.9, -1.5, 0.2;
  dq << 0.4, -0.3, 0.5, 0.2, -0.6, 0.1;
  const auto frame = model_.getFrameId("wrist_1_link");

  pinocchio::Data data(model_);
  auto velocity = [&](const Eigen::VectorXd& qq) {
    pinocchio::forwardKinematics(model_, data, qq, dq);
    pinocchio::updateFramePlacements(model_, data);
    return Eigen::Vector3d(pinocchio::getFrameVelocity(model_, data, frame, pinocchio::LOCAL_WORLD_ALIGNED).linear());
  };
  Eigen::Matrix<double, 3, Eigen::Dynamic> numeric(3, axes);
  constexpr double eps = 1e-6;
  for (Eigen::Index i = 0; i < axes; ++i) {
    Eigen::VectorXd e = Eigen::VectorXd::Zero(axes);
    e(i) = eps;
    numeric.col(i) = (velocity(q + e) - velocity(q - e)) / (2.0 * eps);
  }

  Eigen::Matrix<double, 3, Eigen::Dynamic> v_dq, v_dv;
  pinocchio::computeForwardKinematicsDerivatives(model_, data, q, dq, Eigen::VectorXd::Zero(axes));
  pinocchio::updateFramePlacements(model_, data);
  frameVelocityDerivatives(model_, data, frame, v_dq, v_dv);
  EXPECT_TRUE(v_dq.isApprox(numeric, 1e-6)) << v_dq << "\n" << numeric;

  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, axes);
  pinocchio::getFrameJacobian(model_, data, frame, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
  EXPECT_TRUE(v_dv.isApprox(jacobian.topRows<3>(), 1e-12));
}

// Discrete-time CBF: for every interval i, robot sphere and obstacle sphere the
// row is the linearization of F_i(u) = h(X_{i+1}(u)) - alpha_i h(X_i(u)) around
// the previous inputs u_ref applied from the current state. Rebuild it with
// finite differences on the nonlinear kinematics.
TEST_F(ThorQpIntegrationTest, DiscreteCbfRowsLinearizeTheBarrierAlongTheHorizon)
{
  const auto axes = static_cast<unsigned int>(model_.nv);
  constexpr unsigned int intervals = 3;
  ThorConfig config = baseConfig(intervals, 0.06, 0.01, false);
  config.C = 0.05;
  config.gamma = 3.0;
  config.qmax = constant(2.0 * kPi);
  config.qmin = constant(-2.0 * kPi);
  config.Dqmax = constant(3.0);
  config.DDqmax = constant(20.0);
  config.lambda_acc = 1e-4;
  config.lambda_scaling = 1.0;
  config.lambda_clik = 1e+2;
  config.cbf = true;
  config.cbf_formulation = CbfFormulation::discrete;
  config.cbf_frame_ids = {static_cast<unsigned int>(model_.getFrameId("wrist_1_link")),
                          static_cast<unsigned int>(model_.getFrameId("tool0"))};
  config.cbf_frame_radii = {0.06, 0.05};
  config.n_obstacle_points = 2;

  InspectableOptimizer qp;
  qp.setPinocchioModel(model_);
  ASSERT_NO_THROW(qp.configure(config));

  Eigen::VectorXd state(2 * axes);
  state << 0.0, -kPi / 2.0, kPi / 2.0, -kPi / 2.0, -kPi / 2.0, 0.0,
           0.5, 0.1, -0.2, 0.0, 0.0, 0.0;
  qp.setInitialState(state);

  ThorInput in;
  in.target_dq = Eigen::VectorXd::Zero(axes * intervals);
  for (unsigned int i = 0; i < intervals; ++i)
    in.target_dq(i * axes) = 0.5;
  in.next_target_q = state.head(axes);
  in.x0 = state;
  // Two obstacles approaching the arm, one of them decelerating.
  in.obstacle_pos = {Eigen::Vector3d(0.6, 0.9, 0.7), Eigen::Vector3d(1.2, 0.3, 0.6)};
  in.obstacle_vel = {Eigen::Vector3d(0.0, -0.6, 0.0), Eigen::Vector3d(-0.5, 0.0, 0.0)};
  in.obstacle_acc = {Eigen::Vector3d::Zero(), Eigen::Vector3d(0.8, 0.0, 0.0)};
  in.obstacle_radius = {0.1, 0.2};
  in.obstacle_delta_H = {0.02, 0.05};

  // A first solve gives the previous inputs u_ref used to linearize the second.
  ThorOutput out;
  ASSERT_NO_THROW(out = qp.solve(in));
  qp.updateState(out.next_acc);
  in.x0 = qp.getState();
  const Eigen::Index n_u = static_cast<Eigen::Index>(axes) * intervals;
  const Eigen::VectorXd u_ref = qp.solution().head(n_u);
  ASSERT_GT(u_ref.norm(), 1e-6);
  ASSERT_NO_THROW(out = qp.solve(in));

  const Eigen::VectorXd prediction_time = qp.getPredictionTimeInstant();
  pinocchio::Data data(model_);
  auto barrier = [&](const Eigen::VectorXd& u, unsigned int m, std::size_t j, std::size_t k) {
    const Eigen::VectorXd x = qp.node(in.x0, u, m);
    pinocchio::forwardKinematics(model_, data, x.head(axes), x.tail(axes));
    pinocchio::updateFramePlacements(model_, data);
    const unsigned int frame = config.cbf_frame_ids[j];
    const double t = m == 0 ? 0.0 : prediction_time(m - 1);
    const Eigen::Vector3d& a_o = in.obstacle_acc[k];
    return cbf::barrierTerms(
      data.oMf[frame].translation(),
      pinocchio::getFrameVelocity(model_, data, frame, pinocchio::LOCAL_WORLD_ALIGNED).linear(),
      in.obstacle_pos[k] + in.obstacle_vel[k] * t + 0.5 * a_o * t * t, in.obstacle_vel[k] + a_o * t, a_o,
      config.T_r, config.a_s, config.C,
      config.cbf_frame_radii[j] + in.obstacle_radius[k] + in.obstacle_delta_H[k]).h;
  };

  double h_now = std::numeric_limits<double>::infinity();
  const Eigen::Index n_cols = qp.inequalities().cols();
  for (std::size_t j = 0; j < 2; ++j) {
    for (std::size_t k = 0; k < 2; ++k) {
      h_now = std::min(h_now, barrier(u_ref, 0, j, k));
      for (unsigned int i = 0; i < intervals; ++i) {
        SCOPED_TRACE(::testing::Message() << "frame " << j << " obstacle " << k << " interval " << i);
        const double alpha = std::max(0.0, 1.0 - config.gamma * qp.controlIntervals()(i));
        auto F = [&](const Eigen::VectorXd& u) { return barrier(u, i + 1, j, k) - alpha * barrier(u, i, j, k); };

        Eigen::VectorXd gradient(n_u);
        constexpr double eps = 1e-4;
        for (Eigen::Index n = 0; n < n_u; ++n) {
          Eigen::VectorXd e = Eigen::VectorXd::Zero(n_u);
          e(n) = eps;
          gradient(n) = (F(u_ref + e) - F(u_ref - e)) / (2.0 * eps);
        }

        const Eigen::Index col = n_cols - static_cast<Eigen::Index>(2 * (intervals * (j + 1) - i) - k);
        const Eigen::VectorXd row = qp.inequalities().col(col);
        EXPECT_TRUE(row.head(n_u).isApprox(gradient, 1e-5)) << row.head(n_u).transpose() << "\n" << gradient.transpose();
        EXPECT_TRUE(row.tail(intervals).isZero());
        EXPECT_NEAR(row.head(n_u).dot(u_ref) + qp.solvedOffsets()(col), F(u_ref), 1e-9);

        // The solution satisfies the linearized row, and on the first interval
        // (one control period ahead) the nonlinear condition too.
        const Eigen::VectorXd u = qp.solution().head(n_u);
        EXPECT_GE(row.head(n_u).dot(u) + qp.solvedOffsets()(col), -1e-7);
        if (i == 0)
          EXPECT_GE(F(u), -1e-6);
      }
    }
  }
  EXPECT_NEAR(out.cbf.h_now, h_now, 1e-12);
}

// Repeated linearization: with cbf_linearizations = 2 the second QP linearizes
// the discrete CBF around the plan of the first one (moved by the damping),
// obtained from a twin optimizer with a single linearization. Also checks the
// early stop and the validation of the settings.
TEST_F(ThorQpIntegrationTest, RepeatedLinearizationUsesThePlanOfThePreviousQp)
{
  const auto axes = static_cast<unsigned int>(model_.nv);
  constexpr unsigned int intervals = 3;
  ThorConfig config = baseConfig(intervals, 0.06, 0.01, false);
  config.C = 0.05;
  config.gamma = 3.0;
  config.qmax = constant(2.0 * kPi);
  config.qmin = constant(-2.0 * kPi);
  config.Dqmax = constant(3.0);
  config.DDqmax = constant(20.0);
  config.lambda_acc = 1e-4;
  config.lambda_scaling = 1.0;
  config.lambda_clik = 1e+2;
  config.cbf = true;
  config.cbf_frame_ids = {static_cast<unsigned int>(model_.getFrameId("wrist_1_link")),
                          static_cast<unsigned int>(model_.getFrameId("tool0"))};
  config.cbf_frame_radii = {0.06, 0.05};
  config.n_obstacle_points = 2;

  config.cbf_linearizations = 0;
  EXPECT_THROW(ThorQP(config, model_), std::invalid_argument);
  config.cbf_linearizations = 2;
  for (const double damping : {0.0, 1.5}) {
    config.cbf_linearization_damping = damping;
    EXPECT_THROW(ThorQP(config, model_), std::invalid_argument);
  }
  config.cbf_linearization_damping = 1.0;
  config.cbf_linearization_tolerance = -1.0;
  EXPECT_THROW(ThorQP(config, model_), std::invalid_argument);
  config.cbf_linearization_tolerance = 0.0;

  // Full and damped steps of the linearization point.
  for (const double damping : {1.0, 0.5}) {
    SCOPED_TRACE(::testing::Message() << "damping " << damping);
    InspectableOptimizer qp;
    qp.setPinocchioModel(model_);
    config.cbf_linearizations = 2;
    config.cbf_linearization_tolerance = 0.0;
    config.cbf_linearization_damping = damping;
    ASSERT_NO_THROW(qp.configure(config));

    Eigen::VectorXd state(2 * axes);
    state << 0.0, -kPi / 2.0, kPi / 2.0, -kPi / 2.0, -kPi / 2.0, 0.0,
             0.5, 0.1, -0.2, 0.0, 0.0, 0.0;
    qp.setInitialState(state);

    ThorInput in;
    in.target_dq = Eigen::VectorXd::Zero(axes * intervals);
    for (unsigned int i = 0; i < intervals; ++i)
      in.target_dq(i * axes) = 0.5;
    in.next_target_q = state.head(axes);
    in.x0 = state;
    in.obstacle_pos = {Eigen::Vector3d(0.6, 0.9, 0.7), Eigen::Vector3d(1.2, 0.3, 0.6)};
    in.obstacle_vel = {Eigen::Vector3d(0.0, -0.6, 0.0), Eigen::Vector3d(-0.5, 0.0, 0.0)};
    in.obstacle_acc = {Eigen::Vector3d::Zero(), Eigen::Vector3d(0.8, 0.0, 0.0)};
    in.obstacle_radius = {0.1, 0.2};
    in.obstacle_delta_H = {0.02, 0.05};

    ThorOutput out;
    ASSERT_NO_THROW(out = qp.solve(in));
    EXPECT_EQ(out.cbf_linearizations, 2U);
    qp.updateState(out.next_acc);
    in.x0 = qp.getState();

    // A twin with a single linearization, the same state and the same previous
    // solution u_prev solves only the first QP of this cycle, of solution u_1.
    // The second QP of qp is linearized around u_prev + damping (u_1 - u_prev).
    const Eigen::Index n_u = static_cast<Eigen::Index>(axes) * intervals;
    const Eigen::VectorXd u_prev = qp.solution().head(n_u);
    ThorConfig single = config;
    single.cbf_linearizations = 1;
    ThorOutput first;
    InspectableOptimizer one;
    one.setPinocchioModel(model_);
    ASSERT_NO_THROW(one.configure(single));
    one.setInitialState(in.x0);
    one.setSolution(qp.solution());
    ASSERT_NO_THROW(first = one.solve(in));
    EXPECT_EQ(first.cbf_linearizations, 1U);
    const Eigen::VectorXd u_ref = u_prev + damping * (one.solution().head(n_u) - u_prev);

    ASSERT_NO_THROW(out = qp.solve(in));
    EXPECT_EQ(out.cbf_linearizations, 2U);

    const Eigen::VectorXd prediction_time = qp.getPredictionTimeInstant();
    pinocchio::Data data(model_);
    auto barrier = [&](const Eigen::VectorXd& u, unsigned int m, std::size_t j, std::size_t k) {
      const Eigen::VectorXd x = qp.node(in.x0, u, m);
      pinocchio::forwardKinematics(model_, data, x.head(axes), x.tail(axes));
      pinocchio::updateFramePlacements(model_, data);
      const unsigned int frame = config.cbf_frame_ids[j];
      const double t = m == 0 ? 0.0 : prediction_time(m - 1);
      const Eigen::Vector3d& a_o = in.obstacle_acc[k];
      return cbf::barrierTerms(
        data.oMf[frame].translation(),
        pinocchio::getFrameVelocity(model_, data, frame, pinocchio::LOCAL_WORLD_ALIGNED).linear(),
        in.obstacle_pos[k] + in.obstacle_vel[k] * t + 0.5 * a_o * t * t, in.obstacle_vel[k] + a_o * t, a_o,
        config.T_r, config.a_s, config.C,
        config.cbf_frame_radii[j] + in.obstacle_radius[k] + in.obstacle_delta_H[k]).h;
    };

    const Eigen::Index n_cols = qp.inequalities().cols();
    for (std::size_t j = 0; j < 2; ++j) {
      for (std::size_t k = 0; k < 2; ++k) {
        for (unsigned int i = 0; i < intervals; ++i) {
          SCOPED_TRACE(::testing::Message() << "frame " << j << " obstacle " << k << " interval " << i);
          const double alpha = std::max(0.0, 1.0 - config.gamma * qp.controlIntervals()(i));
          auto F = [&](const Eigen::VectorXd& u) { return barrier(u, i + 1, j, k) - alpha * barrier(u, i, j, k); };
          Eigen::VectorXd gradient(n_u);
          constexpr double eps = 1e-4;
          for (Eigen::Index n = 0; n < n_u; ++n) {
            Eigen::VectorXd e = Eigen::VectorXd::Zero(n_u);
            e(n) = eps;
            gradient(n) = (F(u_ref + e) - F(u_ref - e)) / (2.0 * eps);
          }
          const Eigen::Index col = n_cols - static_cast<Eigen::Index>(2 * (intervals * (j + 1) - i) - k);
          const Eigen::VectorXd row = qp.inequalities().col(col);
          EXPECT_TRUE(row.head(n_u).isApprox(gradient, 1e-5));
          EXPECT_NEAR(row.head(n_u).dot(u_ref) + qp.solvedOffsets()(col), F(u_ref), 1e-9);
        }
      }
    }

    // With every obstacle beyond the activation distance there is nothing to
    // re-linearize: a single QP is solved.
    config.cbf_linearizations = 3;
    ASSERT_NO_THROW(qp.configure(config));
    qp.setInitialState(state);
    in.x0 = state;
    ThorInput far = in;
    far.obstacle_pos = {Eigen::Vector3d(100.0, 100.0, 100.0), Eigen::Vector3d(100.0, 100.0, 100.0)};
    ASSERT_NO_THROW(out = qp.solve(far));
    EXPECT_EQ(out.cbf_linearizations, 1U);

    // Early stop: a huge tolerance accepts the first QP, a zero one runs them all
    // (unless two solutions are identical).
    config.cbf_linearization_tolerance = 1e9;
    ASSERT_NO_THROW(qp.configure(config));
    qp.setInitialState(state);
    ASSERT_NO_THROW(out = qp.solve(in));
    EXPECT_EQ(out.cbf_linearizations, 1U);
    EXPECT_GT(out.cbf_linearization_step, 0.0);
    config.cbf_linearization_tolerance = 0.0;
    ASSERT_NO_THROW(qp.configure(config));
    qp.setInitialState(state);
    ASSERT_NO_THROW(out = qp.solve(in));
    EXPECT_EQ(out.cbf_linearizations, 3U);
  }
}

// The variable bounds are passed to qpmad as simple bounds. The solution must
// be the one of the original problem, with the bounds as general rows of m_CI.
TEST_F(ThorQpIntegrationTest, SimpleBoundsGiveTheSameSolutionAsGeneralRows)
{
  const auto axes = static_cast<unsigned int>(model_.nv);
  constexpr unsigned int intervals = 4;
  constexpr double ddq_max = 1.0;  // low, so that the accelerations saturate
  for (const CbfFormulation formulation : {CbfFormulation::discrete, CbfFormulation::continuous}) {
    SCOPED_TRACE(formulation == CbfFormulation::discrete ? "discrete" : "continuous");
    ThorConfig config = baseConfig(intervals, 0.1, 0.01, false);
    config.C = 0.05;
    config.qmax = constant(2.0 * kPi);
    config.qmin = constant(-2.0 * kPi);
    config.Dqmax = constant(2.0);
    config.DDqmax = constant(ddq_max);
    config.lambda_acc = 1e-4;
    config.lambda_jerk = 1e-3;
    config.lambda_scaling = 10.0;
    config.lambda_clik = 1e+2;
    config.position_bounds = true;
    config.cbf = true;
    config.cbf_formulation = formulation;
    config.cbf_frame_ids = {static_cast<unsigned int>(model_.getFrameId("wrist_1_link")),
                            static_cast<unsigned int>(model_.getFrameId("tool0"))};
    config.cbf_frame_radii = {0.06, 0.05};
    config.n_obstacle_points = 1;

    InspectableOptimizer qp;
    qp.setPinocchioModel(model_);
    ASSERT_NO_THROW(qp.configure(config));
    Eigen::VectorXd state = Eigen::VectorXd::Zero(2 * axes);
    state.head(axes) << 0.0, -kPi / 2.0, kPi / 2.0, -kPi / 2.0, -kPi / 2.0, 0.0;
    qp.setInitialState(state);

    ThorInput in;
    in.target_dq = Eigen::VectorXd::Zero(axes * intervals);
    for (unsigned int i = 0; i < intervals; ++i)
      in.target_dq(i * axes) = 1.5;  // fast enough to hit the bounds
    // A static obstacle close enough to keep the CBF rows in the QP (h below
    // the activation distance), far enough not to stop the robot.
    in.obstacle_pos = {Eigen::Vector3d(1.5, 1.5, 0.7)};
    in.obstacle_vel = {Eigen::Vector3d::Zero()};
    in.obstacle_radius = {0.15};

    // ThorQP also leaves out the inequalities that the bounds already imply and
    // factorizes the cost by blocks; the reference below has neither.
    int cycles_with_active_bounds = 0;
    Eigen::Index redundant = 0;
    for (int cycle = 0; cycle < 20; ++cycle) {
      SCOPED_TRACE(cycle);
      in.x0 = qp.getState();
      in.next_target_q = in.x0.head(axes);
      ThorOutput out;
      ASSERT_NO_THROW(out = qp.solve(in));
      ASSERT_LT(out.cbf.h_min, config.cbf_activation_distance);  // all the CBF rows are in the QP
      redundant += qp.redundantInequalities();

      Eigen::MatrixXd hessian = qp.hessian();
      Eigen::MatrixXd factor;
      ASSERT_TRUE(qp.blockInvertedFactor(factor));
      EXPECT_TRUE((factor * factor.transpose() * hessian).isIdentity(1e-9));
      Eigen::VectorXd reference;
      qpmad::Solver solver;
      const Eigen::Index rows = qp.inequalities().cols();
      ASSERT_EQ(solver.solve(reference, hessian, qp.gradient(), qp.inequalities().transpose(),
                             -qp.solvedOffsets(),
                             Eigen::VectorXd::Constant(rows, std::numeric_limits<double>::infinity())),
                qpmad::Solver::OK);
      EXPECT_TRUE(qp.solution().isApprox(reference, 1e-8))
        << (qp.solution() - reference).lpNorm<Eigen::Infinity>();
      const Eigen::VectorXd u = qp.solution().head(axes * intervals);
      if ((u.cwiseAbs().array() >= ddq_max - 1e-9).any())
        ++cycles_with_active_bounds;
      qp.updateState(out.next_acc);
    }
    EXPECT_GT(cycles_with_active_bounds, 0);  // the simple bounds were exercised
    EXPECT_GT(redundant, 0);                  // and so was the reduction of the QP
  }
}

TEST_F(ThorQpIntegrationTest, ComputesConstrainedStepWithoutCbf)
{
  constexpr unsigned int intervals = 4;
  const auto axes = static_cast<unsigned int>(model_.nv);

  ThorConfig config = baseConfig(intervals, 0.2, 0.01, false);
  config.qmax = constant(kPi);
  config.qmin = constant(-kPi);
  config.Dqmax = constant(2.0);
  config.DDqmax = constant(20.0);
  config.tau_max = constant(100.0);
  config.lambda_acc = 1e-4;
  config.lambda_jerk = 1e-4;
  config.lambda_scaling = 10.0;
  config.lambda_clik = 10.0;

  InspectableOptimizer qp;
  qp.setPinocchioModel(model_);
  ASSERT_NO_THROW(qp.configure(config));

  ThorInput in;
  in.target_dq = Eigen::VectorXd::Zero(axes * intervals);
  in.next_target_q = Eigen::VectorXd::Zero(axes);
  in.target_scaling = 0.8;
  in.x0 = Eigen::VectorXd::Zero(2 * axes);
  qp.setInitialState(in.x0);

  ThorOutput out;
  ASSERT_NO_THROW(out = qp.solve(in));

  ASSERT_EQ(out.next_acc.size(), axes);
  EXPECT_TRUE(out.next_acc.allFinite());
  EXPECT_TRUE(std::isfinite(out.next_scaling));
  EXPECT_GE(out.next_scaling, config.min_scaling - 1e-8);
  EXPECT_LE(out.next_scaling, config.max_scaling + 1e-8);
  EXPECT_EQ(qp.getFirstPredictionPos().size(), axes);
  EXPECT_EQ(qp.getFirstPredictionVel().size(), axes);
  EXPECT_TRUE(qp.getFirstPredictionPos().allFinite());
  EXPECT_TRUE(qp.getFirstPredictionVel().allFinite());
}

TEST_F(ThorQpIntegrationTest, RepeatedConstrainedSolvesPreserveSolutionAndBounds)
{
  const auto axes = static_cast<unsigned int>(model_.nv);
  ThorConfig config = baseConfig(3, 0.15, 0.01, false);
  config.qmax = constant(2.0);
  config.qmin = constant(-1.0);
  config.Dqmax = constant(0.2);
  config.DDqmax = constant(0.5);
  config.tau_max = constant(100.0);
  config.lambda_acc = 1e-3;
  config.lambda_scaling = 10.0;
  config.lambda_clik = 1.0;

  InspectableOptimizer qp;
  qp.setPinocchioModel(model_);
  ASSERT_NO_THROW(qp.configure(config));

  ThorInput in;
  in.target_dq = Eigen::VectorXd::Constant(3 * axes, 2.0);
  in.next_target_q = constant(0.5);
  in.target_scaling = 0.8;
  in.x0 = Eigen::VectorXd::Zero(2 * axes);
  in.x0.tail(axes).setConstant(0.1);
  qp.setInitialState(in.x0);
  Eigen::VectorXd reference;
  for (int iteration = 0; iteration < 4; ++iteration) {
    SCOPED_TRACE(iteration);
    ThorOutput out;
    ASSERT_NO_THROW(out = qp.solve(in));
    ASSERT_TRUE(qp.solution().allFinite());
    EXPECT_GE((qp.inequalities().transpose() * qp.solution() + qp.stateOffsets(in.x0))
                .minCoeff(), -1e-7);
    EXPECT_LE(out.next_acc.cwiseAbs().maxCoeff(), 0.5 + 1e-7);
    EXPECT_LE(qp.velocities().cwiseAbs().maxCoeff(), 0.2 + 1e-7);
    EXPECT_GE(out.next_scaling, 0.05 - 1e-7);
    EXPECT_LE(out.next_scaling, 1.01 + 1e-7);
    if (iteration == 0) reference = qp.solution();
    else EXPECT_TRUE(qp.solution().isApprox(reference, 1e-9));
  }
  // The acceleration bound cannot bring this velocity into range in time.
  in.x0.tail(axes).setConstant(10.0);
  in.target_dq.setZero();
  in.next_target_q.setZero();
  qp.setInitialState(in.x0);
  EXPECT_THROW(qp.solve(in), std::runtime_error);
}

TEST_F(ThorQpIntegrationTest, CopiesConfiguredOptimizer)
{
  const auto axes = static_cast<unsigned int>(model_.nv);
  ThorConfig config = baseConfig(3, 0.15, 0.01, true);
  config.qmax = constant(2.0);
  config.qmin = constant(-2.0);
  config.Dqmax = constant(3.0);
  config.DDqmax = constant(30.0);
  config.tau_max = constant(100.0);
  config.lambda_acc = 1e-3;
  config.lambda_scaling = 10.0;
  config.lambda_clik = 1.0;
  const ThorQP original(config, model_);

  ThorQP copy;
  ASSERT_NO_THROW(copy = original);
  EXPECT_FALSE(copy.needUpdate());
  EXPECT_DOUBLE_EQ(copy.getDt(), 0.01);
  EXPECT_EQ(copy.getNumObstaclePoints(), 0U);

  const Eigen::VectorXd state = Eigen::VectorXd::Zero(2 * axes);
  copy.setInitialState(state);
  Eigen::VectorXd acceleration;
  double scaling = 0.0;
  ASSERT_TRUE(copy.computedUncostrainedSolution(
    Eigen::VectorXd::Zero(3 * axes), Eigen::VectorXd::Zero(axes),
    1.0, state, acceleration, scaling));
  EXPECT_TRUE(acceleration.allFinite());
  EXPECT_TRUE(std::isfinite(scaling));
}

TEST_F(ThorQpIntegrationTest, CbfConfigurationRequiresMatchingModel)
{
  const auto axes = static_cast<unsigned int>(model_.nv);
  ThorConfig config;
  config.n_intervals = 3;
  config.n_joints = axes - 1;
  config.horizon_time = 0.15;
  config.sampling_period = 0.01;
  config.input_blocking = false;
  config.qmax = Eigen::VectorXd::Constant(axes - 1, kPi);
  config.qmin = Eigen::VectorXd::Constant(axes - 1, -kPi);
  config.Dqmax = Eigen::VectorXd::Constant(axes - 1, 3.0);
  config.DDqmax = Eigen::VectorXd::Constant(axes - 1, 30.0);
  config.cbf = true;
  config.a_s = 2.5;

  EXPECT_THROW(ThorQP(config, model_), std::invalid_argument);

  config.n_joints = axes;
  config.qmax = Eigen::VectorXd::Constant(axes, kPi);
  config.qmin = Eigen::VectorXd::Constant(axes, -kPi);
  config.Dqmax = Eigen::VectorXd::Constant(axes, 3.0);
  config.DDqmax = Eigen::VectorXd::Constant(axes, 30.0);
  config.cbf_frame_ids = {static_cast<unsigned int>(model_.frames.size())};
  EXPECT_THROW(ThorQP(config, model_), std::invalid_argument);

  config.cbf_frame_ids = {static_cast<unsigned int>(model_.frames.size() - 1)};
  // The barrier divides by the safety margin C.
  EXPECT_THROW(ThorQP(config, model_), std::invalid_argument);
  config.C = 0.1;
  EXPECT_NO_THROW(ThorQP(config, model_));
  config.delta_H = -0.01;
  EXPECT_THROW(ThorQP(config, model_), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Deprecated API: keeps the legacy setters and computedCostrainedSolution
// wrappers working until they are removed.
// ---------------------------------------------------------------------------
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

TEST_F(ThorQpIntegrationTest, LegacyApiMatchesConfiguredCbfSolve)
{
  constexpr unsigned int intervals = 5;
  const auto axes = static_cast<unsigned int>(model_.nv);

  ThorConfig config = baseConfig(intervals, 0.2, 0.002, true);
  config.qmax = constant(kPi);
  config.qmin = constant(-kPi);
  config.Dqmax = constant(30.0);
  config.DDqmax = constant(500.0);
  config.tau_max = constant(10.0);
  config.lambda_acc = 1.0e-6;
  config.lambda_tau = 1.0e-9;
  config.lambda_scaling = 5e+1;
  config.lambda_clik = 1e+4;
  config.position_bounds = true;
  config.cbf = true;
  config.cbf_frame_ids = cbfFrameIds();
  config.n_obstacle_points = 1;

  InspectableOptimizer legacy;
  legacy.setPinocchioModel(model_);
  legacy.setIntervals(intervals, axes, config.horizon_time, config.sampling_period);
  legacy.setCBFParameters(config.a_s, config.T_r, config.C, config.gamma);
  legacy.setConstraints(config.qmax, config.qmin, config.Dqmax, config.DDqmax,
                        config.tau_max);
  legacy.setWeigthFunction(config.lambda_acc, config.lambda_tau, 0.0,
                           config.lambda_scaling, config.lambda_clik);
  legacy.activatePositionBounds(true);
  legacy.activateCbfBounds(true);
  legacy.setCbfIds(config.cbf_frame_ids, 1);
  legacy.updateMatrices();

  InspectableOptimizer configured;
  configured.setPinocchioModel(model_);
  ASSERT_NO_THROW(configured.configure(config));
  ASSERT_EQ(configured.inequalities().cols(), legacy.inequalities().cols());

  Eigen::VectorXd state = Eigen::VectorXd::Zero(2 * axes);
  state.head(axes) << 0.0, -kPi / 2.0, kPi / 2.0, 0.0, 0.0, 0.0;
  legacy.setInitialState(state);
  configured.setInitialState(state);

  ThorInput in;
  in.target_dq = Eigen::VectorXd::Zero(axes * intervals);
  in.next_target_q = state.head(axes);
  in.x0 = state;
  in.obstacle_pos = {Eigen::Vector3d(0.8, 0.3, 0.65)};
  in.obstacle_vel = {Eigen::Vector3d::Zero()};

  Eigen::VectorXd acceleration;
  double scaling = 0.0;
  std::vector<double> diagnostics;
  ASSERT_NO_THROW(diagnostics = legacy.computedCostrainedSolution(
    in.target_dq, in.next_target_q, in.target_scaling, in.x0,
    acceleration, scaling, in.obstacle_vel, in.obstacle_pos));
  ThorOutput out;
  ASSERT_NO_THROW(out = configured.solve(in));

  EXPECT_TRUE(out.next_acc.isApprox(acceleration, 1e-12)) << out.next_acc.transpose() << "\n" << acceleration.transpose();
  EXPECT_DOUBLE_EQ(out.next_scaling, scaling);
  ASSERT_EQ(diagnostics.size(), 6U);
  EXPECT_LT(out.cbf.h_min, config.cbf_activation_distance);
  EXPECT_DOUBLE_EQ(out.cbf.h_min, diagnostics[0]);
  EXPECT_DOUBLE_EQ(out.cbf.interval, diagnostics[1]);
  EXPECT_DOUBLE_EQ(out.cbf.frame_id, diagnostics[2]);
  EXPECT_DOUBLE_EQ(out.cbf.distance, diagnostics[3]);
  EXPECT_DOUBLE_EQ(out.cbf.v_robot, diagnostics[4]);
  EXPECT_DOUBLE_EQ(out.cbf.v_obstacle, diagnostics[5]);
  EXPECT_EQ(out.cbf.obstacle_index, 0U);

  // Missing obstacle velocity is rejected; no obstacle data means a far-away obstacle.
  ThorInput unpaired = in;
  unpaired.obstacle_vel.clear();
  EXPECT_THROW(configured.solve(unpaired), std::invalid_argument);
  ThorInput no_obstacles = in;
  no_obstacles.obstacle_pos.clear();
  no_obstacles.obstacle_vel.clear();
  ASSERT_NO_THROW(out = configured.solve(no_obstacles));
  EXPECT_GT(out.cbf.h_min, config.cbf_activation_distance);
}

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

}  // namespace
}  // namespace thor::math
