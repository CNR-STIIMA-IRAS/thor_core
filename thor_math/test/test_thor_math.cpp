#include <gtest/gtest.h>

#include <thor_math/thor_math.h>

#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
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
  constexpr double horizon = 0.2;
  constexpr double sampling_period = 0.002;
  const auto axes = static_cast<unsigned int>(model_.nv);

  InspectableOptimizer qp;
  qp.setPinocchioModel(model_);
  qp.setIntervals(intervals, axes, horizon, sampling_period);
  qp.setCBFParameters(2.5, 0.15, 0.5, 3.0);
  qp.setConstraints(Eigen::VectorXd::Constant(axes, kPi),
                    Eigen::VectorXd::Constant(axes, -kPi),
                    Eigen::VectorXd::Constant(axes, 30.0),
                    Eigen::VectorXd::Constant(axes, 500.0),
                    Eigen::VectorXd::Constant(axes, 10.0));
  qp.setWeigthFunction(1.0e-6, 1.0e-9, 0.0, 5e+1, 1e+4);
  qp.activatePositionBounds(true);
  qp.activateTorqueBounds(false);
  qp.activateCbfBounds(true);

  std::vector<unsigned int> frame_ids;
  for (std::size_t index = 0; index < model_.frames.size(); ++index) {
    const pinocchio::Frame &frame = model_.frames[index];
    if (frame.type == pinocchio::JOINT ||
        frame.name.find("intermediate") != std::string::npos) {
      frame_ids.push_back(static_cast<unsigned int>(index));
    }
  }
  ASSERT_FALSE(frame_ids.empty());
  qp.setCbfIds(frame_ids, 1);

  if (qp.needUpdate()) {
    ASSERT_NO_THROW(qp.updateMatrices());
  }

  Eigen::VectorXd initial_position(axes);
  initial_position << 0.0, -kPi / 2.0,
    kPi / 2.0, 0.0, 0.0, 0.0;
  Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(2 * axes);
  initial_state.head(axes) = initial_position;
  qp.setInitialState(initial_state);

  const Eigen::VectorXd target_velocity =
    Eigen::VectorXd::Zero(axes * intervals);
  const Eigen::VectorXd target_position = initial_position;
  const std::vector<Eigen::Vector3d> human_velocity(
    1, Eigen::Vector3d::Zero());
  const std::vector<Eigen::Vector3d> human_position(
    1, Eigen::Vector3d(0.8, 0.3, 0.65));

  Eigen::VectorXd next_acceleration;
  double scaling = 1.0;
  std::vector<double> diagnostics;

  ASSERT_NO_THROW(
    diagnostics = qp.computedCostrainedSolution(
      target_velocity, target_position, 1.0, qp.getState(),
      next_acceleration, scaling, human_velocity, human_position));

  ASSERT_EQ(next_acceleration.size(), axes);
  EXPECT_TRUE(next_acceleration.allFinite());
  EXPECT_TRUE(std::isfinite(scaling));
  EXPECT_FALSE(diagnostics.empty());

  // The CBF is linearized at the initial stationary prediction. Independently
  // reconstruct its offset gamma * max(0, distance - safety_distance).
  pinocchio::Data data(model_);
  pinocchio::forwardKinematics(model_, data, initial_position);
  pinocchio::updateFramePlacements(model_, data);
  const auto cbf_count = intervals * frame_ids.size();
  const Eigen::Index first_cbf = qp.inequalities().cols() - cbf_count;
  ASSERT_LT(diagnostics[0], 5.0);  // full-constraint solver branch
  for (std::size_t frame = 0; frame < frame_ids.size(); ++frame) {
    const double distance =
      (data.oMf[frame_ids[frame]].translation() - human_position[0]).norm();
    const double barrier = 3.0 * std::max(0.0, distance - 0.5);
    const double offset = barrier == 0.0 ? 1e-6 : barrier;
    for (unsigned int interval = 0; interval < intervals; ++interval) {
      const Eigen::Index column = qp.inequalities().cols() -
        intervals * (frame + 1) + interval;
      EXPECT_GE(qp.inequalities().col(column).dot(qp.solution()) + offset,
                -1e-7) << "frame " << frame << " interval " << interval;
    }
  }
  EXPECT_GT(qp.inequalities().rightCols(cbf_count).norm(), 1e-8);
  const Eigen::VectorXd offsets = qp.stateOffsets(initial_state).head(first_cbf);
  EXPECT_GE((qp.inequalities().leftCols(first_cbf).transpose() * qp.solution()
    + offsets).minCoeff(), -1e-7);
  EXPECT_LE(qp.positions().cwiseAbs().maxCoeff(), kPi + 1e-7);
  EXPECT_LE(qp.velocities().cwiseAbs().maxCoeff(), 30.0 + 1e-7);

  ASSERT_NO_THROW(qp.updateState(next_acceleration));
  EXPECT_TRUE(qp.getState().allFinite());

  // Far-away humans select the reduced-constraint branch. Its result must
  // match the same configured optimizer with CBF disabled, including offsets
  // from a nonzero initial velocity.
  initial_state.tail(axes).setConstant(0.01);
  qp.setInitialState(initial_state);
  InspectableOptimizer without_cbf;
  without_cbf = qp;
  without_cbf.activateCbfBounds(false);
  without_cbf.updateMatrices();
  without_cbf.setInitialState(initial_state);
  const std::vector<Eigen::Vector3d> far_human(
    1, Eigen::Vector3d(100.0, 100.0, 100.0));
  ASSERT_NO_THROW(diagnostics = qp.computedCostrainedSolution(
    target_velocity, target_position, 1.0, initial_state,
    next_acceleration, scaling, human_velocity, far_human));
  ASSERT_GT(diagnostics[0], 5.0);
  Eigen::VectorXd reference_acceleration;
  double reference_scaling;
  ASSERT_NO_THROW(without_cbf.computedCostrainedSolution(
    target_velocity, target_position, 1.0, initial_state,
    reference_acceleration, reference_scaling));
  EXPECT_TRUE(qp.solution().isApprox(without_cbf.solution(), 1e-8));
  EXPECT_GE((without_cbf.inequalities().transpose() * qp.solution() +
    without_cbf.stateOffsets(initial_state)).minCoeff(), -1e-7);
}

TEST_F(ThorQpIntegrationTest, ComputesConstrainedStepWithoutCbf)
{
  constexpr unsigned int intervals = 4;
  constexpr double horizon = 0.2;
  constexpr double sampling_period = 0.01;
  const auto axes = static_cast<unsigned int>(model_.nv);

  InspectableOptimizer qp;
  qp.setPinocchioModel(model_);
  qp.setIntervals(intervals, axes, horizon, sampling_period, false);
  qp.setConstraints(Eigen::VectorXd::Constant(axes, kPi),
                    Eigen::VectorXd::Constant(axes, -kPi),
                    Eigen::VectorXd::Constant(axes, 2.0),
                    Eigen::VectorXd::Constant(axes, 20.0),
                    Eigen::VectorXd::Constant(axes, 100.0));
  qp.setWeigthFunction(1e-4, 0.0, 1e-4, 10.0, 10.0);
  qp.setCBFParameters(2.5, 0.15, 0.5, 3.0);
  qp.setCbfIds({}, 0);
  qp.activatePositionBounds(false);
  qp.activateTorqueBounds(false);
  qp.activateCbfBounds(false);
  ASSERT_NO_THROW(qp.updateMatrices());

  Eigen::VectorXd state = Eigen::VectorXd::Zero(2 * axes);
  qp.setInitialState(state);
  Eigen::VectorXd acceleration;
  double scaling = 0.0;
  std::vector<double> diagnostics;

  ASSERT_NO_THROW(
    diagnostics = qp.computedCostrainedSolution(
      Eigen::VectorXd::Zero(axes * intervals),
      Eigen::VectorXd::Zero(axes), 0.8, state,
      acceleration, scaling));

  EXPECT_EQ(diagnostics.size(), 6U);
  ASSERT_EQ(acceleration.size(), axes);
  EXPECT_TRUE(acceleration.allFinite());
  EXPECT_TRUE(std::isfinite(scaling));
  EXPECT_GE(scaling, 0.05 - 1e-8);
  EXPECT_LE(scaling, 1.01 + 1e-8);
  EXPECT_EQ(qp.getFirstPredictionPos().size(), axes);
  EXPECT_EQ(qp.getFirstPredictionVel().size(), axes);
  EXPECT_TRUE(qp.getFirstPredictionPos().allFinite());
  EXPECT_TRUE(qp.getFirstPredictionVel().allFinite());
}

TEST_F(ThorQpIntegrationTest, RepeatedConstrainedSolvesPreserveSolutionAndBounds)
{
  const auto axes = static_cast<unsigned int>(model_.nv);
  InspectableOptimizer qp;
  qp.setPinocchioModel(model_);
  qp.setIntervals(3, axes, 0.15, 0.01, false);
  qp.setConstraints(Eigen::VectorXd::Constant(axes, 2.0),
                    Eigen::VectorXd::Constant(axes, -1.0),
                    Eigen::VectorXd::Constant(axes, 0.2),
                    Eigen::VectorXd::Constant(axes, 0.5),
                    Eigen::VectorXd::Constant(axes, 100.0));
  qp.setWeigthFunction(1e-3, 0.0, 0.0, 10.0, 1.0);
  qp.setCBFParameters(2.5, 0.15, 0.5, 3.0);
  qp.setCbfIds({}, 0);
  qp.activatePositionBounds(false);
  qp.activateCbfBounds(false);
  qp.updateMatrices();
  Eigen::VectorXd state = Eigen::VectorXd::Zero(2 * axes);
  state.tail(axes).setConstant(0.1);
  qp.setInitialState(state);
  Eigen::VectorXd reference;
  for (int iteration = 0; iteration < 4; ++iteration) {
    SCOPED_TRACE(iteration);
    Eigen::VectorXd acceleration;
    double scaling;
    ASSERT_NO_THROW(qp.computedCostrainedSolution(
      Eigen::VectorXd::Constant(3 * axes, 2.0),
      Eigen::VectorXd::Constant(axes, 0.5), 0.8, state, acceleration, scaling));
    ASSERT_TRUE(qp.solution().allFinite());
    EXPECT_GE((qp.inequalities().transpose() * qp.solution() + qp.stateOffsets(state))
                .minCoeff(), -1e-7);
    EXPECT_LE(acceleration.cwiseAbs().maxCoeff(), 0.5 + 1e-7);
    EXPECT_LE(qp.velocities().cwiseAbs().maxCoeff(), 0.2 + 1e-7);
    EXPECT_GE(scaling, 0.05 - 1e-7);
    EXPECT_LE(scaling, 1.01 + 1e-7);
    if (iteration == 0) reference = qp.solution();
    else EXPECT_TRUE(qp.solution().isApprox(reference, 1e-9));
  }
  // The acceleration bound cannot bring this velocity into range in time.
  state.tail(axes).setConstant(10.0);
  qp.setInitialState(state);
  Eigen::VectorXd acceleration;
  double scaling;
  EXPECT_THROW(qp.computedCostrainedSolution(
    Eigen::VectorXd::Zero(3 * axes), Eigen::VectorXd::Zero(axes),
    0.8, state, acceleration, scaling), std::runtime_error);
}

TEST_F(ThorQpIntegrationTest, CopiesConfiguredOptimizer)
{
  const auto axes = static_cast<unsigned int>(model_.nv);
  ThorQP original;
  original.setPinocchioModel(model_);
  original.setIntervals(3, axes, 0.15, 0.01);
  original.setConstraints(Eigen::VectorXd::Constant(axes, 2.0),
                          Eigen::VectorXd::Constant(axes, -2.0),
                          Eigen::VectorXd::Constant(axes, 3.0),
                          Eigen::VectorXd::Constant(axes, 30.0),
                          Eigen::VectorXd::Constant(axes, 100.0));
  original.setWeigthFunction(1e-3, 0.0, 0.0, 10.0, 1.0);
  original.setCBFParameters(2.5, 0.15, 0.5, 3.0);
  original.setCbfIds({}, 0);
  original.updateMatrices();

  ThorQP copy;
  ASSERT_NO_THROW(copy = original);
  EXPECT_FALSE(copy.needUpdate());
  EXPECT_DOUBLE_EQ(copy.getDt(), original.getDt());
  EXPECT_DOUBLE_EQ(copy.getNumPh(), original.getNumPh());

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

}  // namespace
}  // namespace thor::math
