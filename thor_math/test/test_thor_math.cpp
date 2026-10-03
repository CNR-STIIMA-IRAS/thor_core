#include <gtest/gtest.h>

#include <thor_math/thor_math.h>

#include <pinocchio/parsers/urdf.hpp>

#include <cmath>
#include <filesystem>
#include <numbers>
#include <string>
#include <vector>

namespace thor::math {
namespace {

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

  ThorQP qp;
  qp.setPinocchioModel(model_);
  qp.setIntervals(intervals, axes, horizon, sampling_period);
  qp.setCBFParameters(2.5, 0.15, 0.5, 3.0);
  qp.setConstraints(Eigen::VectorXd::Constant(axes, std::numbers::pi),
                    Eigen::VectorXd::Constant(axes, -std::numbers::pi),
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
  initial_position << 0.0, -std::numbers::pi / 2.0,
    std::numbers::pi / 2.0, 0.0, 0.0, 0.0;
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

  ASSERT_NO_THROW(qp.updateState(next_acceleration));
  EXPECT_TRUE(qp.getState().allFinite());
}

TEST_F(ThorQpIntegrationTest, ComputesConstrainedStepWithoutCbf)
{
  constexpr unsigned int intervals = 4;
  constexpr double horizon = 0.2;
  constexpr double sampling_period = 0.01;
  const auto axes = static_cast<unsigned int>(model_.nv);

  ThorQP qp;
  qp.setPinocchioModel(model_);
  qp.setIntervals(intervals, axes, horizon, sampling_period, false);
  qp.setConstraints(Eigen::VectorXd::Constant(axes, std::numbers::pi),
                    Eigen::VectorXd::Constant(axes, -std::numbers::pi),
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
