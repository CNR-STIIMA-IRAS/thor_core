// BSD 3-Clause License
//
// Copyright (c) 2026, Nicola Pedrocchi
// All rights reserved.
//
// See the project license for redistribution terms and warranty disclaimer.

#include <gmock/gmock.h>

#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <controller_interface/test_utils.hpp>

#include "thor_controllers/thor_chainable_controller.hpp"

namespace
{

using controller_interface::activate_succeeds;
using controller_interface::configure_succeeds;

const std::vector<std::string> kJoints = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                          "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
const std::vector<double> kHome = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
constexpr unsigned int kRate = 500;
constexpr double kDt = 1.0 / kRate;

class TestableThorChainableController : public thor_controllers::ThorChainableController
{
public:
  using ThorChainableController::last_output_;
  using ThorChainableController::ros_core_;

  rclcpp::NodeOptions define_custom_node_options() const override { return node_options_; }
  void set_node_options(const rclcpp::NodeOptions & options) { node_options_ = options; }

private:
  rclcpp::NodeOptions node_options_;
};

std::string readUrdf()
{
  std::ifstream file(THOR_TEST_URDF);
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

// A nominal trajectory of the base joint: a smooth rest-to-rest motion of
// `delta` in `duration`, sampled at trajectory time tau.
struct Nominal
{
  double delta = 1.0;
  double duration = 2.0;
  void sample(double tau, double & q, double & dq, double & ddq) const
  {
    const double s = std::clamp(tau / duration, 0.0, 1.0);
    const double w = 2.0 * M_PI / duration;
    q = delta * (s - std::sin(2.0 * M_PI * s) / (2.0 * M_PI));
    dq = (tau > 0.0 && tau < duration) ? delta / duration * (1.0 - std::cos(w * tau)) : 0.0;
    ddq = (tau > 0.0 && tau < duration) ? delta / duration * w * std::sin(w * tau) : 0.0;
  }
};

class ThorChainableControllerTest : public ::testing::Test
{
public:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void TearDown() override
  {
    if (controller_) controller_->get_node()->shutdown();
    controller_.reset();
  }

  controller_interface::return_type init(const std::vector<rclcpp::Parameter> & overrides = {})
  {
    std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("joints", kJoints),
      rclcpp::Parameter("command_interfaces", command_types_),
      rclcpp::Parameter("thor.limits.acceleration", std::vector<double>(6, 10.0)),
      rclcpp::Parameter("thor.limits.velocity", std::vector<double>(6, 2.0)),
      rclcpp::Parameter("thor.robot_spheres.names", std::vector<std::string>{"elbow", "wrist", "tool"}),
      rclcpp::Parameter("thor.robot_spheres.elbow.frame", "forearm_link"),
      rclcpp::Parameter("thor.robot_spheres.elbow.radius", 0.08),
      rclcpp::Parameter("thor.robot_spheres.wrist.frame", "wrist_2_link"),
      rclcpp::Parameter("thor.robot_spheres.wrist.radius", 0.07),
      rclcpp::Parameter("thor.robot_spheres.tool.frame", "tool0"),
      rclcpp::Parameter("thor.robot_spheres.tool.radius", 0.05),
    };
    parameters.insert(parameters.end(), overrides.begin(), overrides.end());
    parameters.insert(parameters.end(), extra_parameters_.begin(), extra_parameters_.end());
    controller_ = std::make_unique<TestableThorChainableController>();
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    controller_->set_node_options(options);

    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "thor_chainable";
    params.robot_description = readUrdf();
    params.update_rate = kRate;
    params.node_namespace = "";
    params.node_options = controller_->define_custom_node_options();
    return controller_->init(params);
  }

  void setUpAndActivate()
  {
    ASSERT_EQ(init(), controller_interface::return_type::OK);
    ASSERT_TRUE(configure_succeeds(controller_));
    for (const auto & r : controller_->export_reference_interfaces()) refs_[r->get_name()] = r;
    const auto states = controller_->export_state_interfaces();
    ASSERT_EQ(states.size(), 1u);
    scaling_ = states[0];

    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> state_ifs;
    for (const auto & type : {"position", "velocity", "acceleration"})
    {
      if (std::find(command_types_.begin(), command_types_.end(), type) == command_types_.end()) continue;
      for (std::size_t i = 0; i < kJoints.size(); ++i)
      {
        auto c = std::make_shared<hardware_interface::CommandInterface>(kJoints[i], type);
        std::ignore = c->set_value(std::string(type) == "position" ? kHome[i] : 0.0);
        cmd_[type].push_back(c);
        commands.emplace_back(c, nullptr);
      }
    }
    for (const auto & type : {"position", "velocity"})
    {
      for (std::size_t i = 0; i < kJoints.size(); ++i)
      {
        auto s = std::make_shared<hardware_interface::StateInterface>(kJoints[i], type);
        std::ignore = s->set_value(std::string(type) == "position" ? kHome[i] : 0.0);
        state_[type].push_back(s);
        state_ifs.emplace_back(s, nullptr);
      }
    }
    controller_->assign_interfaces(std::move(commands), std::move(state_ifs));
    ASSERT_TRUE(controller_->set_chained_mode(true));
    ASSERT_TRUE(activate_succeeds(controller_));
  }

  double scaling() const { return scaling_->get_optional().value(); }
  double position(std::size_t i) const { return state_.at("position")[i]->get_optional().value(); }

  // Runs the chain: a preceding controller that advances its trajectory time
  // by the exported scaling and writes the reference one period ahead, then
  // the THOR controller, then a robot that executes the commands.
  void runFor(double seconds, bool with_reference = true)
  {
    for (int k = 0; k < static_cast<int>(seconds * kRate); ++k)
    {
      if (with_reference)
      {
        tau_ += kDt * scaling();
        double q, dq, ddq;
        nominal_.sample(tau_ + kDt, q, dq, ddq);
        for (std::size_t i = 0; i < kJoints.size(); ++i)
        {
          const std::string prefix = "thor_chainable/" + kJoints[i];
          std::ignore = refs_[prefix + "/position"]->set_value(i == 0 ? kHome[0] + q : kHome[i]);
          std::ignore = refs_[prefix + "/velocity"]->set_value(i == 0 ? dq : 0.0);
          std::ignore = refs_[prefix + "/acceleration"]->set_value(i == 0 ? ddq : 0.0);
        }
      }
      ASSERT_EQ(controller_->update(time_, rclcpp::Duration::from_seconds(kDt)),
                controller_interface::return_type::OK);
      time_ += rclcpp::Duration::from_seconds(kDt);
      for (std::size_t i = 0; i < kJoints.size(); ++i)
      {
        if (cmd_.count("position"))
          std::ignore = state_["position"][i]->set_value(cmd_["position"][i]->get_optional().value());
        else
          std::ignore = state_["position"][i]->set_value(
            position(i) + kDt * cmd_["velocity"][i]->get_optional().value());
      }
      const auto * out = controller_->last_output_;
      min_scaling_ = std::min(min_scaling_, out->scaling);
      min_distance_ = std::min(min_distance_, out->min_distance);
    }
  }

protected:
  std::unique_ptr<TestableThorChainableController> controller_;
  std::vector<std::string> command_types_ = {"position"};
  std::vector<rclcpp::Parameter> extra_parameters_;
  std::map<std::string, hardware_interface::CommandInterface::SharedPtr> refs_;
  hardware_interface::StateInterface::ConstSharedPtr scaling_;
  std::map<std::string, std::vector<hardware_interface::CommandInterface::SharedPtr>> cmd_;
  std::map<std::string, std::vector<hardware_interface::StateInterface::SharedPtr>> state_;
  rclcpp::Time time_{0, 0, RCL_ROS_TIME};
  Nominal nominal_;
  double tau_ = 0.0;
  double min_scaling_ = std::numeric_limits<double>::infinity();
  double min_distance_ = std::numeric_limits<double>::infinity();
};

TEST_F(ThorChainableControllerTest, ExportsReferencesAndSpeedScaling)
{
  ASSERT_NO_FATAL_FAILURE(setUpAndActivate());
  EXPECT_EQ(refs_.size(), 18u);
  EXPECT_TRUE(refs_.count("thor_chainable/shoulder_pan_joint/position"));
  EXPECT_TRUE(refs_.count("thor_chainable/wrist_3_joint/acceleration"));
  EXPECT_EQ(scaling_->get_name(), "thor_chainable/speed_scaling");
  EXPECT_EQ(scaling(), 1.0);
  const auto cmd = controller_->command_interface_configuration();
  EXPECT_THAT(cmd.names, ::testing::ElementsAre(
                           "shoulder_pan_joint/position", "shoulder_lift_joint/position", "elbow_joint/position",
                           "wrist_1_joint/position", "wrist_2_joint/position", "wrist_3_joint/position"));
}

TEST_F(ThorChainableControllerTest, AcceptsTheCommandInterfacesOfJtc)
{
  const std::vector<std::pair<std::vector<std::string>, bool>> cases = {
    {{"position"}, true},
    {{"velocity"}, true},
    {{"position", "velocity"}, true},
    {{"position", "acceleration"}, true},
    {{"position", "velocity", "acceleration"}, true},
    {{"acceleration"}, false},
    {{"velocity", "acceleration"}, false},
  };
  for (const auto & [types, valid] : cases)
  {
    command_types_ = types;
    ASSERT_EQ(init(), controller_interface::return_type::OK);
    EXPECT_EQ(configure_succeeds(controller_), valid) << ::testing::PrintToString(types);
  }
}

TEST_F(ThorChainableControllerTest, HoldsPositionWithoutReferences)
{
  ASSERT_NO_FATAL_FAILURE(setUpAndActivate());
  runFor(0.5, false);
  for (std::size_t i = 0; i < kJoints.size(); ++i) EXPECT_NEAR(position(i), kHome[i], 1e-6);
}

TEST_F(ThorChainableControllerTest, FollowsTheReferenceOfThePrecedingController)
{
  ASSERT_NO_FATAL_FAILURE(setUpAndActivate());
  runFor(1.0);
  double q, dq, ddq;
  nominal_.sample(tau_, q, dq, ddq);
  EXPECT_NEAR(position(0), kHome[0] + q, 2e-3);
  EXPECT_NEAR(scaling(), 1.0, 1e-2);
  runFor(1.5);
  EXPECT_NEAR(position(0), kHome[0] + nominal_.delta, 1e-3);
}

TEST_F(ThorChainableControllerTest, ThePrecedingControllerFollowsTheScaling)
{
  extra_parameters_ = {rclcpp::Parameter("thor.target_scaling", 0.5)};
  ASSERT_NO_FATAL_FAILURE(setUpAndActivate());
  runFor(1.0);
  EXPECT_NEAR(scaling(), 0.5, 2e-2);
  EXPECT_NEAR(tau_, 0.5, 0.05);  // half of the elapsed time
  double q, dq, ddq;
  nominal_.sample(tau_, q, dq, ddq);
  EXPECT_NEAR(position(0), kHome[0] + q, 2e-3);
}

TEST_F(ThorChainableControllerTest, KeepsDistanceFromAnObstacle)
{
  ASSERT_NO_FATAL_FAILURE(setUpAndActivate());
  // A person on the path of the tool.
  thor_controllers::Obstacle person;
  const double angle = std::atan2(0.164, 0.69) + 0.6;
  person.position = Eigen::Vector3d(0.71 * std::cos(angle), 0.71 * std::sin(angle), 0.6);
  person.half_axis = Eigen::Vector3d(0.0, 0.0, 0.5);
  person.radius = 0.15;
  controller_->ros_core_.core().setObstacles({person});
  double max_deviation = 0.0;
  for (int k = 0; k < 150; ++k)
  {
    runFor(0.02);
    for (std::size_t i = 1; i < kJoints.size(); ++i)
      max_deviation = std::max(max_deviation, std::abs(position(i) - kHome[i]));
  }
  EXPECT_GT(min_distance_, 0.05 - 1e-3);
  EXPECT_TRUE(min_scaling_ < 0.9 || max_deviation > 0.05)
    << "min scaling " << min_scaling_ << ", max deviation " << max_deviation;
}

}  // namespace
