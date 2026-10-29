// BSD 3-Clause License
//
// Copyright (c) 2026, Nicola Pedrocchi
// All rights reserved.
//
// See the project license for redistribution terms and warranty disclaimer.

#include <gmock/gmock.h>

#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <controller_interface/test_utils.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "thor_controllers/thor_trajectory_controller.hpp"

namespace
{

using controller_interface::activate_succeeds;
using controller_interface::configure_succeeds;
using trajectory_msgs::msg::JointTrajectory;
using trajectory_msgs::msg::JointTrajectoryPoint;

const std::vector<std::string> kJoints = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                          "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
const std::vector<double> kHome = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
constexpr unsigned int kRate = 500;
constexpr double kDt = 1.0 / kRate;

class TestableThorTrajectoryController : public thor_controllers::ThorTrajectoryController
{
public:
  using JointTrajectoryController::add_new_trajectory_msg;
  using ThorTrajectoryController::last_output_;
  using ThorTrajectoryController::ros_core_;

  rclcpp::NodeOptions define_custom_node_options() const override { return node_options_; }
  void set_node_options(const rclcpp::NodeOptions & options) { node_options_ = options; }
  double scalingFactor() const { return scaling_factor_.load(); }

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

class ThorTrajectoryControllerTest : public ::testing::Test
{
public:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void TearDown() override
  {
    if (controller_) controller_->get_node()->shutdown();
    controller_.reset();
  }

  // Parameters of a working controller, with the barrier on.
  std::vector<rclcpp::Parameter> defaultParameters() const
  {
    return {
      rclcpp::Parameter("joints", kJoints),
      rclcpp::Parameter("command_interfaces", command_types_),
      rclcpp::Parameter("state_interfaces", std::vector<std::string>{"position", "velocity"}),
      rclcpp::Parameter("thor.limits.acceleration", std::vector<double>(6, 10.0)),
      rclcpp::Parameter("thor.limits.velocity", std::vector<double>(6, 2.0)),
      rclcpp::Parameter("thor.obstacles.timeout", 0.0),
      rclcpp::Parameter("thor.robot_spheres.names", std::vector<std::string>{"elbow", "wrist", "tool"}),
      rclcpp::Parameter("thor.robot_spheres.elbow.frame", "forearm_link"),
      rclcpp::Parameter("thor.robot_spheres.elbow.radius", 0.08),
      rclcpp::Parameter("thor.robot_spheres.wrist.frame", "wrist_2_link"),
      rclcpp::Parameter("thor.robot_spheres.wrist.radius", 0.07),
      rclcpp::Parameter("thor.robot_spheres.tool.frame", "tool0"),
      rclcpp::Parameter("thor.robot_spheres.tool.radius", 0.05),
    };
  }

  // Creates and initializes the controller; `overrides` replace the defaults.
  controller_interface::return_type init(const std::vector<rclcpp::Parameter> & overrides = {})
  {
    auto parameters = defaultParameters();
    for (const auto & o : overrides)
    {
      auto it = std::find_if(parameters.begin(), parameters.end(),
                             [&](const rclcpp::Parameter & p) { return p.get_name() == o.get_name(); });
      if (it != parameters.end())
        *it = o;
      else
        parameters.push_back(o);
    }
    controller_ = std::make_unique<TestableThorTrajectoryController>();
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    controller_->set_node_options(options);

    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "thor_controller";
    params.robot_description = readUrdf();
    params.update_rate = kRate;
    params.node_namespace = "";
    params.node_options = controller_->define_custom_node_options();
    return controller_->init(params);
  }

  // Interfaces of a robot that executes the position command exactly.
  void assignInterfaces()
  {
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> states;
    for (std::size_t i = 0; i < kJoints.size(); ++i)
    {
      for (const auto & type : command_types_)
      {
        auto c = std::make_shared<hardware_interface::CommandInterface>(kJoints[i], type);
        std::ignore = c->set_value(type == "position" ? kHome[i] : 0.0);
        cmd_[type].push_back(c);
        commands.emplace_back(c, nullptr);
      }
      auto pos = std::make_shared<hardware_interface::StateInterface>(kJoints[i], "position");
      auto vel = std::make_shared<hardware_interface::StateInterface>(kJoints[i], "velocity");
      std::ignore = pos->set_value(kHome[i]);
      std::ignore = vel->set_value(0.0);
      pos_state_.push_back(pos);
      vel_state_.push_back(vel);
      states.emplace_back(pos, nullptr);
      states.emplace_back(vel, nullptr);
    }
    controller_->assign_interfaces(std::move(commands), std::move(states));
  }

  void setUpAndActivate(const std::vector<rclcpp::Parameter> & overrides = {})
  {
    ASSERT_EQ(init(overrides), controller_interface::return_type::OK);
    ASSERT_TRUE(configure_succeeds(controller_));
    assignInterfaces();
    ASSERT_TRUE(activate_succeeds(controller_));
    executor_.add_node(controller_->get_node()->get_node_base_interface());
  }

  // Runs the controller for `seconds`, mirroring the commands to the state
  // as a robot controlled in position (or integrating the velocity command).
  void runFor(double seconds)
  {
    const int steps = static_cast<int>(seconds * kRate);
    for (int k = 0; k < steps; ++k)
    {
      executor_.spin_some(std::chrono::microseconds(0));
      ASSERT_EQ(controller_->update(time_, rclcpp::Duration::from_seconds(kDt)),
                controller_interface::return_type::OK);
      time_ += rclcpp::Duration::from_seconds(kDt);
      for (std::size_t i = 0; i < kJoints.size(); ++i)
      {
        if (cmd_.count("position"))
        {
          std::ignore = pos_state_[i]->set_value(cmd_["position"][i]->get_optional().value());
        }
        else
        {
          const double q = pos_state_[i]->get_optional().value();
          std::ignore = pos_state_[i]->set_value(q + kDt * cmd_["velocity"][i]->get_optional().value());
        }
        if (cmd_.count("velocity"))
          std::ignore = vel_state_[i]->set_value(cmd_["velocity"][i]->get_optional().value());
      }
      if (controller_->last_output_)
        h_min_ = std::min(h_min_, controller_->last_output_->cbf.h_now);
    }
  }

  // Moves the base joint by `delta` in `duration` seconds.
  std::shared_ptr<JointTrajectory> baseJointMotion(double delta, double duration) const
  {
    auto traj = std::make_shared<JointTrajectory>();
    traj->joint_names = kJoints;
    JointTrajectoryPoint p;
    p.positions = kHome;
    p.positions[0] += delta;
    p.velocities.assign(6, 0.0);
    p.time_from_start = rclcpp::Duration::from_seconds(duration);
    traj->points.push_back(p);
    return traj;
  }

  // Publishes until the subscription is matched and the message processed.
  void publishUntilReceived(
    const rclcpp::Publisher<thor_msgs::msg::ObstacleArray>::SharedPtr & pub,
    const thor_msgs::msg::ObstacleArray & msg)
  {
    for (int k = 0; k < 200 && pub->get_subscription_count() == 0; ++k)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_GT(pub->get_subscription_count(), 0u);
    pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    runFor(0.02);
  }

  double statePosition(std::size_t i) const { return pos_state_[i]->get_optional().value(); }

protected:
  std::unique_ptr<TestableThorTrajectoryController> controller_;
  std::vector<std::string> command_types_ = {"position"};
  std::map<std::string, std::vector<hardware_interface::CommandInterface::SharedPtr>> cmd_;
  std::vector<hardware_interface::StateInterface::SharedPtr> pos_state_, vel_state_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Time time_{0, 0, RCL_ROS_TIME};
  double h_min_ = std::numeric_limits<double>::infinity();
};

TEST_F(ThorTrajectoryControllerTest, ConfiguresAndClaimsTheInterfacesOfJtc)
{
  ASSERT_EQ(init(), controller_interface::return_type::OK);
  ASSERT_TRUE(configure_succeeds(controller_));
  const auto cmd = controller_->command_interface_configuration();
  EXPECT_EQ(cmd.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_THAT(cmd.names, ::testing::Contains("shoulder_pan_joint/position"));
  EXPECT_EQ(cmd.names.size(), 6u);
  const auto state = controller_->state_interface_configuration();
  EXPECT_EQ(state.names.size(), 12u);
  EXPECT_EQ(controller_->thorCore().predictionTimes().size(), 5);
  EXPECT_NEAR(controller_->thorCore().mpcConfig().sampling_period, kDt, 1e-12);
}

TEST_F(ThorTrajectoryControllerTest, RejectsUnsupportedConfigurations)
{
  // Missing acceleration limits.
  ASSERT_EQ(init({rclcpp::Parameter("thor.limits.acceleration", std::vector<double>{})}),
            controller_interface::return_type::OK);
  EXPECT_FALSE(configure_succeeds(controller_));
  // A sphere on a frame that is not in the URDF.
  ASSERT_EQ(init({rclcpp::Parameter("thor.robot_spheres.tool.frame", "nowhere")}),
            controller_interface::return_type::OK);
  EXPECT_FALSE(configure_succeeds(controller_));
  // The open-loop mode of JTC: the MPC has its own. Renamed after Jazzy.
  constexpr const char * from_desired_state =
    requires(joint_trajectory_controller::Params p) { p.interpolate_from_desired_state; }
      ? "interpolate_from_desired_state"
      : "open_loop_control";
  ASSERT_EQ(init({rclcpp::Parameter(from_desired_state, true)}), controller_interface::return_type::OK);
  EXPECT_FALSE(configure_succeeds(controller_));
  // Effort commands.
  ASSERT_EQ(init({rclcpp::Parameter("command_interfaces", std::vector<std::string>{"effort"})}),
            controller_interface::return_type::OK);
  EXPECT_FALSE(configure_succeeds(controller_));
  // Full feedback without measured velocities.
  ASSERT_EQ(init({rclcpp::Parameter("state_interfaces", std::vector<std::string>{"position"}),
                  rclcpp::Parameter("thor.state_feedback", "full")}),
            controller_interface::return_type::OK);
  EXPECT_FALSE(configure_succeeds(controller_));
}

TEST_F(ThorTrajectoryControllerTest, HoldsPositionWithoutTrajectory)
{
  ASSERT_NO_FATAL_FAILURE(setUpAndActivate());
  runFor(0.5);
  for (std::size_t i = 0; i < kJoints.size(); ++i) EXPECT_NEAR(statePosition(i), kHome[i], 1e-6);
  EXPECT_NEAR(controller_->scalingFactor(), 1.0, 1e-3);
}

TEST_F(ThorTrajectoryControllerTest, FollowsTrajectory)
{
  ASSERT_NO_FATAL_FAILURE(setUpAndActivate());
  controller_->add_new_trajectory_msg(baseJointMotion(0.5, 1.0));
  runFor(0.5);
  // Halfway along a smooth motion, at nominal speed.
  EXPECT_GT(statePosition(0), 0.1);
  EXPECT_LT(statePosition(0), 0.4);
  EXPECT_NEAR(controller_->scalingFactor(), 1.0, 1e-2);
  runFor(1.0);
  EXPECT_NEAR(statePosition(0), 0.5, 1e-3);
  for (std::size_t i = 1; i < kJoints.size(); ++i) EXPECT_NEAR(statePosition(i), kHome[i], 1e-3);
}

TEST_F(ThorTrajectoryControllerTest, FollowsTrajectoryWithVelocityCommands)
{
  command_types_ = {"velocity"};
  ASSERT_NO_FATAL_FAILURE(setUpAndActivate({rclcpp::Parameter("thor.state_feedback", "position")}));
  controller_->add_new_trajectory_msg(baseJointMotion(0.5, 1.0));
  runFor(1.5);
  EXPECT_NEAR(statePosition(0), 0.5, 1e-3);
}

TEST_F(ThorTrajectoryControllerTest, AvoidsObstacleFromTopic)
{
  ASSERT_NO_FATAL_FAILURE(setUpAndActivate());
  // A person (capsule) where the tool would be at the end of the motion.
  auto node = std::make_shared<rclcpp::Node>("obstacle_publisher");
  auto pub = node->create_publisher<thor_msgs::msg::ObstacleArray>("/thor_controller/obstacles", 10);
  thor_msgs::msg::ObstacleArray msg;
  msg.header.frame_id = "world";
  thor_msgs::msg::Obstacle person;
  person.position.x = -0.15;
  person.position.y = 0.70;
  person.position.z = 0.65;
  person.radius = 0.15;
  person.half_axis.z = 0.5;
  msg.obstacles.push_back(person);
  publishUntilReceived(pub, msg);
  ASSERT_EQ(controller_->last_output_->n_obstacles, 1u);

  controller_->add_new_trajectory_msg(baseJointMotion(1.5, 2.0));
  runFor(3.0);
  EXPECT_GE(h_min_, -1e-3);
  const Eigen::Vector3d axis(0.0, 0.0, 0.5);
  const Eigen::Vector3d center(person.position.x, person.position.y, person.position.z);
  for (const auto & c : controller_->ros_core_.core().sphereCenters())
  {
    const double s = std::clamp((c - center).dot(axis) / axis.squaredNorm(), -1.0, 1.0);
    EXPECT_GT((c - center - s * axis).norm(), person.radius);
  }
}

TEST_F(ThorTrajectoryControllerTest, DiscardsObstaclesInOtherFrames)
{
  ASSERT_NO_FATAL_FAILURE(setUpAndActivate());
  auto node = std::make_shared<rclcpp::Node>("obstacle_publisher");
  auto pub = node->create_publisher<thor_msgs::msg::ObstacleArray>("/thor_controller/obstacles", 10);
  thor_msgs::msg::ObstacleArray msg;
  msg.header.frame_id = "camera";
  msg.obstacles.resize(1);
  publishUntilReceived(pub, msg);
  EXPECT_EQ(controller_->last_output_->n_obstacles, 0u);
}

}  // namespace
