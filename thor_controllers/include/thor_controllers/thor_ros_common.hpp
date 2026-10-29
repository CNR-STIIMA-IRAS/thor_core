// BSD 3-Clause License
//
// Copyright (c) 2026, Nicola Pedrocchi
// All rights reserved.
//
// See the project license for redistribution terms and warranty disclaimer.

#ifndef THOR_CONTROLLERS__THOR_ROS_COMMON_HPP_
#define THOR_CONTROLLERS__THOR_ROS_COMMON_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include "thor_controllers/thor_core.hpp"
#include "thor_controllers/thor_parameters.hpp"
#include "thor_msgs/msg/obstacle_array.hpp"
#include "thor_msgs/msg/thor_controller_state.hpp"

namespace thor_controllers
{

/// Settings of ThorCore from the "thor." parameters. The sampling period is
/// the controller period.
CoreSettings makeCoreSettings(
  const Params & params, const std::vector<std::string> & joint_names, double sampling_period);

/// Converts the obstacles of a message.
void fromMsg(const thor_msgs::msg::ObstacleArray & msg, std::vector<Obstacle> & obstacles);

/// The ROS side of a THOR controller shared by its variants: the core, the
/// obstacle subscription and the state publisher.
class ThorRosCore
{
public:
  /// Configures the core and creates the subscription and the publisher.
  /// Throws std::invalid_argument on errors.
  void configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const Params & params,
    const std::string & urdf, const std::vector<std::string> & joint_names, double sampling_period);

  /// Forgets the received obstacles.
  void clearObstacles();

  /// Passes the latest obstacles to the core: called in the control loop.
  /// When no new message arrives within the timeout (measured on the clock
  /// of the control loop), the last obstacles are kept but considered still.
  void updateObstacles(const rclcpp::Time & now);

  bool obstaclesStale() const { return obstacles_stale_; }

  /// Publishes the state of a cycle (if the publisher is free).
  void publishState(
    const rclcpp::Time & now, const CoreOutput & out, const std::vector<double> & reference_q);

  ThorCore & core() { return core_; }
  const Params & params() const { return params_; }

private:
  struct ObstacleData
  {
    std::vector<Obstacle> obstacles;
  };

  ThorCore core_;
  Params params_;
  std::vector<std::string> joint_names_;
  std::string obstacle_frame_;
  rclcpp::Logger logger_ = rclcpp::get_logger("thor_controllers");
  rclcpp::Clock::SharedPtr clock_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<ObstacleData>> obstacle_buffer_;
  std::shared_ptr<ObstacleData> last_obstacles_;
  rclcpp::Time last_received_;
  std::vector<Obstacle> still_obstacles_;
  bool obstacles_stale_ = false;
  rclcpp::Subscription<thor_msgs::msg::ObstacleArray>::SharedPtr obstacle_sub_;

  using StatePublisher = realtime_tools::RealtimePublisher<thor_msgs::msg::ThorControllerState>;
  rclcpp::Publisher<thor_msgs::msg::ThorControllerState>::SharedPtr state_pub_;
  std::unique_ptr<StatePublisher> rt_state_pub_;
  thor_msgs::msg::ThorControllerState state_msg_;
};

}  // namespace thor_controllers

#endif  // THOR_CONTROLLERS__THOR_ROS_COMMON_HPP_
