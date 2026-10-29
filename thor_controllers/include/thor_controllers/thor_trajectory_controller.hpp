// BSD 3-Clause License
//
// Copyright (c) 2026, Nicola Pedrocchi
// All rights reserved.
//
// See the project license for redistribution terms and warranty disclaimer.

#ifndef THOR_CONTROLLERS__THOR_TRAJECTORY_CONTROLLER_HPP_
#define THOR_CONTROLLERS__THOR_TRAJECTORY_CONTROLLER_HPP_

#include <memory>
#include <vector>

#include <joint_trajectory_controller/joint_trajectory_controller.hpp>
#include <joint_trajectory_controller/trajectory.hpp>

#include "thor_controllers/thor_ros_common.hpp"

namespace thor_controllers
{

/// joint_trajectory_controller with the THOR MPC between the trajectory and
/// the robot.
///
/// The base class handles goals, tolerances, feedback and holding; in each
/// cycle this class then samples the trajectory over the MPC horizon,
/// solves the MPC (time scaling and control barrier function), overwrites
/// the commands with the integrated MPC state, and sets the time scaling
/// with which the base class advances the trajectory in the next cycle.
///
/// It relies on protected members of JointTrajectoryController (4.x, ROS 2
/// Jazzy), which may change between releases.
class ThorTrajectoryController : public joint_trajectory_controller::JointTrajectoryController
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// The MPC, for tests and diagnostics.
  const ThorCore & thorCore() { return ros_core_.core(); }

protected:
  /// Fills input_.target_dq and next_target_q from the trajectory, from the
  /// current trajectory time over the horizon.
  void samplePreview(double scaling);
  void readMeasuredState();
  void writeCommands(const CoreOutput & out);

  std::shared_ptr<ParamListener> thor_param_listener_;
  Params thor_params_;
  ThorRosCore ros_core_;

  // Private copy of the trajectory of the base class, sampled over the
  // horizon without touching the search state of the original.
  joint_trajectory_controller::Trajectory preview_;
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> preview_msg_;
  rclcpp::Time preview_start_;

  CoreInput input_;
  trajectory_msgs::msg::JointTrajectoryPoint sample_;
  std::vector<double> reference_q_;
  double applied_scaling_ = 1.0;
  const CoreOutput * last_output_ = nullptr;  ///< Result of the last cycle, if any.
};

}  // namespace thor_controllers

#endif  // THOR_CONTROLLERS__THOR_TRAJECTORY_CONTROLLER_HPP_
