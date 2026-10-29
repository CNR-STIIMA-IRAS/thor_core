// BSD 3-Clause License
//
// Copyright (c) 2026, Nicola Pedrocchi
// All rights reserved.
//
// See the project license for redistribution terms and warranty disclaimer.

#ifndef THOR_CONTROLLERS__THOR_CHAINABLE_CONTROLLER_HPP_
#define THOR_CONTROLLERS__THOR_CHAINABLE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>

#include "thor_controllers/thor_chainable_parameters.hpp"
#include "thor_controllers/thor_ros_common.hpp"

namespace thor_controllers
{

/// The THOR MPC as a chainable controller, between any trajectory controller
/// and the robot.
///
/// It exports the reference interfaces `<name>/<joint>/position|velocity|
/// acceleration`, written by the preceding controller (e.g.
/// joint_trajectory_controller with command_joints `<name>/<joint>`), and
/// the state interface `<name>/speed_scaling` with the time scaling that the
/// preceding controller should apply (joint_trajectory_controller:
/// speed_scaling.state_interface). Only the current reference is known, so
/// the trajectory over the horizon is extrapolated with its velocity and
/// acceleration.
class ThorChainableController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// The MPC, for tests and diagnostics.
  const ThorCore & thorCore() { return ros_core_.core(); }

protected:
  std::vector<hardware_interface::StateInterface::SharedPtr> on_export_state_interfaces_list() override;

  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_reference_interfaces_list() override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Reads the references; false if the preceding controller wrote none.
  bool readReferences();
  void readMeasuredState();

  std::shared_ptr<thor_chainable_controller::ParamListener> param_listener_;
  thor_chainable_controller::Params params_;
  std::shared_ptr<ParamListener> thor_param_listener_;
  Params thor_params_;
  ThorRosCore ros_core_;

  bool has_position_command_ = false;
  bool has_velocity_command_ = false;
  bool has_acceleration_command_ = false;
  bool has_velocity_state_ = false;

  // Reference interfaces, per joint: position, velocity, acceleration.
  std::vector<hardware_interface::CommandInterface::SharedPtr> position_refs_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> velocity_refs_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> acceleration_refs_;
  hardware_interface::StateInterface::SharedPtr speed_scaling_;

  CoreInput input_;
  Eigen::VectorXd q_ref_, dq_ref_, ddq_ref_;
  std::vector<double> reference_q_;
  double applied_scaling_ = 1.0;
  const CoreOutput * last_output_ = nullptr;  ///< Result of the last cycle, if any.
};

}  // namespace thor_controllers

#endif  // THOR_CONTROLLERS__THOR_CHAINABLE_CONTROLLER_HPP_
