// BSD 3-Clause License
//
// Copyright (c) 2026, Nicola Pedrocchi
// All rights reserved.
//
// See the project license for redistribution terms and warranty disclaimer.

#include "thor_controllers/thor_chainable_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace thor_controllers
{

using controller_interface::CallbackReturn;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;

namespace
{

bool contains(const std::vector<std::string> & v, const std::string & s)
{
  return std::find(v.begin(), v.end(), s) != v.end();
}

hardware_interface::InterfaceDescription describe(const std::string & prefix, const std::string & name)
{
  hardware_interface::InterfaceInfo info;
  info.name = name;
  info.data_type = "double";
  info.initial_value = "";  // NaN: no reference yet
  return hardware_interface::InterfaceDescription(prefix, info);
}

double valueOr(const hardware_interface::Handle & handle, double fallback)
{
  const auto v = handle.get_optional();
  return v && std::isfinite(*v) ? *v : fallback;
}

}  // namespace

CallbackReturn ThorChainableController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<thor_chainable_controller::ParamListener>(get_node());
    thor_param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration ThorChainableController::command_interface_configuration() const
{
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const char * type : {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
                            hardware_interface::HW_IF_ACCELERATION})
  {
    if (!contains(params_.command_interfaces, type)) continue;
    for (const auto & joint : params_.joints) config.names.push_back(joint + "/" + type);
  }
  return config;
}

InterfaceConfiguration ThorChainableController::state_interface_configuration() const
{
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & joint : params_.joints) config.names.push_back(joint + "/position");
  if (has_velocity_state_)
  {
    for (const auto & joint : params_.joints) config.names.push_back(joint + "/velocity");
  }
  return config;
}

CallbackReturn ThorChainableController::on_configure(const rclcpp_lifecycle::State &)
{
  const auto logger = get_node()->get_logger();
  auto fail = [&](const std::string & what) {
    RCLCPP_ERROR(logger, "%s", what.c_str());
    return CallbackReturn::FAILURE;
  };

  params_ = param_listener_->get_params();
  thor_params_ = thor_param_listener_->get_params();
  const auto & cmd = params_.command_interfaces;
  has_position_command_ = contains(cmd, "position");
  has_velocity_command_ = contains(cmd, "velocity");
  has_acceleration_command_ = contains(cmd, "acceleration");
  has_velocity_state_ = contains(params_.state_interfaces, "velocity");

  // The combinations of joint_trajectory_controller: position, velocity,
  // position + velocity, position + acceleration, position + velocity + acceleration.
  if (!has_position_command_ && !(has_velocity_command_ && cmd.size() == 1))
    return fail("command_interfaces: velocity alone, or position with velocity and/or acceleration");
  if (!contains(params_.state_interfaces, "position"))
    return fail("the position state interface is required");
  if (thor_params_.thor.state_feedback == "full" && !has_velocity_state_)
    return fail("thor.state_feedback 'full' needs the velocity state interface");
  if (get_update_rate() == 0) return fail("the update rate of the controller is unknown");

  try
  {
    ros_core_.configure(
      get_node(), thor_params_, get_robot_description(), params_.joints, 1.0 / get_update_rate());
  }
  catch (const std::exception & e)
  {
    return fail(std::string("Cannot configure the MPC: ") + e.what());
  }

  const auto n = static_cast<Eigen::Index>(params_.joints.size());
  const auto nc = static_cast<Eigen::Index>(thor_params_.thor.horizon.n_intervals);
  input_.measured_q = Eigen::VectorXd::Zero(n);
  input_.measured_dq = Eigen::VectorXd::Zero(n);
  input_.target_dq = Eigen::VectorXd::Zero(n * nc);
  input_.next_target_q = Eigen::VectorXd::Zero(n);
  input_.target_scaling = thor_params_.thor.target_scaling;
  q_ref_ = Eigen::VectorXd::Zero(n);
  dq_ref_ = Eigen::VectorXd::Zero(n);
  ddq_ref_ = Eigen::VectorXd::Zero(n);
  reference_q_.assign(params_.joints.size(), 0.0);
  RCLCPP_INFO(
    logger, "THOR MPC (chainable): %ld joints, %ld intervals over %.3f s, period %.4f s, barrier %s", n,
    nc, thor_params_.thor.horizon.time, 1.0 / get_update_rate(),
    thor_params_.thor.cbf.enabled ? "on" : "off");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
ThorChainableController::on_export_reference_interfaces_list()
{
  const std::string prefix = get_node()->get_name();
  position_refs_.clear();
  velocity_refs_.clear();
  acceleration_refs_.clear();
  std::vector<hardware_interface::CommandInterface::SharedPtr> all;
  for (const auto & joint : params_.joints)
  {
    position_refs_.push_back(
      std::make_shared<hardware_interface::CommandInterface>(describe(prefix, joint + "/position")));
    velocity_refs_.push_back(
      std::make_shared<hardware_interface::CommandInterface>(describe(prefix, joint + "/velocity")));
    acceleration_refs_.push_back(
      std::make_shared<hardware_interface::CommandInterface>(describe(prefix, joint + "/acceleration")));
    all.push_back(position_refs_.back());
    all.push_back(velocity_refs_.back());
    all.push_back(acceleration_refs_.back());
  }
  return all;
}

std::vector<hardware_interface::StateInterface::SharedPtr>
ThorChainableController::on_export_state_interfaces_list()
{
  auto description = describe(get_node()->get_name(), "speed_scaling");
  description.interface_info.initial_value = "1.0";
  speed_scaling_ = std::make_shared<hardware_interface::StateInterface>(description);
  return {speed_scaling_};
}

CallbackReturn ThorChainableController::on_activate(const rclcpp_lifecycle::State &)
{
  readMeasuredState();
  ros_core_.core().reset(input_.measured_q, has_velocity_state_ ? input_.measured_dq : Eigen::VectorXd());
  for (auto & refs : {&position_refs_, &velocity_refs_, &acceleration_refs_})
  {
    for (auto & r : *refs) std::ignore = r->set_value(std::numeric_limits<double>::quiet_NaN());
  }
  applied_scaling_ = 1.0;
  if (speed_scaling_) std::ignore = speed_scaling_->set_value(1.0);
  last_output_ = nullptr;
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ThorChainableController::update_reference_from_subscribers(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Used only in a chain: without a preceding controller the references stay
  // unset and the robot holds its position.
  return controller_interface::return_type::OK;
}

void ThorChainableController::readMeasuredState()
{
  const std::size_t n = params_.joints.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto idx = static_cast<Eigen::Index>(i);
    if (const auto q = state_interfaces_[i].get_optional()) input_.measured_q(idx) = *q;
    if (has_velocity_state_)
    {
      if (const auto dq = state_interfaces_[n + i].get_optional()) input_.measured_dq(idx) = *dq;
    }
  }
}

bool ThorChainableController::readReferences()
{
  for (std::size_t i = 0; i < position_refs_.size(); ++i)
  {
    const auto j = static_cast<Eigen::Index>(i);
    const auto q = position_refs_[i]->get_optional();
    if (!q || !std::isfinite(*q)) return false;
    q_ref_(j) = *q;
    dq_ref_(j) = valueOr(*velocity_refs_[i], 0.0);
    ddq_ref_(j) = valueOr(*acceleration_refs_[i], 0.0);
  }
  return true;
}

controller_interface::return_type ThorChainableController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (thor_param_listener_->is_old(thor_params_))
  {
    thor_params_ = thor_param_listener_->get_params();
    input_.target_scaling = thor_params_.thor.target_scaling;
  }

  readMeasuredState();
  const auto n = static_cast<Eigen::Index>(params_.joints.size());
  const Eigen::VectorXd & times = ros_core_.core().predictionTimes();
  const double dt = ros_core_.core().mpcConfig().sampling_period;
  if (readReferences())
  {
    // The reference is the trajectory one period ahead of its current time
    // (as written by joint_trajectory_controller): extrapolate it to the
    // prediction times with its velocity and acceleration.
    for (Eigen::Index i = 0; i < times.size(); ++i)
    {
      const double ahead = applied_scaling_ * times(i) - dt;
      input_.target_dq.segment(i * n, n) = dq_ref_ + ahead * ddq_ref_;
      if (i == 0) input_.next_target_q = q_ref_ + ahead * dq_ref_ + 0.5 * ahead * ahead * ddq_ref_;
    }
  }
  else
  {
    // No reference: stop where the controller is.
    input_.next_target_q = last_output_ ? last_output_->q : input_.measured_q;
    input_.target_dq.setZero();
    q_ref_ = input_.next_target_q;
  }
  Eigen::VectorXd::Map(reference_q_.data(), n) = q_ref_;

  ros_core_.updateObstacles(time);
  const CoreOutput & out = ros_core_.core().step(input_);
  last_output_ = &out;

  bool ok = true;
  std::size_t c = 0;
  if (has_position_command_)
    for (Eigen::Index j = 0; j < n; ++j) ok &= command_interfaces_[c++].set_value(out.q(j));
  if (has_velocity_command_)
    for (Eigen::Index j = 0; j < n; ++j) ok &= command_interfaces_[c++].set_value(out.dq(j));
  if (has_acceleration_command_)
    for (Eigen::Index j = 0; j < n; ++j) ok &= command_interfaces_[c++].set_value(out.ddq(j));
  if (!ok)
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000, "Cannot write the commands");

  // The preceding controller advances its trajectory by period * scaling.
  applied_scaling_ = out.scaling;
  std::ignore = speed_scaling_->set_value(out.scaling);

  ros_core_.publishState(time, out, reference_q_);
  return controller_interface::return_type::OK;
}

}  // namespace thor_controllers

PLUGINLIB_EXPORT_CLASS(
  thor_controllers::ThorChainableController, controller_interface::ChainableControllerInterface)
