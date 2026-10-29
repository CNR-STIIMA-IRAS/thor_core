// BSD 3-Clause License
//
// Copyright (c) 2026, Nicola Pedrocchi
// All rights reserved.
//
// See the project license for redistribution terms and warranty disclaimer.

#include "thor_controllers/thor_trajectory_controller.hpp"

#include <stdexcept>
#include <string>

#include <pluginlib/class_list_macros.hpp>

namespace thor_controllers
{

using controller_interface::CallbackReturn;

CallbackReturn ThorTrajectoryController::on_init()
{
  const auto ret = JointTrajectoryController::on_init();
  if (ret != CallbackReturn::SUCCESS) return ret;
  try
  {
    thor_param_listener_ = std::make_shared<ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid thor parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ThorTrajectoryController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = JointTrajectoryController::on_configure(previous_state);
  if (ret != CallbackReturn::SUCCESS) return ret;
  const auto logger = get_node()->get_logger();

  auto fail = [&](const std::string & what) {
    RCLCPP_ERROR(logger, "%s", what.c_str());
    return CallbackReturn::FAILURE;
  };
  if (!params_.speed_scaling.state_interface.empty() || !params_.speed_scaling.command_interface.empty())
    return fail("speed_scaling interfaces are not supported: the time scaling is set by the MPC (see thor.target_scaling)");
  // JTC starting from the last command instead of the measured state:
  // open_loop_control up to Jazzy, interpolate_from_desired_state from Kilted.
  const bool from_desired_state = [](const auto & p) {
    bool on = false;
    if constexpr (requires { p.open_loop_control; }) on = on || p.open_loop_control;
    if constexpr (requires { p.interpolate_from_desired_state; }) on = on || p.interpolate_from_desired_state;
    return on;
  }(params_);
  if (from_desired_state)
    return fail(
      "interpolate_from_desired_state (open_loop_control up to Jazzy) must be false: "
      "the initial state of the MPC is set by thor.state_feedback");
  // With velocity commands alone the base class also runs its PID adapter;
  // its output is overwritten, the MPC corrects the position itself.
  if (has_effort_command_interface_)
    return fail("only position, velocity and acceleration command interfaces are supported");
  if (num_cmd_joints_ != dof_)
    return fail("command_joints must be the same as joints");
  if (!has_position_state_interface_)
    return fail("the position state interface is required");

  thor_params_ = thor_param_listener_->get_params();
  const auto & thor_params = thor_params_;
  if (thor_params.thor.state_feedback == "full" && !has_velocity_state_interface_)
    return fail("thor.state_feedback 'full' needs the velocity state interface");
  if (get_update_rate() == 0)
    return fail("the update rate of the controller is unknown");

  try
  {
    ros_core_.configure(
      get_node(), thor_params, get_robot_description(), params_.joints, 1.0 / get_update_rate());
  }
  catch (const std::exception & e)
  {
    return fail(std::string("Cannot configure the MPC: ") + e.what());
  }

  const auto n = static_cast<Eigen::Index>(dof_);
  const auto nc = static_cast<Eigen::Index>(thor_params.thor.horizon.n_intervals);
  input_.measured_q = Eigen::VectorXd::Zero(n);
  input_.measured_dq = Eigen::VectorXd::Zero(n);
  input_.target_dq = Eigen::VectorXd::Zero(n * nc);
  input_.next_target_q = Eigen::VectorXd::Zero(n);
  input_.target_scaling = thor_params.thor.target_scaling;
  reference_q_.assign(dof_, 0.0);
  RCLCPP_INFO(
    logger, "THOR MPC: %zu joints, %ld intervals over %.3f s, period %.4f s, barrier %s", dof_, nc,
    thor_params.thor.horizon.time, 1.0 / get_update_rate(), thor_params.thor.cbf.enabled ? "on" : "off");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ThorTrajectoryController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = JointTrajectoryController::on_activate(previous_state);
  if (ret != CallbackReturn::SUCCESS) return ret;

  readMeasuredState();
  if (has_velocity_state_interface_)
    ros_core_.core().reset(input_.measured_q, input_.measured_dq);
  else
    ros_core_.core().reset(input_.measured_q, Eigen::VectorXd());
  preview_msg_.reset();
  last_output_ = nullptr;
  applied_scaling_ = 1.0;
  scaling_factor_ = 1.0;
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ThorTrajectoryController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Goals, tolerances and the trajectory time, advanced with the scaling
  // that the MPC applied in the last period.
  const auto ret = JointTrajectoryController::update(time, period);
  if (ret != controller_interface::return_type::OK) return ret;

  if (thor_param_listener_->is_old(thor_params_))
  {
    thor_params_ = thor_param_listener_->get_params();
    input_.target_scaling = thor_params_.thor.target_scaling;
  }

  readMeasuredState();
  samplePreview(applied_scaling_);
  ros_core_.updateObstacles(time);
  const CoreOutput & out = ros_core_.core().step(input_);
  last_output_ = &out;
  writeCommands(out);

  // The base class advances the trajectory time by period * scaling_factor_.
  applied_scaling_ = out.scaling;
  scaling_factor_ = out.scaling;

  ros_core_.publishState(time, out, reference_q_);
  return controller_interface::return_type::OK;
}

void ThorTrajectoryController::readMeasuredState()
{
  for (std::size_t i = 0; i < dof_; ++i)
  {
    const auto idx = static_cast<Eigen::Index>(i);
    if (const auto q = joint_state_interface_[0][i].get().get_optional()) input_.measured_q(idx) = *q;
    if (has_velocity_state_interface_)
    {
      if (const auto dq = joint_state_interface_[1][i].get().get_optional())
        input_.measured_dq(idx) = *dq;
    }
  }
}

void ThorTrajectoryController::samplePreview(double scaling)
{
  const auto n = static_cast<Eigen::Index>(dof_);
  const Eigen::VectorXd & times = ros_core_.core().predictionTimes();
  const auto msg = current_trajectory_ ? current_trajectory_->get_trajectory_msg() : nullptr;
  if (!msg || msg->points.empty() || !current_trajectory_->is_sampled_already())
  {
    // Nothing to follow yet: stay where the controller is.
    input_.next_target_q = last_output_ ? last_output_->q : input_.measured_q;
    input_.target_dq.setZero();
    for (std::size_t i = 0; i < dof_; ++i) reference_q_[i] = input_.next_target_q(static_cast<Eigen::Index>(i));
    return;
  }

  // A new trajectory (or a new start time) since the last copy.
  if (msg != preview_msg_ || current_trajectory_->time_from_start() != preview_start_)
  {
    preview_ = *current_trajectory_;
    preview_msg_ = msg;
    preview_start_ = current_trajectory_->time_from_start();
  }
  const rclcpp::Time end = preview_start_ + rclcpp::Duration(msg->points.back().time_from_start);

  // Past the end the trajectory stays at its last point. It is not sampled
  // there, because that would move the search index of the copy to the end.
  auto sample_at = [&](const rclcpp::Time & t) {
    if (t >= end)
    {
      sample_.positions = msg->points.back().positions;
      sample_.velocities.assign(dof_, 0.0);
      return;
    }
    joint_trajectory_controller::TrajectoryPointConstIter start, stop;
    preview_.sample(t, interpolation_method_, sample_, start, stop, false);
  };

  sample_at(traj_time_);
  if (sample_.positions.size() == dof_) reference_q_ = sample_.positions;

  for (Eigen::Index i = 0; i < times.size(); ++i)
  {
    sample_at(traj_time_ + rclcpp::Duration::from_seconds(scaling * times(i)));
    for (Eigen::Index j = 0; j < n; ++j)
    {
      const auto k = static_cast<std::size_t>(j);
      input_.target_dq(i * n + j) = sample_.velocities.size() == dof_ ? sample_.velocities[k] : 0.0;
      if (i == 0 && sample_.positions.size() == dof_) input_.next_target_q(j) = sample_.positions[k];
    }
  }
}

void ThorTrajectoryController::writeCommands(const CoreOutput & out)
{
  for (std::size_t c = 0; c < num_cmd_joints_; ++c)
  {
    const auto j = static_cast<Eigen::Index>(map_cmd_to_joints_[c]);
    bool ok = true;
    if (has_position_command_interface_) ok &= joint_command_interface_[0][c].get().set_value(out.q(j));
    if (has_velocity_command_interface_) ok &= joint_command_interface_[1][c].get().set_value(out.dq(j));
    if (has_acceleration_command_interface_)
      ok &= joint_command_interface_[2][c].get().set_value(out.ddq(j));
    if (!ok)
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000, "Cannot write the command of joint '%s'",
        command_joint_names_[c].c_str());
  }

  // What the base class stores as the last command: used when a new
  // trajectory starts from the desired state.
  auto copy = [](const Eigen::VectorXd & v, std::vector<double> & dst) {
    if (dst.size() == static_cast<std::size_t>(v.size())) Eigen::VectorXd::Map(dst.data(), v.size()) = v;
  };
  copy(out.q, last_commanded_state_.positions);
  copy(out.dq, last_commanded_state_.velocities);
  copy(out.ddq, last_commanded_state_.accelerations);
}

}  // namespace thor_controllers

PLUGINLIB_EXPORT_CLASS(
  thor_controllers::ThorTrajectoryController, controller_interface::ControllerInterface)
