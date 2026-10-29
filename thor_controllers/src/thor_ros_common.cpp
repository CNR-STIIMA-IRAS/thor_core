// BSD 3-Clause License
//
// Copyright (c) 2026, Nicola Pedrocchi
// All rights reserved.
//
// See the project license for redistribution terms and warranty disclaimer.

#include "thor_controllers/thor_ros_common.hpp"

#include <stdexcept>

namespace thor_controllers
{

CoreSettings makeCoreSettings(
  const Params & params, const std::vector<std::string> & joint_names, double sampling_period)
{
  const auto & p = params.thor;
  CoreSettings s;
  s.joint_names = joint_names;
  s.state_feedback = parseStateFeedback(p.state_feedback);
  s.max_obstacles = static_cast<unsigned int>(p.cbf.max_obstacles);

  auto & c = s.mpc;
  c.n_intervals = static_cast<unsigned int>(p.horizon.n_intervals);
  c.horizon_time = p.horizon.time;
  c.sampling_period = sampling_period;
  c.input_blocking = p.horizon.input_blocking;
  c.lambda_acc = p.weights.acceleration;
  c.lambda_jerk = p.weights.jerk;
  c.lambda_scaling = p.weights.scaling;
  c.lambda_clik = p.weights.clik;
  c.Dqmax = Eigen::Map<const Eigen::VectorXd>(
    p.limits.velocity.data(), static_cast<Eigen::Index>(p.limits.velocity.size()));
  c.DDqmax = Eigen::Map<const Eigen::VectorXd>(
    p.limits.acceleration.data(), static_cast<Eigen::Index>(p.limits.acceleration.size()));
  if (c.DDqmax.size() == 0)
    throw std::invalid_argument("thor.limits.acceleration is required: one value per joint");
  c.position_bounds = p.limits.position_bounds;
  c.min_scaling = p.limits.min_scaling;
  c.max_scaling = p.limits.max_scaling;

  c.cbf = p.cbf.enabled;
  c.cbf_formulation = p.cbf.formulation == "continuous" ? thor::math::CbfFormulation::continuous
                                                         : thor::math::CbfFormulation::discrete;
  c.cbf_linearizations = static_cast<unsigned int>(p.cbf.linearizations);
  c.cbf_linearization_damping = p.cbf.linearization_damping;
  c.cbf_linearization_tolerance = p.cbf.linearization_tolerance;
  c.a_s = p.cbf.braking_deceleration;
  c.T_r = p.cbf.reaction_time;
  c.C = p.cbf.safety_margin;
  c.gamma = p.cbf.gamma;
  c.delta_H = p.cbf.distance_error;
  c.cbf_activation_distance = p.cbf.activation_distance;

  for (const auto & name : p.robot_spheres.names)
  {
    const auto & entry = p.robot_spheres.names_map.at(name);
    if (entry.frame.empty())
      throw std::invalid_argument("thor.robot_spheres." + name + ".frame is required");
    RobotSphere sphere;
    sphere.name = name;
    sphere.frame = entry.frame;
    sphere.offset = Eigen::Vector3d(entry.offset[0], entry.offset[1], entry.offset[2]);
    sphere.radius = entry.radius;
    s.spheres.push_back(sphere);
  }
  return s;
}

namespace
{
Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3 & v) { return {v.x, v.y, v.z}; }
Eigen::Vector3d toEigen(const geometry_msgs::msg::Point & p) { return {p.x, p.y, p.z}; }
}  // namespace

void fromMsg(const thor_msgs::msg::ObstacleArray & msg, std::vector<Obstacle> & obstacles)
{
  obstacles.resize(msg.obstacles.size());
  for (std::size_t k = 0; k < msg.obstacles.size(); ++k)
  {
    const auto & m = msg.obstacles[k];
    auto & o = obstacles[k];
    o.position = toEigen(m.position);
    o.velocity = toEigen(m.velocity);
    o.acceleration = toEigen(m.acceleration);
    o.half_axis = toEigen(m.half_axis);
    o.radius = m.radius;
    o.delta_h = m.delta_h;
  }
}

void ThorRosCore::configure(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, const Params & params,
  const std::string & urdf, const std::vector<std::string> & joint_names, double sampling_period)
{
  params_ = params;
  joint_names_ = joint_names;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  core_.configure(urdf, makeCoreSettings(params, joint_names, sampling_period));
  obstacle_frame_ =
    params.thor.obstacles.frame_id.empty() ? core_.rootFrame() : params.thor.obstacles.frame_id;

  clearObstacles();
  obstacle_sub_ = node->create_subscription<thor_msgs::msg::ObstacleArray>(
    params.thor.obstacles.topic, rclcpp::SystemDefaultsQoS(),
    [this](const thor_msgs::msg::ObstacleArray::SharedPtr msg) {
      if (!msg->header.frame_id.empty() && msg->header.frame_id != obstacle_frame_)
      {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000, "Obstacles in frame '%s' discarded: expected '%s'",
          msg->header.frame_id.c_str(), obstacle_frame_.c_str());
        return;
      }
      auto data = std::make_shared<ObstacleData>();
      fromMsg(*msg, data->obstacles);
      obstacle_buffer_.writeFromNonRT(data);
    });

  state_pub_ = node->create_publisher<thor_msgs::msg::ThorControllerState>(
    params.thor.state_topic, rclcpp::SystemDefaultsQoS());
  rt_state_pub_ = std::make_unique<StatePublisher>(state_pub_);
}

void ThorRosCore::clearObstacles()
{
  obstacle_buffer_.writeFromNonRT(nullptr);
  last_obstacles_.reset();
  still_obstacles_.clear();
  still_obstacles_.reserve(64);
  obstacles_stale_ = false;
  if (core_.isConfigured()) core_.setObstacles({});
}

void ThorRosCore::updateObstacles(const rclcpp::Time & now)
{
  const auto latest = *obstacle_buffer_.readFromRT();
  if (!latest) return;
  if (latest != last_obstacles_)
  {
    // New message: the timeout runs on the clock of the control loop.
    last_obstacles_ = latest;
    last_received_ = now;
    obstacles_stale_ = false;
    core_.setObstacles(latest->obstacles);
    return;
  }
  const double timeout = params_.thor.obstacles.timeout;
  if (obstacles_stale_ || timeout <= 0.0 || now.seconds() - last_received_.seconds() <= timeout) return;

  // Stale: keep them where they were last seen, still.
  obstacles_stale_ = true;
  still_obstacles_.assign(latest->obstacles.begin(), latest->obstacles.end());
  for (auto & o : still_obstacles_)
  {
    o.velocity.setZero();
    o.acceleration.setZero();
  }
  core_.setObstacles(still_obstacles_);
}

void ThorRosCore::publishState(
  const rclcpp::Time & now, const CoreOutput & out, const std::vector<double> & reference_q)
{
  if (!rt_state_pub_ || !rt_state_pub_->can_publish()) return;
  auto & m = state_msg_;
  m.header.stamp = now;
  m.status = out.status == CoreOutput::Status::ok ? thor_msgs::msg::ThorControllerState::STATUS_OK
                                                   : thor_msgs::msg::ThorControllerState::STATUS_BRAKING;
  m.scaling = out.scaling;
  m.solve_time = out.solve_time;
  m.n_obstacles = out.n_obstacles;
  m.min_distance = out.min_distance;
  m.obstacles_stale = obstacles_stale_;
  m.h_now = out.cbf.h_now;
  m.h_min = out.cbf.h_min;
  m.distance = out.cbf.distance;
  m.v_robot = out.cbf.v_robot;
  m.v_obstacle = out.cbf.v_obstacle;
  m.robot_frame = out.cbf_frame;
  m.obstacle_index = out.cbf.obstacle_index;
  m.joint_names = joint_names_;
  m.command_positions.assign(out.q.data(), out.q.data() + out.q.size());
  m.command_velocities.assign(out.dq.data(), out.dq.data() + out.dq.size());
  m.command_accelerations.assign(out.ddq.data(), out.ddq.data() + out.ddq.size());
  m.reference_positions = reference_q;
  rt_state_pub_->try_publish(m);
}

}  // namespace thor_controllers
