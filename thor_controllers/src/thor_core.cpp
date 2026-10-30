// BSD 3-Clause License
//
// Copyright (c) 2026, Nicola Pedrocchi
// All rights reserved.
//
// See the project license for redistribution terms and warranty disclaimer.

#include "thor_controllers/thor_core.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace thor_controllers
{

StateFeedback parseStateFeedback(const std::string & name)
{
  if (name == "open_loop") return StateFeedback::open_loop;
  if (name == "position") return StateFeedback::position;
  if (name == "full") return StateFeedback::full;
  throw std::invalid_argument(
    "unknown state feedback '" + name + "': expected open_loop, position or full");
}

namespace
{

// Far, still obstacle used to fill the unused slots of the MPC.
constexpr double kFarAway = 100.0;

// Distance between the surfaces of a robot sphere and an obstacle.
double surfaceDistance(const Eigen::Vector3d & center, double radius, const Obstacle & o)
{
  Eigen::Vector3d closest = o.position;
  const double axis_sq = o.half_axis.squaredNorm();
  if (axis_sq > 0.0)
  {
    const double s = std::clamp((center - o.position).dot(o.half_axis) / axis_sq, -1.0, 1.0);
    closest += s * o.half_axis;
  }
  return (center - closest).norm() - radius - o.radius;
}

}  // namespace

struct ThorCore::Impl
{
  CoreSettings settings;
  thor::math::ThorConfig config;  // in model order
  pinocchio::Model model;
  pinocchio::Data data;
  thor::math::ThorQP qp;
  std::string root_frame;
  bool configured = false;

  // to_model[i]: index in the model configuration of the i-th joint name.
  std::vector<Eigen::Index> to_model;
  std::vector<pinocchio::FrameIndex> sphere_frames;
  Eigen::VectorXd prediction_times;

  // State in model order.
  Eigen::VectorXd q;
  Eigen::VectorXd dq;

  std::vector<Obstacle> obstacles;
  std::vector<double> obstacle_distance;
  std::vector<std::size_t> obstacle_order;

  thor::math::ThorInput in;
  Eigen::VectorXd acc;
  CoreOutput out;

  // Copies a vector in joint-name order into model order, and back.
  void toModel(const Eigen::VectorXd & src, Eigen::VectorXd & dst) const
  {
    for (std::size_t i = 0; i < to_model.size(); ++i) dst(to_model[i]) = src(static_cast<Eigen::Index>(i));
  }
  void fromModel(const Eigen::VectorXd & src, Eigen::VectorXd & dst) const
  {
    for (std::size_t i = 0; i < to_model.size(); ++i) dst(static_cast<Eigen::Index>(i)) = src(to_model[i]);
  }

  void selectObstacles();
};

ThorCore::ThorCore() : impl_(std::make_unique<Impl>()) {}
ThorCore::~ThorCore() = default;

void ThorCore::configure(const std::string & urdf, const CoreSettings & settings)
{
  auto impl = std::make_unique<Impl>();
  impl->settings = settings;
  const auto & names = settings.joint_names;
  if (names.empty()) throw std::invalid_argument("no joints");

  pinocchio::Model full;
  try
  {
    pinocchio::urdf::buildModelFromXML(urdf, full);
  }
  catch (const std::exception & e)
  {
    throw std::invalid_argument(std::string("cannot parse the robot description: ") + e.what());
  }

  // Lock every joint that is not controlled.
  std::vector<pinocchio::JointIndex> to_lock;
  for (pinocchio::JointIndex j = 1; j < full.joints.size(); ++j)
  {
    if (std::find(names.begin(), names.end(), full.names[j]) == names.end()) to_lock.push_back(j);
  }
  const Eigen::VectorXd q_lock = pinocchio::neutral(full);
  impl->model = to_lock.empty() ? full : pinocchio::buildReducedModel(full, to_lock, q_lock);
  auto & model = impl->model;

  for (const auto & name : names)
  {
    if (!model.existJointName(name))
      throw std::invalid_argument("joint '" + name + "' is not in the robot description");
    const auto j = model.getJointId(name);
    if (model.joints[j].nq() != 1 || model.joints[j].nv() != 1)
      throw std::invalid_argument(
        "joint '" + name + "' must be revolute or prismatic with limits (continuous joints are not supported)");
    impl->to_model.push_back(model.joints[j].idx_q());
  }
  const auto n = static_cast<Eigen::Index>(names.size());
  if (model.nq != n || model.nv != n)
    throw std::invalid_argument("joint names must be unique");

  // Robot spheres: a frame for each, at its offset.
  for (const auto & sphere : settings.spheres)
  {
    if (!model.existFrame(sphere.frame))
      throw std::invalid_argument(
        "frame '" + sphere.frame + "' of robot sphere '" + sphere.name + "' is not in the robot description");
    if (!(sphere.radius >= 0.0))
      throw std::invalid_argument("robot sphere '" + sphere.name + "' has a negative radius");
    const auto parent = model.getFrameId(sphere.frame);
    const auto & parent_frame = model.frames[parent];
    pinocchio::SE3 offset = pinocchio::SE3::Identity();
    offset.translation() = sphere.offset;
    impl->sphere_frames.push_back(model.addFrame(pinocchio::Frame(
      "thor_sphere_" + sphere.name, parent_frame.parentJoint, parent,
      parent_frame.placement * offset, pinocchio::OP_FRAME)));
  }
  for (const auto & frame : model.frames)
  {
    if (frame.parentJoint == 0 && frame.type == pinocchio::BODY)
    {
      impl->root_frame = frame.name;
      break;
    }
  }

  // MPC settings in model order.
  auto & config = impl->config;
  config = settings.mpc;
  config.n_joints = static_cast<unsigned int>(n);
  auto permuted = [&](const Eigen::VectorXd & v, const char * what) {
    if (v.size() != n)
    {
      std::ostringstream msg;
      msg << what << " has " << v.size() << " values, expected one per joint (" << n << ")";
      throw std::invalid_argument(msg.str());
    }
    Eigen::VectorXd out(n);
    impl->toModel(v, out);
    return out;
  };
#if PINOCCHIO_VERSION_AT_LEAST(4, 0, 0)
  const Eigen::VectorXd urdf_velocity_limit = model.upperVelocityLimit;
#else
  const Eigen::VectorXd urdf_velocity_limit = model.velocityLimit;
#endif
  config.Dqmax = settings.mpc.Dqmax.size() == 0 ? urdf_velocity_limit
                                                : permuted(settings.mpc.Dqmax, "the velocity limit");
  config.DDqmax = permuted(settings.mpc.DDqmax, "the acceleration limit");
  config.qmin = model.lowerPositionLimit;
  config.qmax = model.upperPositionLimit;
  if (!(config.Dqmax.array() > 0.0).all())
    throw std::invalid_argument("velocity limits must be positive (check the URDF or thor.limits.velocity)");
  config.n_obstacle_points = config.cbf ? static_cast<int>(settings.max_obstacles) : 0;
  config.cbf_frame_ids.clear();
  config.cbf_frame_radii.clear();
  if (config.cbf)
  {
    if (settings.spheres.empty())
      throw std::invalid_argument("the barrier is enabled but there are no robot spheres");
    for (std::size_t k = 0; k < settings.spheres.size(); ++k)
    {
      config.cbf_frame_ids.push_back(static_cast<unsigned int>(impl->sphere_frames[k]));
      config.cbf_frame_radii.push_back(settings.spheres[k].radius);
    }
  }

  impl->data = pinocchio::Data(model);
  impl->qp.setPinocchioModel(model);
  impl->qp.configure(config);  // throws std::invalid_argument
  impl->prediction_times = impl->qp.getPredictionTimeInstant();

  // Buffers of the control loop.
  impl->q = Eigen::VectorXd::Zero(n);
  impl->dq = Eigen::VectorXd::Zero(n);
  impl->acc = Eigen::VectorXd::Zero(n);
  impl->in.target_dq = Eigen::VectorXd::Zero(n * config.n_intervals);
  impl->in.next_target_q = Eigen::VectorXd::Zero(n);
  impl->in.x0 = Eigen::VectorXd::Zero(2 * n);
  const std::size_t m = config.cbf ? settings.max_obstacles : 0;
  impl->in.obstacle_pos.assign(m, Eigen::Vector3d::Constant(kFarAway));
  impl->in.obstacle_vel.assign(m, Eigen::Vector3d::Zero());
  impl->in.obstacle_acc.assign(m, Eigen::Vector3d::Zero());
  impl->in.obstacle_axis.assign(m, Eigen::Vector3d::Zero());
  impl->in.obstacle_radius.assign(m, 0.0);
  impl->in.obstacle_delta_H.assign(m, config.delta_H);
  impl->in.obstacle_sigma.assign(m, config.obstacle_sigma);
  impl->obstacles.reserve(std::max<std::size_t>(m, 16));
  impl->obstacle_distance.reserve(impl->obstacles.capacity());
  impl->obstacle_order.reserve(impl->obstacles.capacity());
  impl->out.q = Eigen::VectorXd::Zero(n);
  impl->out.dq = Eigen::VectorXd::Zero(n);
  impl->out.ddq = Eigen::VectorXd::Zero(n);
  impl->out.cbf_frame.reserve(64);
  impl->configured = true;
  impl_ = std::move(impl);
}

bool ThorCore::isConfigured() const { return impl_->configured; }

void ThorCore::reset(const Eigen::VectorXd & q, const Eigen::VectorXd & dq)
{
  auto & d = *impl_;
  if (!d.configured) throw std::logic_error("ThorCore::reset() before configure()");
  if (q.size() != d.q.size()) throw std::invalid_argument("ThorCore::reset(): wrong size of q");
  d.toModel(q, d.q);
  if (dq.size() == d.dq.size())
    d.toModel(dq, d.dq);
  else
    d.dq.setZero();
  d.in.x0 << d.q, d.dq;
  d.qp.setInitialState(d.in.x0);
}

void ThorCore::setObstacles(const std::vector<Obstacle> & obstacles)
{
  impl_->obstacles.assign(obstacles.begin(), obstacles.end());
}

void ThorCore::Impl::selectObstacles()
{
  const std::size_t m = in.obstacle_pos.size();
  out.min_distance = std::numeric_limits<double>::infinity();
  if (m == 0) return;

  // Distance of each obstacle from the closest robot sphere, at the current state.
  pinocchio::framesForwardKinematics(model, data, q);
  obstacle_distance.resize(obstacles.size());
  obstacle_order.resize(obstacles.size());
  for (std::size_t k = 0; k < obstacles.size(); ++k)
  {
    double d = std::numeric_limits<double>::infinity();
    for (std::size_t s = 0; s < sphere_frames.size(); ++s)
    {
      d = std::min(d, surfaceDistance(data.oMf[sphere_frames[s]].translation(),
                                      settings.spheres[s].radius, obstacles[k]));
    }
    obstacle_distance[k] = d;
  }
  std::iota(obstacle_order.begin(), obstacle_order.end(), 0);
  const std::size_t used = std::min(m, obstacles.size());
  if (used > 0) out.min_distance = *std::min_element(obstacle_distance.begin(), obstacle_distance.end());
  std::partial_sort(obstacle_order.begin(), obstacle_order.begin() + static_cast<std::ptrdiff_t>(used),
                    obstacle_order.end(), [this](std::size_t a, std::size_t b) {
                      return obstacle_distance[a] < obstacle_distance[b];
                    });

  for (std::size_t k = 0; k < m; ++k)
  {
    if (k < used)
    {
      const Obstacle & o = obstacles[obstacle_order[k]];
      in.obstacle_pos[k] = o.position;
      in.obstacle_vel[k] = o.velocity;
      in.obstacle_acc[k] = o.acceleration;
      in.obstacle_axis[k] = o.half_axis;
      in.obstacle_radius[k] = std::max(0.0, o.radius);
      in.obstacle_delta_H[k] = o.delta_h >= 0.0 ? o.delta_h : config.delta_H;
      for (Eigen::Index i = 0; i < 3; ++i)
        in.obstacle_sigma[k](i) = o.sigma(i) >= 0.0 ? o.sigma(i) : config.obstacle_sigma(i);
    }
    else
    {
      in.obstacle_pos[k] = Eigen::Vector3d::Constant(kFarAway);
      in.obstacle_vel[k].setZero();
      in.obstacle_acc[k].setZero();
      in.obstacle_axis[k].setZero();
      in.obstacle_radius[k] = 0.0;
      in.obstacle_delta_H[k] = config.delta_H;
      in.obstacle_sigma[k] = config.obstacle_sigma;
    }
  }
  out.n_obstacles = static_cast<unsigned int>(used);
}

const CoreOutput & ThorCore::step(const CoreInput & input)
{
  auto & d = *impl_;
  if (!d.configured) throw std::logic_error("ThorCore::step() before configure()");
  const auto n = static_cast<Eigen::Index>(d.to_model.size());
  const double dt = d.config.sampling_period;

  // Initial state of the MPC.
  if (d.settings.state_feedback != StateFeedback::open_loop && input.measured_q.size() == n)
    d.toModel(input.measured_q, d.q);
  if (d.settings.state_feedback == StateFeedback::full && input.measured_dq.size() == n)
    d.toModel(input.measured_dq, d.dq);
  d.in.x0.head(n) = d.q;
  d.in.x0.tail(n) = d.dq;

  // Nominal trajectory in model order.
  for (unsigned int i = 0; i < d.config.n_intervals; ++i)
  {
    for (Eigen::Index j = 0; j < n; ++j)
      d.in.target_dq(i * n + d.to_model[static_cast<std::size_t>(j)]) = input.target_dq(i * n + j);
  }
  d.toModel(input.next_target_q, d.in.next_target_q);
  d.in.target_scaling = input.target_scaling;
  d.selectObstacles();

  auto & out = d.out;
  const auto start = std::chrono::steady_clock::now();
  try
  {
    const thor::math::ThorOutput sol = d.qp.solve(d.in);
    d.acc = sol.next_acc;
    out.status = CoreOutput::Status::ok;
    out.scaling = sol.next_scaling;
    out.cbf = sol.cbf;
  }
  catch (const std::runtime_error &)
  {
    // No solution: brake within the limits and stop the trajectory time.
    d.acc = (-d.dq / dt).cwiseMax(-d.config.DDqmax).cwiseMin(d.config.DDqmax);
    out.status = CoreOutput::Status::braking;
    out.scaling = 0.0;
  }
  out.solve_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  if (d.config.cbf && out.cbf.frame_id < d.model.frames.size())
    out.cbf_frame = d.model.frames[out.cbf.frame_id].name;

  // Integrate one period.
  d.q += (d.dq + 0.5 * dt * d.acc) * dt;
  d.dq += dt * d.acc;
  d.fromModel(d.q, out.q);
  d.fromModel(d.dq, out.dq);
  d.fromModel(d.acc, out.ddq);
  return out;
}

const Eigen::VectorXd & ThorCore::predictionTimes() const { return impl_->prediction_times; }

Eigen::VectorXd ThorCore::state() const
{
  const auto n = static_cast<Eigen::Index>(impl_->to_model.size());
  Eigen::VectorXd q(n), dq(n), x(2 * n);
  impl_->fromModel(impl_->q, q);
  impl_->fromModel(impl_->dq, dq);
  x << q, dq;
  return x;
}

std::vector<Eigen::Vector3d> ThorCore::sphereCenters()
{
  auto & d = *impl_;
  pinocchio::framesForwardKinematics(d.model, d.data, d.q);
  std::vector<Eigen::Vector3d> centers;
  for (const auto f : d.sphere_frames) centers.push_back(d.data.oMf[f].translation());
  return centers;
}

const CoreSettings & ThorCore::settings() const { return impl_->settings; }
const thor::math::ThorConfig & ThorCore::mpcConfig() const { return impl_->config; }
const std::string & ThorCore::rootFrame() const { return impl_->root_frame; }

}  // namespace thor_controllers
