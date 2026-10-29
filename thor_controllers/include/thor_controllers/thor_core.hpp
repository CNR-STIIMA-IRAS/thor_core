// BSD 3-Clause License
//
// Copyright (c) 2026, Nicola Pedrocchi
// All rights reserved.
//
// See the project license for redistribution terms and warranty disclaimer.

#ifndef THOR_CONTROLLERS__THOR_CORE_HPP_
#define THOR_CONTROLLERS__THOR_CORE_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <thor_math/thor_math.h>

namespace thor_controllers
{

/// Initial state of the MPC at each cycle.
enum class StateFeedback
{
  open_loop,  ///< The state integrated by the controller: robots commanded in position.
  position,   ///< Measured position, integrated velocity: robots commanded in velocity.
  full,       ///< Measured position and velocity.
};

/// Parses "open_loop", "position" or "full"; throws std::invalid_argument otherwise.
StateFeedback parseStateFeedback(const std::string & name);

/// A sphere covering part of the robot, attached to a frame of the model.
struct RobotSphere
{
  std::string name;
  std::string frame;
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();  ///< Center in the frame [m].
  double radius = 0.0;                               ///< [m]
};

/// An obstacle in the root frame of the robot model: a sphere, or a capsule
/// if half_axis is not zero.
struct Obstacle
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d half_axis = Eigen::Vector3d::Zero();
  double radius = 0.0;
  double delta_h = -1.0;  ///< Negative: ThorConfig::delta_H.
};

/// Settings of ThorCore. The joint-related fields of #mpc (n_joints, limits)
/// and the CBF frames are filled by ThorCore::configure() from the URDF.
struct CoreSettings
{
  std::vector<std::string> joint_names;
  /// MPC settings. sampling_period must be the controller period.
  /// Dqmax and DDqmax are in the order of #joint_names; an empty Dqmax means
  /// the velocity limits of the URDF.
  thor::math::ThorConfig mpc;
  std::vector<RobotSphere> spheres;
  StateFeedback state_feedback = StateFeedback::open_loop;
  /// Obstacles used by the MPC in each cycle (ThorConfig::n_obstacle_points).
  unsigned int max_obstacles = 4;
};

/// Inputs of one cycle, in the order of CoreSettings::joint_names.
struct CoreInput
{
  /// Measured joint positions; used unless the feedback is open_loop.
  Eigen::VectorXd measured_q;
  /// Measured joint velocities; used with full feedback.
  Eigen::VectorXd measured_dq;
  /// Unscaled velocities of the nominal trajectory at the prediction times
  /// (ThorCore::predictionTimes()), stacked: size n_joints * n_intervals.
  Eigen::VectorXd target_dq;
  /// Position of the nominal trajectory at the first prediction time.
  Eigen::VectorXd next_target_q;
  double target_scaling = 1.0;
};

/// Result of one cycle, in the order of CoreSettings::joint_names: the state
/// at the end of the period, to be sent to the robot.
struct CoreOutput
{
  enum class Status
  {
    ok,
    braking,  ///< No solution: braking within the acceleration limits.
  };

  Status status = Status::ok;
  Eigen::VectorXd q;
  Eigen::VectorXd dq;
  Eigen::VectorXd ddq;
  /// Time scaling of the period: advance the trajectory time by scaling * dt.
  double scaling = 1.0;
  double solve_time = 0.0;  ///< [s]
  unsigned int n_obstacles = 0;
  /// Distance between the surfaces of the robot spheres and of the closest
  /// obstacle at the current state [m]; infinite without obstacles.
  double min_distance = std::numeric_limits<double>::infinity();
  /// The pair with the lowest barrier over the horizon: its distance may be
  /// on a predicted state of the nominal trajectory, not on the current one.
  thor::math::CbfDiagnostics cbf;
  std::string cbf_frame;  ///< Name of the robot frame of #cbf (empty without CBF).
};

/// The MPC of a THOR controller, without ROS: robot model, robot spheres,
/// choice of the closest obstacles, fallback braking and integration of the
/// accelerations into position and velocity commands.
///
/// configure() allocates; reset() and step() are meant for the control loop.
class ThorCore
{
public:
  ThorCore();
  ~ThorCore();
  ThorCore(const ThorCore &) = delete;
  ThorCore & operator=(const ThorCore &) = delete;

  /// Builds the model from a URDF, keeping only #CoreSettings::joint_names
  /// (the other joints are locked at zero), adds a frame for each robot
  /// sphere and configures the MPC. Throws std::invalid_argument with a
  /// readable message on any error.
  void configure(const std::string & urdf, const CoreSettings & settings);

  bool isConfigured() const;

  /// Sets the integrated state, e.g. to the measured one on activation.
  void reset(const Eigen::VectorXd & q, const Eigen::VectorXd & dq);

  /// Replaces the obstacles. More than max_obstacles are allowed: in each
  /// cycle the closest to the robot are used. Not real-time safe if the
  /// number of obstacles grows beyond the capacity of a previous call.
  void setObstacles(const std::vector<Obstacle> & obstacles);

  /// Solves one cycle and integrates the state for one period.
  const CoreOutput & step(const CoreInput & input);

  /// End of each control interval, relative to now [s]: the times at which
  /// CoreInput::target_dq is sampled (scaled by target_scaling).
  const Eigen::VectorXd & predictionTimes() const;

  /// Integrated state [q; dq] in the order of the joint names.
  Eigen::VectorXd state() const;

  /// Robot sphere centers in the root frame, at the integrated state.
  std::vector<Eigen::Vector3d> sphereCenters();

  const CoreSettings & settings() const;
  /// Configuration given to ThorQP. Its vectors are in the order of the
  /// model, which may differ from the order of the joint names.
  const thor::math::ThorConfig & mpcConfig() const;
  /// Name of the root frame of the model, in which obstacles are expressed.
  const std::string & rootFrame() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace thor_controllers

#endif  // THOR_CONTROLLERS__THOR_CORE_HPP_
