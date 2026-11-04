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
  /// Standard deviations of the position [m], velocity [m/s] and
  /// acceleration [m/s^2]; a negative component means that component of
  /// ThorConfig::obstacle_sigma.
  Eigen::Vector3d sigma = Eigen::Vector3d::Constant(-1.0);
};

/// Protective stop on the measured distance (ISO 10218): below #distance the
/// robot brakes at the acceleration limits, with all the joints stopping
/// together along the current direction of motion, and then holds its
/// position (category 2 stop: the motors stay powered). The MPC is not used
/// while the stop is active, whatever the robot was doing, because escaping
/// is not guaranteed to be fast enough.
///
/// The distance is the worst case over the perception errors, for every
/// obstacle and robot sphere at the current state:
/// `surface distance - delta_H - sigma_bound * sigma_position`.
///
/// The stop ends automatically when the robot is still and the distance has
/// stayed at least #distance + #resume_margin for #resume_delay, as allowed by
/// ISO/TS 15066 for speed and separation monitoring; the trajectory resumes
/// from where it stopped. It needs robot spheres.
struct ProtectiveStop
{
  bool enabled = true;
  double distance = 0.0;       ///< [m] (>= 0); 0 means ThorConfig::C.
  double resume_margin = 0.05; ///< Hysteresis before resuming [m] (>= 0).
  double resume_delay = 0.5;   ///< Time with the distance restored before resuming [s] (>= 0).
};

/// Settings of ThorCore. The joint-related fields of #mpc (n_joints, limits)
/// and the CBF frames are filled by ThorCore::configure() from the URDF.
/// #mpc tube_bound, if given, is in the order of #joint_names; with it,
/// ThorCore also runs the recovery of the tube (see CoreOutput::Status).
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
  ProtectiveStop protective_stop;
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
  /// Time scaling with which the targets were sampled: usually the scaling
  /// applied in the last cycle (CoreOutput::scaling). See
  /// ThorInput::sampling_scaling; it matters only with the tube.
  double sampling_scaling = 1.0;
};

/// Result of one cycle, in the order of CoreSettings::joint_names: the state
/// at the end of the period, to be sent to the robot.
struct CoreOutput
{
  enum class Status
  {
    ok,
    /// No solution: braking at the acceleration limits, all the joints
    /// stopping together. If the robot stops outside the tube, a recovery
    /// follows.
    braking,
    /// Protective stop (ProtectiveStop): braking, then holding still.
    protective_stop,
    /// Back into the tube after a stop (Eqs. 25-27 of the paper): the
    /// trajectory time is stopped and the MPC moves the robot towards the
    /// reference, with a temporary tube that shrinks monotonically to
    /// ThorConfig::tube_bound.
    recovery,
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
  /// #min_distance reduced by the perception errors, as used by the
  /// protective stop (ProtectiveStop) [m].
  double guard_distance = std::numeric_limits<double>::infinity();
  /// Largest |q - q_ref| over the joints, with q_ref the reference now [rad
  /// or m]; 0 without the tube.
  double tube_error = 0.0;
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

  /// Sets the integrated state and clears stop/recovery history on activation.
  void reset(const Eigen::VectorXd & q, const Eigen::VectorXd & dq);

  /// Replaces the obstacles. More than max_obstacles are allowed: in each
  /// cycle the closest to the robot are used. Invalid geometry or non-finite
  /// data forces braking until replaced by valid data, even with CBF/stop disabled.
  /// Not real-time safe if the
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
