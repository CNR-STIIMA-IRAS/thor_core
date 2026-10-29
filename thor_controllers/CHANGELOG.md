# Changelog

All notable changes to `thor_controllers` are documented here.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the
project uses [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added

- `ThorCore`: the THOR MPC without ROS. It reduces the URDF model to the
  controlled joints, places the robot spheres on frames with offsets, chooses
  the closest obstacles, brakes when the QP has no solution and integrates the
  accelerations into position and velocity commands. The initial state is set
  by `thor.state_feedback` (`open_loop`, `position` or `full`).
- `ThorTrajectoryController`: a `JointTrajectoryController` that solves the
  MPC over the horizon of the trajectory, overwrites the commands and sets the
  time scaling of the trajectory.
- `ThorChainableController`: the MPC as a chainable controller after any
  trajectory controller, with position, velocity and acceleration reference
  interfaces and a `speed_scaling` state interface for
  `joint_trajectory_controller`.
- Parameters under `thor.*`, generated with `generate_parameter_library`, and
  the `ThorControllerState` message on `thor.state_topic`, with the actual
  distance from the closest obstacle.

### Changed

- The same sources build on Jazzy and on Rolling (Kilted and later):
  - `ThorTrajectoryController` rejects the JTC mode that starts from the last
    command instead of the measured state under both of its names,
    `open_loop_control` (up to Jazzy) and `interpolate_from_desired_state`
    (from Kilted);
  - the obstacles reach the control loop through a
    `realtime_tools::RealtimeThreadSafeBox`, available on both, instead of
    `RealtimeBuffer`, deprecated in Rolling. The control loop does not wait
    for the lock: while the subscription writes, it keeps the last obstacles.
- Build: no CMP0167 warnings with CMake 3.30 or later (Boost for Pinocchio is
  found through `BoostConfig.cmake`).
