# thor_controllers

ros2_control controllers (ROS 2 Jazzy) that put the THOR MPC of
[thor_math](../thor_math) between a joint trajectory and the robot. In each
cycle the MPC:

- scales the speed of the trajectory: the trajectory time advances by
  `scaling * period`, with `scaling` in `[thor.limits.min_scaling, thor.limits.max_scaling]`;
- keeps a safe distance from moving obstacles (spheres and capsules) with a
  control barrier function, slowing down or moving the joints around them;
- respects the position, velocity and acceleration limits.

The MPC returns joint accelerations. The controller integrates them and
writes position, velocity and/or acceleration commands.

## Controllers

| Plugin | Use |
| --- | --- |
| `thor_controllers/ThorTrajectoryController` | Replaces `joint_trajectory_controller`. It **derives** from it: goals, tolerances, feedback and holding are those of JTC, and the MPC sees the whole trajectory over its horizon. |
| `thor_controllers/ThorChainableController` | Goes **after** any trajectory controller, in a chain. It sees only the current reference (position, velocity, acceleration) and extrapolates it over the horizon; it exports the time scaling for the preceding controller. |

### ThorTrajectoryController

In each cycle `update()`:

1. calls `JointTrajectoryController::update()`, which handles goals and
   advances the trajectory time by `period * scaling` of the previous cycle;
2. samples a private copy of the trajectory from the current trajectory time
   over the horizon (the copy keeps the search state of the original intact);
3. solves the MPC and overwrites the commands written by JTC;
4. sets the scaling with which JTC advances the trajectory in the next cycle.

It relies on protected members of `JointTrajectoryController` (4.x, Jazzy):
`current_trajectory_`, `traj_time_`, `scaling_factor_`,
`joint_command_interface_`, `joint_state_interface_`,
`last_commanded_state_`. They may change between releases of
ros2_controllers; the tests catch it.

Parameters of JTC that matter:

- `command_interfaces`: `[position]`, `[velocity]`, `[position, velocity]`,
  `[position, acceleration]` or `[position, velocity, acceleration]`. Effort
  is not supported. With `[velocity]` JTC also runs its PID adapter, whose
  output is overwritten.
- `interpolate_from_desired_state` (`open_loop_control` up to Jazzy) must be
  false and `speed_scaling.*` empty: the initial state and the scaling are
  set by the MPC.
- Path tolerances (`constraints.<joint>.trajectory`) are checked against the
  nominal trajectory, while the MPC moves away from it near obstacles: keep
  them at 0 (off), and `constraints.goal_time` at 0 since the trajectory slows down.

### ThorChainableController

- Reference interfaces: `<name>/<joint>/position`, `velocity`, `acceleration`.
- Exported state interface: `<name>/speed_scaling`.
- Own parameters: `joints`, `command_interfaces` (same combinations as
  above), `state_interfaces` (`position`, optionally `velocity`).

With `joint_trajectory_controller` in front:

```yaml
joint_trajectory_controller:
  ros__parameters:
    joints: [j1, j2, ...]
    command_joints: [thor_chainable/j1, thor_chainable/j2, ...]
    command_interfaces: [position, velocity, acceleration]
    speed_scaling:
      state_interface: thor_chainable/speed_scaling
```

Activate both together, THOR first:
`spawner thor_chainable joint_trajectory_controller --activate-as-group`.
Without references (not chained, or nothing written yet) the robot holds
its position.

The reference is extrapolated with constant acceleration over the horizon
(0.2 s by default): exact for the first interval, approximate at sharp
changes of the trajectory. Use `ThorTrajectoryController` when the exact
preview matters.

## Parameters (`thor.*`, both controllers)

The full list, with defaults and validation, is in
[src/thor_parameters.yaml](src/thor_parameters.yaml). The main ones:

| Parameter | Meaning |
| --- | --- |
| `thor.state_feedback` | Initial state of the MPC: `open_loop` (integrated state, for position commands), `position` (measured position, integrated velocity, for velocity commands), `full` (measured position and velocity). |
| `thor.target_scaling` | Desired speed (1 = nominal); can be changed at run time. |
| `thor.horizon.{n_intervals,time}` | Horizon of the MPC. |
| `thor.weights.{acceleration,jerk,scaling,clik}` | Cost weights. |
| `thor.limits.acceleration` | One value per joint (required). |
| `thor.limits.velocity` | One value per joint; empty means the URDF limits. Position limits come from the URDF. |
| `thor.cbf.*` | Barrier: `braking_deceleration`, `reaction_time`, `safety_margin`, `gamma`, `distance_error`, `max_obstacles`, ... |
| `thor.cbf.{obstacle_sigma,sigma_bound,robust_rate}` | Perception uncertainty: default standard deviations of the obstacle position, velocity and acceleration, the number of standard deviations of the box (default 3), and the robust rate term of the continuous formulation. See below. |
| `thor.cbf.escape_margin` | Scale of the escape term of the barrier; 0 means `safety_margin`. |
| `thor.robot_spheres.names` and `thor.robot_spheres.<name>.{frame,offset,radius}` | Spheres covering the robot, attached to frames of the URDF. |
| `thor.obstacles.{topic,frame_id,timeout}` | Obstacle input. |
| `thor.state_topic` | State output (default `~/thor_state`). |

The robot model comes from `robot_description`; joints that the controller
does not command are locked at zero. Only revolute and prismatic joints with
limits are supported.

## Topics

- **Obstacles** (`thor.obstacles.topic`, default `~/obstacles`):
  `thor_msgs/ObstacleArray`, in the root frame of the URDF (or
  `thor.obstacles.frame_id`); messages in other frames are discarded. Each
  obstacle has position, velocity, acceleration, radius, a capsule half axis
  (zero for a sphere), its own distance error and its own standard
  deviations (`sigma_position`, `sigma_velocity`, `sigma_acceleration`;
  negative means `thor.cbf.obstacle_sigma`). When more than
  `thor.cbf.max_obstacles` are given, the closest to the robot are used.
  Without new messages for `thor.obstacles.timeout`, the last obstacles are
  kept but considered still.
- **State** (`thor.state_topic`): `thor_msgs/ThorControllerState` every
  cycle: status, scaling, solve time, actual distance from the closest
  obstacle (`min_distance`), barrier (`h_now`), commands and reference.

## Safety notes

- When the QP has no solution (e.g. an obstacle already inside the margin
  while the robot moves fast), the controller brakes within the acceleration
  limits, stops the trajectory time and reports `STATUS_BRAKING`.
- `h_now` includes the braking distance against the worst case. When a
  person walks towards the robot, `h_now` can go below zero while the actual
  distance stays above `thor.cbf.safety_margin`: the discrete barrier lets
  `h` recover gradually instead of declaring the problem infeasible. A few
  millimeters below zero also come from the linearization along the horizon.
- The obstacle input is not filtered: perception must give positions and
  velocities in the right frame, with `delta_h` or the standard deviations
  covering its errors.
- **Perception uncertainty.** With `thor.cbf.obstacle_sigma` (or the
  per-obstacle `sigma_*`), the barrier is evaluated at the worst obstacle
  state within `sigma_bound` standard deviations: closer, faster towards the
  robot, accelerating towards it. The box grows along the horizon. Use either
  `distance_error` or `sigma_position` for the same position noise, not both.
  With a smaller `safety_margin`, raise `escape_margin` so that the escape
  term keeps its slope positive at the usual speeds
  (`v < escape_margin / reaction_time`). The method is described in
  [thor_math/docs/worst_case_h_design.md](../thor_math/docs/worst_case_h_design.md)
  and [thor_math/docs/robust_cbf_method.md](../thor_math/docs/robust_cbf_method.md).

## Tests

```bash
colcon test --packages-select thor_controllers
```

- `test_thor_core`: the MPC without ROS (model, joint order, tracking,
  obstacles, braking, feedback).
- `test_thor_trajectory_controller`, `test_thor_chainable_controller`: the
  controllers with test interfaces (configuration, interfaces, tracking,
  scaling, obstacles).

The integration test with the controller manager is in
[thor_bringup](../thor_bringup).
