# Changelog

All notable changes to `thor_math` are documented here.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the
project uses [Semantic Versioning](https://semver.org/).

## [Unreleased]

The control barrier function (CBF) inside `ThorQP` now uses the formulation of
the paper and of its reference implementation,
[`cbf_python`](https://github.com/JRL-CARI-CNR-UNIBS/cbf_python/tree/dynamic_parameters)
(`controllers/kernels/ssm_cbf_acc.py`), with one deliberate correction (see
*Fixed*). `ThorQP` is still a model predictive controller over `n_intervals`
control intervals, and the CBF can be enabled or disabled with `ThorConfig::cbf`.

The CBF is no longer specific to people. It avoids any dynamic or static
obstacle, given as spheres with position, velocity and acceleration. The robot
is covered by spheres as well.

### Added

- `thor_math/cbf.h`: the barrier as free functions, ported from `cbf_python` and
  checked against its output to 1e-12.
  - `cbf::minimumDistance()`: minimum robot-obstacle distance while the robot
    reacts (`T_r`) and then brakes (`a_s`), with the obstacle keeping a constant
    acceleration, and its Jacobian with respect to `psi = [d, v_r, v_o, a_o]`.
  - `cbf::barrier()`: `h = d_min - C`, plus `(C - d_min)/C * T_r * v_r` when
    `d_min < C` so that a receding robot increases `h`.
  - `cbf::barrierTerms()`: barrier value, Lie derivatives `L_f h`, `L_g h` and the
    distance sensitivity `h_chi` for a robot sphere and an obstacle sphere
    (`radius` is the sum of the radii; 0 for points).
  - `cbf::perceptionMargin()`: distance estimation error `delta_H` from the
    perception noise (Eq. 9 of the paper).
- `ThorConfig`, `ThorInput`, `ThorOutput` and `CbfDiagnostics`, with
  `ThorQP::configure()` and `ThorQP::solve()`:
  - `configure()` validates the whole configuration and throws
    `std::invalid_argument` without changing the optimizer;
  - `solve()` validates the input sizes. It throws `std::logic_error` if the
    matrices are outdated and `std::runtime_error` if the problem is infeasible.
- `ThorQP(const ThorConfig&, const pinocchio::Model&)` constructor.
- Dynamic obstacles as spheres:
  - `ThorInput::obstacle_pos`, `obstacle_vel` and `obstacle_acc` (empty
    accelerations are taken as zero);
  - `ThorInput::obstacle_radius`: one radius per obstacle sphere (empty means
    points);
  - `ThorConfig::cbf_frame_radii`: one radius per robot sphere (empty means
    points). `C` is now the margin between the surfaces, and no longer has to
    include the size of the links and of the obstacles.
- Distance estimation error: `ThorConfig::delta_H` (default) and
  `ThorInput::obstacle_delta_H` (one per obstacle, e.g. noisier hand
  keypoints). The barrier is evaluated at the worst-case distance
  `d - delta_H`.
- `ThorConfig::n_obstacle_points` and `ThorQP::getNumObstaclePoints()`.
- `CbfDiagnostics` fields `v_robot`, `v_obstacle` and `obstacle_index`.
- `ThorConfig::min_scaling`, `max_scaling` and `cbf_activation_distance`, which
  were hard-coded before.
- `ThorQP::doSolve()`: protected virtual customization point for subclasses.
- `examples/cbf_mpc_example.cpp`, built with `-DBUILD_EXAMPLES=ON`: a closed-loop
  UR10 example, with and without the CBF. The robot is covered by spheres;
  the scene has a person (torso and hand spheres, each with its own
  `delta_H`) and a static fixture. It reports the clearance per obstacle and
  the MPC computation time (mean, median, p99, max), compared with the control
  period.
- API documentation:
  - `docs/api.md`: formulation, lifecycle, reference of every public symbol,
    and migration from the legacy API;
  - Doxygen comments on all public headers;
  - an optional `thor_math_docs` target (`-DBUILD_DOCS=ON`) that builds the
    HTML reference without warnings.
- Unit tests for the CBF kernels: analytic cases, finite-difference Jacobians,
  Lie derivatives (points and spheres) and `cbf_python` reference values.
  Integration tests check the CBF rows built by `ThorQP`, the radii, the
  per-obstacle `delta_H` and the input validation.

### Changed

- **The CBF formulation.** The barrier was `h(d, v_rel)`, written by hand for a
  few velocity cases. It is now `h(d, v_r, v_o, a_o)`, built on `d_min` along
  the braking trajectory. Its Lie derivatives include the rotation of the
  robot-obstacle direction and the obstacle acceleration. Each interval,
  robot sphere and obstacle sphere gets the constraint
  `L_g h (J u_i + dJ dq) + L_f h + gamma h >= 0`. Here `h` uses the worst-case
  distance between the surfaces,
  `|p_r - p_o| - r_robot - r_obstacle - delta_H`.
- The CBF of interval `i` is now written at the **start** of the interval:
  - the current state `x0` for `i = 0`;
  - for later intervals, the end of interval `i-1` of the previous prediction.

  Before, it used the end of interval `i`. This matches the acceleration `u_i`
  applied in that interval. With `n_intervals = 1` and `delta_H = 0` the
  constraint is the same as in the one-step controller of `cbf_python`.
- Obstacle points are extrapolated along the horizon with constant
  acceleration, as in the `d_min` model. Before, they were extrapolated with
  constant velocity.
- With the CBF enabled, `configure()` requires `C > 0`, `T_r >= 0` and
  `delta_H >= 0`.
- Kinematics is computed once per interval instead of once per interval and
  frame.
- `solve()` no longer prints the barrier diagnostics on `std::cout`; they are
  returned in `ThorOutput::cbf`.
- `thor_math.h` no longer includes `<iostream>`. Code that relied on that
  include must now include it itself.
- `eigen_utils::standard_deviation()` and `correlation()` no longer use static
  buffers, so they are reentrant and work with vectors of any size.
  `correlation()` accepts two different expression types and returns 0 for
  constant vectors.

### Deprecated

- The legacy setters, replaced by `ThorConfig` and `configure()`:
  `setCBFParameters`, `setConstraints`, `activatePositionBounds`,
  `activateCbfBounds`, `setIntervals`, `setWeigthFunction` and `setCbfIds`.
- `computedCostrainedSolution()`, replaced by `solve()`. It is no longer
  virtual; override `doSolve()` instead.
- `activateTorqueBounds()`: torque bounds are not implemented and the call has no
  effect.
- `activateCbfMoveAway()`: the call has no effect, because `d_min` already handles
  a receding robot.
- `getNumPh()`, replaced by `getNumObstaclePoints()`.

### Removed

- `ThorConfig::cbf_move_away` and the protected `ThorQP::compute_h()` /
  `ThorQP::compute_theta()`.
- The "human" names of the new API, renamed before release:
  - `human_pos`/`human_vel`/`human_acc` → `obstacle_pos`/`obstacle_vel`/`obstacle_acc`;
  - `n_human_points` → `n_obstacle_points`;
  - `CbfDiagnostics::v_rel`/`vh_proj`/`human_index` → `v_robot`/`v_obstacle`/`obstacle_index`.
- The `1e-6` replacement of a zero CBF offset.

### Fixed

- **Robust term.** `cbf_python` and the paper add `+ h_chi delta_H` to the CBF
  condition. That relaxes the constraint, and it adds a distance (m) to a rate
  (m/s). `thor_math` instead evaluates the barrier at the worst-case distance
  `d - delta_H`, which is the same as inflating the obstacle sphere by
  `delta_H`.

  Its first-order form, `gamma (h - h_chi delta_H)`, is not used: it leaves the
  escape term of the barrier at `d_min = C`. A robot stopped on the robust
  boundary then has no control authority (`L_g h = 0`), and the QP stays
  infeasible forever; a regression test covers this case. See
  `docs/robust_margin_deadlock.md`.

  With `delta_H = 0` all the forms coincide.
- The CBF columns are written into buffers that are zeroed first, because
  Pinocchio only writes the Jacobian columns of the frame's supporting joints.

## [0.1.0]

- First release: `ThorQP` MPC with time scaling, velocity, acceleration,
  position and invariance constraints, an optional CBF, and the qpmad solver.
