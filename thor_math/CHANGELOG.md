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
obstacle, given as spheres or capsules (cylinders with hemispherical ends)
with position, velocity and acceleration. The robot is covered by spheres.

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
- Discrete-time CBF along the horizon, `ThorConfig::cbf_formulation =
  CbfFormulation::discrete` (the default).
  - The constraint is `h(X_{i+1}) >= max(0, 1 - gamma dt_i) h(X_i)`, with `h`
    linearized around the previous solution applied from the current state.
  - The continuous-time formulation is still available as
    `CbfFormulation::continuous`. It only constrains the rate of `h` at the
    start of each interval, so the predicted barrier could become strongly
    negative along the long intervals of the horizon.
  - On the example scene, the lowest barrier along the predicted plan goes
    from -1.19 m (continuous) to -0.04 m (discrete), for about 30 µs more
    per solve.
- Repeated linearization of the CBF within a cycle.
  - `ThorConfig::cbf_linearizations` (maximum QPs per cycle, default 1): each
    QP after the first re-linearizes the CBF around the plan just found.
  - `cbf_linearization_damping` (default 1): moves the linearization point only
    part of the way towards the new solution, which damps the oscillations of
    full steps.
  - `cbf_linearization_tolerance` (default 0, never): stops as soon as the
    solution changes by at most the tolerance.
  - `ThorOutput::cbf_linearizations` and `cbf_linearization_step` report what
    happened in the cycle. If a later QP fails, the previous solution is kept.
  - On the example scene, `N = 4`, damping 0.7 and tolerance 0.5 rad/s² bring
    the lowest predicted barrier from -4 cm to -1 mm, with no cycle below
    -1 cm, at 1.25 QPs per cycle on average. Full steps (damping 1) oscillate:
    `N = 6` is worse than `N = 2`.
- `CbfDiagnostics::h_now`: the barrier on the current state, i.e. the actual
  safety margin.
- `cbf::BarrierTerms::grad_p`: gradient of the barrier with respect to the
  robot center position.
- `frameVelocityDerivatives()`: derivatives of the velocity of a frame origin
  in world coordinates with respect to `q` and `dq`. The linear rows of
  `pinocchio::getFrameVelocityDerivatives()` in `LOCAL_WORLD_ALIGNED` are not
  this quantity.
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
  Lie derivatives and state gradients (points and spheres), and `cbf_python`
  reference values.
- Integration tests for `ThorQP`:
  - the CBF rows of both formulations; the discrete rows are rebuilt by finite
    differences on the nonlinear kinematics, also after a re-linearization;
  - `frameVelocityDerivatives()`;
  - the solution with simple bounds, the inequalities implied by the bounds
    left out and the Hessian factorized by blocks against the solution of the
    original problem with the bounds as general rows, over 20 cycles with
    saturated accelerations, for both formulations; the block factor is also
    checked against the inverse of the Hessian;
  - a reused qpmad solver against a fresh one on problems whose number of
    general constraints shrinks and grows, with and without simple bounds;
  - the radii, the per-obstacle `delta_H` and the input validation.
- Build: with the Ninja generators the parallel compile and link jobs are
  capped by the physical memory, assuming `THOR_MATH_JOB_MEMORY_MB` (3072)
  per job and leaving `THOR_MATH_RESERVED_MEMORY_MB` (4096) to the system.
  With Makefiles the `-j` of the caller decides.
- Build: `THOR_MATH_ARCH_FLAGS` (e.g. `-march=x86-64-v3`) is added to the
  public compile options of the target, so that the programs that link
  `thor_math::thor_math` share the flags that change the layout of Eigen
  objects. `scripts/build_thor.sh` (in the repository root) builds the
  Pinocchio of `.repos` and `thor_math` with the same flags into one prefix.
- Capsule obstacles (cylinders with hemispherical ends), e.g. a person from
  the floor to the head, or a column:
  - `ThorInput::obstacle_axis`: one half axis per obstacle, from
    `obstacle_pos` to one end; zero for a sphere, empty means spheres. It is
    the last field of `ThorInput`, so positional initializers still compile,
    and leaving it empty does not make `solve()` copy the input;
  - `cbf::capsuleBarrierTerms()`: beyond the ends the barrier of the sphere
    at the end, along the side the barrier of the state projected on the
    plane orthogonal to the axis (exact, since the motion along the axis does
    not change the distance);
  - the example models the body of the person as a vertical capsule.

- **Perception bounds on the whole obstacle state.** The barrier can be
  evaluated at the worst corner of the box of possible true obstacle states,
  not only at the worst-case distance. See `docs/worst_case_h_design.md`.
  - `ThorInput::obstacle_sigma` and `ThorConfig::obstacle_sigma`: standard
    deviations of the obstacle position, velocity and acceleration;
    `ThorConfig::sigma_bound` (default 3) sets the box half-width. The box
    grows along the extrapolation of the obstacle over the horizon.
  - `cbf::PerceptionBounds` and the new trailing arguments `bounds` of
    `cbf::barrierTerms()` and `cbf::capsuleBarrierTerms()`. `d_min` is monotone
    in every component of `psi`, so its minimum over the box is a fixed corner;
    `h`, the Lie derivatives and the gradients are evaluated there. The
    position bound also rotates `u`, which widens the bounds of the projected
    velocities near the obstacle (`BarrierTerms::psi_bound`).
  - `BarrierTerms::rate_bound`: bound of the error of `dh/dt` when the true
    state is elsewhere in the box. `ThorConfig::cbf_robust_rate` subtracts it
    in the continuous formulation; it is a rate and is not multiplied by
    `gamma`.
  - `BarrierTerms::dh_dpsi`: the full gradient of `h` with respect to `psi`.
  - With zero standard deviations nothing changes. With only `sigma_p` and a
    radial motion the result is the same as `delta_H = sigma_bound sigma_p`.
- **Scale of the escape term.** `ThorConfig::C_escape` and the trailing
  argument `C_escape` of `cbf::barrier()`, `cbf::barrierTerms()` and
  `cbf::capsuleBarrierTerms()`: the escape term becomes
  `(C - d_min)/C_escape * T_r * v_r`. It still starts at `d_min = C`; only its
  slope changes. `h` stays increasing in `d_min` up to `v_r = C_escape / T_r`,
  so a small `C` no longer flips the slope at ordinary speeds. 0 (the
  default) means `C`.

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
- Faster solve, measured on the example scene: from ~115 to ~58 µs
  (discrete), from ~83 to ~36 µs (continuous), and from ~32 to ~9 µs
  without CBF. The solution is unchanged.
  - The bounds on the accelerations and on the time scaling are passed to
    qpmad as simple bounds instead of identity rows of the constraint
    matrix. `m_CI` keeps its layout.
  - The QP solver and all the per-cycle buffers are members, sized by
    `configure()`/`updateMatrices()`. The CBF rows and the cost are built
    without heap allocations, and `solve()` copies the input only when it
    must complete it with defaults. Heap allocations per `solve()` go from
    437 to 1, the returned `ThorOutput::next_acc`.
  - The constant acceleration block of the Hessian is factorized once by
    `updateMatrices()`; each cycle only completes the inverted Cholesky
    factor for the time scalings and passes it to qpmad, once for all the
    linearizations of the cycle.
  - The inequalities that hold for every input within the bounds (most of
    them on the example scene) are not passed to qpmad.
  - The rows of the discrete CBF are computed only on the inputs that the
    state at the end of each interval depends on.
- The bundled qpmad keeps the buffers sized by the number of constraints
  instead of reallocating them when it changes (changes marked `thor_math:`
  in `include/qpmad/implementation.h`).
- `thor_math.h` includes `<qpmad/solver.h>`, which is installed with the
  package.
- When the QP fails, the stored solution is reset to zero. The next cycle uses
  it for the jerk term and the CBF linearization, and must not start from a
  partial or non-finite result.
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

- **Jerk weight accumulating over cycles.** `computeActualMatrices()` added
  `lambda_jerk J^T J` to the Hessian at every cycle without resetting it, so
  with `lambda_jerk > 0` the jerk weight grew with time. The term is constant
  and is now part of the fixed Hessian. A regression test checks that the
  cost does not change across cycles with the same inputs.

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
- Build: no CMP0167 warnings with CMake 3.30 or later. Boost, which Pinocchio
  looks for, is found through the `BoostConfig.cmake` of Boost instead of the
  removed `FindBoost` module.

## [0.1.0]

- First release: `ThorQP` MPC with time scaling, velocity, acceleration,
  position and invariance constraints, an optional CBF, and the qpmad solver.
