# thor_math API reference

`thor_math` provides `thor::math::ThorQP`, a model predictive controller (MPC)
for robot manipulators. It scales the speed of a nominal joint trajectory and,
optionally, keeps a safe distance from dynamic obstacles through a control
barrier function (CBF). Obstacles are spheres or capsules (cylinders with
hemispherical ends) with position, velocity and acceleration: people (e.g. skeleton keypoints), vehicles, other robots and
static fixtures are handled the same way.

| Header | Content |
| --- | --- |
| `thor_math/thor_math.h` | `ThorQP`, its configuration and I/O structs, evolution matrices |
| `thor_math/cbf.h` | CBF kernels: `d_min`, barrier, Lie derivatives, perception margin |
| `thor_math/eigen_matrix_utils.h` | statistics helpers (`eigen_utils`) |

Link with the CMake target `thor_math::thor_math`. It depends on Eigen 3 and
Pinocchio. The QP solver ([qpmad](https://github.com/asherikov/qpmad)) is
bundled.

The Doxygen reference is generated with `-DBUILD_DOCS=ON` and
`cmake --build <build> --target thor_math_docs`. A complete, commented
closed-loop program is in
[examples/cbf_mpc_example.cpp](../examples/cbf_mpc_example.cpp).

---

## 1. Quick start

```cpp
#include <thor_math/thor_math.h>
#include <pinocchio/parsers/urdf.hpp>

pinocchio::Model model;
pinocchio::urdf::buildModel("robot.urdf", model);
const unsigned int n = model.nv;

thor::math::ThorConfig config;
config.n_intervals     = 5;
config.n_joints        = n;
config.horizon_time    = 0.2;
config.sampling_period = 0.002;           // 500 Hz
config.lambda_scaling  = 50.0;
config.lambda_clik     = 1e4;
config.lambda_acc      = 1e-6;
config.qmin   = model.lowerPositionLimit;
config.qmax   = model.upperPositionLimit;
config.Dqmax  = Eigen::VectorXd::Constant(n, 2.0);
config.DDqmax = Eigen::VectorXd::Constant(n, 10.0);
config.position_bounds = true;
config.min_scaling = 0.0;
config.max_scaling = 1.0;

config.cbf   = true;                       // false: the same MPC without CBF
config.a_s   = 2.5;
config.T_r   = 0.15;
config.C     = 0.10;
config.gamma = 5.0;
config.delta_H = thor::math::cbf::perceptionMargin(3.291, 0.015, 0.072, 0.196, 0.3);
config.cbf_frame_ids     = {static_cast<unsigned int>(model.getFrameId("tool0"))};
config.cbf_frame_radii   = {0.05};          // robot sphere around tool0
config.n_obstacle_points = 1;

thor::math::ThorQP qp(config, model);      // throws std::invalid_argument on bad config
qp.setInitialState(x0);                    // x0 = [q; dq]

double tau = 0.0, s = 1.0;                 // trajectory time and time scaling
const Eigen::VectorXd t = qp.getPredictionTimeInstant();
for (;;) {
  thor::math::ThorInput in;
  in.x0 = qp.getState();                   // or the measured state
  in.target_dq.resize(n * config.n_intervals);
  for (unsigned int i = 0; i < config.n_intervals; ++i)
    in.target_dq.segment(i * n, n) = nominal_velocity(tau + s * t(i));
  in.next_target_q = nominal_position(tau + s * t(0));
  in.obstacle_pos    = {p_o};
  in.obstacle_vel    = {v_o};
  in.obstacle_acc    = {a_o};              // optional
  in.obstacle_radius = {0.2};              // optional: obstacle sphere radius
  // in.obstacle_axis = {Eigen::Vector3d(0, 0, 0.6)};  // optional: a vertical capsule instead

  try {
    const thor::math::ThorOutput out = qp.solve(in);
    qp.updateState(out.next_acc);          // or send out.next_acc to the robot
    s = out.next_scaling;
  } catch (const std::runtime_error&) {    // infeasible: brake safely
    ...
  }
  tau += s * config.sampling_period;
}
```

---

## 2. Formulation

### 2.1 Horizon

The horizon `horizon_time` is split into `nc = n_intervals` control
intervals. Interval `i` ends at the prediction time `t_i`
(`ThorQP::getPredictionTimeInstant()`):

- `input_blocking = true` (default): the intervals grow quadratically (see
  `quadraticControlIntervals()`). The first one lasts `sampling_period`, and
  the last prediction time is `horizon_time` rounded to a multiple of
  `sampling_period`. This needs `nc >= 2`.
- `input_blocking = false`: all intervals last `horizon_time / nc`, rounded to
  a multiple of `sampling_period` (see `constantControlIntervals()`).

Only the command of the first interval is applied; the others make the
controller look ahead.

### 2.2 Decision variables and prediction

For `nax = n_joints` joints, the decision vector has size `nc*(nax+1)`:

```
x = [ u_0; u_1; ...; u_{nc-1}; s_0; s_1; ...; s_{nc-1} ]
```

- `u_i` (size `nax`) is the joint acceleration, constant over interval `i`.
- `s_i` is the time scaling of interval `i`.

Each joint is a double integrator. The predicted positions and velocities at
`t_i` are affine in the current state `x0 = [q; dq]` and in `u`
(`computeEvolutionMatrix()`):

```
q(t_i)  = Pf_i x0  + PF_i u
dq(t_i) = Vf_i dq0 + VF_i u
```

### 2.3 Time scaling

The controller does not follow time `t` but a trajectory time `tau`, with
`d(tau)/dt = s`:

- `s = 1` follows the nominal trajectory at nominal speed;
- `s < 1` slows it down;
- `s = 0` stops along the path.

The caller owns `tau`:

- sample the nominal trajectory at `tau + s * t_i`, and pass the **unscaled**
  velocities as `target_dq`;
- after each cycle, advance `tau += next_scaling * dt`.

### 2.4 Cost

`ThorQP` minimizes (qpmad form: `1/2 x' H x + f' x`):

```
1/2 sum_i || dq(t_i) - s_i target_dq_i ||^2                  velocity tracking
+ lambda_acc/2     sum_i || u_i ||^2                         effort
+ lambda_scaling/2 sum_i ( s_i - target_scaling )^2          keep the nominal speed
+ lambda_clik/2    || q(t_0) - next_target_q ||^2            position drift correction
+ lambda_jerk/2    sum_i || (u_i - u_{i-1}) / dt_i ||^2      smoothness
```

`u_{-1}` is the first acceleration of the previous solution. `lambda_tau` is
accepted but has no effect.

The velocity tracking term has unit weight; the other weights are relative to
it. Typical starting values are those of the quick start. Raise
`lambda_scaling` to make the robot keep its speed, and raise `lambda_clik` to
make it stay on the path (for example, slow down rather than deviate when the
CBF is active).

### 2.5 Constraints

For every interval `i`:

| Constraint | Enabled by |
| --- | --- |
| `-DDqmax <= u_i <= DDqmax` | always |
| `min_scaling <= s_i <= max_scaling` | always |
| `-Dqmax <= dq(t_i) <= Dqmax` | always |
| `qmin <= q(t_i) <= qmax` | `position_bounds` |
| `K (q(t_i) - qmin) + dq(t_i) >= 0`, `K (qmax - q(t_i)) - dq(t_i) >= 0`, with `K = 0.99 DDqmax / Dqmax` | `position_bounds` |
| CBF, one row per (interval, robot sphere, obstacle) | `cbf` |

The invariance rows limit the velocity towards a position limit, so that the
robot can always stop before reaching it.

### 2.6 Control barrier function

The robot is covered by spheres centered at `cbf_frame_ids`, with radii
`cbf_frame_radii`. Each obstacle point `k` is a sphere centered at
`obstacle_pos[k]`, with radius `obstacle_radius[k]` and distance estimation
error `delta_H_k` (`obstacle_delta_H[k]`, or `ThorConfig::delta_H`). If
`obstacle_axis[k]` is not zero, the obstacle is a capsule of the same radius
around the segment from `obstacle_pos[k] - obstacle_axis[k]` to
`obstacle_pos[k] + obstacle_axis[k]`: a cylinder with hemispherical ends, e.g.
a person from the floor to the head, or a column. For every robot sphere `j`
and obstacle `k`:

- **Distance.** The barrier uses the worst-case distance between the
  surfaces, `d = |p_r - p_o| - r_j - r_k - delta_H_k`. The perception error
  inflates the obstacle. For a capsule `p_o` is the point of the axis
  closest to `p_r`: beyond an end it is that end (a sphere), along the side
  the barrier is that of the positions, velocities and accelerations
  projected on the plane orthogonal to the axis, since the motion along the
  axis does not change the distance (`cbf::capsuleBarrierTerms()`).
- **Barrier.** `h = d_min - C`, where `d_min` is the minimum of that distance
  if the robot reacts after `T_r` and then brakes at `a_s`, while the
  obstacle keeps its acceleration (section 5). `C` is the safety margin
  between the surfaces. The safe set is `h >= 0`. Inside the margin an
  escape term with scale `C_escape` (default `C`) lets a receding robot
  raise `h`.
- **Perception bounds.** With `obstacle_sigma[k]` (or
  `ThorConfig::obstacle_sigma`) the obstacle position, velocity and
  acceleration are known within `sigma_bound` standard deviations. The box
  grows along the extrapolation to the prediction time `t`:
  `b_p = k (sigma_p + sigma_v t + sigma_a t²/2)`, `b_v = k (sigma_v + sigma_a t)`,
  `b_a = k sigma_a`. `d_min` is monotone in every component of
  `psi = [d, v_r, v_o, a_o]`, so the barrier, its Lie derivatives and its
  gradients are evaluated at the worst corner of the box
  (`cbf::barrierTerms()`; design and limits in
  [worst_case_h_design.md](worst_case_h_design.md); the full method, with
  proofs, is in [robust_cbf_method.md](robust_cbf_method.md)). With only `sigma_p` and
  a radial motion this is the same as `delta_H = sigma_bound sigma_p`; do not
  count the same noise in both.
- **Condition.** One row for every interval `i`, robot sphere and
  obstacle. `ThorConfig::cbf_formulation` selects how it is written; the two
  formulations are described below.
- **Obstacle prediction.** The obstacle centers are extrapolated to each
  prediction time with constant acceleration; capsule axes translate without
  rotating.
- **Activation distance.** If the lowest `h` over the horizon is above
  `cbf_activation_distance`, the CBF rows are dropped from the QP: the
  solution is the same, and the QP is smaller.

Let `X_0 = x0` be the current state and `X_{i+1}` the predicted state at the
end of interval `i`, of duration `dt_i`.

#### Discrete formulation (`CbfFormulation::discrete`, default)

```
h(X_{i+1}) >= alpha_i h(X_i),     alpha_i = max(0, 1 - gamma dt_i)
```

- **Meaning.** `h` stays non-negative at every predicted state. When it is
  negative, it recovers at least geometrically. The clamp at 0 keeps
  `h(X_{i+1}) >= 0` when `gamma dt_i > 1`.
- **Linearization.** `h(X_m)` is linearized around a reference trajectory:
  the inputs of the previous solution applied from the current state. Each
  row depends on all the accelerations `u_0 ... u_i`:

  ```
  h(X_m(u)) ≈ h(X̄_m) + G_m (u - u_ref)
  G_m = (grad_p^T J + lie_g^T dv/dq) dq_m/du + lie_g^T J ddq_m/du
  ```

  - `grad_p` and `lie_g` are the gradients of `h` with respect to the robot
    center position and velocity (`cbf::barrierTerms()`).
  - `dv/dq` comes from `frameVelocityDerivatives()`.
  - `h(X_0)` is exact.
- **Accuracy.** On the first interval the linearization is exact to first
  order in one control period. On the longer intervals of the horizon the
  error grows with the distance between the new plan and the reference.
- **Cost.** It takes one kinematics evaluation more per cycle than the
  continuous formulation. The rows are dense in `u`.

#### Continuous formulation (`CbfFormulation::continuous`)

```
L_g h (J u_i + dJ dq) + L_f h + gamma h >= 0
```

- **Where it is imposed.** At the start of interval `i`: on the current
  state for `i = 0`, and for `i > 0` on the state at `t_{i-1}` of the
  previous prediction, which `solve()` stores.
- **Terms.** `L_f h` and `L_g h` are the Lie derivatives of `h`;
  `J u_i + dJ dq` is the acceleration of the frame. Radii and `delta_H` are
  constant, so the Lie derivatives are the same as for points.
- **Robust rate (`cbf_robust_rate`).** With perception bounds, the condition
  also subtracts `BarrierTerms::rate_bound`, the bound of the error of
  `dh/dt` when the true obstacle state is elsewhere in the box. It is a
  rate, not multiplied by `gamma`. The discrete formulation constrains `h`
  at every node and does not use it.
- **Relation to the paper.** With `n_intervals = 1` this is the one-step
  controller of the paper.
- **Limitation.** It constrains only the rate of `h` at one instant per
  interval. Along the long intervals of the horizon the predicted `h` can
  become strongly negative even when the executed motion is safe.

#### Repeated linearization (`cbf_linearizations`)

Every cycle solves up to `N = cbf_linearizations` QPs. The CBF is linearized
around reference inputs `u_ref`:

```
u_ref = solution of the previous cycle
for k = 1 … N:
    u* = QP(CBF linearized around u_ref)
    stop if k == N, if the CBF rows are inactive,
         or if |u* - u_ref|_inf <= cbf_linearization_tolerance
    u_ref = u_ref + cbf_linearization_damping · (u* - u_ref)
command = u* of the last QP solved
```

- **What is reported.** `ThorOutput::cbf_linearizations` is the number of
  QPs solved; `ThorOutput::cbf_linearization_step` is the last
  `|u* - u_ref|_inf` [rad/s²], useful to tune the tolerance.
- **Failures.** If a QP after the first is infeasible, the solution of the
  previous one is kept: it satisfies its own linearization.
- **Why the command is never damped.** Only the linearization point is
  damped. The command is always a QP solution, so it satisfies the
  constraints of its own linearization.
- **When `N = 1` is enough.** `N = 1` (default) is enough for safety,
  because the executed interval is linearized over one control period only.
  A larger `N` makes the predicted barrier follow the nonlinear one along the
  long intervals of the horizon.

Measured on the example scene (3000 cycles; lowest nonlinear `h` along the
plan of each cycle):

| `N` | damping | tolerance [rad/s²] | lowest `h` along the plan | cycles with `h < -1 cm` | QPs per cycle | mean / max solve time |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | – | – | -0.040 m | 53 | 1.00 | 77 / 167 µs |
| 2 | 1 | 0 | -0.027 m | 6 | 2.00 | 149 / 508 µs |
| 3 | 1 | 0 | -0.027 m | 28 | 2.92 | 223 / 429 µs |
| 6 | 1 | 0 | -0.027 m | 50 | 5.16 | 401 / 796 µs |
| 6 | 0.7 | 0 | -0.010 m | 0 | 6.00 | 454 / 747 µs |
| 6 | 0.7 | 0.1 | -0.010 m | 0 | 1.89 | 149 / 580 µs |
| 4 | 0.7 | 0.5 | -0.001 m | 0 | 1.25 | 98 / 540 µs |

- **Full steps oscillate.** With `damping = 1` more linearizations can make
  the plan worse (`N = 6` is worse than `N = 2`), because the linearization
  point jumps back and forth.
- **Damping stabilizes large `N`.** With `damping = 0.7`, `N = 6` has no
  cycle below -1 cm. The trend is not monotonic in `N`, though.
- **The early stop makes it affordable.** Far from the obstacles a second
  QP changes little and the cycle stops after one or two QPs. The mean cost
  falls to that of `N = 1`–`2`, while the critical cycles still get up to
  `N` QPs.
- **Size the real-time budget on the maximum, not the mean.** The early stop
  does not lower the worst case, which is `N` QPs per cycle.
- **These numbers come from one scene.** Use them as a starting point
  (`N = 4`, `damping = 0.7`, `tolerance = 0.5`), not as a general tuning.

With the continuous formulation, re-linearizing does not help: with `N = 2`
the lowest `h` along the plan is still -1.04 m, because that formulation
only constrains the rate of `h`.

#### Comparison

On the example scene (5 robot spheres, 3 obstacles, 5 intervals over
0.2 s), the executed motion is equally safe with both formulations. The
difference is in the plan, measured as the nonlinear `h` along the predicted
states of the new solution over 3000 cycles:

| | lowest `h` along the plan | cycles with `h < -1 cm` | mean solve time |
| --- | --- | --- | --- |
| continuous | -1.19 m | 254 | ~55 µs |
| discrete | -0.04 m | 53 | ~80 µs |

---

## 3. ThorQP lifecycle

```
ThorQP()  ──setPinocchioModel()──>  configure(config)  ──>  setInitialState(x0)
                                         │                          │
                                         └── on new settings ◄──────┤
                                                                    ▼
                                            loop:  solve(in)  ──>  updateState(acc)
```

1. **Model.** `setPinocchioModel(model)` is required before `configure()` if
   `config.cbf` is true. The model is copied, and must have
   `nq == nv == n_joints`. The constructor `ThorQP(config, model)` does both
   steps.
2. **Configure.** `configure(config)` validates everything, then applies the
   settings and rebuilds the matrices. On invalid settings it throws
   `std::invalid_argument` and leaves the optimizer unchanged. It also resets
   the previous solution.
3. **Initial state.** `setInitialState(x0)` sets the internal state and a
   prediction of a robot standing still at `q`.
4. **Each cycle:**
   - `solve(in)` returns the acceleration and time scaling of the first
     interval, and stores the prediction for the next cycle.
   - `updateState(acc)` integrates the internal state for one period. In a
     real system, pass the measured state as `in.x0` instead.

**Infeasibility.** `solve()` throws `std::runtime_error` when no command
satisfies all constraints, e.g. an obstacle moving faster than the robot can
react. The controller must then apply a safe command itself. For example,
brake within the limits, `acc = clamp(-dq / dt, -DDqmax, DDqmax)`, and freeze
the trajectory time.

**Thread safety.** `ThorQP` keeps per-cycle state and Pinocchio buffers, so it
is not thread-safe. Use one instance per control loop. Instances can be copied
with `operator=`; the copy includes the model, the configuration, the state
and the last solution.

**Cost per cycle.** It grows with
`n_intervals * (n_joints + 1)` variables and with
`n_intervals * robot spheres * obstacles` CBF rows. There is one
kinematics evaluation per interval (continuous) or per interval plus one
(discrete), times `cbf_linearizations`. With 6 joints, 5 intervals, 5 robot
spheres and 3 obstacles, a solve takes about 35–60 µs in Release
builds with one linearization (see the example and section 3.1).

### 3.1 Performance

Profile of one solve on the example scene before the optimizations below
(discrete formulation, one linearization, Release build, AMD Ryzen 9
5900HX):

| phase | time |
| --- | --- |
| QP solver (qpmad) | 52 µs |
| kinematics (Pinocchio, one pass per prediction node) | 25 µs |
| barriers and CBF rows | 22 µs |
| cost and constraint assembly | 13 µs |
| **total** | **~115 µs** |

After the optimizations, with the same solutions:

| | discrete | continuous | without CBF | N = 4, β = 0.7, tol. 0.5 |
| --- | --- | --- | --- | --- |
| before | ~115 µs | ~83 µs | ~32 µs | ~142 µs |
| first round | ~77 µs | ~53 µs | ~19 µs | ~96 µs |
| second round | **~58 µs** | **~36 µs** | **~9 µs** | **~72 µs** |

Heap allocations per `solve()` went from 437 to 1 in the first round, and
the second round adds none.

First round:

- **Simple bounds.** The bounds on the accelerations and on the time scaling
  are passed to qpmad as simple bounds `lb <= x <= ub`. They are not passed
  as general constraints (identity rows), which the active-set solver would
  treat like any other row. `m_CI` still stores them in its first columns,
  but they are not sent to the solver. The problem and the solution are the
  same.
- **No allocation in the loop.** The solver and every buffer (QP matrices,
  CBF rows, kinematic Jacobians) are members, sized once by `configure()`
  and reused. Products are written with `noalias()`. `solve()` copies the
  input only when some obstacle vectors must be completed with defaults.
  The one allocation left is the `ThorOutput::next_acc` returned by value.
  This also makes the solve time more deterministic.
- **Eigen is not the bottleneck.** At these sizes (35 variables, ~255
  constraints) the time goes into the active-set iterations, the kinematics
  and, before, the allocations, not into the matrix products.

Second round, after a new profile (discrete: CBF rows ~33 µs, of which
~14 µs for the rows `G` of the linearized barrier; qpmad ~27 µs, of which
~7 µs to factorize and invert the Hessian; ~88% of the general constraints
implied by the bounds):

- **Hessian factorized by blocks.** The block of `m_H` on the accelerations
  is constant; only the rows of the time scalings change with the target
  velocity. `updateMatrices()` computes the inverted Cholesky factor of the
  constant block once, and each cycle completes the factor with the Schur
  complement of the scalings (5×5) and hands it to qpmad as
  `HESSIAN_INVERTED_CHOLESKY_FACTOR`. The factor is computed once per cycle,
  not once per linearization. If a derived class changes the constant block,
  `solve()` falls back to the full factorization.
- **Inequalities implied by the bounds are left out.** A row
  `CI^T x + ci0 >= 0` whose minimum over `lb <= x <= ub` is already >= 0
  cannot be violated, so leaving it out does not change the feasible set or
  the solution. qpmad multiplies and checks every general constraint at every
  iteration; on the example scene ~224 of the 255 rows are of this kind. The
  minimum is precomputed for the constant rows and computed at every
  linearization for the CBF rows. `m_CI` and `m_ci0` keep their layout.
- **Causal CBF rows.** The state at the end of interval `i` depends only on
  the inputs of intervals `0..i`, so only those columns of each row `G` are
  computed. `updateMatrices()` finds the nonzero columns of the forced
  responses instead of assuming them.
- **qpmad keeps its buffers.** The number of rows passed to qpmad now
  changes from cycle to cycle, and qpmad reallocated two internal vectors at
  every change. The bundled copy is modified so that they grow but never
  shrink (marked `thor_math:` in `include/qpmad/implementation.h`), and
  `ThorQP` reserves them for the largest problem.

**Build options**, measured on the same scene with Pinocchio 3.3.1 built
from source (−% of the mean solve time, discrete / continuous / without
CBF). None of them changes the solution:

| option | gain |
| --- | --- |
| `-march=x86-64-v3` (AVX2/FMA) on Pinocchio and `thor_math` | 5 / 8 / 10% |
| `-march=x86-64-v3 -fno-semantic-interposition -fno-plt` | 6 / 8 / 9% |
| `-march=x86-64-v3`, Pinocchio without template instantiation | 1 / 10 / 10% |
| `-march=x86-64-v3` with LTO on `thor_math` | 3 / 5 / 3% |
| `-march=x86-64-v3 -fno-math-errno -fno-trapping-math` | 3 / 4 / 2% |

- **SIMD needs coherent flags.** Pinocchio from the ROS packages is built for
  generic x86-64. Building `thor_math` with `-march=native` against it makes
  the two binary-incompatible (Eigen changes the layout of the objects), and
  the example crashes, even with `EIGEN_MAX_ALIGN_BYTES=16`. Pinocchio,
  `thor_math` and the application must all be built with the same flags.
- **Pinocchio without template instantiation**
  (`-DENABLE_TEMPLATE_INSTANTIATION=OFF`) builds in ~20 s instead of
  ~3.5 min with 3 jobs, with no loss in speed.

Remaining options, not implemented: skip the kinematics and the barriers of
the pairs far from every obstacle, warm-start the active set (qpmad has no
hot start; qpOASES has), benchmark other QP solvers (HPIPM, ProxQP).

---

## 4. Reference: thor_math.h

### 4.1 `struct ThorConfig`

Settings applied by `ThorQP::configure()`. Vectors have size `n_joints`.

| Field | Default | Meaning | Validation |
| --- | --- | --- | --- |
| `n_intervals` | 0 | control intervals `nc` | > 0; >= 2 with `input_blocking` |
| `n_joints` | 0 | joints `nax` | > 0 |
| `horizon_time` | 0 | horizon [s] | >= `sampling_period` |
| `sampling_period` | 0 | control period `dt` [s] | > 0 |
| `input_blocking` | true | quadratic (true) or equal (false) intervals | every interval must be > 0 |
| `lambda_acc` | 0 | acceleration weight | |
| `lambda_tau` | 0 | not implemented | |
| `lambda_jerk` | 0 | jerk weight | |
| `lambda_scaling` | 0 | scaling weight | |
| `lambda_clik` | 0 | position drift weight | |
| `qmax`, `qmin` | – | position limits [rad] | size; `qmin <= qmax` |
| `Dqmax` | – | velocity limits [rad/s] | size; > 0 |
| `DDqmax` | – | acceleration limits [rad/s²] | size; >= 0 |
| `tau_max` | empty | stored, not used | empty or size `n_joints` |
| `position_bounds` | false | position and invariance constraints | |
| `min_scaling`, `max_scaling` | 0.05, 1.01 | bounds of `s_i` | `min <= max` |
| `cbf` | false | enables the CBF | model `nq == nv == n_joints` |
| `cbf_formulation` | `discrete` | `CbfFormulation::discrete` or `continuous` (section 2.6) | |
| `cbf_linearizations` | 1 | maximum QPs per cycle, each re-linearizing the CBF around the previous plan (section 2.6) | >= 1 (CBF) |
| `cbf_linearization_damping` | 1 | step of the linearization point towards each new solution | in (0, 1] (CBF) |
| `cbf_linearization_tolerance` | 0 | stop re-linearizing when the solution changes by at most this value [rad/s²]; 0 never stops early | >= 0 (CBF) |
| `a_s` | 0 | robot braking deceleration [m/s²] | > 0 (CBF) |
| `T_r` | 0 | reaction time [s] | >= 0 (CBF) |
| `C` | 0 | safety margin [m] | > 0 (CBF) |
| `C_escape` | 0 | scale of the escape term of the barrier [m]; 0 means `C` | >= 0 (CBF) |
| `gamma` | 0 | CBF gain [1/s] | |
| `delta_H` | 0 | default distance estimation error [m], inflates every obstacle | >= 0 (CBF) |
| `obstacle_sigma` | 0 | default standard deviations of the obstacle position [m], velocity [m/s] and acceleration [m/s²] | >= 0 (CBF) |
| `sigma_bound` | 3 | standard deviations of the perception box | >= 0 (CBF) |
| `cbf_robust_rate` | false | continuous formulation: subtract the bound of the `dh/dt` error | |
| `cbf_activation_distance` | 5.0 | drop the CBF rows above this `h` [m] | |
| `cbf_frame_ids` | empty | Pinocchio frame ids of the robot points | < `model.frames.size()` |
| `cbf_frame_radii` | empty | radius of each robot sphere [m]; empty means points | empty or size of `cbf_frame_ids`; >= 0 |
| `n_obstacle_points` | 0 | obstacles (spheres or capsules) per cycle | >= 0 |

### 4.2 `struct ThorInput`

| Field | Size | Meaning |
| --- | --- | --- |
| `target_dq` | `n_joints*n_intervals` | unscaled target velocities, `[dq_0; ...; dq_{nc-1}]` |
| `next_target_q` | `n_joints` | target position at `t_0` |
| `target_scaling` | – | desired time scaling (default 1) |
| `x0` | `2*n_joints` | current state `[q; dq]` |
| `obstacle_pos` | ≥ `n_obstacle_points` | obstacle sphere centers, or capsule axis midpoints [m], world frame |
| `obstacle_vel` | same as `obstacle_pos` | velocities [m/s] |
| `obstacle_acc` | empty or same | accelerations [m/s²]; empty means zero |
| `obstacle_radius` | empty or same | sphere or capsule radii [m] (>= 0); empty means points |
| `obstacle_delta_H` | empty or same | distance estimation error of each obstacle [m] (>= 0); empty means `ThorConfig::delta_H` |
| `obstacle_axis` | empty or same | capsule half axes [m], from `obstacle_pos` to one end, finite; zero for a sphere, empty means spheres |
| `obstacle_sigma` | empty or same | standard deviations of the position [m], velocity [m/s] and acceleration [m/s²] of each obstacle (>= 0); empty means `ThorConfig::obstacle_sigma` |

With the CBF disabled, the obstacle fields are ignored. With the CBF enabled
and `obstacle_pos`, `obstacle_vel` and `obstacle_acc` all empty,
`n_obstacle_points` still points at `(100, 100, 100)` are used (nothing
around). Only the first `n_obstacle_points` spheres are used.

### 4.3 `struct ThorOutput`

| Field | Meaning |
| --- | --- |
| `next_acc` | joint accelerations for the next control period |
| `next_scaling` | time scaling of the first interval |
| `cbf` | `CbfDiagnostics` of the most critical pair (CBF enabled), from the last linearization |
| `cbf_linearizations` | QPs solved successfully in this cycle: 1 without CBF, up to `ThorConfig::cbf_linearizations` with it |
| `cbf_linearization_step` | last `\|u* - u_ref\|_inf` [rad/s²] between a solution and its linearization point; 0 with a single QP |

### 4.4 `struct CbfDiagnostics`

The robot-obstacle pair with the lowest `h` over the horizon, and the lowest
`h` on the current state. All fields are 0 with the CBF disabled.

The predicted states are the linearization points of the CBF: the reference
trajectory (discrete) or the previous prediction (continuous), not the new
solution. `h_min` can therefore be negative even when the executed motion is
safe; `h_now` is the actual margin.

| Field | Meaning |
| --- | --- |
| `h_now` | lowest barrier value on the current state [m] |
| `h_min` | lowest barrier value over the horizon [m] |
| `interval` | prediction node of `h_min`: 0 current state, `i` start of interval `i`, `n_intervals` end of the horizon (discrete only) |
| `frame_id` | Pinocchio frame id (`model.frames[frame_id].name`) |
| `distance` | distance `d` [m] |
| `v_robot` | robot velocity along `(p_r - p_o)/\|p_r - p_o\|` [m/s], negative when approaching |
| `v_obstacle` | obstacle velocity along the same direction, positive when approaching |
| `obstacle_index` | index in `ThorInput::obstacle_pos` |

### 4.5 `class ThorQP`

| Member | Description |
| --- | --- |
| `ThorQP()` | unconfigured optimizer |
| `ThorQP(const ThorConfig&, const pinocchio::Model& = {})` | `setPinocchioModel()` + `configure()` |
| `void configure(const ThorConfig&)` | validate, apply, rebuild; throws `std::invalid_argument` |
| `ThorOutput solve(const ThorInput&)` | solve one cycle; throws `std::logic_error` (not configured), `std::invalid_argument` (sizes), `std::runtime_error` (infeasible) |
| `void setPinocchioModel(const pinocchio::Model&)` | robot model for the CBF (copied) |
| `void setInitialState(const Eigen::VectorXd& x0)` | internal state and still prediction; after `configure()` |
| `void updateState(const Eigen::VectorXd& acc)` | integrate the internal state over `dt` |
| `Eigen::VectorXd getState()` | internal state `[q; dq]` |
| `Eigen::VectorXd getPredictionTimeInstant()` | prediction times `t_i` |
| `Eigen::VectorXd getFirstPredictionPos()` / `Vel()` | predicted `q(t_0)`, `dq(t_0)` |
| `double getDt()` | control period |
| `unsigned int getNumObstaclePoints() const` | `n_obstacle_points` (`getNumPh()` is deprecated) |
| `bool needUpdate()` | settings changed, matrices must be rebuilt |
| `bool arePositionBoundsActive()` | `position_bounds` |
| `virtual void updateMatrices()` | rebuild the constant matrices (needed only after legacy setters) |
| `virtual bool computedUncostrainedSolution(...)` | minimize the cost ignoring all constraints (debugging) |
| `ThorQP& operator=(const ThorQP&)` | deep copy |

**Extending.** Subclasses can override these protected virtual functions:

- `doSolve(const ThorInput&, ThorOutput&)`: the constrained solver. It is
  called by `solve()` with validated, completed inputs.
- `computeActualMatrices(...)`: the per-cycle cost.
- `updateMatrices()`: the constant matrices.

The protected data members are implementation details and may change.

### 4.6 Evolution matrices

These free functions are used by `ThorQP` and are public because they are
useful on their own. The state of `nax` joints is `[q; dq]`.

| Function | Result |
| --- | --- |
| `freeResponse(t, nax)` | `[I, tI; 0, I]`, `2nax x 2nax` |
| `forcedResponse(t, nax)` | `[t²/2 I; t I]`, `2nax x nax` |
| `computeEvolutionMatrix(pred_time, intervals, nax, free, forced)` | stacked `[q; dq](t_p) = free_p x0 + forced_p u`; false if a vector is empty |
| `computeJerkEvolutionMatrix(pred_time, intervals, nax, free, forced)` | `jerk = free u_prev + forced u`, with `jerk_i = (u_i - u_{i-1})/dt_i` |
| `splitResponses(free, vel_free, pos_free, forced, vel_forced, pos_forced, nax)` | separates position and velocity rows; `vel_free` multiplies `dq0` only |
| `quadraticControlIntervals(T, nc, first, intervals, pred_time)` | quadratically growing intervals |
| `constantControlIntervals(T, nc, dt, intervals, pred_time)` | equal intervals, multiple of `dt` |

`frameVelocityDerivatives(model, data, frame, dv_dq, dv_ddq)` returns the
Jacobians of the velocity of a frame origin in world coordinates with respect
to `q` and `dq`. It needs `pinocchio::computeForwardKinematicsDerivatives()`
and `updateFramePlacements()`. The linear rows that Pinocchio returns for
`LOCAL_WORLD_ALIGNED` are not this derivative, so it is built from the
`LOCAL` ones: `dv/dq_i = R dv_local/dq_i + omega_i × v`.

---

## 5. Reference: cbf.h

Namespace `thor::math::cbf`. For a robot sphere centered at `p_r` and an
obstacle sphere centered at `p_o` (for a capsule, the closest point of its
axis), `u = (p_r - p_o)/|p_r - p_o|` and `d` is the distance between the
surfaces. The barrier depends on
`psi = [d, v_r, v_o, a_o]`, the distance and the robot velocity, obstacle
velocity and obstacle acceleration projected on `u`. A negative `v_r` and a
positive `v_o` mean approaching.

### `double minimumDistance(d, v_r, v_o, a_o, T_r, a_s, atol, Eigen::Vector4d& jacobian)`

The minimum of the distance while the robot keeps its velocity for `T_r` and
then brakes at `a_s` until it stops, and the obstacle keeps `a_o`:

```
d(t) = d + (v_r - v_o) t - a_o t²/2 + [t > T_r] a_s (t - T_r)²/2
```

The candidates are:

- `t = 0`;
- `t = T_r` for a receding robot, or the stop time `T_r - v_r/a_s` for an
  approaching robot;
- the interior stationary points.

`jacobian = ∂d_min/∂psi` uses the envelope theorem, plus the dependence of the
stop time on `v_r`. The result can be negative, which means a contact is
unavoidable. Throws `std::invalid_argument` if `a_s <= 0`.

### `double barrier(d, v_r, v_o, a_o, T_r, a_s, C, atol, Eigen::Vector4d& jacobian, C_escape = 0)`

`h = d_min - C`. If `d_min < C`, `h += (C - d_min)/C_e · T_r · v_r`, with
`C_e = C_escape` (or `C` if it is 0), so that inside the margin a receding
robot raises `h` and can escape. The term starts at `d_min = C`; `C_e` only
sets its slope. Inside the margin `∂h/∂d_min = 1 - T_r v_r / C_e`, negative
for `v_r > C_e / T_r`. `jacobian` is `∂h/∂psi`.

### `BarrierTerms barrierTerms(p_r, v_r, p_o, v_o, a_o, T_r, a_s, C, radius = 0, atol = 1e-12, bounds = {}, C_escape = 0)`

Returns `h`, `lie_f`, `lie_g`, `grad_p`, `h_chi`, `dh_dpsi`, `d`, `v_r`,
`v_o`, `a_o`, `psi_bound` and `rate_bound`.

- **Spheres.** `radius` is the sum of the radii (plus `delta_H` for the
  worst-case distance): `d = |p_r - p_o| - radius`, negative if the spheres
  overlap.
- **Lie derivatives.** Along the motion, `dh/dt = lie_f + lie_g · a_r`, where
  `a_r` is the robot center acceleration. They account for the rotation of
  `u`, `du/dt = (I - u uᵀ)(v_r - v_o)/|p_r - p_o|`, and for the obstacle
  acceleration.
- **`h_chi`.** `h_chi = ∂h/∂d`: it is 1 outside the margin and larger inside.
- **Coincident centers.** If `|p_r - p_o| <= atol`, `u` is the x axis.
- **Perception bounds.** `bounds` (`PerceptionBounds{p, v, a}`) are
  half-widths on the obstacle position, velocity and acceleration. They map
  to `psi_bound = (b_p, b_p|P v_r|/D, b_v + b_p|P v_o|/D, b_a + b_p|P a_o|/D)`,
  with `P = I - u uᵀ` and `D = |p_r - p_o|`: the position error also
  rotates `u`. All the outputs are at the corner
  `(d - b_0, v_r - b_1, v_o + b_2, a_o + b_3)`, which is the minimum of
  `d_min` over the box. It is also the minimum of `h` unless
  `v_r > C_e / T_r` there. `rate_bound` bounds the error of `dh/dt` when
  the true state is elsewhere in the box.

### `BarrierTerms capsuleBarrierTerms(p_r, v_r, p_o, half_axis, v_o, a_o, T_r, a_s, C, radius = 0, atol = 1e-12, bounds = {}, C_escape = 0)`

As `barrierTerms()`, for an obstacle capsule: a cylinder with hemispherical
ends around the segment from `p_o - half_axis` to `p_o + half_axis`, which
translates with `v_o` and `a_o` without rotating. `p_o` and `u` refer to the
point `c` of the axis closest to `p_r`, `d = |p_r - c| - radius`.

- **Beyond an end.** `c` is that end: the result is `barrierTerms()` of the
  sphere centered there.
- **Along the side.** `c` slides on the axis: the result is `barrierTerms()`
  of `p_r`, `v_r`, `p_o`, `v_o` and `a_o` projected on the plane orthogonal
  to the axis. It is exact, since the motion along the axis does not change
  the distance; `lie_g` and `grad_p` lie in that plane.
- **Continuity.** `d` and its gradient are continuous across the regions.
- **Degenerate cases.** A zero `half_axis` is a sphere (`barrierTerms()`). If
  `p_r` is on the axis, `u` is a direction orthogonal to it.

### `double perceptionMargin(beta, sigma_p, sigma_v, sigma_a, T_f_bar)`

`delta_H = beta (sigma_p + sigma_v T_f_bar + sigma_a T_f_bar²/2)` (Eq. 28).
It is the distance error accumulated over the stopping time `T_f_bar` by
noisy position, velocity and acceleration estimates. `beta = 3.291` gives
99.9% confidence for Gaussian noise. Pass the result as
`ThorConfig::delta_H` or `ThorInput::obstacle_delta_H`: the barrier is then
evaluated at the worst-case distance `d - delta_H`.

---

## 6. Reference: eigen_matrix_utils.h

Namespace `eigen_utils`.

| Function | Result |
| --- | --- |
| `standard_deviation(v)` | population standard deviation `sqrt(Σ(v - mean)²/n)` |
| `correlation(v1, v2)` | Pearson correlation in `[-1, 1]`; 0 if either vector is constant |
| `factorial(n)` | `n!` as `unsigned int` (overflows for `n > 12`) |

---

## 7. Migrating from the legacy API

The legacy setters still compile with a deprecation warning. They skip
validation and need a manual `updateMatrices()`.

| Legacy call | Replacement |
| --- | --- |
| `setIntervals(nc, nax, T, dt[, blocking])` | `n_intervals`, `n_joints`, `horizon_time`, `sampling_period`, `input_blocking` |
| `setWeigthFunction(acc, tau, jerk, scaling, clik)` | `lambda_acc`, `lambda_tau`, `lambda_jerk`, `lambda_scaling`, `lambda_clik` |
| `setConstraints(qmax, qmin, Dqmax, DDqmax, tau_max)` | `qmax`, `qmin`, `Dqmax`, `DDqmax`, `tau_max` |
| `activatePositionBounds(b)` | `position_bounds` |
| `activateCbfBounds(b)` | `cbf` |
| `setCBFParameters(a_s, T_r, C, gamma)` | `a_s`, `T_r`, `C`, `gamma` (+ `delta_H`) |
| `setCbfIds(ids, n)` | `cbf_frame_ids`, `n_obstacle_points` (+ `cbf_frame_radii`) |
| `activateTorqueBounds(b)` | none (not implemented) |
| `activateCbfMoveAway(b)` | none (handled by `d_min`) |
| `updateMatrices()` | `configure(config)` |
| `computedCostrainedSolution(dq, q, s, x0, acc, scaling, v_o, p_o)` | `solve({dq, q, s, x0, p_o, v_o, a_o, radius, delta_H})` — note the order of positions and velocities |
| `getNumPh()` | `getNumObstaclePoints()` |

The legacy result vector `{h_min, interval, frame_id, distance, v_robot,
v_obstacle}` corresponds to the fields of `ThorOutput::cbf`.

---

## 8. Differences from the paper and cbf_python

`thor_math` follows the CBF of the paper and of
[`cbf_python`](https://github.com/JRL-CARI-CNR-UNIBS/cbf_python/tree/dynamic_parameters).
The kernels in `cbf.h` reproduce its values to 1e-12. The differences are:

- **MPC instead of one step.** `cbf_python` solves one control period with the
  acceleration of the time scaling as input. `ThorQP` keeps its multi-interval
  MPC with the time scaling as a decision variable, and imposes the CBF on
  every interval.
  - By default it uses the discrete-time formulation, which constrains the
    barrier at the predicted states.
  - The continuous formulation is the CBF of the paper; with
    `n_intervals = 1` the CBF row is the same.
- **Robust term.** The paper and `cbf_python` add `+ h_chi delta_H` to the
  condition, which relaxes it and adds metres to metres per second. `ThorQP`
  evaluates the barrier at the worst-case distance `d - delta_H` (the obstacle
  sphere is inflated by `delta_H`).

  The first-order version of the same idea, `gamma (h - h_chi delta_H)`, is
  not used. It moves the boundary of the safe set away from `d_min = C`, where
  the escape term of the barrier gives the robot control authority. A robot
  stopped on that boundary would then make the QP infeasible forever.

  With `delta_H = 0` all three forms coincide. The deadlock and its fix are
  described in [robust_margin_deadlock.md](robust_margin_deadlock.md).

  The perception bounds (`obstacle_sigma`) extend the same idea to the
  velocity and acceleration: the barrier is evaluated at the worst corner of
  the box around the measured state, with the sensitivities at the actual
  time of the minimum distance instead of `T_f_bar`
  ([worst_case_h_design.md](worst_case_h_design.md)).
- **Obstacles, not only people, and spheres, not only points.** The paper
  monitors points of a person with one margin `C`. `ThorQP` takes any obstacle
  spheres, with a radius per robot frame and per obstacle and a `delta_H` per
  obstacle; `C` is the margin between the surfaces.
- **Supervisory logic not included.** The trajectory tube, the missed-deadline
  handling, the fallback braking and the tube recovery of `cbf_python` are left
  to the caller. The example shows the fallback braking.
