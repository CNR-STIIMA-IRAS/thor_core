# thor_math API reference

`thor_math` provides `thor::math::ThorQP`, a model predictive controller (MPC)
for robot manipulators. It scales the speed of a nominal joint trajectory and,
optionally, keeps a safe distance from people through a control barrier
function (CBF).

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
config.cbf_frame_ids  = {static_cast<unsigned int>(model.getFrameId("tool0"))};
config.n_human_points = 1;

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
  in.human_pos = {p_h};
  in.human_vel = {v_h};
  in.human_acc = {a_h};                    // optional

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
| CBF, one row per (interval, frame, human point) | `cbf` |

The invariance rows limit the velocity towards a position limit, so that the
robot can always stop before reaching it.

### 2.6 Control barrier function

For each monitored robot frame (`cbf_frame_ids`) and human point:

- **Barrier.** `h = d_min - C`, where `d_min` is the minimum distance reached
  if the robot reacts after `T_r` and then brakes at `a_s`, while the person
  keeps its acceleration (section 5). The safe set is `h >= 0`.
- **Condition.** Imposed at the start of interval `i`:

  ```
  L_g h (J u_i + dJ dq) + L_f h + gamma (h - h_chi delta_H) >= 0
  ```

  - `L_f h` and `L_g h` are the Lie derivatives of `h` (`cbf::barrierTerms()`);
    `J u_i + dJ dq` is the acceleration of the frame.
  - `h_chi = dh/dd`. Since the true distance is at least `d - delta_H`, the
    condition keeps `h - h_chi delta_H >= 0`: the barrier is non-negative
    even for the worst-case distance.
- **Robot state per interval.** For `i = 0` it is the current state `x0`. For
  `i > 0` it is the state at `t_{i-1}` of the previous prediction, which
  `solve()` stores. With `n_intervals = 1` the constraint is the one-step
  controller of the paper.
- **Human prediction.** The human points are extrapolated to the start of
  each interval with constant acceleration.
- **Activation distance.** If the lowest `h` over the horizon is above
  `cbf_activation_distance`, the CBF rows are dropped from the QP: the
  solution is the same, and the QP is smaller.

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
satisfies all constraints, e.g. a person moving faster than the robot can
react. The controller must then apply a safe command itself. For example,
brake within the limits, `acc = clamp(-dq / dt, -DDqmax, DDqmax)`, and freeze
the trajectory time.

**Thread safety.** `ThorQP` keeps per-cycle state and Pinocchio buffers, so it
is not thread-safe. Use one instance per control loop. Instances can be copied
with `operator=`; the copy includes the model, the configuration, the state
and the last solution.

**Cost per cycle.** It grows with
`n_intervals * (n_joints + 1)` variables and with
`n_intervals * frames * human points` CBF rows; there is one kinematics
evaluation per interval. With 6 joints, 5 intervals, 5 frames and 2 points,
a solve takes about 50–150 µs in Release builds (see the example).

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
| `a_s` | 0 | robot braking deceleration [m/s²] | > 0 (CBF) |
| `T_r` | 0 | reaction time [s] | >= 0 (CBF) |
| `C` | 0 | safety margin [m] | > 0 (CBF) |
| `gamma` | 0 | CBF gain [1/s] | |
| `delta_H` | 0 | distance estimation error [m] | >= 0 (CBF) |
| `cbf_activation_distance` | 5.0 | drop the CBF rows above this `h` [m] | |
| `cbf_frame_ids` | empty | Pinocchio frame ids of the robot points | < `model.frames.size()` |
| `n_human_points` | 0 | human points per cycle | >= 0 |

### 4.2 `struct ThorInput`

| Field | Size | Meaning |
| --- | --- | --- |
| `target_dq` | `n_joints*n_intervals` | unscaled target velocities, `[dq_0; ...; dq_{nc-1}]` |
| `next_target_q` | `n_joints` | target position at `t_0` |
| `target_scaling` | – | desired time scaling (default 1) |
| `x0` | `2*n_joints` | current state `[q; dq]` |
| `human_pos` | ≥ `n_human_points` | human point positions [m], world frame |
| `human_vel` | same as `human_pos` | velocities [m/s] |
| `human_acc` | empty or same | accelerations [m/s²]; empty means zero |

With the CBF disabled, the human fields are ignored. With the CBF enabled and
all three vectors empty, `n_human_points` still points at `(100, 100, 100)`
are used (nobody around). Only the first `n_human_points` points are used.

### 4.3 `struct ThorOutput`

| Field | Meaning |
| --- | --- |
| `next_acc` | joint accelerations for the next control period |
| `next_scaling` | time scaling of the first interval |
| `cbf` | `CbfDiagnostics` of the most critical pair (CBF enabled) |

### 4.4 `struct CbfDiagnostics`

The robot-human pair with the lowest `h` over the horizon. All fields are 0
with the CBF disabled.

| Field | Meaning |
| --- | --- |
| `h_min` | lowest barrier value [m] |
| `interval` | control interval |
| `frame_id` | Pinocchio frame id (`model.frames[frame_id].name`) |
| `distance` | distance `d` [m] |
| `v_rel` | robot velocity along `(p_r - p_h)/d` [m/s], negative when approaching |
| `vh_proj` | human velocity along the same direction, positive when approaching |
| `human_index` | index in `ThorInput::human_pos` |

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
| `double getNumPh()` | `n_human_points` |
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

---

## 5. Reference: cbf.h

Namespace `thor::math::cbf`. For a robot point `p_r` and a human point `p_h`,
`d = |p_r - p_h|` and `u = (p_r - p_h)/d`. The barrier depends on
`psi = [d, v_r, v_h, a_h]`, the distance and the robot velocity, human
velocity and human acceleration projected on `u`. A negative `v_r` and a
positive `v_h` mean approaching.

### `double minimumDistance(d, v_r, v_h, a_h, T_r, a_s, atol, Eigen::Vector4d& jacobian)`

The minimum of the distance while the robot keeps its velocity for `T_r` and
then brakes at `a_s` until it stops, and the human keeps `a_h`:

```
d(t) = d + (v_r - v_h) t - a_h t²/2 + [t > T_r] a_s (t - T_r)²/2
```

The candidates are:

- `t = 0`;
- `t = T_r` for a receding robot, or the stop time `T_r - v_r/a_s` for an
  approaching robot;
- the interior stationary points.

`jacobian = ∂d_min/∂psi` uses the envelope theorem, plus the dependence of the
stop time on `v_r`. The result can be negative, which means a contact is
unavoidable. Throws `std::invalid_argument` if `a_s <= 0`.

### `double barrier(d, v_r, v_h, a_h, T_r, a_s, C, atol, Eigen::Vector4d& jacobian)`

`h = d_min - C`. If `d_min < C`, `h += (C - d_min)/C · T_r · v_r`, so that
inside the margin a receding robot raises `h` and can escape. `jacobian` is
`∂h/∂psi`.

### `BarrierTerms barrierTerms(p_r, v_r, p_h, v_h, a_h, T_r, a_s, C, atol = 1e-12)`

Returns `h`, `lie_f`, `lie_g`, `h_chi`, `d`, `v_r`, `v_h` and `a_h`.

- **Lie derivatives.** Along the motion, `dh/dt = lie_f + lie_g · a_r`, where
  `a_r` is the robot point acceleration. They account for the rotation of
  `u`, `du/dt = (I - u uᵀ)(v_r - v_h)/d`, and for the human acceleration.
- **`h_chi`.** `h_chi = ∂h/∂d`: it is 1 outside the margin and larger inside.
- **Coincident points.** If `d <= atol`, `u` is the x axis.

### `double perceptionMargin(beta, sigma_p, sigma_v, sigma_a, T_f_bar)`

`delta_H = beta (sigma_p + sigma_v T_f_bar + sigma_a T_f_bar²/2)` (Eq. 9).
It is the distance error accumulated over the stopping time `T_f_bar` by
noisy position, velocity and acceleration estimates. `beta = 3.291` gives
99.9% confidence for Gaussian noise. Pass the result as
`ThorConfig::delta_H`.

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
| `setCbfIds(ids, n)` | `cbf_frame_ids`, `n_human_points` |
| `activateTorqueBounds(b)` | none (not implemented) |
| `activateCbfMoveAway(b)` | none (handled by `d_min`) |
| `updateMatrices()` | `configure(config)` |
| `computedCostrainedSolution(dq, q, s, x0, acc, scaling, v_h, p_h)` | `solve({dq, q, s, x0, p_h, v_h, a_h})` — note the order of positions and velocities |

The legacy result vector `{h_min, interval, frame_id, distance, v_rel,
vh_proj}` corresponds to the fields of `ThorOutput::cbf`.

---

## 8. Differences from the paper and cbf_python

`thor_math` follows the CBF of the paper and of
[`cbf_python`](https://github.com/JRL-CARI-CNR-UNIBS/cbf_python/tree/dynamic_parameters).
The kernels in `cbf.h` reproduce its values to 1e-12. The differences are:

- **MPC instead of one step.** `cbf_python` solves one control period with the
  acceleration of the time scaling as input. `ThorQP` keeps its multi-interval
  MPC with the time scaling as a decision variable. It imposes the CBF on every
  interval; with `n_intervals = 1`, the CBF row is the same.
- **Sign of the robust term.** The paper and `cbf_python` add `+ h_chi delta_H`,
  which relaxes the constraint and adds metres to metres per second. `ThorQP`
  uses `gamma (h - h_chi delta_H)`, the condition on the worst-case barrier.
  With `delta_H = 0` the two coincide.
- **Supervisory logic not included.** The trajectory tube, the missed-deadline
  handling, the fallback braking and the tube recovery of `cbf_python` are left
  to the caller. The example shows the fallback braking.
