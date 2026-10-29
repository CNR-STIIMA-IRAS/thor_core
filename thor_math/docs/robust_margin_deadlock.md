# Deadlock of the robust CBF term, and its fix

This note explains a deadlock found in the robust control barrier function
(CBF) of `ThorQP`, why it happens, and how it was fixed. The CBF is the one of
the paper and of
[`cbf_python`](https://github.com/JRL-CARI-CNR-UNIBS/cbf_python/tree/dynamic_parameters);
see [api.md](api.md) for the full formulation.

**In short.** The perception margin `delta_H` was applied to the CBF
condition as the first-order term `gamma (h - h_chi delta_H)`. That moves the
boundary of the safe set without moving the part of the barrier that lets a
stopped robot escape. A robot that stops on that boundary has no control
authority over the barrier, so the QP becomes infeasible and stays infeasible
forever. The fix evaluates the barrier at the worst-case distance
`d - delta_H`, which means inflating the obstacle sphere by `delta_H`. This
moves the boundary and the escape term together.

---

## 1. Background

For a robot sphere and an obstacle sphere, `d` is the distance between the
surfaces and `u` the unit vector from the obstacle to the robot. The barrier
depends on `psi = [d, v_r, v_o, a_o]`: the distance, plus the robot velocity,
obstacle velocity and obstacle acceleration projected on `u`.

```
d_min = min over t of  d + (v_r - v_o) t - a_o t²/2 + [t > T_r] a_s (t - T_r)²/2
h     = d_min - C                                    if d_min >= C
h     = d_min - C + (C - d_min)/C · T_r · v_r        if d_min <  C   (escape term)
```

Here `d_min` is the minimum distance while the robot reacts for `T_r` and
then brakes at `a_s`, with the obstacle keeping its acceleration. The CBF
condition imposed on the robot acceleration `a_r` is

```
dh/dt + gamma h = L_f h + L_g h · a_r + gamma h >= 0,      L_g h = (∂h/∂v_r) u
```

The robot acceleration enters `dh/dt` only through `v_r`. So the controller
can influence the barrier only where `∂h/∂v_r ≠ 0`.

### When `∂h/∂v_r = 0`

Take a robot at rest or moving away (`v_r >= 0`) from an obstacle that is not
approaching faster. The distance does not decrease, so its minimum is now:
`d_min = d` at `t = 0`.

In that case:

- `d_min` does not depend on `v_r`;
- if `d_min >= C`, the escape term is off, so `∂h/∂v_r = 0` and `L_g h = 0`.

The CBF row then no longer involves the acceleration. It becomes a condition
on the state alone, `L_f h + gamma h >= 0`.

This is harmless when `h >= 0`, because the row is then satisfied. The escape
term exists for exactly this region: when `d_min` falls below `C`, it makes
`∂h/∂v_r = (C - d_min) T_r / C > 0`. The robot regains control authority
right at the boundary `h = 0` (`d_min = C`), and can always accelerate away.

## 2. The faulty robust term

The distance is estimated with an error of at most `delta_H` (Eq. 9 of the
paper, `cbf::perceptionMargin()`). The paper and `cbf_python` add
`+ h_chi delta_H` to the condition, with `h_chi = ∂h/∂d`. This is wrong in
two ways: it relaxes the constraint instead of tightening it, and it adds a
distance (m) to a rate (m/s).

The first correction, now superseded, flipped it into the first-order robust
term:

```
L_f h + L_g h · a_r + gamma (h - h_chi delta_H) >= 0
```

This keeps `h >= h_chi delta_H`, i.e. a non-negative barrier for the
worst-case distance, but only to first order. Outside the margin
`h_chi = 1`, so the new boundary of the safe set is `d_min = C + delta_H`.

**The new boundary is outside the region of the escape term** (`d_min < C`).
On that boundary, a robot at rest has `∂h/∂v_r = 0`, and the CBF row
degenerates to

```
0 · a_r  >=  -(L_f h + gamma (h - h_chi delta_H))
```

With a static obstacle and the robot at rest, `L_f h = 0`, so the row
requires `h >= h_chi delta_H` and no acceleration can change it.

Discrete time and the linearization make tiny violations of the boundary
unavoidable. When a violation as small as 1e-5 m occurs:

1. the QP is infeasible;
2. the controller falls back to braking, so the robot stays at rest;
3. at rest the row is the same and still violated, so the QP is infeasible
   again, forever.

The robot stays blocked even when the nominal trajectory turns away from the
obstacle, because every command, including moving away, is rejected.

## 3. How it showed up

The closed-loop example
([examples/cbf_mpc_example.cpp](../examples/cbf_mpc_example.cpp)) runs a UR10
covered by 5 spheres. The scene has a person (torso r = 0.20 m, hand
r = 0.06 m) and a static fixture. The hand has the largest perception error,
`delta_H = 3.291 · (0.03 + 0.15 · 0.3 + 0.4 · 0.3²/2) ≈ 0.306 m`.

With the first-order term, the robot stopped against the hand at
`t = 2.752 s`. A diagnostic build printed the pairs at the boundary at the
first infeasible solve:

| robot sphere | obstacle | `h` | `h - h_chi delta_H` | `\|L_g h\|` | `v_r` |
|---|---|---|---|---|---|
| wrist_1_link | hand | 0.3061 | −0.0000 | 0.150 | −0.0001 |
| wrist_3_link | hand | 0.3061 | −0.0000 | 0.150 | −0.0001 |
| tool0 | hand | 0.3061 | −0.0000 | 0.150 | −0.0001 |

The robot is practically at rest, exactly on the robust boundary. From then
on every solve failed.

To check that the cause is the robust term and not the scene, I ran the same
scene with three settings of `delta_H`:

| `delta_H` | infeasible solves (of 3000) | trajectory progress | min. clearance hand / fixture |
|---|---|---|---|
| per obstacle, hand 0.306 m (first-order term) | 1624, from t = 2.75 s | 2.74 s of 6 | 0.356 / 0.434 m |
| 0.149 m for torso and hand (first-order term) | 1434, from t = 3.13 s | 3.11 s of 6 | 0.199 / 0.434 m |
| 0 | 0 | 6 s of 6 | 0.056 / 0.056 m |

With `delta_H = 0`, the boundary is `d_min = C`, where the escape term acts,
and the robot moves around both obstacles. With any `delta_H > 0`, the robot
deadlocks.

The same structure is in `cbf_python`, whatever the sign of the term: the
barrier and the escape term are the same, and only the boundary moves. With
`+ h_chi delta_H` the boundary moves inwards, so the robot reaches it later,
with less clearance than intended. It can still stop at rest on it, with the
same consequence.

## 4. The fix: evaluate the barrier at the worst-case distance

The true distance is at least `d - delta_H`. Instead of linearizing, the
barrier is evaluated exactly at that distance:

```
h_rob(d, v_r, v_o, a_o) = h(d - delta_H, v_r, v_o, a_o)
condition:  L_f h_rob + L_g h_rob · a_r + gamma h_rob >= 0
```

- **Simple.** `d - delta_H` is the distance between the robot sphere and the
  obstacle sphere inflated by `delta_H`. The implementation is a single line:
  `delta_H` is added to the sum of the radii passed to `cbf::barrierTerms()`.
- **Exact.** `delta_H` is constant during a solve, so the Lie derivatives of
  `h_rob` are those of `h` evaluated at the shifted distance. The geometric
  terms, such as the rotation of `u`, still use the distance between the
  centers.
- **Removes the deadlock.** The boundary of the safe set is now
  `d_min(d - delta_H) = C`, and the escape term activates exactly there, with
  `∂h_rob/∂v_r > 0` just inside. A robot at rest on the robust boundary can
  always accelerate away.
- **Same idea, done right.** To first order `h(d - delta_H) ≈ h - h_chi
  delta_H`, the term that was used before. Outside the margin the two coincide
  exactly, since `h_chi = 1` and the shift equals enlarging `C` to
  `C + delta_H`. The difference is only where it matters: the escape term
  moves with the boundary.

The worst case is exact where `h` grows with the distance, i.e.
`h_chi > 0`. That covers every case except a robot leaving the margin faster
than `C / T_r`, which is already moving away.

### Changes in the code

| File | Change |
|---|---|
| `src/thor_math/thor_math.cpp` | In `doSolve()`, `radius = r_robot + r_obstacle + delta_H`. The CBF offset is `L_g h · dJ dq + L_f h + gamma h` (no `h_chi delta_H` term). `CbfDiagnostics::distance` adds `delta_H` back, so it reports the estimated distance between the surfaces. |
| `include/thor_math/thor_math.h`, `cbf.h` | Documentation of `delta_H`, `obstacle_delta_H`, `CbfDiagnostics` and `perceptionMargin()`. |
| `test/test_thor_math.cpp` | Integration tests use the worst-case distance. New regression test `RobotAtRestInsideTheRobustBandCanMoveAway`. |
| `examples/cbf_mpc_example.cpp` | Comments on the CBF condition. The summary reports the clearance per obstacle, instead of the predicted barrier value (see section 6). |
| `docs/api.md`, `CHANGELOG.md` | Formulation and differences from the paper. |

## 5. Verification

**Regression test.** `RobotAtRestInsideTheRobustBandCanMoveAway` places a
robot at rest 2 mm inside the robust band of a static obstacle
(`delta_H = 0.15 m`). It checks that the QP is solved and that the command
moves the tool away from the obstacle. Against the first-order term, the same
test fails with `Infeasible inequality constraints`. I checked this by
temporarily restoring the old code, then restored the fix.

**Example, same scene as section 3:**

| | infeasible solves | progress | min. clearance torso / hand / fixture |
|---|---|---|---|
| first-order term | 1624 | 2.74 s of 6 | 0.543 / 0.356 / 0.434 m |
| worst-case distance (fix) | 0 | 5.97 s of 6 | 0.476 / 0.355 / 0.064 m |
| no CBF | – | 6 s of 6 | 0.170 / −0.057 / −0.041 m (collision) |

With the fix, the clearances match the intended `C + delta_H`:

- hand: 0.355 m, against `0.05 + 0.306 = 0.356 m`;
- fixture: 0.064 m, against `0.05 + 0.01 = 0.06 m`.

**Other checks.** All 43 unit and integration tests pass. The kernel tests
check the Lie derivatives against finite differences for points and for
spheres, and still match the `cbf_python` reference values to 1e-12 (with
`delta_H = 0` nothing changes).

## 6. What remains open

- **Predicted barrier values.** For intervals after the first, the CBF uses
  the state predicted in the previous cycle. The CBF of interval `i`
  constrains `dh/dt` at the start of the interval, not `h` in the predicted
  state, so `CbfDiagnostics::h_min` can be briefly negative. In the example
  this happens on the last interval only, while the actual clearance stays
  above `C + delta_H`. This is a property of the MPC, not of the robust term.
- **Infeasibility for other reasons.** Other causes can still make the QP
  infeasible, for example an obstacle faster than the robot can react, or
  conflicts between several CBF rows and the joint limits. The caller then
  brakes. A softened CBF, with a heavily penalized slack, would keep the QP
  feasible at the cost of the formal guarantee; it is worth discussing
  together with the per-pair activation of the CBF rows.
- **The paper.** The robust term of the paper (`+ h_chi delta_H`) should be
  replaced by the worst-case distance. Its first-order form
  (`- h_chi delta_H`) should not be used either, for the reason above.
