/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Nicola Pedrocchi
 * All rights reserved.
 *
 * See the project license for redistribution terms and warranty disclaimer.
 */

/**
 * @file cbf.h
 * @brief Control barrier function for speed and separation monitoring with
 *        dynamic obstacles.
 *
 * C++ port of `cbf_python/controllers/kernels/ssm_cbf_acc.py`
 * (JRL-CARI-CNR-UNIBS/cbf_python, branch `dynamic_parameters`), checked
 * against its output to 1e-12. The paper considers people; the formulation
 * holds for any obstacle point that keeps its acceleration while the robot
 * stops (a person, a vehicle, another robot, a static object).
 *
 * The robot point and the obstacle point are spheres of radii `r_r` and `r_o`
 * (zero for points); obstacles can also be capsules, see capsuleBarrierTerms().
 * With `u = (p_r - p_o)/|p_r - p_o|` the barrier depends on
 * `psi = [d, v_r, v_o, a_o]`:
 * - `d = |p_r - p_o| - r_r - r_o` distance between the surfaces [m];
 * - `v_r` robot velocity along `u` [m/s] (negative: approaching);
 * - `v_o` obstacle velocity along `u` [m/s] (positive: approaching);
 * - `a_o` obstacle acceleration along `u` [m/s^2].
 *
 * The robot point acceleration is the control input. ThorQP maps it to the
 * joint accelerations with the frame Jacobian: `a_r = J ddq + dJ dq`.
 *
 * The obstacle state can be uncertain (PerceptionBounds). The barrier is then
 * evaluated at the worst corner of the box around the measured `psi`, see
 * barrierTerms() and docs/worst_case_h_design.md.
 */

#ifndef THOR_MATH__THOR_MATH__CBF__H
#define THOR_MATH__THOR_MATH__CBF__H

#include <Eigen/Dense>

namespace thor
{
namespace math
{
/// Control barrier function kernels (see cbf.h).
namespace cbf
{

/**
 * @brief Minimum robot-obstacle distance over the robot stopping manoeuvre.
 *
 * The robot keeps its velocity for the reaction time @p T_r, then brakes
 * with deceleration @p a_s until it stops; the obstacle keeps the constant
 * acceleration @p a_o. Along `u` the distance is
 * `d(t) = d + (v_r - v_o) t - a_o t^2/2 (+ a_s (t - T_r)^2/2 for t > T_r)`.
 * The minimum is searched among the start, the end of the reaction (receding
 * robot) or the stop (approaching robot), and the interior stationary points.
 *
 * @param d, v_r, v_o, a_o  components of `psi`.
 * @param T_r     reaction time [s] (>= 0).
 * @param a_s     robot braking deceleration [m/s^2] (> 0).
 * @param atol    tolerance to compare times and to skip null denominators.
 * @param[out] jacobian `d(d_min)/d(psi)`, from the envelope theorem plus the
 *        dependence of the stopping time on `v_r`.
 * @return `d_min` [m]; it can be negative if a contact cannot be avoided.
 * @throws std::invalid_argument if @p a_s is not positive.
 */
double minimumDistance( double d, double v_r, double v_o, double a_o,
                        double T_r, double a_s, double atol,
                        Eigen::Vector4d& jacobian );

/**
 * @brief Barrier function `h(psi)`.
 *
 * `h = d_min - C`. When `d_min < C` the escape term
 * `(C - d_min)/C_e * T_r * v_r` is added, so that inside the margin a robot
 * moving away (`v_r > 0`) increases `h` and is allowed to escape. The term
 * starts at `d_min = C`, the boundary of the safe set; `C_e` only sets its
 * slope. Inside the margin `dh/d(d_min) = 1 - T_r v_r / C_e`, which is negative
 * for `v_r > C_e / T_r`.
 *
 * @param d, v_r, v_o, a_o, T_r, a_s, atol  as in minimumDistance().
 * @param C       static safety margin [m] (> 0).
 * @param[out] jacobian `dh/d(psi)`.
 * @param C_escape scale `C_e` of the escape term [m] (>= 0); 0 means `C`.
 * @return `h` [m]; the safe set is `h >= 0`.
 * @throws std::invalid_argument if @p C_escape is negative.
 */
double barrier( double d, double v_r, double v_o, double a_o,
                double T_r, double a_s, double C, double atol,
                Eigen::Vector4d& jacobian, double C_escape = 0.0 );

/**
 * @brief Half-widths of the box of possible true obstacle states around the
 *        measured one.
 *
 * Isotropic bounds on the obstacle position, velocity and acceleration, e.g.
 * `k sigma` for `k` standard deviations. The robot state is taken as exact.
 * The barrier maps them to bounds on `psi`, see BarrierTerms::psi_bound.
 */
struct PerceptionBounds
{
  double p = 0.0;  ///< Obstacle position bound [m] (>= 0).
  double v = 0.0;  ///< Obstacle velocity bound [m/s] (>= 0).
  double a = 0.0;  ///< Obstacle acceleration bound [m/s^2] (>= 0).
};

/**
 * @brief Barrier value and Lie derivatives of one robot-obstacle pair, from
 *        barrierTerms() or capsuleBarrierTerms().
 *
 * All values are at the worst corner of the box around the measured `psi`
 * (#psi_bound); without perception bounds that is the measured `psi`.
 */
struct BarrierTerms
{
  double          h     = 0.0;  ///< Barrier value [m].
  double          lie_f = 0.0;  ///< `L_f h`: rate of `h` with zero robot point acceleration [m/s].
  Eigen::Vector3d lie_g = Eigen::Vector3d::Zero();  ///< `L_g h`: `dh/dt = L_f h + L_g h . a_r` [s]. It is also `dh/dv_r`, the gradient with respect to the robot center velocity.
  Eigen::Vector3d grad_p = Eigen::Vector3d::Zero(); ///< `dh/dp_r`: gradient with respect to the robot center position, obstacle fixed.
  double          h_chi = 0.0;  ///< `dh/dd`, sensitivity of `h` to a distance error.
  Eigen::Vector4d dh_dpsi = Eigen::Vector4d::Zero();  ///< `dh/d(psi)`; `h_chi` is its first component.
  double          d     = 0.0;  ///< Distance between the surfaces [m] (negative if they overlap); measured: `d + psi_bound(0)`.
  double          v_r   = 0.0;  ///< Robot velocity along `u` [m/s]; measured: `v_r + psi_bound(1)`.
  double          v_o   = 0.0;  ///< Obstacle velocity along `u` [m/s]; measured: `v_o - psi_bound(2)`.
  double          a_o   = 0.0;  ///< Obstacle acceleration along `u` [m/s^2]; measured: `a_o - psi_bound(3)`.
  Eigen::Vector4d psi_bound = Eigen::Vector4d::Zero();  ///< Half-widths of the box around the measured `psi`; the worst corner is `psi - (b_0, b_1, -b_2, -b_3)`.
  /**
   * @brief Bound of the error of `dh/dt = L_f h + L_g h . a_r` when the true
   *        `psi` is elsewhere in the box [m/s] (>= 0); 0 without bounds.
   *
   * The robust barrier is `h` at the corner, so its rate is
   * `dh/dpsi(corner) . dpsi/dt(true)`, while #lie_f uses `dpsi/dt(corner)`. To
   * first order, with errors that do not change over time:
   * - the control part `(dh/dv_r) u . a_r` is the same, since `u` is;
   * - the true velocities and acceleration differ from the corner by
   *   `[0, 2 b]` in the favourable direction: they contribute only where
   *   `dh/dd < 0` or `dh/dv_o > 0`, i.e. where the escape term has a negative
   *   slope;
   * - the rotation terms of `dpsi/dt`, which scale with `1/|p_r - p_o|`,
   *   change with the distance error `+-b_d`.
   *
   * It is a rate: subtract it from the CBF condition without multiplying it
   * by `gamma`. It does not cover errors that change over time, nor the
   * rotation of `u` caused by the position error.
   */
  double          rate_bound = 0.0;
};

/**
 * @brief Barrier value and Lie derivatives of a robot sphere and an obstacle
 *        sphere.
 *
 * The barrier uses the distance between the surfaces,
 * `d = |p_r - p_o| - radius`. The Lie derivatives include the rotation of `u`
 * (`du/dt = (I - u u^T)(v_r - v_o)/|p_r - p_o|`) and the obstacle
 * acceleration; since the radii are constant they are the same as for points.
 * The CBF condition used by ThorQP is `L_f h + L_g h . a_r + gamma h >= 0`,
 * with @p radius including the distance estimation error `delta_H`: the
 * barrier is evaluated at the worst-case distance.
 *
 * With perception bounds, the box around the measured `psi` has half-widths
 * (to first order in the position bound, which also rotates `u`):
 * - `b_d   = b_p`;
 * - `b_v_r = b_p |P v_r| / |p_r - p_o|`;
 * - `b_v_o = b_v + b_p |P v_o| / |p_r - p_o|`;
 * - `b_a_o = b_a + b_p |P a_o| / |p_r - p_o|`;
 *
 * with `P = I - u u^T`. `d_min` is non-decreasing in `d` and `v_r` and
 * non-increasing in `v_o` and `a_o`, so its minimum over the box is at the
 * corner `(d - b_d, v_r - b_v_r, v_o + b_v_o, a_o + b_a_o)`. The barrier and
 * its Lie derivatives are evaluated there, by moving the velocities along `u`
 * and inflating the radius. The corner is also the minimum of `h`, unless the
 * escape term has a negative slope in the box (`v_r > C_e / T_r`, see
 * barrier()). The bounds are frozen: their own dependence on the state is
 * left out of the Lie derivatives and of #grad_p.
 *
 * If the centers coincide (`|p_r - p_o| <= atol`), `u` is the x axis.
 *
 * @param p_r, v_r       robot sphere center position [m] and velocity [m/s].
 * @param p_o, v_o, a_o  obstacle sphere center position, velocity and acceleration.
 * @param T_r, a_s, C    as in barrier().
 * @param radius         sum of the two radii `r_r + r_o` [m] (0 for points).
 * @param atol           as in minimumDistance().
 * @param bounds         obstacle perception bounds; zero means an exact state.
 * @param C_escape       as in barrier().
 */
BarrierTerms barrierTerms( const Eigen::Vector3d& p_r, const Eigen::Vector3d& v_r,
                           const Eigen::Vector3d& p_o, const Eigen::Vector3d& v_o,
                           const Eigen::Vector3d& a_o,
                           double T_r, double a_s, double C,
                           double radius = 0.0, double atol = 1e-12,
                           const PerceptionBounds& bounds = PerceptionBounds(),
                           double C_escape = 0.0 );

/**
 * @brief Barrier value and Lie derivatives of a robot sphere and an obstacle
 *        capsule (a cylinder with hemispherical ends).
 *
 * The capsule axis is the segment from `p_o - half_axis` to `p_o + half_axis`;
 * the obstacle translates rigidly with @p v_o and @p a_o, without rotating.
 * The barrier uses the distance between the surfaces, `d = |p_r - c| - radius`,
 * where `c` is the point of the axis closest to `p_r`, and `u = (p_r - c)/|p_r - c|`:
 * - beyond an end, `c` is that end and the result is barrierTerms() of the
 *   sphere centered there;
 * - along the side, `c` slides on the axis: the result is barrierTerms() of
 *   the positions, velocities and accelerations projected on the plane
 *   orthogonal to the axis. It is exact, since the robot motion along the axis
 *   does not change the distance.
 *
 * `d` and its gradient are continuous across the two regions. A capsule with
 * a long axis models a cylinder whose ends are never reached (e.g. a person
 * from the floor to the head, or a column).
 *
 * If @p half_axis is (numerically) zero this is barrierTerms(). If `p_r` is on
 * the axis, `u` is a direction orthogonal to it.
 *
 * @param p_r, v_r       robot sphere center position [m] and velocity [m/s].
 * @param p_o            capsule center (axis midpoint) [m].
 * @param half_axis      vector from the center to one end of the axis [m].
 * @param v_o, a_o       capsule velocity [m/s] and acceleration [m/s^2].
 * @param T_r, a_s, C    as in barrier().
 * @param radius         sum of the robot sphere and capsule radii `r_r + r_o` [m].
 * @param atol           as in minimumDistance().
 * @param bounds, C_escape  as in barrierTerms(); along the side the bounds
 *        apply to the projected vectors.
 */
BarrierTerms capsuleBarrierTerms( const Eigen::Vector3d& p_r, const Eigen::Vector3d& v_r,
                                  const Eigen::Vector3d& p_o, const Eigen::Vector3d& half_axis,
                                  const Eigen::Vector3d& v_o, const Eigen::Vector3d& a_o,
                                  double T_r, double a_s, double C,
                                  double radius = 0.0, double atol = 1e-12,
                                  const PerceptionBounds& bounds = PerceptionBounds(),
                                  double C_escape = 0.0 );

/**
 * @brief Maximum error of the estimated robot-obstacle distance (Eq. 28 of the paper).
 *
 * `delta_H = beta (sigma_p + sigma_v T_f_bar + sigma_a T_f_bar^2 / 2)`: the
 * position error accumulated over the stopping time by noisy position,
 * velocity and acceleration estimates.
 *
 * @param beta    confidence coefficient (3.291 for 99.9% of a Gaussian).
 * @param sigma_p position noise standard deviation [m].
 * @param sigma_v velocity noise standard deviation [m/s].
 * @param sigma_a acceleration noise standard deviation [m/s^2].
 * @param T_f_bar upper bound of the stopping time [s].
 * @return `delta_H` [m], to be used as ThorConfig::delta_H or
 *         ThorInput::obstacle_delta_H. The barrier is evaluated at the
 *         worst-case distance `d - delta_H`, which is the same as inflating the
 *         obstacle (sphere or capsule) by `delta_H`.
 *
 * This is the margin with the stopping time bounded by @p T_f_bar. The
 * perception bounds of barrierTerms() give the same sensitivities at the
 * actual time of the minimum distance, and also cover the velocities; do not
 * use both for the same noise.
 */
double perceptionMargin( double beta, double sigma_p, double sigma_v, double sigma_a, double T_f_bar );

}  // namespace cbf
}  // namespace math
}  // namespace thor

#endif  // THOR_MATH__THOR_MATH__CBF__H
