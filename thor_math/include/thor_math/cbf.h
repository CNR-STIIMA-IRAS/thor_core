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
 * @brief Control barrier function for speed and separation monitoring.
 *
 * C++ port of `cbf_python/controllers/kernels/ssm_cbf_acc.py`
 * (JRL-CARI-CNR-UNIBS/cbf_python, branch `dynamic_parameters`), checked
 * against its output to 1e-12.
 *
 * For a robot point `p_r` and a human point `p_h`, with `d = |p_r - p_h|` and
 * `u = (p_r - p_h)/d`, the barrier depends on `psi = [d, v_r, v_h, a_h]`:
 * - `d`   distance [m];
 * - `v_r` robot velocity along `u` [m/s] (negative: approaching);
 * - `v_h` human velocity along `u` [m/s] (positive: approaching);
 * - `a_h` human acceleration along `u` [m/s^2].
 *
 * The robot point acceleration is the control input. ThorQP maps it to the
 * joint accelerations with the frame Jacobian: `a_r = J ddq + dJ dq`.
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
 * @brief Minimum robot-human distance over the robot stopping manoeuvre.
 *
 * The robot keeps its velocity for the reaction time @p T_r, then brakes
 * with deceleration @p a_s until it stops; the human keeps the constant
 * acceleration @p a_h. Along `u` the distance is
 * `d(t) = d + (v_r - v_h) t - a_h t^2/2 (+ a_s (t - T_r)^2/2 for t > T_r)`.
 * The minimum is searched among the start, the end of the reaction (receding
 * robot) or the stop (approaching robot), and the interior stationary points.
 *
 * @param d, v_r, v_h, a_h  components of `psi`.
 * @param T_r     reaction time [s] (>= 0).
 * @param a_s     robot braking deceleration [m/s^2] (> 0).
 * @param atol    tolerance to compare times and to skip null denominators.
 * @param[out] jacobian `d(d_min)/d(psi)`, from the envelope theorem plus the
 *        dependence of the stopping time on `v_r`.
 * @return `d_min` [m]; it can be negative if a contact cannot be avoided.
 * @throws std::invalid_argument if @p a_s is not positive.
 */
double minimumDistance( double d, double v_r, double v_h, double a_h,
                        double T_r, double a_s, double atol,
                        Eigen::Vector4d& jacobian );

/**
 * @brief Barrier function `h(psi)`.
 *
 * `h = d_min - C`. When `d_min < C` the term `(C - d_min)/C * T_r * v_r` is
 * added, so that inside the margin a robot moving away (`v_r > 0`) increases
 * `h` and is allowed to escape.
 *
 * @param d, v_r, v_h, a_h, T_r, a_s, atol  as in minimumDistance().
 * @param C       static safety margin [m] (> 0).
 * @param[out] jacobian `dh/d(psi)`.
 * @return `h` [m]; the safe set is `h >= 0`.
 */
double barrier( double d, double v_r, double v_h, double a_h,
                double T_r, double a_s, double C, double atol,
                Eigen::Vector4d& jacobian );

/// @brief Barrier value and Lie derivatives of one robot-human pair, from barrierTerms().
struct BarrierTerms
{
  double          h     = 0.0;  ///< Barrier value [m].
  double          lie_f = 0.0;  ///< `L_f h`: rate of `h` with zero robot point acceleration [m/s].
  Eigen::Vector3d lie_g = Eigen::Vector3d::Zero();  ///< `L_g h`: `dh/dt = L_f h + L_g h . a_r` [s].
  double          h_chi = 0.0;  ///< `dh/dd`, sensitivity of `h` to a distance error.
  double          d     = 0.0;  ///< Distance [m].
  double          v_r   = 0.0;  ///< Robot velocity along `u` [m/s].
  double          v_h   = 0.0;  ///< Human velocity along `u` [m/s].
  double          a_h   = 0.0;  ///< Human acceleration along `u` [m/s^2].
};

/**
 * @brief Barrier value and Lie derivatives of a robot point and a human point.
 *
 * The Lie derivatives include the rotation of `u` (`du/dt = (I - u u^T)(v_r -
 * v_h)/d`) and the human acceleration. The CBF condition used by ThorQP is
 * `L_f h + L_g h . a_r + gamma (h - h_chi delta_H) >= 0`.
 *
 * If the points coincide (`d <= atol`), `u` is the x axis and `d = atol`.
 *
 * @param p_r, v_r       robot point position [m] and velocity [m/s].
 * @param p_h, v_h, a_h  human point position, velocity and acceleration.
 * @param T_r, a_s, C    as in barrier().
 * @param atol           as in minimumDistance().
 */
BarrierTerms barrierTerms( const Eigen::Vector3d& p_r, const Eigen::Vector3d& v_r,
                           const Eigen::Vector3d& p_h, const Eigen::Vector3d& v_h,
                           const Eigen::Vector3d& a_h,
                           double T_r, double a_s, double C, double atol = 1e-12 );

/**
 * @brief Maximum error of the estimated robot-human distance (Eq. 9 of the paper).
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
 * @return `delta_H` [m], to be used as ThorConfig::delta_H.
 */
double perceptionMargin( double beta, double sigma_p, double sigma_v, double sigma_a, double T_f_bar );

}  // namespace cbf
}  // namespace math
}  // namespace thor

#endif  // THOR_MATH__THOR_MATH__CBF__H
