/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Nicola Pedrocchi
 * Copyright (c) Manuel Beschi (manuel.beschi@unibs.it)
 * Copyright (c) Marco Faroni (marco.faroni@polimi.it)
 * Copyright (c) Federico Parma (federico.parma@unibs.it)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file thor_math.h
 * @brief Model predictive controller with time scaling and an optional control
 *        barrier function for collision avoidance with dynamic obstacles
 *        (people, vehicles, other robots, static objects).
 *
 * The main entry point is thor::math::ThorQP. The free functions build the
 * prediction (evolution) matrices of a chain of double integrators and are
 * used internally by ThorQP; they are public because they are useful on their
 * own. See docs/api.md for the mathematical formulation.
 */

#ifndef THOR_MATH__THOR_MATH__THOR_MATH__H
#define THOR_MATH__THOR_MATH__THOR_MATH__H


#include <vector>
#include <Eigen/Dense>
#include <qpmad/solver.h>
#include <thor_math/cbf.h>
// coal/narrowphase/narrowphase.h (pulled in by Pinocchio built with collision
// support) disables -Wdeprecated-declarations without restoring it, which would
// silence the [[deprecated]] attributes below. Restore the caller's settings.
#if defined(__GNUC__)
#pragma GCC diagnostic push
#endif
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

namespace thor 
{
/// Math utilities of THOR: MPC, evolution matrices and control barrier function.
namespace math
{

/**
 * @name Evolution matrices
 * Each joint is a double integrator driven by a piecewise-constant
 * acceleration. The state of @p nax joints is stacked as `x = [q; dq]`
 * (size `2*nax`), the inputs as `u = [ddq_0; ...; ddq_{nc-1}]` (size `nc*nax`),
 * one acceleration per control interval.
 * @{
 */

/**
 * @brief Predicted state at each prediction time as an affine function of the
 *        initial state and of the control inputs.
 *
 * For every prediction time `t_p`:
 * `[q(t_p); dq(t_p)] = free_p * x0 + forced_p * u`, where `free_p` and
 * `forced_p` are the `p`-th blocks of `2*nax` rows of the outputs.
 * Control interval `ic` starts at `control_intervals(0) + ... +
 * control_intervals(ic-1)` and lasts `control_intervals(ic)`; its input has no
 * effect on predictions before it starts.
 *
 * @param prediction_time   prediction times from now [s], size `np`.
 * @param control_intervals durations of the control intervals [s], size `nc`.
 * @param nax               number of joints.
 * @param[out] free_response   `(2*nax*np) x (2*nax)` matrix.
 * @param[out] forced_response `(2*nax*np) x (nc*nax)` matrix.
 * @return false if @p prediction_time or @p control_intervals is empty
 *         (outputs unchanged), true otherwise.
 */
bool computeEvolutionMatrix( const Eigen::Ref<Eigen::VectorXd> prediction_time,
                             const Eigen::Ref<Eigen::VectorXd> control_intervals,
                             const unsigned int& nax,
                             Eigen::MatrixXd& free_response,
                             Eigen::MatrixXd& forced_response
                            );

/**
 * @brief Finite-difference jerk of the control inputs.
 *
 * `jerk_i = (u_i - u_{i-1}) / control_intervals(i)`, with `u_{-1}` the
 * acceleration applied in the previous cycle:
 * `jerk = free_response * u_prev + forced_response * u`.
 *
 * @param prediction_time   prediction times [s], size `np` (only its size is used).
 * @param control_intervals durations of the control intervals [s], size `nc`.
 * @param nax               number of joints.
 * @param[out] free_response   `(np*nax) x nax` matrix, multiplies `u_prev`.
 * @param[out] forced_response `(np*nax) x (nc*nax)` matrix, multiplies `u`.
 * @return false if either vector is empty, true otherwise.
 */
bool computeJerkEvolutionMatrix ( const Eigen::Ref< Eigen::VectorXd > prediction_time,
                                  const Eigen::Ref< Eigen::VectorXd > control_intervals,
                                  const unsigned int& nax,
                                  Eigen::MatrixXd& free_response,
                                  Eigen::MatrixXd& forced_response );

/**
 * @brief State transition of @p nax double integrators over time @p t.
 * @return `(2*nax) x (2*nax)` matrix `[I, t*I; 0, I]`.
 */
Eigen::MatrixXd freeResponse(const double& t, const unsigned int& nax);

/**
 * @brief Effect on the state of a constant acceleration applied for time @p t.
 * @return `(2*nax) x nax` matrix `[t^2/2*I; t*I]`.
 */
Eigen::MatrixXd forcedResponse(const double& t, const unsigned int& nax);


/**
 * @brief Splits the output of computeEvolutionMatrix() into positions and
 *        velocities.
 *
 * @param free_response   free response, `(2*nax*np) x (2*nax)`.
 * @param[out] velocity_free_response   `(nax*np) x nax`, multiplies `dq0`
 *        (velocities do not depend on the initial positions).
 * @param[out] position_free_response   `(nax*np) x (2*nax)`, multiplies `x0`.
 * @param forced_response forced response, `(2*nax*np) x (nc*nax)`.
 * @param[out] velocity_forced_response `(nax*np) x (nc*nax)`.
 * @param[out] position_forced_response `(nax*np) x (nc*nax)`.
 * @param nax             number of joints.
 */
void splitResponses( const Eigen::MatrixXd& free_response, 
                     Eigen::MatrixXd& velocity_free_response, 
                     Eigen::MatrixXd& position_free_response, 
                     const Eigen::MatrixXd& forced_response, 
                     Eigen::MatrixXd& velocity_forced_response, 
                     Eigen::MatrixXd& position_forced_response, 
                     const unsigned int& nax);

/**
 * @brief Control intervals growing quadratically along the horizon
 *        ("input blocking").
 *
 * The prediction times are multiples of @p first_interval:
 * `t_i = first_interval * round(a*(i+1)^2 + b*(i+1) + c)`, so that the first
 * one is @p first_interval and the last one is the horizon (rounded down to a
 * multiple of @p first_interval). Short intervals near now give an accurate
 * first command; long intervals far away extend the horizon cheaply.
 *
 * @param control_horizon_time horizon [s].
 * @param n_control            number of intervals; must be at least 2 (with 1
 *                             the coefficients are not finite).
 * @param first_interval       duration of the first interval [s], usually the
 *                             control period.
 * @param[out] control_intervals interval durations, size @p n_control.
 * @param[out] prediction_time   end time of each interval, size @p n_control.
 * @return true. Invalid arguments are only checked by `assert`; with too many
 *         intervals for the horizon some durations are zero (ThorQP::configure()
 *         rejects that).
 */
bool quadraticControlIntervals( const double& control_horizon_time, const unsigned int& n_control, const double& first_interval, Eigen::VectorXd& control_intervals, Eigen::VectorXd& prediction_time );

/**
 * @brief Control intervals of equal duration.
 *
 * The duration is `control_horizon_time / n_control` rounded to a multiple of
 * @p sampling_period.
 *
 * @param control_horizon_time horizon [s].
 * @param n_control            number of intervals.
 * @param sampling_period      control period [s].
 * @param[out] control_intervals interval durations, size @p n_control.
 * @param[out] prediction_time   end time of each interval, size @p n_control.
 * @return true. A zero duration (horizon shorter than `n_control` periods) is
 *         rejected by ThorQP::configure().
 */
bool constantControlIntervals ( const double& control_horizon_time, const unsigned int& n_control, const double& sampling_period, Eigen::VectorXd& control_intervals, Eigen::VectorXd& prediction_time );

/// @}

/**
 * @brief Jacobians of the velocity of a frame origin, in world coordinates.
 *
 * With `v = J(q) dq` the velocity of the origin of @p frame_id in world
 * coordinates (the linear part of the LOCAL_WORLD_ALIGNED frame velocity):
 * - @p dv_ddq `= dv/ddq = J`, the linear frame Jacobian;
 * - @p dv_dq  `= dv/dq`, from the LOCAL derivatives as
 *   `dv/dq_i = R dv_local/dq_i + omega_i x v`, with `omega_i` the angular
 *   Jacobian column in world coordinates. The linear rows returned by
 *   Pinocchio for LOCAL_WORLD_ALIGNED are not this derivative.
 *
 * pinocchio::computeForwardKinematicsDerivatives() and
 * pinocchio::updateFramePlacements() must have been called for `(q, dq)`.
 *
 * @param model, data  Pinocchio model and data.
 * @param frame_id     frame index.
 * @param[out] dv_dq   `3 x nv` matrix.
 * @param[out] dv_ddq  `3 x nv` matrix.
 */
void frameVelocityDerivatives( const pinocchio::Model& model, pinocchio::Data& data,
                               pinocchio::FrameIndex frame_id,
                               Eigen::Matrix<double, 3, Eigen::Dynamic>& dv_dq,
                               Eigen::Matrix<double, 3, Eigen::Dynamic>& dv_ddq );


/**
 * @brief How the CBF is imposed along the horizon (see docs/api.md).
 *
 * Both are written for every robot sphere, obstacle and control
 * interval `i`. `X_0` is the current state and `X_{i+1}` the predicted state
 * at the end of interval `i`.
 */
enum class CbfFormulation
{
  /**
   * `h(X_{i+1}) >= alpha_i h(X_i)`, `alpha_i = max(0, 1 - gamma dt_i)`, with
   * `h` linearized around the trajectory obtained by applying the previous
   * solution from the current state. It constrains the barrier at every
   * predicted state and accounts for the length of each interval.
   */
  discrete,
  /**
   * `L_g h (J u_i + dJ dq) + L_f h + gamma h >= 0` at the start of interval
   * `i`, on the current state for `i = 0` and on the previous prediction for
   * `i > 0`. It constrains only the rate of `h`; with `n_intervals = 1` it is
   * the one-step controller of the paper.
   */
  continuous
};

/**
 * @brief Settings of ThorQP that change rarely.
 *
 * Applied with ThorQP::configure(), which validates them and rebuilds the QP
 * matrices. Units are SI (m, s, rad); vectors have size #n_joints.
 */
struct ThorConfig
{
  /// @name Horizon
  /// @{
  unsigned int n_intervals = 0;    ///< Number of control intervals `nc` (> 0; >= 2 with #input_blocking).
  unsigned int n_joints    = 0;    ///< Number of joints `nax` (> 0).
  double horizon_time      = 0.0;  ///< Prediction horizon [s] (>= #sampling_period).
  double sampling_period   = 0.0;  ///< Control period `dt` [s] (> 0): duration of the applied command.
  bool   input_blocking    = true; ///< Quadratically growing intervals (quadraticControlIntervals()) if true, equal intervals (constantControlIntervals()) if false.
  /// @}

  /// @name Cost weights
  /// The velocity tracking term has unit weight; see docs/api.md for the cost.
  /// @{
  double lambda_acc     = 0.0;  ///< Weight of the joint accelerations `||u||^2`.
  double lambda_tau     = 0.0;  ///< Torque weight. Not implemented: it has no effect.
  double lambda_jerk    = 0.0;  ///< Weight of the finite-difference jerk, including the step from the last applied acceleration.
  double lambda_scaling = 0.0;  ///< Weight of `||s_i - target_scaling||^2`: keeps the nominal speed.
  double lambda_clik    = 0.0;  ///< Weight of `||q(t_0) - next_target_q||^2`: corrects the position drift (closed-loop inverse kinematics).
  /// @}

  /// @name Limits
  /// @{
  Eigen::VectorXd qmax;     ///< Upper position limits [rad], used if #position_bounds.
  Eigen::VectorXd qmin;     ///< Lower position limits [rad] (<= #qmax), used if #position_bounds.
  Eigen::VectorXd Dqmax;    ///< Velocity limits [rad/s] (> 0), imposed at every prediction time.
  Eigen::VectorXd DDqmax;   ///< Acceleration limits [rad/s^2] (>= 0), imposed on every interval.
  Eigen::VectorXd tau_max;  ///< Torque limits; may be empty. Stored but not used.
  bool   position_bounds = false;  ///< Adds position limits and the invariance constraint `K (q - qmin) + dq >= 0`, `K (qmax - q) - dq >= 0`, `K = 0.99 DDqmax/Dqmax`.
  double min_scaling     = 0.05;   ///< Lower bound of the time scaling of each interval.
  double max_scaling     = 1.01;   ///< Upper bound of the time scaling (>= #min_scaling).
  /// @}

  /// @name Control barrier function
  /// See cbf.h. Checked only when #cbf is true. The robot is monitored at
  /// spheres centered at #cbf_frame_ids, the obstacles are spheres or capsules
  /// centered at ThorInput::obstacle_pos (see ThorInput::obstacle_axis); the
  /// barrier uses the distance between surfaces.
  /// @{
  bool   cbf     = false;  ///< Enables the CBF constraints. Needs a Pinocchio model with `nq == nv == n_joints`.
  CbfFormulation cbf_formulation = CbfFormulation::discrete;  ///< How the CBF is imposed along the horizon.
  unsigned int cbf_linearizations = 1;  ///< Maximum QPs solved per cycle (>= 1). 1 linearizes the CBF around the previous cycle; `N > 1` re-linearizes up to `N - 1` more times around the plan just found, reducing the linearization error on the long intervals at up to `N` times the solve cost.
  double cbf_linearization_damping = 1.0;    ///< Step of the linearization point towards each new solution, in (0, 1]: `u_ref <- u_ref + damping (u* - u_ref)`. 1 is the full step; smaller values damp oscillations between linearizations.
  double cbf_linearization_tolerance = 0.0;  ///< Stop re-linearizing when `|u* - u_ref|_inf` is not above this value [rad/s^2] (>= 0); 0 never stops early.
  double a_s     = 0.0;    ///< Robot braking deceleration in the `d_min` formula [m/s^2] (> 0).
  double T_r     = 0.0;    ///< Reaction time before braking [s] (>= 0).
  double C       = 0.0;    ///< Safety margin `C_0` between the robot and obstacle surfaces [m] (> 0).
  double gamma   = 0.0;    ///< Class-K gain of `dh/dt + gamma h >= 0` [1/s].
  double delta_H = 0.0;    ///< Default distance estimation error [m] (>= 0), used when ThorInput::obstacle_delta_H is empty. The barrier is evaluated at the worst-case distance, i.e. the obstacle is inflated by delta_H; see cbf::perceptionMargin().
  double cbf_activation_distance = 5.0;  ///< If the lowest barrier over the horizon is above this value [m], the CBF rows are dropped from the QP.
  std::vector<unsigned int> cbf_frame_ids;  ///< Pinocchio frame ids of the centers of the monitored robot spheres.
  std::vector<double> cbf_frame_radii;  ///< Radius of each robot sphere [m] (>= 0), same size as #cbf_frame_ids; empty means points.
  int    n_obstacle_points = 0;  ///< Number of obstacle points per cycle (>= 0).
  /// @}
};

/**
 * @brief Inputs of one control cycle of ThorQP::solve().
 *
 * The obstacle vectors are ignored when the CBF is disabled. When it is enabled:
 * - if #obstacle_pos, #obstacle_vel and #obstacle_acc are all empty,
 *   ThorConfig::n_obstacle_points still points at `(100, 100, 100)` are used
 *   (no obstacle);
 * - otherwise #obstacle_pos and #obstacle_vel must have the same size, at
 *   least ThorConfig::n_obstacle_points;
 * - #obstacle_acc, #obstacle_radius, #obstacle_axis and #obstacle_delta_H may
 *   be empty (defaults: zero acceleration, zero radius, spheres,
 *   ThorConfig::delta_H) or have the size of #obstacle_pos.
 *
 * Each obstacle is a sphere, or a capsule (cylinder with hemispherical ends)
 * if its #obstacle_axis is not zero: see cbf::capsuleBarrierTerms().
 *
 * Only the first ThorConfig::n_obstacle_points points are used. Positions,
 * velocities and accelerations are in the Pinocchio world frame.
 */
struct ThorInput
{
  Eigen::VectorXd target_dq;       ///< Unscaled target joint velocities of every interval, `[dq_0; ...; dq_{nc-1}]`, size `n_joints*n_intervals`.
  Eigen::VectorXd next_target_q;   ///< Target position at the first prediction time, size `n_joints`.
  double          target_scaling = 1.0;  ///< Desired time scaling (1 = nominal speed).
  Eigen::VectorXd x0;              ///< Current state `[q; dq]`, size `2*n_joints`.
  std::vector<Eigen::Vector3d> obstacle_pos;  ///< Obstacle sphere centers, or capsule axis midpoints [m].
  std::vector<Eigen::Vector3d> obstacle_vel;  ///< Obstacle velocities [m/s].
  std::vector<Eigen::Vector3d> obstacle_acc;  ///< Obstacle accelerations [m/s^2]; empty means zero.
  std::vector<double> obstacle_radius;        ///< Obstacle sphere or capsule radii [m] (>= 0); empty means points.
  std::vector<double> obstacle_delta_H;       ///< Distance estimation error of each point [m] (>= 0), added to its radius by the barrier; empty means ThorConfig::delta_H for all.
  std::vector<Eigen::Vector3d> obstacle_axis; ///< Capsule half axes [m]: the axis goes from `obstacle_pos - obstacle_axis` to `obstacle_pos + obstacle_axis`, fixed in orientation. Zero for a sphere; empty means spheres for all.
};

/**
 * @brief Robot-obstacle pair with the lowest barrier value over the horizon.
 *
 * Barrier values are at the worst-case distance. The states after the current
 * one are the linearization points of the CBF: the previous solution applied
 * from the current state (CbfFormulation::discrete) or the previous prediction
 * (CbfFormulation::continuous), not the new solution. All fields are zero when
 * the CBF is disabled.
 */
struct CbfDiagnostics
{
  double       h_now          = 0.0;  ///< Lowest barrier value on the current state [m]: the actual safety margin.
  double       h_min          = 0.0;  ///< Lowest barrier value over the horizon [m]; it can be negative on predicted states even if the actual motion is safe.
  unsigned int interval       = 0;    ///< Prediction node of #h_min: 0 is the current state, `i` the start of interval `i`, `n_intervals` the end of the horizon (discrete only).
  unsigned int frame_id       = 0;    ///< Pinocchio frame id of the robot sphere.
  double       distance       = 0.0;  ///< Estimated distance between the robot and obstacle surfaces [m], not reduced by delta_H (negative if they overlap).
  double       v_robot        = 0.0;  ///< Robot velocity along `(p_r - p_o)/|p_r - p_o|` [m/s], with `p_o` the closest point of a capsule axis; negative when approaching.
  double       v_obstacle     = 0.0;  ///< Obstacle velocity along the same direction [m/s]; positive when approaching.
  unsigned int obstacle_index = 0;    ///< Index of the obstacle point in ThorInput::obstacle_pos.
};

/// @brief Result of ThorQP::solve().
struct ThorOutput
{
  Eigen::VectorXd next_acc;            ///< Joint accelerations to apply during the next control period, size `n_joints`.
  double          next_scaling = 0.0;  ///< Time scaling of the first interval: advance the trajectory time by `next_scaling * dt`.
  CbfDiagnostics  cbf;                 ///< Most critical robot-obstacle pair (CBF enabled only), from the last linearization.
  unsigned int    cbf_linearizations = 0;  ///< QPs solved successfully in this cycle: 1 without CBF, up to ThorConfig::cbf_linearizations with it (fewer if a later QP failed, the iteration converged or the CBF rows were inactive).
  double          cbf_linearization_step = 0.0;  ///< Last `|u* - u_ref|_inf` [rad/s^2] between a solution and its linearization point; 0 with a single QP.
};


/**
 * @brief Model predictive controller with time scaling and an optional
 *        control barrier function (CBF).
 *
 * At every control cycle it solves a quadratic program over `n_intervals`
 * control intervals. The decision variables are one joint acceleration vector
 * and one time scaling per interval. The cost tracks the scaled target
 * velocities; the constraints bound accelerations, velocities, optionally
 * positions, and, with the CBF, keep a barrier on the distance from dynamic
 * obstacles non-negative (see cbf.h and docs/api.md).
 *
 * Typical use:
 * @code
 * thor::math::ThorQP qp;
 * qp.setPinocchioModel(model);    // only needed with the CBF
 * qp.configure(config);           // validates and builds the matrices
 * qp.setInitialState(x0);
 * while (running) {
 *   thor::math::ThorInput in;     // targets, x0, obstacle points
 *   ...
 *   thor::math::ThorOutput out = qp.solve(in);
 *   qp.updateState(out.next_acc); // or read x0 from the robot
 * }
 * @endcode
 *
 * The class keeps the previous solution: the jerk term and the CBF of the
 * intervals after the first are computed from the previous prediction. It is
 * not thread-safe; use one instance per control loop.
 */
class ThorQP
{
  protected:
    /// @cond THOR_INTERNAL
    bool m_are_matrices_updated;
    double m_min_scaling = 0.05;
    double m_max_scaling = 1.01;
    double m_cbf_activation_distance = 5.0;
    bool m_use_input_blocking;
    
    Eigen::MatrixXd m_weigth_matrix;
    Eigen::MatrixXd m_H_fixed;
    Eigen::MatrixXd m_H_variable;
    Eigen::VectorXd m_f_scaling;  // m_lambda_scaling* ones to be multiplied by sref
    Eigen::MatrixXd m_f_vel; // forced'*free    to be multiplied by v0
    Eigen::MatrixXd m_f_pos; // m_lambda_clik* forced'*free  to be multiplied by x0 (works only on the first step)
    
    Eigen::MatrixXd m_H;
    Eigen::VectorXd m_f; 
    
    Eigen::VectorXd m_sol;
    
    Eigen::VectorXd m_ub;
    Eigen::VectorXd m_lb;
    Eigen::MatrixXd m_CE;
    Eigen::VectorXd m_ce0;
    
    Eigen::MatrixXd m_CI;
    Eigen::VectorXd m_ci0;
    
    Eigen::VectorXd m_qmax;
    Eigen::VectorXd m_qmin;
    Eigen::VectorXd m_Dqmax;
    Eigen::VectorXd m_DDqmax;
    Eigen::VectorXd m_tau_max;

    bool m_are_position_bounds_active;
    bool m_are_torque_bounds_active;
    
    Eigen::VectorXd m_prediction_pos;
    Eigen::VectorXd m_prediction_vel;
    
    Eigen::VectorXd m_control_intervals;
    Eigen::VectorXd m_prediction_time;
    Eigen::MatrixXd m_forced_response;
    Eigen::MatrixXd m_free_response;
    Eigen::MatrixXd m_position_free_resp;
    Eigen::MatrixXd m_position_forced_resp;
    Eigen::MatrixXd m_next_position_free_resp;
    Eigen::MatrixXd m_next_position_forced_resp;
    Eigen::MatrixXd m_velocity_free_resp;
    Eigen::MatrixXd m_velocity_forced_resp;
    Eigen::MatrixXd m_do_scaling; // applying the scaling to trajectory
    Eigen::MatrixXd m_invariance_free_resp;

    Eigen::MatrixXd m_jerk_free_response;
    Eigen::MatrixXd m_jerk_forced_response;

    Eigen::JacobiSVD<Eigen::MatrixXd>  m_svd;
    
    unsigned int m_nc; //number of control and prediction intervals
    unsigned int m_nax; //number of joints
    double m_control_horizon_time;
    double m_dt;
    
    double m_lambda_acc;
    double m_lambda_tau;
    double m_lambda_scaling;
    double m_lambda_clik;
    double m_lambda_jerk;

    Eigen::VectorXd m_state;
    
    bool               m_use_cbf;
    pinocchio::Model   m_model;           // robot model (built from URDF once)
    pinocchio::Data    m_data;            // Pinocchio runtime buffers
    double             m_a_s, m_T_r, m_C; // parameters from d_min formula
    double             m_gamma;           // CBF gain
    double             m_delta_H;         // distance estimation error
    double             m_h;               // barrier value
    int                m_n_obstacle_points; // number of obstacle points considered
    std::vector<unsigned int> m_frameIds;
    std::vector<double> m_frame_radii;    // robot sphere radii, one per frame
    CbfFormulation     m_cbf_formulation = CbfFormulation::discrete;
    unsigned int       m_cbf_linearizations = 1;
    double             m_cbf_linearization_damping = 1.0;
    double             m_cbf_linearization_tolerance = 0.0;
    Eigen::VectorXd    m_cbf_reference;   // inputs around which the discrete CBF is linearized

    // Preallocated workspace of solve(), sized by updateMatrices(): solve()
    // reuses it instead of allocating at every cycle.
    struct Workspace
    {
      // Cost
      Eigen::MatrixXd dqt;                         // targetDq.asDiagonal() * m_do_scaling
      Eigen::VectorXd tmp_u;                       // intermediate products of size n_joints*n_intervals
      // QP passed to qpmad: the variable bounds go as simple bounds, the
      // other inequalities (and the equalities) as general constraints.
      Eigen::MatrixXd qp_H, qp_A;
      Eigen::VectorXd qp_lower, qp_upper;
      // The block of m_H on the accelerations is constant: its inverted
      // Cholesky factor is computed once by updateMatrices(), and each cycle
      // only completes the factor for the time scalings.
      Eigen::MatrixXd acc_chol_inv_t;              // L_a^-T, upper triangular
      bool acc_chol_valid = false;
      Eigen::MatrixXd scaling_coupling;            // L_a^-1 times the acceleration-scaling block of m_H
      Eigen::MatrixXd scaling_schur;               // its Schur complement, factorized in place
      // Minimum over the variable bounds of CI^T x for every column of m_CI;
      // used to skip the inequalities that no x within the bounds can violate.
      Eigen::VectorXd ci_min;
      Eigen::VectorXd ci0, ci0_base, last_sol, last_ci0;
      Eigen::MatrixXd last_cbf_columns;
      // CBF construction
      Eigen::VectorXd q_ref, dq_ref, zero_acc, c;
      Eigen::MatrixXd G;
      // Inputs that the state at the end of interval i can depend on: the
      // nonzero columns of its rows in the forced responses (causality).
      std::vector<Eigen::Index> node_inputs;
      Eigen::RowVectorXd dh_dq, dh_ddq;
      Eigen::Matrix<double, 6, Eigen::Dynamic> J, dJ, local_dq, local_dv;
      Eigen::Matrix<double, 3, Eigen::Dynamic> v_dq, v_dv;
    };
    Workspace          m_ws;
    qpmad::Solver      m_solver;

    /// @brief Sizes the workspace of solve() for the current settings.
    void resizeWorkspace();
    /**
     * @brief Writes into `m_ws.qp_H` the inverted Cholesky factor of `m_H`,
     *        reusing the factor of the constant acceleration block.
     * @return false if the block structure does not hold (e.g. a derived class
     *         changed it); `m_H` must then be factorized from scratch.
     */
    bool invertedCholeskyFactor();
    Eigen::VectorXd    m_ci0_solved;      // inequality offsets of the last solve, CBF rows included
    /// @endcond

    /**
     * @brief Builds the cost of the current cycle (`m_H`, `m_f`) from the
     *        targets and the state. Called by doSolve() and
     *        computedUncostrainedSolution().
     */
    virtual void computeActualMatrices( const Eigen::VectorXd& targetDq,
                                const Eigen::VectorXd& next_targetQ,
                                const double& target_scaling,
                                const Eigen::VectorXd& x0);

    /**
     * @brief Customization point of the constrained solver.
     *
     * Called by solve() with validated inputs; with the CBF enabled the
     * obstacle vectors have at least `n_obstacle_points` elements and
     * ThorInput::obstacle_acc, obstacle_radius and obstacle_delta_H are filled; obstacle_axis is empty or has their size. Subclasses override this, so that the public solve() and
     * computedCostrainedSolution() are never hidden.
     * @throws std::runtime_error if the problem is infeasible.
     */
    virtual void doSolve( const ThorInput& in, ThorOutput& out );

    /**
     * @brief Fills the CBF columns of `m_CI` and their offsets in @p ci0 with
     *        the CbfFormulation::discrete constraints, and the diagnostics.
     * @param in   validated, completed inputs (see doSolve()).
     * @param ci0  inequality offsets of this cycle.
     * @param diag barrier diagnostics.
     */
    void buildDiscreteCbf( const ThorInput& in, Eigen::VectorXd& ci0, CbfDiagnostics& diag );

    /// @brief As buildDiscreteCbf(), with the CbfFormulation::continuous constraints.
    void buildContinuousCbf( const ThorInput& in, Eigen::VectorXd& ci0, CbfDiagnostics& diag );

    /// @brief Legacy return value: `{h_min, interval, frame_id, distance, v_robot, v_obstacle}`.
    static std::vector<double> toLegacyDiagnostics( const CbfDiagnostics& cbf );
    /// @cond THOR_INTERNAL
    int id = 0;
    /// @endcond
  public:
    /**
     * @brief Copies the whole optimizer, including the Pinocchio model, the
     *        configuration, the state and the last solution.
     */
    ThorQP& operator=(const ThorQP& other) {
      if (this != &other) {
        m_are_matrices_updated = other.m_are_matrices_updated;
        m_min_scaling = other.m_min_scaling;
        m_max_scaling = other.m_max_scaling;
        m_cbf_activation_distance = other.m_cbf_activation_distance;
        id = other.id+1;
        m_use_input_blocking = other.m_use_input_blocking;

        m_weigth_matrix = other.m_weigth_matrix;
        m_H_fixed = other.m_H_fixed;
        m_H_variable = other.m_H_variable;
        m_f_scaling = other.m_f_scaling;
        m_f_vel = other.m_f_vel;
        m_f_pos = other.m_f_pos;

        m_H = other.m_H;
        m_f = other.m_f;
        m_sol = other.m_sol;

        m_ub = other.m_ub;
        m_lb = other.m_lb;
        m_CE = other.m_CE;
        m_ce0 = other.m_ce0;

        m_CI = other.m_CI;
        m_ci0 = other.m_ci0;

        m_qmax = other.m_qmax;
        m_qmin = other.m_qmin;
        m_Dqmax = other.m_Dqmax;
        m_DDqmax = other.m_DDqmax;
        m_tau_max = other.m_tau_max;

        m_are_position_bounds_active = other.m_are_position_bounds_active;
        m_are_torque_bounds_active = other.m_are_torque_bounds_active;

        m_prediction_pos = other.m_prediction_pos;
        m_prediction_vel = other.m_prediction_vel;

        m_control_intervals = other.m_control_intervals;
        m_prediction_time = other.m_prediction_time;
        m_forced_response = other.m_forced_response;
        m_free_response = other.m_free_response;
        m_position_free_resp = other.m_position_free_resp;
        m_position_forced_resp = other.m_position_forced_resp;
        m_next_position_free_resp = other.m_next_position_free_resp;
        m_next_position_forced_resp = other.m_next_position_forced_resp;
        m_velocity_free_resp = other.m_velocity_free_resp;
        m_velocity_forced_resp = other.m_velocity_forced_resp;
        m_do_scaling = other.m_do_scaling;
        m_invariance_free_resp = other.m_invariance_free_resp;

        m_jerk_free_response = other.m_jerk_free_response;
        m_jerk_forced_response = other.m_jerk_forced_response;

        m_svd = other.m_svd;

        m_nc = other.m_nc;
        m_nax = other.m_nax;
        m_control_horizon_time = other.m_control_horizon_time;
        m_dt = other.m_dt;

        m_lambda_acc = other.m_lambda_acc;
        m_lambda_tau = other.m_lambda_tau;
        m_lambda_scaling = other.m_lambda_scaling;
        m_lambda_clik = other.m_lambda_clik;
        m_lambda_jerk = other.m_lambda_jerk;

        m_state = other.m_state;

        // CBF parameters cloning
        m_a_s = other.m_a_s;
        m_T_r = other.m_T_r;
        m_C = other.m_C;
        m_gamma = other.m_gamma;
        m_delta_H = other.m_delta_H;

        m_use_cbf = other.m_use_cbf;
        m_model = other.m_model;
        m_data = other.m_data;
        m_frameIds = other.m_frameIds;
        m_frame_radii = other.m_frame_radii;
        m_cbf_formulation = other.m_cbf_formulation;
        m_cbf_linearizations = other.m_cbf_linearizations;
        m_cbf_linearization_damping = other.m_cbf_linearization_damping;
        m_cbf_linearization_tolerance = other.m_cbf_linearization_tolerance;
        m_cbf_reference = other.m_cbf_reference;
        m_ws = other.m_ws;
        m_ci0_solved = other.m_ci0_solved;
        m_h = other.m_h;
        m_n_obstacle_points = other.m_n_obstacle_points;

      }
      return *this;
    }

    /// @cond THOR_INTERNAL
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /// @endcond

    /// @brief Unconfigured optimizer: call configure() before solve().
    ThorQP();

    /**
     * @brief Sets the model and applies the configuration.
     * @param config see configure().
     * @param model  robot model; required only if `config.cbf` is true.
     * @throws std::invalid_argument as configure().
     */
    explicit ThorQP( const ThorConfig& config, const pinocchio::Model& model = pinocchio::Model() );
    virtual ~ThorQP() = default;

    /**
     * @brief Validates the configuration, applies it and rebuilds the matrices.
     *
     * Resets the previous solution. Call setInitialState() afterwards.
     * The Pinocchio model must be set before enabling the CBF.
     * @throws std::invalid_argument on inconsistent settings (sizes, signs,
     *         a horizon that cannot hold `n_intervals` non-empty intervals,
     *         a model that does not match `n_joints`, unknown frame ids). The
     *         optimizer is left unchanged.
     */
    void configure( const ThorConfig& config );

    /**
     * @brief Validates the inputs and solves the constrained problem.
     *
     * Also updates the internal prediction used, in the next cycle, by the
     * jerk term and by the CBF of the intervals after the first.
     * @throws std::logic_error if the matrices are outdated (configure() or
     *         updateMatrices() not called after a change).
     * @throws std::invalid_argument on wrongly sized inputs (see ThorInput).
     * @throws std::runtime_error if the problem is infeasible; the caller must
     *         then apply a safe command, e.g. brake within the limits.
     */
    ThorOutput solve( const ThorInput& in );

    /**
     * @brief Sets the robot model used by the CBF (copied).
     *
     * Must be called before configure() when the CBF is enabled.
     */
    void setPinocchioModel ( const pinocchio::Model& model );

    /// @name Legacy setters
    /// Deprecated: they skip the validation done by configure() and require
    /// a call to updateMatrices().
    /// @{

    /// @deprecated Use ThorConfig::a_s, T_r, C, gamma and configure().
    [[deprecated("set ThorConfig::a_s, T_r, C, gamma and call configure()")]]
    void setCBFParameters ( const double& a_s, const double& T_r, const double& C, const double& gamma);

    /// @deprecated Use the ThorConfig limits and configure().
    [[deprecated("set the ThorConfig limits and call configure()")]]
    void setConstraints(const Eigen::VectorXd& qmax,
                  const Eigen::VectorXd& qmin,
                  const Eigen::VectorXd& Dqmax,
                  const Eigen::VectorXd& DDqmax,
                  const Eigen::VectorXd& tau_max);

    /// @deprecated Use ThorConfig::position_bounds and configure().
    [[deprecated("set ThorConfig::position_bounds and call configure()")]]
    void activatePositionBounds(const bool enable_pos_bounds);

    /// @deprecated Torque bounds are not implemented: only a flag is stored.
    [[deprecated("torque bounds are not implemented and have no effect")]]
    void activateTorqueBounds(const bool enable_tau_bounds);

    /// @deprecated Use ThorConfig::cbf and configure().
    [[deprecated("set ThorConfig::cbf and call configure()")]]
    void activateCbfBounds(const bool enable_cbf_bounds);

    /// @deprecated No effect: the barrier handles a receding robot through `d_min`.
    [[deprecated("the barrier handles a receding robot through d_min: this has no effect")]]
    void activateCbfMoveAway(const bool enable_cbf_move_away);

    /// @}

    /// @brief True if position and invariance constraints are enabled.
    bool arePositionBoundsActive();

    /// @name Legacy setters (continued)
    /// @{

    /// @deprecated Use the ThorConfig horizon fields and configure().
    [[deprecated("set the ThorConfig horizon fields and call configure()")]]
    void setIntervals(const unsigned int& num_of_intervals,
                      const unsigned int& num_of_joints,
                      const double& control_horizon_time,
                      const double& computing_period);

    /// @deprecated Use the ThorConfig horizon fields and configure().
    [[deprecated("set the ThorConfig horizon fields and call configure()")]]
    void setIntervals ( const unsigned int& num_of_intervals,
                      const unsigned int& num_of_joints,
                      const double& control_horizon_time,
                      const double& computing_period,
                      const bool & use_input_blocking);

    /// @deprecated Use the ThorConfig lambda_* fields and configure().
    [[deprecated("set the ThorConfig lambda_* fields and call configure()")]]
    void setWeigthFunction( const double& lambda_acc, const double& lambda_tau, const double& lambda_jerk, const double& lambda_scaling, const double& lambda_clik );

    /// @deprecated Use ThorConfig::cbf_frame_ids, n_obstacle_points and configure().
    /// The robot spheres get zero radius.
    [[deprecated("set ThorConfig::cbf_frame_ids and n_obstacle_points and call configure()")]]
    void setCbfIds(const std::vector<unsigned int>& frameIds, const int& num_ph);
    /// @}


    /// @brief True if the configuration changed and updateMatrices() (or configure()) must be called.
    bool needUpdate(){return !m_are_matrices_updated;};

    /**
     * @brief Rebuilds the constant QP matrices from the stored settings.
     *
     * configure() calls it; call it directly only after the legacy setters.
     */
    virtual void updateMatrices();
    
    /**
     * @brief Minimizes the cost ignoring every constraint (SVD least squares).
     *
     * Useful for debugging the cost weights.
     * @param targetDq       as ThorInput::target_dq.
     * @param next_targetQ   as ThorInput::next_target_q.
     * @param target_scaling as ThorInput::target_scaling.
     * @param x0             as ThorInput::x0.
     * @param[out] next_acc     accelerations of the first interval.
     * @param[out] next_scaling time scaling of the first interval.
     * @return true.
     */
    virtual bool computedUncostrainedSolution(  const Eigen::VectorXd& targetDq,
                                        const Eigen::VectorXd& next_targetQ,
                                        const double& target_scaling,
                                        const Eigen::VectorXd& x0,
                                        Eigen::VectorXd& next_acc,
                                        double& next_scaling
                                    );
    /**
     * @deprecated Use solve().
     * @return `{h_min, interval, frame_id, distance, v_robot, v_obstacle}`
     *         (see CbfDiagnostics). Note the order of the obstacle arguments:
     *         velocities, then positions.
     */
    [[deprecated("use solve()")]]
    std::vector<double> computedCostrainedSolution(  const Eigen::VectorXd& targetDq,
                                        const Eigen::VectorXd& next_targetQ,
                                        const double& target_scaling,
                                        const Eigen::VectorXd& x0,
                                        Eigen::VectorXd& next_acc,
                                        double& next_scaling,
                                        const std::vector<Eigen::Vector3d> &v_o_vector,
                                        const std::vector<Eigen::Vector3d> &p_o_vector
    );
    /// @deprecated Use solve(). Same as above with no obstacle data (far-away obstacles).
    [[deprecated("use solve()")]]
    std::vector<double> computedCostrainedSolution(  const Eigen::VectorXd& targetDq,
                                        const Eigen::VectorXd& next_targetQ,
                                        const double& target_scaling,
                                        const Eigen::VectorXd& x0,
                                        Eigen::VectorXd& next_acc,
                                        double& next_scaling
    );
    /**
     * @brief Sets the internal state `[q; dq]` (size `2*n_joints`) and resets
     *        the prediction to a robot standing still at `q`.
     *
     * Call it after configure().
     */
    void setInitialState(const Eigen::VectorXd& x0);

    /// @brief Integrates the internal state for one control period with constant acceleration @p next_acc.
    void updateState( const Eigen::VectorXd& next_acc );

    /// @brief Internal state `[q; dq]`.
    Eigen::VectorXd getState();

    /// @brief End time of each control interval [s], size `n_intervals`.
    Eigen::VectorXd getPredictionTimeInstant(){return m_prediction_time;};
    
    /// @brief Predicted positions at the first prediction time.
    /// Valid after setInitialState() or solve(); empty before configure().
    Eigen::VectorXd getFirstPredictionPos();
    /// @brief Predicted velocities at the first prediction time.
    /// Valid after setInitialState() or solve(); empty before configure().
    Eigen::VectorXd getFirstPredictionVel();
    /// @brief Control period [s].
    double getDt(){return m_dt;};
    /// @brief Number of obstacle points (ThorConfig::n_obstacle_points).
    unsigned int getNumObstaclePoints() const {return static_cast<unsigned int>(m_n_obstacle_points);};
    /// @deprecated Use getNumObstaclePoints().
    [[deprecated("use getNumObstaclePoints()")]]
    double getNumPh(){return m_n_obstacle_points;};
};

}  // namespace math
}  // namespace thor

#endif  // THOR_MATH__THOR_MATH__THOR_MATH__H
