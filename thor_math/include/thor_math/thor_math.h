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
 *        barrier function for human-robot collaboration.
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
  /// See cbf.h. Checked only when #cbf is true.
  /// @{
  bool   cbf     = false;  ///< Enables the CBF constraints. Needs a Pinocchio model with `nq == nv == n_joints`.
  double a_s     = 0.0;    ///< Robot braking deceleration in the `d_min` formula [m/s^2] (> 0).
  double T_r     = 0.0;    ///< Reaction time before braking [s] (>= 0).
  double C       = 0.0;    ///< Static safety margin `C_0` [m] (> 0).
  double gamma   = 0.0;    ///< Class-K gain of `dh/dt + gamma h >= 0` [1/s].
  double delta_H = 0.0;    ///< Distance estimation error [m] (>= 0); see cbf::perceptionMargin().
  double cbf_activation_distance = 5.0;  ///< If the lowest barrier over the horizon is above this value [m], the CBF rows are dropped from the QP.
  std::vector<unsigned int> cbf_frame_ids;  ///< Pinocchio frame ids of the monitored robot points.
  int    n_human_points = 0;  ///< Number of human points per cycle (>= 0).
  /// @}
};

/**
 * @brief Inputs of one control cycle of ThorQP::solve().
 *
 * The human vectors are ignored when the CBF is disabled. When it is enabled:
 * - if #human_pos, #human_vel and #human_acc are all empty, ThorConfig::n_human_points
 *   still points at `(100, 100, 100)` are used (no person);
 * - otherwise #human_pos and #human_vel must have the same size, at least
 *   ThorConfig::n_human_points, and #human_acc must be empty (zero
 *   accelerations) or have that size too. Only the first
 *   ThorConfig::n_human_points points are used.
 *
 * Positions, velocities and accelerations are in the Pinocchio world frame.
 */
struct ThorInput
{
  Eigen::VectorXd target_dq;       ///< Unscaled target joint velocities of every interval, `[dq_0; ...; dq_{nc-1}]`, size `n_joints*n_intervals`.
  Eigen::VectorXd next_target_q;   ///< Target position at the first prediction time, size `n_joints`.
  double          target_scaling = 1.0;  ///< Desired time scaling (1 = nominal speed).
  Eigen::VectorXd x0;              ///< Current state `[q; dq]`, size `2*n_joints`.
  std::vector<Eigen::Vector3d> human_pos;  ///< Human point positions [m].
  std::vector<Eigen::Vector3d> human_vel;  ///< Human point velocities [m/s].
  std::vector<Eigen::Vector3d> human_acc;  ///< Human point accelerations [m/s^2]; empty means zero.
};

/**
 * @brief Robot-human pair with the lowest barrier value over the horizon.
 *
 * All fields are zero when the CBF is disabled.
 */
struct CbfDiagnostics
{
  double       h_min       = 0.0;  ///< Lowest barrier value [m].
  unsigned int interval    = 0;    ///< Control interval where it occurs.
  unsigned int frame_id    = 0;    ///< Pinocchio frame id of the robot point.
  double       distance    = 0.0;  ///< Robot-human distance `d` [m].
  double       v_rel       = 0.0;  ///< Robot velocity along `(p_r - p_h)/d` [m/s]; negative when approaching.
  double       vh_proj     = 0.0;  ///< Human velocity along `(p_r - p_h)/d` [m/s]; positive when approaching.
  unsigned int human_index = 0;    ///< Index of the human point in ThorInput::human_pos.
};

/// @brief Result of ThorQP::solve().
struct ThorOutput
{
  Eigen::VectorXd next_acc;            ///< Joint accelerations to apply during the next control period, size `n_joints`.
  double          next_scaling = 0.0;  ///< Time scaling of the first interval: advance the trajectory time by `next_scaling * dt`.
  CbfDiagnostics  cbf;                 ///< Most critical robot-human pair (CBF enabled only).
};


/**
 * @brief Model predictive controller with time scaling and an optional
 *        control barrier function (CBF).
 *
 * At every control cycle it solves a quadratic program over `n_intervals`
 * control intervals. The decision variables are one joint acceleration vector
 * and one time scaling per interval. The cost tracks the scaled target
 * velocities; the constraints bound accelerations, velocities, optionally
 * positions, and, with the CBF, keep a barrier on the robot-human distance
 * non-negative (see cbf.h and docs/api.md).
 *
 * Typical use:
 * @code
 * thor::math::ThorQP qp;
 * qp.setPinocchioModel(model);    // only needed with the CBF
 * qp.configure(config);           // validates and builds the matrices
 * qp.setInitialState(x0);
 * while (running) {
 *   thor::math::ThorInput in;     // targets, x0, human points
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
    int                m_num_ph;          // number of human points considered  
    std::vector<unsigned int> m_frameIds;
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
     * Called by solve() with validated inputs; with the CBF enabled the human
     * vectors have at least `n_human_points` elements and #ThorInput::human_acc
     * is filled. Subclasses override this, so that the public solve() and
     * computedCostrainedSolution() are never hidden.
     * @throws std::runtime_error if the problem is infeasible.
     */
    virtual void doSolve( const ThorInput& in, ThorOutput& out );

    /// @brief Legacy return value: `{h_min, interval, frame_id, distance, v_rel, vh_proj}`.
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
        m_h = other.m_h;
        m_num_ph = other.m_num_ph;

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

    /// @deprecated Use ThorConfig::cbf_frame_ids, n_human_points and configure().
    [[deprecated("set ThorConfig::cbf_frame_ids and n_human_points and call configure()")]]
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
     * @return `{h_min, interval, frame_id, distance, v_rel, vh_proj}`
     *         (see CbfDiagnostics). Note the order: velocities, then positions.
     */
    [[deprecated("use solve()")]]
    std::vector<double> computedCostrainedSolution(  const Eigen::VectorXd& targetDq,
                                        const Eigen::VectorXd& next_targetQ,
                                        const double& target_scaling,
                                        const Eigen::VectorXd& x0,
                                        Eigen::VectorXd& next_acc,
                                        double& next_scaling,
                                        const std::vector<Eigen::Vector3d> &v_h_vector,
                                        const std::vector<Eigen::Vector3d> &p_h_vector
    );
    /// @deprecated Use solve(). Same as above with no human data (far-away humans).
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
    /// @brief Number of human points (ThorConfig::n_human_points).
    double getNumPh(){return m_num_ph;};
};

}  // namespace math
}  // namespace thor

#endif  // THOR_MATH__THOR_MATH__THOR_MATH__H
