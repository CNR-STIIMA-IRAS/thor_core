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

#ifndef THOR_MATH__THOR_MATH__THOR_MATH__H
#define THOR_MATH__THOR_MATH__THOR_MATH__H


#include <Eigen/Dense>
#include <eigen_matrix_utils/eiquadprog.hpp>
#include <iostream>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>

namespace thor 
{
namespace math
{

bool computeEvolutionMatrix( const Eigen::Ref<Eigen::VectorXd> prediction_time,
                             const Eigen::Ref<Eigen::VectorXd> control_intervals,
                             const unsigned int& nax,
                             Eigen::MatrixXd& free_response,
                             Eigen::MatrixXd& forced_response
                            );

bool computeJerkEvolutionMatrix ( const Eigen::Ref< Eigen::VectorXd > prediction_time,
                                  const Eigen::Ref< Eigen::VectorXd > control_intervals,
                                  const unsigned int& nax,
                                  Eigen::MatrixXd& free_response,
                                  Eigen::MatrixXd& forced_response );

Eigen::MatrixXd freeResponse(const double& t, const unsigned int& nax);
Eigen::MatrixXd forcedResponse(const double& t, const unsigned int& nax);


void splitResponses( const Eigen::MatrixXd& free_response, 
                     Eigen::MatrixXd& velocity_free_response, 
                     Eigen::MatrixXd& position_free_response, 
                     const Eigen::MatrixXd& forced_response, 
                     Eigen::MatrixXd& velocity_forced_response, 
                     Eigen::MatrixXd& position_forced_response, 
                     const unsigned int& nax);

bool quadraticControlIntervals( const double& control_horizon_time, const unsigned int& n_control, const double& first_interval, Eigen::VectorXd& control_intervals, Eigen::VectorXd& prediction_time );

bool constantControlIntervals ( const double& control_horizon_time, const unsigned int& n_control, const double& sampling_period, Eigen::VectorXd& control_intervals, Eigen::VectorXd& prediction_time );


class ThorQP
{
  protected:
    bool m_are_matrices_updated;
    const float m_min_scaling = 0.0;
    const float m_max_scaling = 1.1;
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
    double             m_h;               // barrier value
    int                m_num_ph;          // number of human points considered  
    std::vector<unsigned int> m_frameIds;

    bool m_use_cbf_move_away = false; // if true, the CBF will try to mantain safaty distance also when the robot is moving away

    void compute_h(const double& v_h, const double& v_r, const double& d, double& h);
    void compute_theta(const double& v_h, const double& v_r, const double& d, std::vector<double>& theta);

    virtual void computeActualMatrices( const Eigen::VectorXd& targetDq,
                                const Eigen::VectorXd& next_targetQ,
                                const double& target_scaling,
                                const Eigen::VectorXd& x0);
    int id = 0;
  public:
    // TODO: Add copy of pinocchio model and data 
    ThorQP& operator=(const ThorQP& other) {
      if (this != &other) {
        m_are_matrices_updated = other.m_are_matrices_updated;
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
        setCBFParameters(other.m_a_s, other.m_T_r, other.m_C, other.m_gamma);
        
        m_use_cbf = other.m_use_cbf;
        m_model = other.m_model;
        m_data = other.m_data;
        m_frameIds = other.m_frameIds;
        m_h = other.m_h;
        m_num_ph = other.m_num_ph;
        m_use_cbf_move_away = other.m_use_cbf_move_away;

      }
      return *this;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ThorQP();

    void setCBFParameters ( const double& a_s, const double& T_r, const double& C, const double& gamma);

    void setPinocchioModel ( const pinocchio::Model& model );

    void setConstraints(const Eigen::VectorXd& qmax,
                  const Eigen::VectorXd& qmin,
                  const Eigen::VectorXd& Dqmax,
                  const Eigen::VectorXd& DDqmax,
                  const Eigen::VectorXd& tau_max);

    void activatePositionBounds(const bool enable_pos_bounds);

    void activateTorqueBounds(const bool enable_tau_bounds);

    void activateCbfBounds(const bool enable_cbf_bounds);
    
    void activateCbfMoveAway(const bool enable_cbf_move_away);

    bool arePositionBoundsActive();

    void setIntervals(const unsigned int& num_of_intervals,
                      const unsigned int& num_of_joints,
                      const double& control_horizon_time,
                      const double& computing_period);

    void setIntervals ( const unsigned int& num_of_intervals,
                      const unsigned int& num_of_joints,
                      const double& control_horizon_time,
                      const double& computing_period,
                      const bool & use_input_blocking);

    void setWeigthFunction( const double& lambda_acc, const double& lambda_tau, const double& lambda_jerk, const double& lambda_scaling, const double& lambda_clik );

    void setCbfIds(const std::vector<unsigned int>& frameIds, const int& num_ph);


    bool needUpdate(){return !m_are_matrices_updated;};

    virtual void updateMatrices();
    
    virtual bool computedUncostrainedSolution(  const Eigen::VectorXd& targetDq,
                                        const Eigen::VectorXd& next_targetQ,
                                        const double& target_scaling,
                                        const Eigen::VectorXd& x0,
                                        Eigen::VectorXd& next_acc,
                                        double& next_scaling
                                    );
    virtual std::vector<double> computedCostrainedSolution(  const Eigen::VectorXd& targetDq,
                                        const Eigen::VectorXd& next_targetQ,
                                        const double& target_scaling,
                                        const Eigen::VectorXd& x0,
                                        Eigen::VectorXd& next_acc,
                                        double& next_scaling,
                                        const std::vector<Eigen::Vector3d> &v_h_vector, 
                                        const std::vector<Eigen::Vector3d> &p_h_vector
    );
    // if ph_vector and vh_vector are not passed, initialize them to default values 
    std::vector<double> computedCostrainedSolution(  const Eigen::VectorXd& targetDq,
                                        const Eigen::VectorXd& next_targetQ,
                                        const double& target_scaling,
                                        const Eigen::VectorXd& x0,
                                        Eigen::VectorXd& next_acc,
                                        double& next_scaling
    )
    {
      std::vector<Eigen::Vector3d> default_v_h(m_num_ph, Eigen::Vector3d::Zero() );
      std::vector<Eigen::Vector3d> default_p_h(m_num_ph, Eigen::Vector3d::Constant(100.0));

      return computedCostrainedSolution(targetDq, next_targetQ, target_scaling,
                                        x0, next_acc, next_scaling,
                                        default_v_h, default_p_h);
    }
    void setInitialState(const Eigen::VectorXd& x0);
    void updateState( const Eigen::VectorXd& next_acc );
    Eigen::VectorXd getState();
    Eigen::VectorXd getPredictionTimeInstant(){return m_prediction_time;};
    
    Eigen::VectorXd getFirstPredictionPos();
    Eigen::VectorXd getFirstPredictionVel();
    double getDt(){return m_dt;};
    double getNumPh(){return m_num_ph;};
  // ThorQP clone();
};

}  // namespace math
}  // namespace thor

#endif  // THOR_MATH__THOR_MATH__THOR_MATH__H
