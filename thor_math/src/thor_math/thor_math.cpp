#include <iostream>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <qpmad/solver.h>

#include <thor_math/thor_math.h>
// #include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
// #include <pinocchio/algorithm/frames.hpp>

namespace thor 
{
namespace math
{
  int computeRank(const Eigen::MatrixXd& M, double tol = 1e-10) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M);
    const Eigen::VectorXd& singularValues = svd.singularValues();
    int rank = 0;
    for (int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tol) {
            ++rank;
        }
    }
    return rank;
}

  Eigen::MatrixXd freeResponse ( const double& t, const unsigned int& nax )
  {
    Eigen::MatrixXd mtx(2*nax,2*nax);
    mtx.setIdentity();
    mtx.block(0,nax,nax,nax)=Eigen::MatrixXd::Identity(nax,nax)*t;
    return mtx;
  }
  
  Eigen::MatrixXd forcedResponse ( const double& t, const unsigned int& nax )
  {
    Eigen::MatrixXd mtx(2*nax,nax);
    mtx.block(0,   0,nax,nax)=std::pow(t,2)*0.5*Eigen::MatrixXd::Identity(nax,nax);
    mtx.block(nax, 0,nax,nax)=Eigen::MatrixXd::Identity(nax,nax)*t;
    return mtx;
  }

  bool computeEvolutionMatrix ( const Eigen::Ref< Eigen::VectorXd > prediction_time, const Eigen::Ref< Eigen::VectorXd > control_intervals, const unsigned int& nax, Eigen::MatrixXd& free_response, Eigen::MatrixXd& forced_response )
  {
    unsigned int np=prediction_time.size();
    if (np==0)
      return false;
    
    unsigned int nc=control_intervals.size();
    if (nc==0)
      return false;
    
    Eigen::VectorXd stop_control_time(nc);
    Eigen::VectorXd start_control_time(nc);
    start_control_time(0)=0;
    stop_control_time(0)=control_intervals(0);
    for (unsigned int ic=1;ic<nc;ic++)
    {
      start_control_time(ic) = stop_control_time(ic-1);
      stop_control_time(ic)  = stop_control_time(ic-1)  + control_intervals(ic);
    }
    free_response.resize(2*nax*np,2*nax);
    forced_response.resize(2*nax*np,nc*nax);
    free_response.setZero();
    forced_response.setZero();
    for (unsigned int ip=0;ip<np;ip++)
    {
      free_response.block(ip*2*nax, 0,   2*nax, 2*nax) = freeResponse(prediction_time(ip),nax);
      for (unsigned int ic=0;ic<nc;ic++)
      {
        if (prediction_time(ip)>stop_control_time(ic))
        {
          double free_response_time=prediction_time(ip)-stop_control_time(ic);
          forced_response.block(ip*2*nax,nax*ic,2*nax,nax)= freeResponse(free_response_time,nax)*forcedResponse(control_intervals(ic),nax);
        }
        else if (prediction_time(ip)>start_control_time(ic))
        {
          forced_response.block(ip*2*nax,nax*ic,2*nax,nax)= forcedResponse(prediction_time(ip)-start_control_time(ic),nax);
        }
        // else zero
      }
    }
    return true;
  }

  bool computeJerkEvolutionMatrix ( const Eigen::Ref< Eigen::VectorXd > prediction_time, const Eigen::Ref< Eigen::VectorXd > control_intervals, const unsigned int& nax, Eigen::MatrixXd& free_response, Eigen::MatrixXd& forced_response )
  {
    unsigned int np=prediction_time.size();
    if (np==0)
      return false;

    unsigned int nc=control_intervals.size();
    if (nc==0)
      return false;

    forced_response.resize(np*nax,nc*nax);
    free_response.resize(np*nax,nax);

    free_response.setZero();
    free_response.block(0,0,nax,nax)=-Eigen::MatrixXd::Identity(nax,nax)*1/control_intervals(0);

    forced_response.setZero();
    forced_response.block(0,0,nax,nax)=Eigen::MatrixXd::Identity(nax,nax)*1/control_intervals(0);
    for (unsigned int idx=1;idx<nc;idx++)
    {
      forced_response.block(nax*idx,nax*(idx-1),nax,nax)=-Eigen::MatrixXd::Identity(nax,nax)*1/control_intervals(idx);
      forced_response.block(nax*idx,nax*idx,nax,nax)    = Eigen::MatrixXd::Identity(nax,nax)*1/control_intervals(idx);
    }

    return true;
  } 

  bool quadraticControlIntervals ( const double& control_horizon_time, const unsigned int& n_control, const double& first_interval, Eigen::VectorXd& control_intervals, Eigen::VectorXd& prediction_time )
  {
    assert(first_interval>0);
    assert(control_horizon_time>0);
    assert(n_control>0);
    
    unsigned int n_steps=control_horizon_time/first_interval;
    
    double a = (n_steps-1)/std::pow(n_control-1,2);
    double b = -2.0*(n_steps-1)/std::pow(n_control-1,2);
    double c = 1+a;
    
    prediction_time.resize(n_control);
    control_intervals.resize(n_control);
    for (unsigned int ic=0;ic<n_control;ic++)
    {
      prediction_time(ic)=first_interval*std::round(a* std::pow(ic+1,2)+b*(ic+1)+c);
      if (ic==0)
        control_intervals(ic)=prediction_time(ic);
      else
        control_intervals(ic)=prediction_time(ic)-prediction_time(ic-1);
    }
    return true;
  }

  bool constantControlIntervals ( const double& control_horizon_time, const unsigned int& n_control, const double& sampling_period, Eigen::VectorXd& control_intervals, Eigen::VectorXd& prediction_time )
  {
    assert(control_horizon_time>0);
    assert(n_control>0);

    double step_size=sampling_period*std::round(control_horizon_time/n_control/sampling_period);

    prediction_time.resize(n_control);
    control_intervals.resize(n_control);
    for (unsigned int ic=0;ic<n_control;ic++)
    {
      prediction_time(ic)=step_size*(ic+1);
      if (ic==0)
        control_intervals(ic)=prediction_time(ic);
      else
        control_intervals(ic)=prediction_time(ic)-prediction_time(ic-1);
    }
    return true;
  }

  void splitResponses ( const Eigen::MatrixXd& free_response, 
                        Eigen::MatrixXd& velocity_free_response, 
                        Eigen::MatrixXd& position_free_response, 
                        const Eigen::MatrixXd& forced_response, 
                        Eigen::MatrixXd& velocity_forced_response, 
                        Eigen::MatrixXd& position_forced_response, 
                        const unsigned int& nax )
  {
    velocity_free_response.resize(free_response.rows()/2,free_response.cols()/2);
    position_free_response.resize(free_response.rows()/2,free_response.cols());
    velocity_forced_response.resize(forced_response.rows()/2,forced_response.cols());
    position_forced_response.resize(forced_response.rows()/2,forced_response.cols());
    
    unsigned int np = free_response.rows()/2/nax;
    for (unsigned int ip=0; ip<np;ip++)
    {
      velocity_free_response.block(   ip*nax,0,nax,free_response.cols()/2)   = free_response.block(  ip*2*nax+nax,nax,nax,free_response.cols()/2);
      position_free_response.block(   ip*nax,0,nax,free_response.cols())   = free_response.block(  ip*2*nax,    0,nax,free_response.cols());
      velocity_forced_response.block( ip*nax,0,nax,forced_response.cols()) = forced_response.block(ip*2*nax+nax,0,nax,forced_response.cols());
      position_forced_response.block( ip*nax,0,nax,forced_response.cols()) = forced_response.block(ip*2*nax,    0,nax,forced_response.cols());
    }
    
  }

  namespace
  {
    // frameVelocityDerivatives() with caller-provided 6 x nv buffers, so that
    // ThorQP does not allocate at every cycle.
    void frameVelocityDerivatives ( const pinocchio::Model& model, pinocchio::Data& data,
                                    pinocchio::FrameIndex frame_id,
                                    Eigen::Matrix<double, 6, Eigen::Dynamic>& local_dq,
                                    Eigen::Matrix<double, 6, Eigen::Dynamic>& local_dv,
                                    Eigen::Matrix<double, 3, Eigen::Dynamic>& dv_dq,
                                    Eigen::Matrix<double, 3, Eigen::Dynamic>& dv_ddq )
    {
      // Pinocchio only writes the columns of the frame's supporting joints.
      local_dq.setZero(6, model.nv);
      local_dv.setZero(6, model.nv);
      pinocchio::getFrameVelocityDerivatives(model, data, frame_id, pinocchio::LOCAL, local_dq, local_dv);
      const Eigen::Matrix3d R = data.oMf[frame_id].rotation();
      const Eigen::Vector3d v = R * pinocchio::getFrameVelocity(model, data, frame_id, pinocchio::LOCAL).linear();

      // v = R v_local: d/dq_i = R dv_local/dq_i + (dR/dq_i) v_local, dR/dq_i = [omega_i]x R.
      dv_ddq.resize(3, model.nv);
      dv_dq.resize(3, model.nv);
      dv_ddq.noalias() = R * local_dv.topRows<3>();
      dv_dq.noalias() = R * local_dq.topRows<3>();
      for (Eigen::Index i = 0; i < model.nv; ++i)
      {
        const Eigen::Vector3d omega = R * local_dv.col(i).tail<3>();
        dv_dq.col(i) += omega.cross(v);
      }
    }
  }

  void frameVelocityDerivatives ( const pinocchio::Model& model, pinocchio::Data& data,
                                  pinocchio::FrameIndex frame_id,
                                  Eigen::Matrix<double, 3, Eigen::Dynamic>& dv_dq,
                                  Eigen::Matrix<double, 3, Eigen::Dynamic>& dv_ddq )
  {
    Eigen::Matrix<double, 6, Eigen::Dynamic> local_dq, local_dv;
    frameVelocityDerivatives(model, data, frame_id, local_dq, local_dv, dv_dq, dv_ddq);
  }

  ThorQP::ThorQP()
    : m_are_matrices_updated(false),
      m_use_input_blocking(true),
      m_are_position_bounds_active(false),
      m_are_torque_bounds_active(false),
      m_nc(0),
      m_nax(0),
      m_control_horizon_time(0.0),
      m_dt(0.0),
      m_lambda_acc(0.0),
      m_lambda_tau(0.0),
      m_lambda_scaling(0.0),
      m_lambda_clik(0.0),
      m_lambda_jerk(0.0),
      m_use_cbf(false),
      m_a_s(0.0),
      m_T_r(0.0),
      m_C(0.0),
      m_gamma(0.0),
      m_delta_H(0.0),
      m_h(0.0),
      m_n_obstacle_points(0)
  {
  }

  void ThorQP::setCBFParameters ( const double& a_s, const double& T_r, const double& C, const double& gamma )
  {
    m_a_s=a_s;
    m_T_r=T_r;
    m_C=C;
    m_gamma=gamma;
    
  }

  void ThorQP::setPinocchioModel ( const pinocchio::Model& model )
  {
    m_model=model;
    m_data=pinocchio::Data(m_model);
    // m_eeFrameId = model.getFrameId("end_effector"); // Example, if you have an end-effector frame
  }

  void ThorQP::setConstraints ( const Eigen::VectorXd& qmax, const Eigen::VectorXd& qmin, const Eigen::VectorXd& Dqmax, const Eigen::VectorXd& DDqmax, const Eigen::VectorXd& tau_max )
  {
    m_qmax=qmax;
    m_qmin=qmin;
    m_Dqmax=Dqmax;
    m_DDqmax=DDqmax;
    m_tau_max=tau_max;
    m_are_matrices_updated=false;
    
  }

  void ThorQP::activatePositionBounds(const bool enable_pos_bounds)
  {
    if (m_are_position_bounds_active!=enable_pos_bounds)
    {
      m_are_position_bounds_active=enable_pos_bounds;
      m_are_matrices_updated=false;
  //    ROS_INFO("Position bounds activated. Execute update matrices to load the new options.");
    }
  }

  void ThorQP::activateTorqueBounds(const bool enable_tau_bounds)
  {
    if (m_are_torque_bounds_active!=enable_tau_bounds)
    {
      m_are_torque_bounds_active=enable_tau_bounds;
      m_are_matrices_updated=false;
  //    ROS_INFO("Torque bounds activated. Execute update matrices to load the new options.");
    }
  }

  void ThorQP::activateCbfBounds(const bool enable_cbf_bounds)
  {
    if (m_use_cbf!=enable_cbf_bounds)
    {
      m_use_cbf=enable_cbf_bounds;
      m_are_matrices_updated=false;
    }
  }

  void ThorQP::activateCbfMoveAway(const bool /*enable_cbf_move_away*/)
  {
  }

  bool ThorQP::arePositionBoundsActive()
  {
    return m_are_position_bounds_active;
  }

  void ThorQP::setIntervals ( const unsigned int& num_of_intervals,
                              const unsigned int& num_of_joints,
                              const double& control_horizon_time, 
                              const double& computing_period )
  {
    m_dt=computing_period;
    m_control_horizon_time=control_horizon_time;
    m_nax=num_of_joints;
    m_nc=num_of_intervals;
    
    m_are_matrices_updated=false;

    m_use_input_blocking=true;
    
    m_sol.resize( (m_nax+1)*m_nc);
    m_sol.setZero();
    
  }

  void ThorQP::setIntervals ( const unsigned int& num_of_intervals,
                              const unsigned int& num_of_joints,
                              const double& control_horizon_time,
                              const double& computing_period,
                              const bool & use_input_blocking)
  {
    m_dt=computing_period;
    m_control_horizon_time=control_horizon_time;
    m_nax=num_of_joints;
    m_nc=num_of_intervals;

    m_are_matrices_updated=false;

    if (use_input_blocking)
      m_use_input_blocking=true;
    else
      m_use_input_blocking=false;

    m_sol.resize( (m_nax+1)*m_nc);
    m_sol.setZero();

  }

  void ThorQP::setWeigthFunction ( const double& lambda_acc, 
                                  const double& lambda_tau, 
                                  const double& lambda_jerk, 
                                  const double& lambda_scaling, 
                                  const double& lambda_clik )
{
  m_lambda_acc=lambda_acc;
  m_lambda_tau=lambda_tau;
  m_lambda_jerk=lambda_jerk;
  m_lambda_scaling=lambda_scaling;
  m_lambda_clik=lambda_clik;
  m_are_matrices_updated=false;
}

  void ThorQP::setCbfIds(const std::vector<unsigned int>& frameIds, const int& num_ph)
  {
    m_frameIds = frameIds;
    m_frame_radii.assign(frameIds.size(), 0.0);
    m_n_obstacle_points = num_ph;
    m_are_matrices_updated = false;
  }

  void ThorQP::updateMatrices()
  {
    m_CE.resize((m_nax+1)*m_nc,0);
    m_ce0.resize(0);
    if (m_use_input_blocking)
      quadraticControlIntervals(m_control_horizon_time,m_nc,m_dt,m_control_intervals,m_prediction_time);
    else
      constantControlIntervals(m_control_horizon_time,m_nc,m_dt,m_control_intervals,m_prediction_time);
    computeEvolutionMatrix(m_prediction_time,m_control_intervals,m_nax,m_free_response,m_forced_response);
    computeJerkEvolutionMatrix(m_prediction_time,m_control_intervals,m_nax,m_jerk_free_response,m_jerk_forced_response);
    thor::math::splitResponses(m_free_response,m_velocity_free_resp,m_position_free_resp,m_forced_response,m_velocity_forced_resp,m_position_forced_resp,m_nax);
    m_lb.resize(m_nc*(m_nax+1));
    m_ub.resize(m_nc*(m_nax+1));
    for (unsigned int ic=0;ic<m_nc;ic++)
    {
      //// std::cout << "DDqMAX= "<<m_DDqmax << std::endl;
      m_lb.segment(ic*m_nax,m_nax) = -m_DDqmax;
      m_ub.segment(ic*m_nax,m_nax) =  m_DDqmax;
    }
    m_lb.tail(m_nc).setConstant(m_min_scaling);
    m_ub.tail(m_nc).setConstant(m_max_scaling);

    /*
    * I u > lb    ->  I*u+lb>0
    * I u < ub    -> -I*u+ub>0
    * Fv u +fv*v0 > -Dqmax    ->   Fv*u+fv*v0+Dqmax>0
    * Fv u +fv*v0 <  Dqmax    ->  -Fv*u-fv*v0+Dqmax>0
    * 
    * A^T =[I            -I          Fv^T    -Fv^T ]
    *       nc*(nax+1)   nc*(nax+1)  nc*nax  nc*nax
    * 
    * 
    */
    m_CI.resize(m_nc*(m_nax+1),  4*m_nc*m_nax+2*m_nc);
    m_CI.setZero();
    m_CI.block(0,0,m_nc*(m_nax+1),m_nc*(m_nax+1)).setIdentity();
    m_CI.block(0,m_nc*(m_nax+1),m_nc*(m_nax+1),m_nc*(m_nax+1))=-m_CI.block(0,0,m_nc*(m_nax+1),m_nc*(m_nax+1));
    m_CI.block(0,2*m_nc*(m_nax+1),           m_nc*m_nax,m_nc*m_nax)=m_velocity_forced_resp.transpose();
    m_CI.block(0,2*m_nc*(m_nax+1)+m_nc*m_nax,m_nc*m_nax,m_nc*m_nax)=-m_velocity_forced_resp.transpose();
    m_ci0.resize(m_CI.cols());
    m_ci0.setZero();
    m_ci0.head(m_nc*(m_nax+1))=-m_lb; 
    m_ci0.segment(m_nc*(m_nax+1),m_nc*(m_nax+1))=m_ub;
    for (unsigned int ic=0;ic<m_nc;ic++)
    {
      m_ci0.segment(2*m_nc*(m_nax+1)           +ic*m_nax,m_nax)=m_Dqmax;  // A^T*u>-Dqmax -> A^T*u+Dqmax>0
      m_ci0.segment(2*m_nc*(m_nax+1)+m_nax*m_nc+ic*m_nax,m_nax)=m_Dqmax; // A^T*u<Dqmax ->  A^T*u-Dqmax<0 ->  -A^T*u+Dqmax>0
    }

    /*
    * POSITION BOUNDS
    *
    * Fp u + fp*[p0;v0] > qmin ->  Fp*u+fp*[p0;v0]-qmin>0
    * Fp u + fp*[p0;v0] < qmax -> -Fp*u-fp*[p0;q0]+qmax>0
    *
    * INVARIANCE CONSTRAINTS
    *
    * K=0.99*DDqmax/Dqmax
    *  (K*Fp+Fv)*u+(K*fp+fv)*[p0;v0]-K*qmin>0
    * -(K*Fp+Fv)*u-(K*fp+fv)*[p0;v0]+K*qmax>0
    *
    *
    * A^T = [ A^T   Fp^T   -Fp^T   (K*Fp+Fv)^T   -(K*Fp+Fv)^T ]
    *              nc*nax  nc*nax    nc*nax          nc*nax
    *
    */

    if (m_are_position_bounds_active)
    {
      m_CI.conservativeResize(m_nc*(m_nax+1),  8*m_nc*m_nax+2*m_nc);
      m_CI.block(0,4*m_nc*m_nax+2*m_nc,m_nc*(m_nax+1),4*m_nc*m_nax).setZero();

      m_CI.block(0,4*m_nc*m_nax+2*m_nc,m_nc*m_nax,m_nc*m_nax)=m_position_forced_resp.transpose();
      m_CI.block(0,5*m_nc*m_nax+2*m_nc,m_nc*m_nax,m_nc*m_nax)=-m_position_forced_resp.transpose();

      m_ci0.conservativeResize(8*m_nc*m_nax+2*m_nc);
      m_ci0.tail(4*m_nc*m_nax).setZero();
      for (unsigned int ic=0;ic<m_nc;ic++)
      {
        m_ci0.segment(4*m_nc*m_nax+2*m_nc+ic*m_nax,m_nax)=-m_qmin;
        m_ci0.segment(5*m_nc*m_nax+2*m_nc+ic*m_nax,m_nax)= m_qmax;
      }

      Eigen::MatrixXd Kinv(m_nc*m_nax,m_nc*m_nax);
      Kinv.setZero();
      Eigen::MatrixXd Kinv_block=0.99*(m_DDqmax.cwiseQuotient(m_Dqmax)).asDiagonal();
      for (unsigned int ic=0;ic<m_nc;ic++)
      {
        Kinv.block(ic*m_nax,ic*m_nax,m_nax,m_nax)=Kinv_block;
        m_ci0.segment(6*m_nc*m_nax+2*m_nc+ic*m_nax,m_nax)=-Kinv_block*m_qmin;
        m_ci0.segment(7*m_nc*m_nax+2*m_nc+ic*m_nax,m_nax)= Kinv_block*m_qmax;
      }
      m_CI.block(0,6*m_nc*m_nax+2*m_nc,m_nc*m_nax,m_nc*m_nax)=(Kinv*m_position_forced_resp+m_velocity_forced_resp).transpose();
      m_CI.block(0,7*m_nc*m_nax+2*m_nc,m_nc*m_nax,m_nc*m_nax)=-(Kinv*m_position_forced_resp+m_velocity_forced_resp).transpose();

      m_invariance_free_resp=Kinv*m_position_free_resp;

      m_invariance_free_resp.rightCols(m_nax)+=m_velocity_free_resp;
    }

    /*
    * H u + b > -tau_max    ->  H*u+b+tau_max>0
    * H u + b <  tau_max    -> -H*u-b+tau_max>0
    * A^T =[H            -H          ]
    *       nc*(nax+1)   nc*(nax+1)
    *
    *
    */
  //   if (m_are_torque_bounds_active)
  //   {
  //     m_CI.conservativeResize(m_nc*(m_nax+1),  10*m_nc*m_nax+2*m_nc);
  //     m_CI.block(0,8*m_nc*m_nax+2*m_nc,m_nc*(m_nax+1),2*m_nc*m_nax).setZero();

  // //    m_CI.block(0,8*m_nc*m_nax+2*m_nc,m_nc*m_nax,m_nc*m_nax)=m_position_forced_resp.transpose();
  // //    m_CI.block(0,9*m_nc*m_nax+2*m_nc,m_nc*m_nax,m_nc*m_nax)=-m_position_forced_resp.transpose();

  //     m_ci0.conservativeResize(10*m_nc*m_nax+2*m_nc);
  //     m_ci0.tail(2*m_nc*m_nax).setZero();
  //     for (unsigned int ic=0;ic<m_nc;ic++)
  //     {
  //       m_ci0.segment(8*m_nc*m_nax+2*m_nc+ic*m_nax,m_nax)= m_tau_max;
  //       m_ci0.segment(9*m_nc*m_nax+2*m_nc+ic*m_nax,m_nax)= m_tau_max;
  //     }
  //   }
    // 1 constraint for each robot sphere, obstacle point and prediction interval
    if (m_use_cbf)
    {
      int old_cols=m_CI.cols();
      m_CI.conservativeResize(m_CI.rows(), m_CI.cols() + m_n_obstacle_points * m_frameIds.size() * m_nc);
      m_ci0.conservativeResize(m_ci0.size() + m_n_obstacle_points * m_frameIds.size() * m_nc);

      m_CI.block(0,old_cols, m_CI.rows(), m_n_obstacle_points * m_frameIds.size()*m_nc).setZero();
      m_ci0.tail(m_n_obstacle_points * m_frameIds.size()*m_nc).setZero();
    }

    m_next_position_forced_resp=m_position_forced_resp.topRows(m_nax);
    m_next_position_free_resp=m_position_free_resp.topRows(m_nax);
    
    // forced'*free    to be multiplied by v0
    m_f_vel.resize((m_nax+1)*m_nc,m_nax);
    m_f_vel.setZero();
    m_f_vel.topRows(m_nc*m_nax)=m_velocity_forced_resp.transpose()*m_velocity_free_resp;
    
    // m_lambda_clik* forced'*free  to be multiplied by x0 (works only on the first step)
    m_f_pos.resize((m_nax+1)*m_nc,2*m_nax);
    m_f_pos.setZero();
    m_f_pos.topRows(m_nc*m_nax)=m_lambda_clik*m_next_position_forced_resp.transpose()*m_next_position_free_resp;

    // m_lambda_scaling* ones to be multiplied by sref
    m_f_scaling.resize((m_nax+1)*m_nc);
    m_f_scaling.setZero();
    m_f_scaling.tail(m_nc).setConstant(-m_lambda_scaling);
    
    m_do_scaling.resize(m_nax*m_nc,m_nc);
    m_do_scaling.setZero();
    for (unsigned int ic=0;ic<m_nc;ic++)
      m_do_scaling.block(ic*m_nax,ic,m_nax,1).setOnes();
    
    m_H_fixed.resize((m_nax+1)*m_nc,(m_nax+1)*m_nc);
    m_H_fixed.setZero();

    m_H_variable.resize((m_nax+1)*m_nc,(m_nax+1)*m_nc);
    m_H_variable.setZero();
    
    m_H_fixed.block(0,0,m_nax*m_nc,m_nax*m_nc) =  m_velocity_forced_resp.transpose()*m_velocity_forced_resp+
                                                  m_lambda_acc*Eigen::MatrixXd::Identity(m_nax*m_nc,m_nax*m_nc) +
                                                  m_lambda_clik*m_next_position_forced_resp.transpose()*m_next_position_forced_resp +
                                                  m_lambda_jerk*m_jerk_forced_response.transpose()*m_jerk_forced_response;
    m_H_fixed.block(m_nax*m_nc,m_nax*m_nc,m_nc,m_nc)  =Eigen::MatrixXd::Identity(m_nc,m_nc)*m_lambda_scaling;
    
    m_H.resize((m_nax+1)*m_nc,(m_nax+1)*m_nc);
    m_f.resize((m_nax+1)*m_nc);
    m_H.setZero();
    m_f.setZero();
    
    m_prediction_pos.resize(m_nax*m_nc);
    m_prediction_vel.resize(m_nax*m_nc);
    resizeWorkspace();
    m_are_matrices_updated=true;
  }

  void ThorQP::resizeWorkspace()
  {
    const Eigen::Index n_x = static_cast<Eigen::Index>(m_nc) * (m_nax + 1);
    const Eigen::Index n_u = static_cast<Eigen::Index>(m_nc) * m_nax;
    // The first 2 n_x columns of m_CI are the variable bounds, passed to qpmad
    // as simple bounds.
    const Eigen::Index n_general = m_CE.cols() + m_CI.cols() - 2 * n_x;
    m_ws.dqt.resize(n_u, m_nc);
    m_ws.tmp_u.resize(n_u);
    m_ws.qp_H.resize(n_x, n_x);
    m_ws.qp_A.resize(n_general, n_x);
    m_ws.qp_lower.resize(n_general);
    m_ws.qp_upper.resize(n_general);
    m_ws.ci0.resize(m_CI.cols());
    m_ws.ci0_base.resize(m_CI.cols());
    m_ws.last_sol.resize(n_x);
    m_ws.last_ci0.resize(m_CI.cols());
    m_sol.conservativeResize(n_x);
    m_cbf_reference.resize(n_u);

    if (!m_use_cbf)
      return;
    const Eigen::Index n_cbf = static_cast<Eigen::Index>(m_n_obstacle_points) * m_nc * m_frameIds.size();
    const Eigen::Index n_nodes = static_cast<Eigen::Index>(m_nc + 1) * m_frameIds.size() * m_n_obstacle_points;
    const Eigen::Index nv = m_model.nv;
    m_ws.last_cbf_columns.resize(m_CI.rows(), n_cbf);
    m_ws.q_ref.resize(n_u);
    m_ws.dq_ref.resize(n_u);
    m_ws.zero_acc.setZero(m_nax);
    m_ws.c.resize(n_nodes);
    m_ws.G.resize(n_nodes, n_u);
    m_ws.dh_dq.resize(m_nax);
    m_ws.dh_ddq.resize(m_nax);
    m_ws.J.resize(6, nv);
    m_ws.dJ.resize(6, nv);
    m_ws.local_dq.resize(6, nv);
    m_ws.local_dv.resize(6, nv);
    m_ws.v_dq.resize(3, nv);
    m_ws.v_dv.resize(3, nv);
  }

  void ThorQP::computeActualMatrices ( const Eigen::VectorXd& targetDq, const Eigen::VectorXd& next_targetQ, const double& target_scaling, const Eigen::VectorXd& x0 )
  {
    // Products are written with noalias() into preallocated buffers, so that
    // the cost is built without heap allocations.
    Eigen::MatrixXd& DQT = m_ws.dqt;
    DQT.noalias() = targetDq.asDiagonal()*m_do_scaling;
    m_H_variable.block(0,m_nax*m_nc,m_nax*m_nc,m_nc).noalias()=-m_velocity_forced_resp.transpose()*DQT;
    m_H_variable.block(m_nax*m_nc,0,m_nc,m_nax*m_nc)=m_H_variable.block(0,m_nax*m_nc,m_nax*m_nc,m_nc).transpose();
    m_H_variable.block(m_nax*m_nc,m_nax*m_nc,m_nc,m_nc).noalias()=DQT.transpose()*DQT;

    m_f.noalias() = m_f_vel*x0.tail(m_nax);
    m_f.noalias() += m_f_pos*x0;
    m_f += m_f_scaling*target_scaling;
    m_f.head(m_nc*m_nax).noalias() -= m_lambda_clik*(m_next_position_forced_resp.transpose()*next_targetQ);
    m_ws.tmp_u.noalias() = m_velocity_free_resp*x0.tail(m_nax);
    m_f.tail(m_nc).noalias() -= DQT.transpose()*m_ws.tmp_u;

    // The jerk Hessian is constant and lives in m_H_fixed; only the term of the
    // acceleration applied in the previous cycle changes.
    m_ws.tmp_u.noalias() = m_jerk_free_response*m_sol.head(m_nax);
    m_f.head(m_nax*m_nc).noalias() += m_lambda_jerk * (m_jerk_forced_response.transpose()*m_ws.tmp_u);

    m_H=m_H_fixed+m_H_variable;
  }

  void ThorQP::setInitialState ( const Eigen::VectorXd& x0 )
  {
    assert(x0.size()==m_nax*2);
    // std::cout << "Setting initial state." << std::endl;
    m_state=x0;
    // std::cout << "Initial state: " << m_state.transpose() << std::endl;
    for (unsigned int ic=0; ic<m_nc; ic++)
    {
      // std::cout << "Setting prediction pos and vel for control interval " << ic << std::endl;
      m_prediction_pos.block(ic*m_nax,0,m_nax,1)=x0.head(m_nax);
      m_prediction_vel.block(ic*m_nax,0,m_nax,1).setZero();
    }
    
  }

  void ThorQP::updateState ( const Eigen::VectorXd& next_acc )
  {
    m_state.head(m_nax)+=(m_state.tail(m_nax)+0.5*next_acc*m_dt)*m_dt;
    m_state.tail(m_nax)+=next_acc*m_dt;
  }

  Eigen::VectorXd ThorQP::getState()
  {
    return m_state;
  }

  void ThorQP::doSolve ( const ThorInput& in, ThorOutput& out )
  {
    const Eigen::VectorXd& targetDq = in.target_dq;
    const Eigen::VectorXd& next_targetQ = in.next_target_q;
    const double& target_scaling = in.target_scaling;
    const Eigen::VectorXd& x0 = in.x0;
    Eigen::VectorXd& next_acc = out.next_acc;
    double& next_scaling = out.next_scaling;
    out.cbf = CbfDiagnostics();
    // Build cost matrices and baseline inequality vector
    computeActualMatrices(targetDq,next_targetQ,target_scaling,x0);
    Eigen::VectorXd& ci0 = m_ws.ci0;
    ci0 = m_ci0;
    // Velocity bounds
    ci0.segment(2*m_nc*(m_nax+1)           ,m_nax*m_nc).noalias()+=m_velocity_free_resp*x0.tail(m_nax); // vel lower bounds
    ci0.segment(2*m_nc*(m_nax+1)+m_nax*m_nc,m_nax*m_nc).noalias()-=m_velocity_free_resp*x0.tail(m_nax); // vel upper bounds
    // Position bounds
    if (m_are_position_bounds_active)
    {
      ci0.segment(2*m_nc*(m_nax+1)+2*m_nax*m_nc,m_nax*m_nc).noalias()+=m_position_free_resp*x0; // pos lower bounds
      ci0.segment(2*m_nc*(m_nax+1)+3*m_nax*m_nc,m_nax*m_nc).noalias()-=m_position_free_resp*x0; // pos upper bounds
      ci0.segment(2*m_nc*(m_nax+1)+4*m_nax*m_nc,m_nax*m_nc).noalias()+=m_invariance_free_resp*x0; // invariance lower constraint
      ci0.segment(2*m_nc*(m_nax+1)+5*m_nax*m_nc,m_nax*m_nc).noalias()-=m_invariance_free_resp*x0; // invariance upper constraint
    }
    // if  (m_are_torque_bounds_active)
    // {
    //   for (unsigned int idx=0;idx<m_nc;idx++)
    //   {
    //     ci0.segment(2*m_nc*(m_nax+1)+6*m_nax*m_nc+idx*m_nc,m_nax)+=torque_nonlinear_part; // torque lower bounds
    //     ci0.segment(2*m_nc*(m_nax+1)+7*m_nax*m_nc+idx*m_nc,m_nax)-=torque_nonlinear_part; // torque upper bounds
    //   }
    // }

    // Solve once, or, with the CBF, up to ThorConfig::cbf_linearizations times.
    // The CBF is linearized around the reference inputs u_ref: the solution of
    // the previous cycle for the first QP, then
    //   u_ref <- u_ref + damping (u* - u_ref)
    // after each QP of solution u*. The iteration stops early when
    // |u* - u_ref|_inf <= tolerance. The command is always the solution u* of
    // the last QP solved; a failure after the first QP keeps the previous one,
    // which satisfies its own linearization.
    m_ws.ci0_base = ci0;
    const Eigen::Index n_x = m_H.rows();
    const Eigen::Index n_cbf = m_use_cbf ? static_cast<Eigen::Index>(m_n_obstacle_points) * m_nc * m_frameIds.size() : 0;
    const Eigen::Index n_u = static_cast<Eigen::Index>(m_nc) * m_nax;
    // The first 2 n_x columns of m_CI are the variable bounds (identity rows):
    // qpmad receives them as simple bounds m_lb <= x <= m_ub, which it handles
    // much more cheaply than general constraints.
    const Eigen::Index n_bounds = 2 * n_x;
    const Eigen::Index n_eq = m_CE.cols();
    const unsigned int linearizations = m_use_cbf ? m_cbf_linearizations : 1;
    m_cbf_reference = m_sol.head(n_u);
    if (!m_cbf_reference.allFinite())
      m_cbf_reference.setZero();
    out.cbf_linearizations = 0;
    out.cbf_linearization_step = 0.0;
    for (unsigned int iteration = 0; iteration < linearizations; ++iteration)
    {
      ci0 = m_ws.ci0_base;
      CbfDiagnostics diag;
      bool cbf_rows = false;
      // CBF constraints: one for each control interval, robot sphere and
      // obstacle sphere (see buildDiscreteCbf() and buildContinuousCbf()).
      if (m_use_cbf)
      {
        m_CI.rightCols(n_cbf).setZero();
        if (m_cbf_formulation == CbfFormulation::discrete)
          buildDiscreteCbf(in, ci0, diag);
        else
          buildContinuousCbf(in, ci0, diag);
        m_h = diag.h_min;
        cbf_rows = !(m_h > m_cbf_activation_distance);
      }
      const Eigen::Index n_in = m_CI.cols() - n_bounds - (m_use_cbf && !cbf_rows ? n_cbf : 0);
      const Eigen::Index n_general = n_eq + n_in;
      m_ws.qp_A.topRows(n_eq) = m_CE.transpose();
      m_ws.qp_A.middleRows(n_eq, n_in) = m_CI.middleCols(n_bounds, n_in).transpose();
      m_ws.qp_lower.head(n_eq) = -m_ce0;
      m_ws.qp_upper.head(n_eq) = -m_ce0;
      m_ws.qp_lower.segment(n_eq, n_in) = -ci0.segment(n_bounds, n_in);
      m_ws.qp_upper.segment(n_eq, n_in).setConstant(std::numeric_limits<double>::infinity());

      // qpmad factorizes H in place; retain the original cost matrix in ThorQP.
      m_ws.qp_H = m_H;
      std::string failure;
      try
      {
        const qpmad::Solver::ReturnStatus status = m_solver.solve(
          m_sol, m_ws.qp_H, m_f, m_lb, m_ub,
          m_ws.qp_A.topRows(n_general), m_ws.qp_lower.head(n_general), m_ws.qp_upper.head(n_general));
        if (status != qpmad::Solver::OK || !m_sol.allFinite())
          failure = "Problem is not feasible. Check the constraints and the target values.";
      }
      catch (const std::runtime_error& error)
      {
        failure = std::string("Quadratic program failed: ") + error.what();
      }

      if (!failure.empty())
      {
        if (iteration == 0)
        {
          // The solution is used by the next cycle (jerk term, CBF linearization).
          m_sol.setZero();
          throw std::runtime_error(failure);
        }
        m_sol = m_ws.last_sol;
        m_ci0_solved = m_ws.last_ci0;
        m_CI.rightCols(n_cbf) = m_ws.last_cbf_columns;
        break;
      }

      out.cbf = diag;
      out.cbf_linearizations = iteration + 1;
      m_ci0_solved = ci0;
      if (!cbf_rows || iteration + 1 == linearizations)
        break;
      out.cbf_linearization_step = (m_sol.head(n_u) - m_cbf_reference).lpNorm<Eigen::Infinity>();
      if (out.cbf_linearization_step <= m_cbf_linearization_tolerance)
        break;
      m_ws.last_sol = m_sol;
      m_ws.last_ci0 = ci0;
      m_ws.last_cbf_columns = m_CI.rightCols(n_cbf);

      m_cbf_reference += m_cbf_linearization_damping * (m_sol.head(n_u) - m_cbf_reference);
      // The prediction is the linearization point of the continuous CBF.
      m_prediction_vel.noalias() = m_velocity_forced_resp*m_cbf_reference;
      m_prediction_vel.noalias() += m_velocity_free_resp*x0.tail(m_nax);
      m_prediction_pos.noalias() = m_position_forced_resp*m_cbf_reference;
      m_prediction_pos.noalias() += m_position_free_resp*x0;
    }
    // The prediction of the command, used by the next cycle.
    m_prediction_vel.noalias() = m_velocity_forced_resp*m_sol.head(n_u);
    m_prediction_vel.noalias() += m_velocity_free_resp*x0.tail(m_nax);
    m_prediction_pos.noalias() = m_position_forced_resp*m_sol.head(n_u);
    m_prediction_pos.noalias() += m_position_free_resp*x0;

    // std::cout << "m_ci0 size: " << m_ci0.size() << " x 1" << std::endl;
    next_acc=m_sol.head(m_nax);
    // std::cout << "Next acceleration: " << next_acc.transpose() << std::endl;
    next_scaling=m_sol (m_nax*m_nc);
    // std::cout << "Predicted position: " << m_prediction_pos.transpose() << std::endl;
    // std::cout << "Prediction instants: " << m_prediction_time.transpose() << std::endl;
    // std::cout << "control intervals: " << m_control_intervals.transpose() << std::endl;
    m_next_position_forced_resp=m_position_forced_resp.topRows(m_nax);
    // std::cout << "Solution computed" << std::endl;
  }

  void ThorQP::buildContinuousCbf ( const ThorInput& in, Eigen::VectorXd& ci0, CbfDiagnostics& diag )
  {
    // Continuous-time CBF at the start of every interval. Interval i starts
    // from the current state (i == 0) or from the end of interval i-1 of the
    // previous prediction, and u_i is its acceleration:
    //   L_g h (J u_i + dJ dq) + L_f h + gamma h >= 0
    // h is evaluated at the worst-case distance between the surfaces,
    // d = |p_r - p_o| - r_robot - r_obstacle - delta_H: the perception error
    // inflates the obstacle sphere. cbf_python (and the paper) add
    // + h_chi delta_H instead, which relaxes the constraint and mixes m with
    // m/s; its first-order correction - h_chi delta_H would move the safe-set
    // boundary away from where the escape term of the barrier acts, and a robot
    // stopped on it could never move again (docs/robust_margin_deadlock.md).
    // With a single interval and delta_H = 0 this is the one-step controller of cbf_python.
    const Eigen::VectorXd& x0 = in.x0;
    const Eigen::Index n_cols = m_CI.cols();

    // Pinocchio needs 6 x nv buffers and only writes the columns of the frame's
    // supporting joints, so they must be zeroed before each use. All the
    // buffers are preallocated in m_ws (see resizeWorkspace()).
    Eigen::Matrix<double, 6, Eigen::Dynamic>& J = m_ws.J;
    Eigen::Matrix<double, 6, Eigen::Dynamic>& dJ = m_ws.dJ;
    double h_min = std::numeric_limits<double>::infinity();
    double h_now = std::numeric_limits<double>::infinity();

    for (unsigned int i = 0; i < m_nc; ++i)
    {
      const double t_start = (i == 0) ? 0.0 : m_prediction_time(i-1);
      using ConstRef = Eigen::Ref<const Eigen::VectorXd>;
      const ConstRef q  = (i == 0) ? ConstRef(x0.head(m_nax)) : ConstRef(m_prediction_pos.segment(m_nax*(i-1), m_nax));
      const ConstRef dq = (i == 0) ? ConstRef(x0.tail(m_nax)) : ConstRef(m_prediction_vel.segment(m_nax*(i-1), m_nax));
      pinocchio::computeForwardKinematicsDerivatives(m_model, m_data, q, dq, m_ws.zero_acc);
      pinocchio::updateFramePlacements(m_model, m_data);
      pinocchio::computeJointJacobiansTimeVariation(m_model, m_data, q, dq);

      for (std::size_t j = 0; j < m_frameIds.size(); ++j)
      {
        const unsigned int frameId = m_frameIds[j];
        const Eigen::Vector3d p_r = m_data.oMf[frameId].translation();
        const Eigen::Vector3d v_r = pinocchio::getFrameVelocity(m_model, m_data, frameId, pinocchio::LOCAL_WORLD_ALIGNED).linear();
        J.setZero();
        pinocchio::getFrameJacobian(m_model, m_data, frameId, pinocchio::LOCAL_WORLD_ALIGNED, J);
        dJ.setZero();
        pinocchio::getFrameJacobianTimeVariation(m_model, m_data, frameId, pinocchio::LOCAL_WORLD_ALIGNED, dJ);
        const Eigen::Vector3d dJdq = dJ.topRows<3>() * dq;

        for (std::size_t k = 0; k < static_cast<std::size_t>(m_n_obstacle_points); ++k)
        {
          // Obstacle point extrapolated with constant acceleration, as in d_min.
          const Eigen::Vector3d& a_o = in.obstacle_acc[k];
          const Eigen::Vector3d p_o = in.obstacle_pos[k] + in.obstacle_vel[k] * t_start + 0.5 * a_o * t_start * t_start;
          const Eigen::Vector3d v_o = in.obstacle_vel[k] + a_o * t_start;
          const double radius = m_frame_radii[j] + in.obstacle_radius[k] + in.obstacle_delta_H[k];

          const cbf::BarrierTerms b = cbf::barrierTerms(p_r, v_r, p_o, v_o, a_o, m_T_r, m_a_s, m_C, radius);

          if (i == 0)
            h_now = std::min(h_now, b.h);
          if (b.h < h_min)
          {
            h_min = b.h;
            diag.h_min = b.h;
            diag.interval = i;
            diag.frame_id = frameId;
            diag.distance = b.d + in.obstacle_delta_H[k];
            diag.v_robot = b.v_r;
            diag.v_obstacle = b.v_o;
            diag.obstacle_index = static_cast<unsigned int>(k);
          }

          // Store each barrier constraint as a column: CI^T x + ci0 >= 0.
          const Eigen::Index col = n_cols - m_n_obstacle_points * m_nc * (j+1) + m_n_obstacle_points * i + k;
          m_CI.col(col).segment(m_nax * i, m_nax).noalias() = J.topRows<3>().transpose() * b.lie_g;
          ci0(col) = b.lie_g.dot(dJdq) + b.lie_f + m_gamma * b.h;
        }
      }
    }
    diag.h_now = h_now;
  }

  void ThorQP::buildDiscreteCbf ( const ThorInput& in, Eigen::VectorXd& ci0, CbfDiagnostics& diag )
  {
    // Discrete-time CBF along the horizon. X_0 is the current state, X_m the
    // predicted state at the end of interval m-1 (affine in the inputs u).
    // For every interval i, robot sphere and obstacle sphere:
    //   h(X_{i+1}) >= alpha_i h(X_i),   alpha_i = max(0, 1 - gamma dt_i)
    // h is evaluated at the worst-case distance (radii + delta_H, see
    // buildContinuousCbf()) and linearized around the trajectory obtained by
    // applying the reference inputs u_ref (see doSolve()) from the current state:
    //   h(X_m(u)) ~ c_m + G_m u,   G_m = dh/dq dq_m/du + dh/ddq ddq_m/du
    // with dh/dq = grad_p^T J + lie_g^T dv/dq and dh/ddq = lie_g^T J.
    const Eigen::VectorXd& x0 = in.x0;
    const Eigen::Index n_cols = m_CI.cols();
    const Eigen::Index n_u = static_cast<Eigen::Index>(m_nc) * m_nax;
    const std::size_t n_frames = m_frameIds.size();
    const std::size_t n_obstacles = static_cast<std::size_t>(m_n_obstacle_points);

    // All the buffers are preallocated in m_ws (see resizeWorkspace()).
    const Eigen::VectorXd& u_ref = m_cbf_reference;
    Eigen::VectorXd& q_ref = m_ws.q_ref;
    Eigen::VectorXd& dq_ref = m_ws.dq_ref;
    q_ref.noalias() = m_position_forced_resp * u_ref;
    q_ref.noalias() += m_position_free_resp * x0;
    dq_ref.noalias() = m_velocity_forced_resp * u_ref;
    dq_ref.noalias() += m_velocity_free_resp * x0.tail(m_nax);

    // Affine barrier h ~ c + G u of every node and pair, indexed by node(m, j, k).
    auto node = [&](std::size_t m, std::size_t j, std::size_t k) { return static_cast<Eigen::Index>((m * n_frames + j) * n_obstacles + k); };
    Eigen::VectorXd& c = m_ws.c;
    Eigen::MatrixXd& G = m_ws.G;
    G.topRows(static_cast<Eigen::Index>(n_frames * n_obstacles)).setZero();  // the current state does not depend on u

    Eigen::Matrix<double, 3, Eigen::Dynamic>& v_dq = m_ws.v_dq;
    Eigen::Matrix<double, 3, Eigen::Dynamic>& v_dv = m_ws.v_dv;
    double h_min = std::numeric_limits<double>::infinity();
    double h_now = std::numeric_limits<double>::infinity();

    for (unsigned int m = 0; m <= m_nc; ++m)
    {
      const double t = (m == 0) ? 0.0 : m_prediction_time(m-1);
      using ConstRef = Eigen::Ref<const Eigen::VectorXd>;
      const ConstRef q  = (m == 0) ? ConstRef(x0.head(m_nax)) : ConstRef(q_ref.segment(m_nax*(m-1), m_nax));
      const ConstRef dq = (m == 0) ? ConstRef(x0.tail(m_nax)) : ConstRef(dq_ref.segment(m_nax*(m-1), m_nax));
      pinocchio::computeForwardKinematicsDerivatives(m_model, m_data, q, dq, m_ws.zero_acc);
      pinocchio::updateFramePlacements(m_model, m_data);

      for (std::size_t j = 0; j < n_frames; ++j)
      {
        const unsigned int frameId = m_frameIds[j];
        const Eigen::Vector3d p_r = m_data.oMf[frameId].translation();
        const Eigen::Vector3d v_r = pinocchio::getFrameVelocity(m_model, m_data, frameId, pinocchio::LOCAL_WORLD_ALIGNED).linear();
        frameVelocityDerivatives(m_model, m_data, frameId, m_ws.local_dq, m_ws.local_dv, v_dq, v_dv);

        for (std::size_t k = 0; k < n_obstacles; ++k)
        {
          // Obstacle point extrapolated with constant acceleration, as in d_min.
          const Eigen::Vector3d& a_o = in.obstacle_acc[k];
          const Eigen::Vector3d p_o = in.obstacle_pos[k] + in.obstacle_vel[k] * t + 0.5 * a_o * t * t;
          const Eigen::Vector3d v_o = in.obstacle_vel[k] + a_o * t;
          const double radius = m_frame_radii[j] + in.obstacle_radius[k] + in.obstacle_delta_H[k];

          const cbf::BarrierTerms b = cbf::barrierTerms(p_r, v_r, p_o, v_o, a_o, m_T_r, m_a_s, m_C, radius);

          if (m == 0)
            h_now = std::min(h_now, b.h);
          if (b.h < h_min)
          {
            h_min = b.h;
            diag.h_min = b.h;
            diag.interval = m;
            diag.frame_id = frameId;
            diag.distance = b.d + in.obstacle_delta_H[k];
            diag.v_robot = b.v_r;
            diag.v_obstacle = b.v_o;
            diag.obstacle_index = static_cast<unsigned int>(k);
          }

          const Eigen::Index n = node(m, j, k);
          if (m == 0)
          {
            c(n) = b.h;
            continue;
          }
          m_ws.dh_dq.noalias()  = b.grad_p.transpose() * v_dv;
          m_ws.dh_dq.noalias() += b.lie_g.transpose() * v_dq;
          m_ws.dh_ddq.noalias() = b.lie_g.transpose() * v_dv;
          G.row(n).noalias()  = m_ws.dh_dq * m_position_forced_resp.middleRows(m_nax*(m-1), m_nax);
          G.row(n).noalias() += m_ws.dh_ddq * m_velocity_forced_resp.middleRows(m_nax*(m-1), m_nax);
          c(n) = b.h - G.row(n).dot(u_ref);
        }
      }
    }
    diag.h_now = h_now;

    // Store each barrier constraint as a column: CI^T x + ci0 >= 0.
    for (unsigned int i = 0; i < m_nc; ++i)
    {
      const double alpha = std::max(0.0, 1.0 - m_gamma * m_control_intervals(i));
      for (std::size_t j = 0; j < n_frames; ++j)
      {
        for (std::size_t k = 0; k < n_obstacles; ++k)
        {
          const auto col = n_cols - static_cast<Eigen::Index>(n_obstacles * (m_nc * (j+1) - i) - k);
          const Eigen::Index next = node(i+1, j, k);
          const Eigen::Index prev = node(i, j, k);
          m_CI.col(col).head(n_u) = (G.row(next) - alpha * G.row(prev)).transpose();
          ci0(col) = c(next) - alpha * c(prev);
        }
      }
    }
  }

  namespace
  {
    void requireSize(const Eigen::VectorXd& v, Eigen::Index size, const char* name)
    {
      if (v.size() != size)
        throw std::invalid_argument(std::string(name) + " has size " + std::to_string(v.size()) +
                                    ", expected " + std::to_string(size));
    }
  }

  ThorQP::ThorQP ( const ThorConfig& config, const pinocchio::Model& model )
    : ThorQP()
  {
    setPinocchioModel(model);
    configure(config);
  }

  void ThorQP::configure ( const ThorConfig& config )
  {
    // Validate everything before touching the optimizer state.
    if (config.n_intervals == 0 || config.n_joints == 0)
      throw std::invalid_argument("n_intervals and n_joints must be positive");
    if (!(config.sampling_period > 0.0) || !(config.horizon_time >= config.sampling_period))
      throw std::invalid_argument("sampling_period must be positive and not larger than horizon_time");

    Eigen::VectorXd control_intervals, prediction_time;
    if (config.input_blocking)
      quadraticControlIntervals(config.horizon_time, config.n_intervals, config.sampling_period,
                                control_intervals, prediction_time);
    else
      constantControlIntervals(config.horizon_time, config.n_intervals, config.sampling_period,
                               control_intervals, prediction_time);
    if (!control_intervals.allFinite() || !(control_intervals.array() > 0.0).all())
      throw std::invalid_argument("the horizon cannot be split into n_intervals non-empty intervals: "
                                  "reduce n_intervals or sampling_period, or increase horizon_time");

    const Eigen::Index nax = config.n_joints;
    requireSize(config.qmax, nax, "qmax");
    requireSize(config.qmin, nax, "qmin");
    requireSize(config.Dqmax, nax, "Dqmax");
    requireSize(config.DDqmax, nax, "DDqmax");
    if (config.tau_max.size() != 0)
      requireSize(config.tau_max, nax, "tau_max");
    if ((config.qmin.array() > config.qmax.array()).any())
      throw std::invalid_argument("qmin must not exceed qmax");
    if (!(config.Dqmax.array() > 0.0).all() || (config.DDqmax.array() < 0.0).any())
      throw std::invalid_argument("Dqmax must be positive and DDqmax non-negative");
    if (!(config.min_scaling <= config.max_scaling))
      throw std::invalid_argument("min_scaling must not exceed max_scaling");

    if (config.cbf)
    {
      if (m_model.nq != static_cast<int>(nax) || m_model.nv != static_cast<int>(nax))
        throw std::invalid_argument("the CBF needs a Pinocchio model with nq == nv == n_joints; "
                                    "set the model before configuring");
      if (!(config.a_s > 0.0) || !(config.C > 0.0))
        throw std::invalid_argument("a_s and C must be positive");
      if (!(config.T_r >= 0.0) || !(config.delta_H >= 0.0))
        throw std::invalid_argument("T_r and delta_H must not be negative");
      if (config.cbf_linearizations < 1)
        throw std::invalid_argument("cbf_linearizations must be at least 1");
      if (!(config.cbf_linearization_damping > 0.0 && config.cbf_linearization_damping <= 1.0))
        throw std::invalid_argument("cbf_linearization_damping must be in (0, 1]");
      if (!(config.cbf_linearization_tolerance >= 0.0))
        throw std::invalid_argument("cbf_linearization_tolerance must not be negative");
      if (config.n_obstacle_points < 0)
        throw std::invalid_argument("n_obstacle_points must not be negative");
      for (const unsigned int frame_id : config.cbf_frame_ids)
        if (frame_id >= m_model.frames.size())
          throw std::invalid_argument("cbf frame id " + std::to_string(frame_id) + " is not in the model");
      if (!config.cbf_frame_radii.empty() && config.cbf_frame_radii.size() != config.cbf_frame_ids.size())
        throw std::invalid_argument("cbf_frame_radii must be empty or have the size of cbf_frame_ids");
      for (const double radius : config.cbf_frame_radii)
        if (!(radius >= 0.0))
          throw std::invalid_argument("cbf_frame_radii must not be negative");
    }

    m_nc = config.n_intervals;
    m_nax = config.n_joints;
    m_control_horizon_time = config.horizon_time;
    m_dt = config.sampling_period;
    m_use_input_blocking = config.input_blocking;
    m_sol.setZero((m_nax+1)*m_nc);

    m_lambda_acc = config.lambda_acc;
    m_lambda_tau = config.lambda_tau;
    m_lambda_jerk = config.lambda_jerk;
    m_lambda_scaling = config.lambda_scaling;
    m_lambda_clik = config.lambda_clik;

    m_qmax = config.qmax;
    m_qmin = config.qmin;
    m_Dqmax = config.Dqmax;
    m_DDqmax = config.DDqmax;
    m_tau_max = config.tau_max;
    m_are_position_bounds_active = config.position_bounds;
    m_min_scaling = config.min_scaling;
    m_max_scaling = config.max_scaling;

    m_use_cbf = config.cbf;
    m_a_s = config.a_s;
    m_T_r = config.T_r;
    m_C = config.C;
    m_gamma = config.gamma;
    m_delta_H = config.delta_H;
    m_cbf_formulation = config.cbf_formulation;
    m_cbf_linearizations = config.cbf_linearizations;
    m_cbf_linearization_damping = config.cbf_linearization_damping;
    m_cbf_linearization_tolerance = config.cbf_linearization_tolerance;
    m_cbf_activation_distance = config.cbf_activation_distance;
    m_frameIds = config.cbf_frame_ids;
    m_frame_radii = config.cbf_frame_radii;
    m_frame_radii.resize(m_frameIds.size(), 0.0);
    m_n_obstacle_points = config.n_obstacle_points;

    updateMatrices();
  }

  ThorOutput ThorQP::solve ( const ThorInput& in )
  {
    if (needUpdate())
      throw std::logic_error("the QP matrices are outdated: call configure() or updateMatrices() first");

    requireSize(in.target_dq, static_cast<Eigen::Index>(m_nax) * m_nc, "target_dq");
    requireSize(in.next_target_q, m_nax, "next_target_q");
    requireSize(in.x0, 2 * static_cast<Eigen::Index>(m_nax), "x0");

    ThorOutput out;
    if (!m_use_cbf)
    {
      doSolve(in, out);
      return out;
    }
    // Inputs that leave some obstacle vectors empty are completed with the
    // defaults on a copy; complete inputs are used as they are, so that a
    // control loop that fills everything does not allocate here.
    const bool complete = !in.obstacle_pos.empty() && !in.obstacle_acc.empty() &&
                          !in.obstacle_radius.empty() && !in.obstacle_delta_H.empty();
    ThorInput with_defaults;
    if (!complete)
    {
      with_defaults = in;
      if (in.obstacle_pos.empty() && in.obstacle_vel.empty() && in.obstacle_acc.empty())
      {
        // No obstacle data: assume n_obstacle_points far-away, still points.
        with_defaults.obstacle_pos.assign(m_n_obstacle_points, Eigen::Vector3d::Constant(100.0));
        with_defaults.obstacle_vel.assign(m_n_obstacle_points, Eigen::Vector3d::Zero());
      }
      const std::size_t n = with_defaults.obstacle_pos.size();
      if (in.obstacle_acc.empty())
        with_defaults.obstacle_acc.assign(n, Eigen::Vector3d::Zero());
      if (in.obstacle_radius.empty())
        with_defaults.obstacle_radius.assign(n, 0.0);
      if (in.obstacle_delta_H.empty())
        with_defaults.obstacle_delta_H.assign(n, m_delta_H);
    }
    const ThorInput& full = complete ? in : with_defaults;

    const std::size_t n_obstacles = full.obstacle_pos.size();
    if (full.obstacle_vel.size() != n_obstacles || full.obstacle_acc.size() != n_obstacles ||
        full.obstacle_radius.size() != n_obstacles || full.obstacle_delta_H.size() != n_obstacles ||
        n_obstacles < static_cast<std::size_t>(m_n_obstacle_points))
    {
      throw std::invalid_argument("obstacle_pos, obstacle_vel and, if given, obstacle_acc, obstacle_radius and "
                                  "obstacle_delta_H must have the same size, at least n_obstacle_points (" +
                                  std::to_string(m_n_obstacle_points) + ")");
    }
    for (std::size_t k = 0; k < n_obstacles; ++k)
    {
      if (!(full.obstacle_radius[k] >= 0.0) || !(full.obstacle_delta_H[k] >= 0.0))
        throw std::invalid_argument("obstacle_radius and obstacle_delta_H must not be negative");
    }
    doSolve(full, out);
    return out;
  }

  std::vector<double> ThorQP::toLegacyDiagnostics ( const CbfDiagnostics& cbf )
  {
    return {cbf.h_min, static_cast<double>(cbf.interval), static_cast<double>(cbf.frame_id),
            cbf.distance, cbf.v_robot, cbf.v_obstacle};
  }

  std::vector<double> ThorQP::computedCostrainedSolution ( const Eigen::VectorXd& targetDq,
                                                           const Eigen::VectorXd& next_targetQ,
                                                           const double& target_scaling,
                                                           const Eigen::VectorXd& x0,
                                                           Eigen::VectorXd& next_acc,
                                                           double& next_scaling,
                                                           const std::vector<Eigen::Vector3d> &v_o_vector,
                                                           const std::vector<Eigen::Vector3d> &p_o_vector )
  {
    const ThorOutput out = solve({targetDq, next_targetQ, target_scaling, x0, p_o_vector, v_o_vector, {}, {}, {}});
    next_acc = out.next_acc;
    next_scaling = out.next_scaling;
    return toLegacyDiagnostics(out.cbf);
  }

  std::vector<double> ThorQP::computedCostrainedSolution ( const Eigen::VectorXd& targetDq,
                                                           const Eigen::VectorXd& next_targetQ,
                                                           const double& target_scaling,
                                                           const Eigen::VectorXd& x0,
                                                           Eigen::VectorXd& next_acc,
                                                           double& next_scaling )
  {
    const ThorOutput out = solve({targetDq, next_targetQ, target_scaling, x0, {}, {}, {}, {}, {}});
    next_acc = out.next_acc;
    next_scaling = out.next_scaling;
    return toLegacyDiagnostics(out.cbf);
  }

  bool ThorQP::computedUncostrainedSolution ( const Eigen::VectorXd& targetDq,
                                              const Eigen::VectorXd& next_targetQ,
                                              const double& target_scaling,
                                              const Eigen::VectorXd& x0,
                                              Eigen::VectorXd& next_acc,
                                              double& next_scaling )
  {
    computeActualMatrices(targetDq,next_targetQ,target_scaling,x0);
    
    m_svd.compute( m_H, Eigen::ComputeThinU | Eigen::ComputeThinV );
    m_sol=-m_svd.solve(m_f);
    next_acc=m_sol.head(m_nax);
    next_scaling=m_sol (m_nax*m_nc);
    return true;
  }

  Eigen::VectorXd ThorQP::getFirstPredictionPos()
  {
    if (m_prediction_pos.size()>0)
    {
      return m_prediction_pos.head(m_nax);
    }
    else
    {
      Eigen::VectorXd empty;
      return empty;
    }

  }
  Eigen::VectorXd ThorQP::getFirstPredictionVel()
  {
    if (m_prediction_vel.size()>0)
    {
      return m_prediction_vel.head(m_nax);
    }
    else
    {
      Eigen::VectorXd empty;
      return empty;
    }
  }

}
}
