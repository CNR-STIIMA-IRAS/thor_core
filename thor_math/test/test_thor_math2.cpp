#include <thor_math/thor_math.h>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <fstream>
// ...


void cubic_spline(double t, double T, double q_init, double q_final, double dq_init, double dq_final,
                  double& pos, double& vel, double& acc)
{
    if (t > T) t = T;
    double s = t / T;
    double s2 = s * s;
    double s3 = s2 * s;

    // Cubic Hermite spline (includes initial/final velocities if desired)
    pos = (2*s3 - 3*s2 + 1) * q_init
        + (s3 - 2*s2 + s) * T * dq_init
        + (-2*s3 + 3*s2) * q_final
        + (s3 - s2) * T * dq_final;

    vel = (6*s2 - 6*s) * (q_final - q_init) / T
        + (3*s2 - 4*s + 1) * dq_init
        + (-3*s2 + 2*s) * dq_final;

    acc = (12*s - 6) * (q_final - q_init) / (T * T)
        + (6*s - 4) * dq_init / T
        + (-6*s + 2) * dq_final / T;
}

void triangularWave(double t, const double& scaling, double& pos, double& vel, double& acc)
{
  double amplitude=0.4;
  double period=0.5;
  
  while(t>period)
    t-=period;
  
  acc=0;
  if (t<0.5*period)
  {
    pos=amplitude*t/(0.5*period);
    vel=amplitude/(0.5*period)*scaling;
  }
  else
  {
    pos=amplitude-amplitude*(t-0.5*period)/(0.5*period);
    vel=-amplitude/(0.5*period)*scaling;
  }
  
}
void human_circle(double t,  Eigen::Vector3d& pos, Eigen::Vector3d& vel)
{
  double radius = 0.3;               // Smaller, so stays inside workspace
  double omega = 2*M_PI/5.0;
  double center_x = 0.8;             // Further in front of the robot
  double center_y = 0.3;
  double center_z = 0.65;             // Higher up

  pos(0) = radius*cos(omega*t) + center_x;
  pos(1) = radius*sin(omega*t) + center_y;
  pos(2) = center_z;

  // Velocity is the derivative of position with respect to time
  vel(0) = -radius * omega * sin(omega * t);   // dx/dt
  vel(1) =  radius * omega * cos(omega * t);   // dy/dt
  vel(2) = 0.0;                                // dz/dt

}

 int main(int argc, char **argv){

    using namespace thor::math;
    double toll = 0.001; // Tolerance for convergence
    double mean_pos_error = 10.0;
    Eigen::VectorXd rel_error;

    std::ofstream logfile("/home/galileo/projects/thor_ws/src/thor_core/thor_math/trajectory_log.csv");
    logfile << "time,q1,q2,q3,q4,q5,q6,ph_x,ph_y,ph_z,h,n_c,frameId,d,vr,vh\n";

    std::string urdf_path = "/home/galileo/projects/thor_ws/src/thor_core/thor_math/ur10/ur10_with_intermediates.urdf"; // Replace with your URDF path
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);

    ThorQP qp;
    qp.setPinocchioModel(model);
    // Dummy initialization
    unsigned int nax = model.nv; // Number of joints in the model
    unsigned int nc = 5;
    double horizon = 0.2; // Control horizon time in seconds
    double st = 0.002;

    Eigen::VectorXd q_init(nax); // nax = number of joints
    q_init << 0.0, -M_PI/2, M_PI/2, 0.0, 0.0, 0.0; // for example

    Eigen::VectorXd q_final(nax);
    q_final << 
        M_PI/4,      // shoulder pan: 45° to the left
       -M_PI/3,     // shoulder lift: a bit higher (-60°)
        M_PI/3,      // elbow: extends more (60°)
       -M_PI/4,     // wrist 1: tilts down (-45°)
        M_PI/6,      // wrist 2: small positive rotation (30°)
       -M_PI/6;     // wrist 3: small negative rotation (-30°)
    double T = 2.0; // seconds (duration of motion)

    std::cout << "Setting up ThorQP with " << nax << " joints and " << nc << " intervals." << std::endl;
    
    qp.setIntervals(nc, nax, horizon, st);
    qp.setCBFParameters(2.5,0.15,0.5, 3.0);
    qp.setConstraints(Eigen::VectorXd::Constant(nax, M_PI),   // qmax
                        Eigen::VectorXd::Constant(nax, -M_PI),  // qmin
                        Eigen::VectorXd::Constant(nax, 30.0),   // Dqmax
                        Eigen::VectorXd::Constant(nax, 500.0),   // DDqmax
                        Eigen::VectorXd::Constant(nax, 10.0)); // tau_max

    std::cout << "Setting weight functions." << std::endl;
    qp.setWeigthFunction(1.0e-6, 1.0e-9, 0.0, 5e+1, 1e+4);

    qp.activatePositionBounds(true);
    qp.activateTorqueBounds(false);
    qp.activateCbfBounds(true);


    std::vector<unsigned int> frame_ids;
    // frame_ids.push_back(43);  
    for (std::size_t i = 0; i < model.frames.size(); ++i)
    {
        const pinocchio::Frame &f = model.frames[i];

        if (f.type == pinocchio::JOINT || f.name.find("intermediate") != std::string::npos)
        {
            frame_ids.push_back(static_cast<int>(i));
            std::cout << "Frame ID: " << i << ", Name: " << f.name << std::endl;
        }
      }
    qp.setCbfIds(frame_ids, 1);
    if (qp.needUpdate()) 
    {
       qp.updateMatrices();
      //  std::cout << "Matrices updated." << std::endl;
    } 
 

        // Set initial state (zero position + velocity)
        Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2 * nax);
        x0.segment(0, nax) =q_init; // Initial position
        // std::cout << "Initial state set to:\n" << x0.transpose() << std::endl;
       //x0.tail(nax) << 0.1,0.1,0.1,0.1,0.1,0.1; // Initial velocity
        qp.setInitialState(x0);
        //  std::cout << "Setting initial state." << std::endl;
        // std::cout << "Initial state: " << qp.getState().transpose() << std::endl;
        // Define dummy targets
        Eigen::VectorXd targetDq;
        targetDq.resize(nax * nc);
        Eigen::VectorXd next_targetQ;
        next_targetQ.resize(nax); 
        double target_scaling = 1;
       
        Eigen::VectorXd next_acc;
        double scaling;
        std::vector<Eigen::Vector3d> p_h(1); // Human position vector with 1 element
        std::vector<Eigen::Vector3d> vh(1);  // Human velocity vector with 1 element
        // size_t frameId = model.getFrameId("wrist_3_joint"); // Example frame ID
             
        // Run constrained QP solution
        Eigen::VectorXd prediction_time = qp.getPredictionTimeInstant();
        int iter = 0;
        double nominal_t = 0.0;
        double t= 0.0;
        int max_iter = 5000;
        std::cout << "starting ... " << std::endl;
        while (mean_pos_error > toll && iter < max_iter)
        {

            if (qp.needUpdate()) 
            {
              qp.updateMatrices();
              // std::cout << "Matrices updated." << std::endl;
            } 
            nominal_t+=scaling*st;
            t+=st;
            for (unsigned int iax = 0; iax < nax; iax++) {

              double pos, vel, acc;

              // Current time step for this joint
            cubic_spline(nominal_t, T, q_init(iax), q_final(iax), 0, 0, pos, vel, acc);
            next_targetQ(iax) = pos;
            targetDq(iax) = vel;

            // Fill the prediction horizon for this joint
            for (unsigned int ic = 1; ic < nc; ic++) {
                double future_t = nominal_t + prediction_time(ic);
                cubic_spline(future_t, T, q_init(iax), q_final(iax), 0, 0, pos, vel, acc);
                targetDq(iax + ic * nax) = vel;
            }
        }


            human_circle(nominal_t, p_h.at(0), vh.at(0));
            std::cout << __LINE__ << " ... " << std::endl;
            printf("aaa\n");
            std::vector<double> res = qp.computedCostrainedSolution(targetDq,next_targetQ,target_scaling,qp.getState(), next_acc,scaling, vh, p_h);
            // std::cout << __LINE__ << " ... " << std::endl;
            // std::cout << "Next Acceleration: " << next_acc.transpose() << std::endl;
            // std::cout << "Next Scaling: " << scaling << std::endl;
            qp.updateState(next_acc);
            std::cout << "Updated pos: " << qp.getState().head(nax).transpose() << std::endl;
            // std::cout << "targetq: " << next_targetQ.transpose() << std::endl;
            double eps = 1e-6;
            mean_pos_error = (
                ((qp.getState().head(nax) - q_final).cwiseAbs().array())
                / (next_targetQ.cwiseAbs().array() + eps)
            ).mean() * 100.0;

            rel_error = ((qp.getState().head(nax) - q_final).cwiseAbs().array())
                 / (q_final.cwiseAbs().array() + eps);

          
            logfile << t;
                    for (int i = 0; i < nax; ++i) logfile << "," << qp.getState()[i];   // robot joints
                    logfile << "," << p_h.at(0)(0) << "," << p_h.at(0)(1) << "," << p_h.at(0)(2) << "," << res.at(0) << "," << res.at(1) << "," << res.at(2) << "," << res.at(3) << "," << res.at(4) << "," << res.at(5) <<  "\n"; // human pos
            iter++;

    }
      std::cout << "Mean position error: " << mean_pos_error << std::endl;
      // std::cout << "Updated_vel: " << qp.getState().tail(nax).transpose() << std::endl;
      std::cout << "target q: " << next_targetQ.transpose() << std::endl;
      // std::cout << "Relative error: " << rel_error.transpose() << std::endl;
      if (iter >= max_iter)
      {
        std::cout << "Maximum iterations reached without convergence." << std::endl;
      }
      else
      {
        std::cout << "Convergence achieved." << std::endl;
      }
    logfile.close();
    return 0;  
    }





