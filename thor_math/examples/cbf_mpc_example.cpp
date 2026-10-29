// Closed-loop example of thor::math::ThorQP with dynamic and static obstacles.
//
// A UR10 sweeps its base joint back and forth. The scene contains:
//   - a person, tracked as a vertical capsule (body) and a sphere (hand), who
//     walks towards the robot, stops, and reaches into the workspace with the
//     hand;
//   - a static fixture, a sphere standing on the other side of the sweep.
// The robot itself is covered by spheres centered at some of its frames.
//
// Run:
//   thor_math_cbf_mpc_example [path/to/robot.urdf] [--no-cbf] [--continuous-cbf]
//                             [--linearizations N] [--damping B] [--tolerance T]
//
// With the CBF the MPC changes the motion so that, for every robot sphere and
// every obstacle, the barrier h stays non-negative: the robot can always
// stop at least C + delta_H away from the obstacle surfaces. With --no-cbf the
// same MPC tracks the trajectory ignoring the obstacles, to compare the
// minimum clearance reached. With --continuous-cbf the CBF is imposed in
// continuous time at the start of each interval (the formulation of the
// paper) instead of in discrete time along the whole horizon. With
// --linearizations N each cycle solves up to N QPs, re-linearizing the CBF
// around the plan of the previous one; --damping B moves the linearization
// point only by a fraction B of the way, and --tolerance T stops as soon as
// the solution changes by less than T [rad/s^2].
//
// The CBF does not know what an obstacle is: it only sees spheres and capsules
// (cylinders with hemispherical ends) with position, velocity and
// acceleration. People, vehicles, other robots and fixtures are handled the
// same way; only the numbers (radius, axis, motion, delta_H) change.
//
// The example shows, in order:
//   1. loading the robot model and covering it with spheres;
//   2. filling ThorConfig (horizon, weights, limits, CBF parameters);
//   3. the control loop: nominal trajectory sampled at the scaled time,
//      obstacle measurements, ThorQP::solve(), fallback braking, integration;
//   4. the computation time of the MPC, compared with the control period.

#include <thor_math/thor_math.h>
#include <thor_math/cbf.h>

#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;

// ---------------------------------------------------------------------------
// Nominal trajectory, parametrized by the trajectory time tau.
//
// The MPC does not follow time t but tau, with d(tau)/dt = s (the time
// scaling computed by the optimizer). s = 1 follows the trajectory at nominal
// speed, s < 1 slows it down, s = 0 stops along the path.
// ---------------------------------------------------------------------------
struct NominalTrajectory
{
  Eigen::VectorXd q0;            // configuration at tau = 0
  double amplitude = 1.0;        // base joint sweep amplitude [rad]
  double omega     = 1.0;        // base joint sweep frequency [rad/s]

  Eigen::VectorXd position(double tau) const
  {
    Eigen::VectorXd q = q0;
    q(0) += amplitude * std::sin(omega * tau);
    return q;
  }

  Eigen::VectorXd velocity(double tau) const
  {
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(q0.size());
    dq(0) = amplitude * omega * std::cos(omega * tau);
    return dq;
  }
};

// ---------------------------------------------------------------------------
// Robot spheres: a frame and a radius that covers the link around it.
// ---------------------------------------------------------------------------
struct RobotSphere
{
  const char* frame;
  double      radius;  // [m]
};

// ---------------------------------------------------------------------------
// Obstacle spheres and capsules.
//
// In a real application they come from a perception system: skeleton
// keypoints with a radius per body part, tracked objects, or fixed elements of
// the cell. ThorQP needs, for each obstacle, the center position, velocity and
// (optionally) acceleration in the robot base frame, its radius and,
// optionally, its own distance estimation error delta_H. A capsule also needs
// its half axis, from the center to one end; a zero axis is a sphere. The CBF
// extrapolates the centers along the horizon with constant acceleration, the
// same assumption used by the d_min formula; the axis does not rotate.
// ---------------------------------------------------------------------------
struct Obstacles
{
  std::vector<const char*>     name;
  std::vector<Eigen::Vector3d> pos, vel, acc, axis;
  std::vector<double>          radius, delta_H;
};

Obstacles scene(double t)
{
  // Person. The body walks along -x at 0.6 m/s, then brakes at 1.5 m/s^2
  // until it stops; the hand moves with the body, ahead of the torso. The
  // body is a vertical capsule from 0.2 m to 1.4 m above the robot base, so
  // the robot can neither pass over the shoulders nor under the hips.
  const Eigen::Vector3d start(1.85, 0.72, 0.8);
  const double v_walk = 0.6, a_brake = 1.5;
  const double t_walk = 1.5, t_brake = v_walk / a_brake;

  double x = 0.0, v = 0.0, a = 0.0;
  if (t < t_walk)
  {
    x = -v_walk * t;
    v = -v_walk;
  }
  else if (t < t_walk + t_brake)
  {
    const double tb = t - t_walk;
    x = -v_walk * t_walk - v_walk * tb + 0.5 * a_brake * tb * tb;
    v = -v_walk + a_brake * tb;
    a = a_brake;
  }
  else
  {
    x = -v_walk * t_walk - 0.5 * v_walk * t_brake;
  }
  const Eigen::Vector3d torso = start + Eigen::Vector3d(x, 0.0, 0.0);
  const Eigen::Vector3d hand  = torso + Eigen::Vector3d(-0.35, -0.2, -0.1);

  // delta_H of each obstacle from its perception noise (Eq. 9 of the paper):
  // 99.9% confidence (beta = 3.291), position/velocity/acceleration noise
  // standard deviations, and an upper bound of the stopping time. A hand
  // keypoint is much noisier than the torso; the fixture is measured once,
  // with a small, constant error.
  using thor::math::cbf::perceptionMargin;
  const double delta_torso   = perceptionMargin(3.291, 0.015, 0.072, 0.196, 0.3);
  const double delta_hand    = perceptionMargin(3.291, 0.030, 0.150, 0.400, 0.3);
  const double delta_fixture = 0.01;

  // Static fixture on the other side of the sweep: zero velocity and
  // acceleration. Nothing else distinguishes it from the person.
  const Eigen::Vector3d fixture(0.70, -0.42, 0.65);

  Obstacles o;
  o.name    = {"body", "hand", "fixture"};
  o.pos     = {torso, hand, fixture};
  o.axis    = {Eigen::Vector3d(0.0, 0.0, 0.6), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  o.vel     = {Eigen::Vector3d(v, 0.0, 0.0), Eigen::Vector3d(v, 0.0, 0.0), Eigen::Vector3d::Zero()};
  o.acc     = {Eigen::Vector3d(a, 0.0, 0.0), Eigen::Vector3d(a, 0.0, 0.0), Eigen::Vector3d::Zero()};
  o.radius  = {0.20, 0.06, 0.10};
  o.delta_H = {delta_torso, delta_hand, delta_fixture};
  return o;
}

// Computation time statistics of the MPC. A real-time loop must finish every
// solve within the control period, so the tail (99th percentile, maximum)
// matters more than the mean.
struct TimingStats
{
  double mean = 0.0, median = 0.0, p99 = 0.0, max = 0.0;
  std::size_t over_budget = 0;  // solves longer than the control period
};

TimingStats timingStats(std::vector<double> samples, double budget)
{
  TimingStats stats;
  if (samples.empty())
    return stats;
  std::sort(samples.begin(), samples.end());
  const auto percentile = [&](double p) {
    return samples[static_cast<std::size_t>(p * static_cast<double>(samples.size() - 1))];
  };
  double sum = 0.0;
  for (const double v : samples)
    sum += v;
  stats.mean = sum / static_cast<double>(samples.size());
  stats.median = percentile(0.5);
  stats.p99 = percentile(0.99);
  stats.max = samples.back();
  stats.over_budget = static_cast<std::size_t>(
    samples.end() - std::upper_bound(samples.begin(), samples.end(), budget));
  return stats;
}

// Current clearance (distance between surfaces) of every obstacle from the
// robot spheres, computed independently of the optimizer to compare runs with
// and without CBF. The distance from a capsule is the distance from the
// closest point of its axis.
std::vector<double> clearances(const pinocchio::Model& model, pinocchio::Data& data,
                               const Eigen::VectorXd& q, const std::vector<unsigned int>& frames,
                               const std::vector<double>& frame_radii, const Obstacles& obstacles)
{
  pinocchio::framesForwardKinematics(model, data, q);
  std::vector<double> c(obstacles.pos.size(), std::numeric_limits<double>::infinity());
  for (std::size_t j = 0; j < frames.size(); ++j)
    for (std::size_t k = 0; k < obstacles.pos.size(); ++k)
    {
      const Eigen::Vector3d r = data.oMf[frames[j]].translation() - obstacles.pos[k];
      const Eigen::Vector3d& axis = obstacles.axis[k];
      const double s = axis.squaredNorm() > 0.0 ? std::clamp(r.dot(axis) / axis.squaredNorm(), -1.0, 1.0) : 0.0;
      c[k] = std::min(c[k], (r - s * axis).norm() - frame_radii[j] - obstacles.radius[k]);
    }
  return c;
}

}  // namespace

int main(int argc, char** argv)
{
  std::string urdf = THOR_MATH_EXAMPLE_URDF;
  bool use_cbf = true;
  auto formulation = thor::math::CbfFormulation::discrete;
  unsigned int linearizations = 1;
  double damping = 1.0, tolerance = 0.0;
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--no-cbf")
      use_cbf = false;
    else if (arg == "--continuous-cbf")
      formulation = thor::math::CbfFormulation::continuous;
    else if (arg == "--linearizations" && i + 1 < argc)
      linearizations = static_cast<unsigned int>(std::stoul(argv[++i]));
    else if (arg == "--damping" && i + 1 < argc)
      damping = std::stod(argv[++i]);
    else if (arg == "--tolerance" && i + 1 < argc)
      tolerance = std::stod(argv[++i]);
    else
      urdf = arg;
  }

  // -------------------------------------------------------------------------
  // 1. Robot model and robot spheres
  //
  // The CBF needs a Pinocchio model with nq == nv == number of joints. The
  // robot is covered by spheres centered at Pinocchio frames: a few links plus
  // an intermediate frame added along the forearm, so that the whole arm, not
  // only the tool, keeps its distance. Each radius covers the link around its
  // frame. Without radii (cbf_frame_radii empty) the frames are points and the
  // link size must be folded into the margin C.
  // -------------------------------------------------------------------------
  pinocchio::Model model;
  try
  {
    pinocchio::urdf::buildModel(urdf, model);
  }
  catch (const std::exception& e)
  {
    std::cerr << "cannot load " << urdf << ": " << e.what() << "\n";
    return 1;
  }
  const auto n_joints = static_cast<unsigned int>(model.nv);

  const RobotSphere robot_spheres[] = {
    {"forearm_link", 0.08}, {"elbow_joint_intermediate_2", 0.07}, {"wrist_1_link", 0.06},
    {"wrist_3_link", 0.05}, {"tool0", 0.05}};
  std::vector<unsigned int> cbf_frames;
  std::vector<double> cbf_radii;
  for (const RobotSphere& sphere : robot_spheres)
  {
    if (model.existFrame(sphere.frame))
    {
      cbf_frames.push_back(static_cast<unsigned int>(model.getFrameId(sphere.frame)));
      cbf_radii.push_back(sphere.radius);
    }
  }
  if (cbf_frames.empty())
  {
    std::cerr << "none of the monitored frames is in the model\n";
    return 1;
  }

  // -------------------------------------------------------------------------
  // 2. Configuration
  // -------------------------------------------------------------------------
  thor::math::ThorConfig config;

  // Horizon: n_intervals control intervals over horizon_time. With
  // input_blocking the intervals grow quadratically: the first one equals the
  // sampling period (it is the command actually applied), the later ones are
  // longer and only make the MPC look ahead.
  const double dt = 0.002;  // 500 Hz control loop
  config.n_intervals     = 5;
  config.n_joints        = n_joints;
  config.horizon_time    = 0.2;
  config.sampling_period = dt;
  config.input_blocking  = true;

  // Cost weights. The MPC minimizes, over the horizon,
  //   || dq_pred - s * target_dq ||^2            velocity tracking
  // + lambda_acc     || ddq ||^2                 effort
  // + lambda_scaling || s - target_scaling ||^2  keep the nominal speed
  // + lambda_clik    || q_next - next_target_q ||^2  position drift correction
  config.lambda_acc     = 1e-6;
  config.lambda_scaling = 50.0;
  config.lambda_clik    = 1e4;
  config.lambda_jerk    = 0.0;

  // Joint limits. Velocity and acceleration limits are imposed on every
  // predicted interval; position_bounds adds position and invariance limits.
  config.qmin   = model.lowerPositionLimit;
  config.qmax   = model.upperPositionLimit;
  config.Dqmax  = Eigen::VectorXd::Constant(n_joints, 2.0);
  config.DDqmax = Eigen::VectorXd::Constant(n_joints, 10.0);
  config.position_bounds = true;

  // Time scaling bounds: 0 lets the MPC stop the progress along the path.
  config.min_scaling = 0.0;
  config.max_scaling = 1.0;

  // Control barrier function (see thor_math/cbf.h). For each robot sphere and
  // obstacle the barrier is
  //   h = d_min - C
  // where d_min is the minimum distance between the two surfaces reached if
  // the robot reacts after T_r and then brakes at a_s while the obstacle keeps
  // its acceleration. The distance is the worst case allowed by the
  // perception error, d - delta_H: in practice each obstacle is
  // inflated by its delta_H. h >= 0 means that the robot can always stop C
  // away from the inflated obstacle.
  //
  // With X_0 the current state and X_{i+1} the predicted state at the end of
  // interval i, the discrete formulation imposes on every interval
  //   h(X_{i+1}) >= (1 - gamma dt_i) h(X_i)
  // so h stays non-negative at every predicted state, and decays at most
  // geometrically when it is not. The continuous formulation imposes
  //   dh/dt + gamma h >= 0
  // at the start of each interval: it constrains only the rate of h, which
  // is enough for the first interval (one control period) but not along the
  // long intervals of the horizon.
  config.cbf     = use_cbf;
  config.cbf_formulation = formulation;
  // The CBF is linearized around the plan of the previous cycle. With N > 1
  // each cycle solves N QPs, each linearized around the plan of the previous
  // one: the predicted barrier gets closer to the nonlinear one along the long
  // intervals, at about N times the solve time. 1 is enough for safety, since
  // the first interval, the one executed, is linearized over one period only.
  config.cbf_linearizations = linearizations;
  // Each new linearization point moves towards the new solution by a fraction
  // `damping` of the way, which damps the oscillations of a full step; the
  // iteration stops when the solution changes by at most `tolerance`.
  config.cbf_linearization_damping   = damping;
  config.cbf_linearization_tolerance = tolerance;
  config.a_s     = 2.5;    // robot braking deceleration [m/s^2]
  config.T_r     = 0.15;   // reaction time [s]
  config.C       = 0.05;   // safety margin between the surfaces [m]: sizes are in the radii
  config.gamma   = 5.0;    // class-K gain [1/s]: higher reacts later but harder
  // Default delta_H, used when ThorInput::obstacle_delta_H is empty. Here each
  // obstacle provides its own, so this value is not used.
  config.delta_H = thor::math::cbf::perceptionMargin(3.291, 0.015, 0.072, 0.196, 0.3);
  config.cbf_frame_ids     = cbf_frames;
  config.cbf_frame_radii   = cbf_radii;
  config.n_obstacle_points = 3;  // body, hand, fixture
  // Above this barrier value the CBF rows are dropped from the QP (all
  // obstacles far away): a smaller and faster problem with the same solution.
  config.cbf_activation_distance = 5.0;

  // The model must be given before configure() when the CBF is enabled.
  // configure() validates everything and throws std::invalid_argument on
  // inconsistent settings, leaving the optimizer unchanged.
  thor::math::ThorQP qp;
  qp.setPinocchioModel(model);
  try
  {
    qp.configure(config);
  }
  catch (const std::invalid_argument& e)
  {
    std::cerr << "invalid configuration: " << e.what() << "\n";
    return 1;
  }

  // -------------------------------------------------------------------------
  // 3. Control loop
  // -------------------------------------------------------------------------
  NominalTrajectory trajectory;
  trajectory.q0 = Eigen::VectorXd::Zero(n_joints);
  trajectory.q0 << 0.0, -kPi / 2.0, kPi / 2.0, -kPi / 2.0, -kPi / 2.0, 0.0;
  trajectory.amplitude = 1.2;
  trajectory.omega     = 1.0;

  // The state is x = [q; dq], here starting at rest. ThorQP integrates it
  // with updateState(); in a real application x0 comes from the robot
  // encoders at every cycle.
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2 * n_joints);
  x0.head(n_joints) = trajectory.position(0.0);
  qp.setInitialState(x0);

  const Eigen::VectorXd prediction_time = qp.getPredictionTimeInstant();
  pinocchio::Data data(model);

  double tau = 0.0;      // trajectory time
  double scaling = 1.0;  // time scaling applied in the last cycle
  std::vector<double> min_clearance(config.n_obstacle_points, std::numeric_limits<double>::infinity());
  double min_h_now = std::numeric_limits<double>::infinity();
  int fallback_cycles = 0;

  // Wall-clock time of every ThorQP::solve() call [us]. steady_clock is
  // monotonic, so it is not affected by system clock adjustments.
  using Clock = std::chrono::steady_clock;
  const double t_end = 6.0;
  std::vector<double> solve_us;
  solve_us.reserve(static_cast<std::size_t>(t_end / dt) + 1);

  std::printf("CBF %s, %zu robot spheres, %u obstacles\n",
              !use_cbf ? "disabled"
                       : formulation == thor::math::CbfFormulation::discrete ? "enabled (discrete)"
                                                                            : "enabled (continuous)",
              cbf_frames.size(), config.n_obstacle_points);
  if (use_cbf)
    std::printf("CBF linearizations per cycle: up to %u, damping %.2f, tolerance %.3g rad/s^2\n",
                config.cbf_linearizations, config.cbf_linearization_damping,
                config.cbf_linearization_tolerance);
#ifndef NDEBUG
  std::printf("warning: built without optimizations, timings are not representative "
              "(configure with -DCMAKE_BUILD_TYPE=Release)\n");
#endif
  std::printf("%6s %7s %7s %8s %8s %8s %9s %10s  %s\n",
              "t[s]", "tau[s]", "s", "h_now", "h_min", "dist", "obstacle", "solve[us]", "robot frame");

  for (int k = 0; k * dt < t_end; ++k)
  {
    const double t = k * dt;
    x0 = qp.getState();

    thor::math::ThorInput in;
    in.x0 = x0;

    // Target velocities for every interval, sampled at the trajectory time
    // the robot is expected to reach at that interval (the MPC multiplies
    // them by its own scaling). Layout: [dq_interval_0; dq_interval_1; ...].
    in.target_dq.resize(n_joints * config.n_intervals);
    for (unsigned int i = 0; i < config.n_intervals; ++i)
      in.target_dq.segment(i * n_joints, n_joints) = trajectory.velocity(tau + scaling * prediction_time(i));
    // Position the robot should have at the end of the first interval.
    in.next_target_q  = trajectory.position(tau + scaling * prediction_time(0));
    // Desired scaling: 1 = nominal speed. A supervisor could lower it.
    in.target_scaling = 1.0;

    // Obstacles (ignored when the CBF is disabled). Leaving pos, vel and acc
    // all empty means "no obstacle": n_obstacle_points far-away, still
    // points. acc, radius, axis and delta_H may each be left empty: zero
    // acceleration, points, spheres, and ThorConfig::delta_H for every
    // obstacle.
    const Obstacles obstacles = scene(t);
    in.obstacle_pos     = obstacles.pos;
    in.obstacle_vel     = obstacles.vel;
    in.obstacle_acc     = obstacles.acc;
    in.obstacle_radius  = obstacles.radius;
    in.obstacle_axis    = obstacles.axis;
    in.obstacle_delta_H = obstacles.delta_H;

    // solve() throws std::runtime_error if the QP is infeasible, e.g. when an
    // obstacle moves faster than the robot can react. A real controller must
    // then fall back to a safe command: here, brake as hard as allowed and
    // freeze the progress along the path (as the cbf_python controller).
    //
    // Only solve() is timed: it builds the cost, evaluates the robot
    // kinematics and the barrier for every interval, robot sphere and
    // obstacle, and runs the QP solver. A failed solve is timed too,
    // since in a real loop it also consumes the cycle.
    thor::math::ThorOutput out;
    bool solved = true;
    const Clock::time_point start = Clock::now();
    try
    {
      out = qp.solve(in);
    }
    catch (const std::runtime_error&)
    {
      solved = false;
    }
    const double elapsed_us = std::chrono::duration<double, std::micro>(Clock::now() - start).count();
    solve_us.push_back(elapsed_us);

    Eigen::VectorXd acc;
    if (solved)
    {
      acc     = out.next_acc;      // joint accelerations of the first interval
      scaling = out.next_scaling;  // time scaling of the first interval
      if (use_cbf)
        min_h_now = std::min(min_h_now, out.cbf.h_now);

      if (k % 250 == 0)
      {
        // out.cbf.h_now is the barrier on the current state: the actual
        // safety margin. The other fields describe the most critical pair over
        // the whole horizon, on the predicted states used to linearize the CBF
        // (h_min); distance is measured between the surfaces.
        const auto nan = std::nan("");
        std::printf("%6.2f %7.3f %7.3f %8.3f %8.3f %8.3f %9s %10.1f  %s\n", t, tau, scaling,
                    use_cbf ? out.cbf.h_now : nan, use_cbf ? out.cbf.h_min : nan,
                    use_cbf ? out.cbf.distance : nan,
                    use_cbf ? obstacles.name[out.cbf.obstacle_index] : "-", elapsed_us,
                    use_cbf ? model.frames[out.cbf.frame_id].name.c_str() : "-");
      }
    }
    else
    {
      ++fallback_cycles;
      const Eigen::VectorXd dq = x0.tail(n_joints);
      acc = (-dq / dt).cwiseMax(-config.DDqmax).cwiseMin(config.DDqmax);
      scaling = 0.0;
    }

    // Integrate the robot state and the trajectory time. In a real
    // application acc (or the integrated q, dq) is sent to the robot.
    qp.updateState(acc);
    tau += scaling * dt;

    const std::vector<double> c =
      clearances(model, data, qp.getState().head(n_joints), cbf_frames, cbf_radii, obstacles);
    for (std::size_t o = 0; o < c.size(); ++o)
      min_clearance[o] = std::min(min_clearance[o], c[o]);
  }

  // Clearance is the distance between the robot and obstacle surfaces: a
  // negative value is a collision.
  std::printf("\nminimum clearance per obstacle:\n");
  const Obstacles names = scene(0.0);
  for (std::size_t o = 0; o < min_clearance.size(); ++o)
    std::printf("  %-8s %7.3f m%s\n", names.name[o], min_clearance[o],
                min_clearance[o] < 0.0 ? "   <- collision" : "");
  if (use_cbf)
    std::printf("minimum barrier (h_now):   %.3f m\n", min_h_now);
  std::printf("trajectory progress:       %.2f s of %.2f s\n", tau, t_end);
  std::printf("fallback braking cycles:   %d\n", fallback_cycles);

  // The budget is the control period: a solve longer than dt misses the
  // deadline of the 500 Hz loop. The CBF adds n_intervals * robot spheres *
  // obstacles rows (here 5 * 5 * 3 = 75) and one kinematics evaluation
  // per interval, so compare with --no-cbf to see its cost.
  const double budget_us = dt * 1e6;
  const TimingStats timing = timingStats(solve_us, budget_us);
  std::printf("\nMPC computation time over %zu solves [us]:\n", solve_us.size());
  std::printf("  mean %.1f   median %.1f   p99 %.1f   max %.1f\n",
              timing.mean, timing.median, timing.p99, timing.max);
  std::printf("  budget %.0f us (control period): %zu solves over budget, worst-case load %.0f%%\n",
              budget_us, timing.over_budget, 100.0 * timing.max / budget_us);
  return 0;
}
