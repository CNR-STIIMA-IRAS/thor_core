// BSD 3-Clause License
//
// Copyright (c) 2026, Nicola Pedrocchi
// All rights reserved.
//
// See the project license for redistribution terms and warranty disclaimer.

#include <gmock/gmock.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "thor_controllers/thor_core.hpp"

namespace
{

using thor_controllers::CoreInput;
using thor_controllers::CoreOutput;
using thor_controllers::CoreSettings;
using thor_controllers::Obstacle;
using thor_controllers::ThorCore;

const std::vector<std::string> kJoints = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                          "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
constexpr double kDt = 0.002;

std::string readUrdf()
{
  std::ifstream file(THOR_TEST_URDF);
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

CoreSettings defaultSettings(bool cbf = true)
{
  CoreSettings s;
  s.joint_names = kJoints;
  s.mpc.n_intervals = 5;
  s.mpc.horizon_time = 0.2;
  s.mpc.sampling_period = kDt;
  s.mpc.lambda_acc = 1e-6;
  s.mpc.lambda_scaling = 50.0;
  s.mpc.lambda_clik = 1e4;
  s.mpc.DDqmax = Eigen::VectorXd::Constant(6, 10.0);
  s.mpc.Dqmax = Eigen::VectorXd::Constant(6, 2.0);
  s.mpc.position_bounds = true;
  s.mpc.min_scaling = 0.0;
  s.mpc.max_scaling = 1.0;
  s.mpc.cbf = cbf;
  s.mpc.a_s = 2.5;
  s.mpc.T_r = 0.15;
  s.mpc.C = 0.05;
  s.mpc.gamma = 5.0;
  s.max_obstacles = 2;
  s.spheres = {
    {"elbow", "forearm_link", Eigen::Vector3d::Zero(), 0.08},
    {"forearm", "forearm_link", Eigen::Vector3d(-0.3, 0.0, 0.0), 0.07},
    {"wrist", "wrist_2_link", Eigen::Vector3d::Zero(), 0.07},
    {"tool", "tool0", Eigen::Vector3d::Zero(), 0.05},
  };
  return s;
}

const Eigen::VectorXd kHome = (Eigen::VectorXd(6) << 0.0, -1.57, 1.57, -1.57, -1.57, 0.0).finished();

// Input that moves the base joint at constant speed from q0, at trajectory time tau.
CoreInput constantVelocityInput(const ThorCore & core, const Eigen::VectorXd & q0, double tau, double v)
{
  const Eigen::VectorXd & t = core.predictionTimes();
  CoreInput in;
  in.target_dq = Eigen::VectorXd::Zero(6 * t.size());
  for (Eigen::Index i = 0; i < t.size(); ++i) in.target_dq(i * 6) = v;
  in.next_target_q = q0;
  in.next_target_q(0) += v * (tau + t(0));
  return in;
}

// Result of a closed-loop run.
struct RunResult
{
  double h_min = std::numeric_limits<double>::infinity();  ///< lowest h_now
  double min_scaling = std::numeric_limits<double>::infinity();
  double final_scaling = 0.0;
  double max_deviation = 0.0;  ///< largest displacement of joints 2..6 from q0
  double min_distance = std::numeric_limits<double>::infinity();
};

// Runs the closed loop for `steps` cycles.
RunResult run(ThorCore & core, const Eigen::VectorXd & q0, double v, int steps)
{
  RunResult r;
  double tau = 0.0;
  for (int k = 0; k < steps; ++k)
  {
    const CoreOutput & out = core.step(constantVelocityInput(core, q0, tau, v));
    tau += out.scaling * kDt;
    if (core.settings().mpc.cbf) r.h_min = std::min(r.h_min, out.cbf.h_now);
    r.min_scaling = std::min(r.min_scaling, out.scaling);
    r.final_scaling = out.scaling;
    r.min_distance = std::min(r.min_distance, out.min_distance);
    r.max_deviation = std::max(r.max_deviation, (out.q.tail(5) - q0.tail(5)).cwiseAbs().maxCoeff());
  }
  return r;
}

TEST(ThorCore, ConfiguresFromUrdf)
{
  ThorCore core;
  EXPECT_FALSE(core.isConfigured());
  ASSERT_NO_THROW(core.configure(readUrdf(), defaultSettings()));
  EXPECT_TRUE(core.isConfigured());
  EXPECT_EQ(core.rootFrame(), "world");
  EXPECT_EQ(core.predictionTimes().size(), 5);
  EXPECT_NEAR(core.predictionTimes()(0), kDt, 1e-12);
  EXPECT_EQ(core.mpcConfig().n_obstacle_points, 2);
  EXPECT_EQ(core.mpcConfig().cbf_frame_ids.size(), 4u);
  // Position limits come from the URDF.
  EXPECT_NEAR(core.mpcConfig().qmax(2), M_PI, 1e-9);
}

TEST(ThorCore, VelocityLimitsDefaultToUrdf)
{
  auto s = defaultSettings();
  s.mpc.Dqmax.resize(0);
  ThorCore core;
  core.configure(readUrdf(), s);
  EXPECT_NEAR(core.mpcConfig().Dqmax(0), 2.0943951023931953, 1e-12);
  EXPECT_NEAR(core.mpcConfig().Dqmax(5), M_PI, 1e-12);
}

TEST(ThorCore, RejectsInvalidSettings)
{
  ThorCore core;
  auto s = defaultSettings();
  s.joint_names[0] = "missing_joint";
  EXPECT_THROW(core.configure(readUrdf(), s), std::invalid_argument);

  s = defaultSettings();
  s.spheres[0].frame = "missing_frame";
  EXPECT_THROW(core.configure(readUrdf(), s), std::invalid_argument);

  s = defaultSettings();
  s.mpc.DDqmax = Eigen::VectorXd::Constant(5, 10.0);
  EXPECT_THROW(core.configure(readUrdf(), s), std::invalid_argument);

  s = defaultSettings();
  s.spheres.clear();
  EXPECT_THROW(core.configure(readUrdf(), s), std::invalid_argument);

  s = defaultSettings();
  s.joint_names.push_back(s.joint_names[0]);
  EXPECT_THROW(core.configure(readUrdf(), s), std::invalid_argument);

  EXPECT_THROW(core.configure("<robot", defaultSettings()), std::invalid_argument);
  EXPECT_FALSE(core.isConfigured());
  EXPECT_THROW(core.step(CoreInput()), std::logic_error);
}

TEST(ThorCore, ParsesStateFeedback)
{
  EXPECT_EQ(thor_controllers::parseStateFeedback("open_loop"), thor_controllers::StateFeedback::open_loop);
  EXPECT_EQ(thor_controllers::parseStateFeedback("position"), thor_controllers::StateFeedback::position);
  EXPECT_EQ(thor_controllers::parseStateFeedback("full"), thor_controllers::StateFeedback::full);
  EXPECT_THROW(thor_controllers::parseStateFeedback("closed"), std::invalid_argument);
}

TEST(ThorCore, SubsetOfJointsLocksTheOthers)
{
  auto s = defaultSettings(false);
  s.joint_names = {"shoulder_pan_joint", "elbow_joint"};
  s.mpc.DDqmax = Eigen::VectorXd::Constant(2, 10.0);
  s.mpc.Dqmax = Eigen::VectorXd::Constant(2, 2.0);
  ThorCore core;
  ASSERT_NO_THROW(core.configure(readUrdf(), s));
  EXPECT_EQ(core.mpcConfig().n_joints, 2u);
}

TEST(ThorCore, TracksTrajectoryWithoutObstacles)
{
  ThorCore core;
  core.configure(readUrdf(), defaultSettings());
  core.reset(kHome, Eigen::VectorXd());
  const RunResult r = run(core, kHome, 0.5, 1000);  // 2 s
  EXPECT_NEAR(r.final_scaling, 1.0, 1e-3);
  const Eigen::VectorXd x = core.state();
  EXPECT_NEAR(x(0), kHome(0) + 0.5 * 2.0, 0.02);
  EXPECT_NEAR(x(6), 0.5, 1e-2);
  EXPECT_LT((x.segment(1, 5) - kHome.tail(5)).cwiseAbs().maxCoeff(), 1e-3);
}

TEST(ThorCore, OutputIsIntegratedAcceleration)
{
  ThorCore core;
  core.configure(readUrdf(), defaultSettings(false));
  core.reset(kHome, Eigen::VectorXd());
  const Eigen::VectorXd x0 = core.state();
  const CoreOutput & out = core.step(constantVelocityInput(core, kHome, 0.0, 0.5));
  EXPECT_EQ(out.status, CoreOutput::Status::ok);
  const Eigen::VectorXd dq = x0.tail(6) + kDt * out.ddq;
  const Eigen::VectorXd q = x0.head(6) + (x0.tail(6) + 0.5 * kDt * out.ddq) * kDt;
  EXPECT_TRUE(out.dq.isApprox(dq, 1e-12));
  EXPECT_TRUE(out.q.isApprox(q, 1e-12));
  EXPECT_LE(out.ddq.cwiseAbs().maxCoeff(), 10.0 + 1e-9);
}

TEST(ThorCore, JointOrderDoesNotMatter)
{
  // The same controller with the joints in reverse order, and the
  // per-joint limits reversed accordingly.
  auto sa = defaultSettings();
  sa.mpc.DDqmax << 5.0, 6.0, 7.0, 8.0, 9.0, 10.0;
  auto sb = sa;
  std::reverse(sb.joint_names.begin(), sb.joint_names.end());
  sb.mpc.DDqmax = sa.mpc.DDqmax.reverse().eval();
  ThorCore a, b;
  a.configure(readUrdf(), sa);
  b.configure(readUrdf(), sb);

  const Eigen::VectorXd home_rev = kHome.reverse();
  a.reset(kHome, Eigen::VectorXd());
  b.reset(home_rev, Eigen::VectorXd());
  for (int k = 0; k < 50; ++k)
  {
    CoreInput ia = constantVelocityInput(a, kHome, k * kDt, 0.5);
    CoreInput ib = ia;
    ib.next_target_q = ia.next_target_q.reverse();
    for (Eigen::Index i = 0; i < a.predictionTimes().size(); ++i)
      ib.target_dq.segment(i * 6, 6) = ia.target_dq.segment(i * 6, 6).reverse();
    const Eigen::VectorXd qa = a.step(ia).q;
    const Eigen::VectorXd qb = b.step(ib).q;
    ASSERT_TRUE(qa.isApprox(qb.reverse(), 1e-9)) << "cycle " << k;
  }
}

TEST(ThorCore, KeepsDistanceFromObstacleOnThePath)
{
  // The tool sweeps around the base; a person stands on its path.
  ThorCore free_core, safe_core;
  free_core.configure(readUrdf(), defaultSettings(false));
  safe_core.configure(readUrdf(), defaultSettings(true));
  free_core.reset(kHome, Eigen::VectorXd());
  safe_core.reset(kHome, Eigen::VectorXd());

  // Where the tool goes without obstacles after 1 s.
  run(free_core, kHome, 0.8, 500);
  const Eigen::Vector3d tool_ahead = free_core.sphereCenters().back();

  Obstacle person;
  person.position = tool_ahead;
  person.radius = 0.1;
  person.half_axis = Eigen::Vector3d(0.0, 0.0, 0.5);
  safe_core.setObstacles({person});

  const RunResult r = run(safe_core, kHome, 0.8, 1500);
  // Safe all the time...
  EXPECT_GE(r.h_min, -1e-3);
  EXPECT_GT(r.min_distance, 0.05 - 1e-3);  // the safety margin C
  for (const auto & c : safe_core.sphereCenters())
  {
    const Eigen::Vector3d d = c - person.position;
    const double axial = std::clamp(d.z() / 0.5, -1.0, 1.0) * 0.5;
    EXPECT_GT((d - Eigen::Vector3d(0, 0, axial)).norm(), person.radius + 0.05);
  }
  // ...by slowing down or moving the other joints around the person.
  EXPECT_TRUE(r.min_scaling < 0.9 || r.max_deviation > 0.05)
    << "min scaling " << r.min_scaling << ", max deviation " << r.max_deviation;
  std::cout << "min scaling " << r.min_scaling << ", final scaling " << r.final_scaling
            << ", max deviation " << r.max_deviation << " rad, min h " << r.h_min << " m\n";
}

TEST(ThorCore, UsesTheClosestObstacles)
{
  auto s = defaultSettings();
  s.max_obstacles = 1;
  ThorCore core;
  core.configure(readUrdf(), s);
  core.reset(kHome, Eigen::VectorXd());
  const Eigen::Vector3d tool = core.sphereCenters().back();

  Obstacle far, near;
  far.position = tool + Eigen::Vector3d(3.0, 0.0, 0.0);
  near.position = tool + Eigen::Vector3d(0.0, 0.6, 0.0);
  core.setObstacles({far, near});
  const CoreOutput & out = core.step(constantVelocityInput(core, kHome, 0.0, 0.0));
  EXPECT_EQ(out.n_obstacles, 1u);
  EXPECT_EQ(out.cbf.obstacle_index, 0u);
  EXPECT_GT(out.min_distance, 0.3);
  EXPECT_LT(out.min_distance, 0.6);
  // The near obstacle, seen from the closest robot sphere, not the far one.
  EXPECT_GT(out.cbf.distance, 0.3);
  EXPECT_LT(out.cbf.distance, 0.6);
}

// The standard deviations of an obstacle come from the obstacle itself or,
// component by component when negative, from ThorConfig::obstacle_sigma. With
// the robot at rest and a static obstacle only the position one acts: the
// barrier is that of an obstacle sigma_bound * sigma_position closer.
TEST(ThorCore, PerceptionSigmaPerObstacleOrDefault)
{
  constexpr double sigma_p = 0.02;
  auto h_now = [](const Eigen::Vector3d & default_sigma, const Eigen::Vector3d & obstacle_sigma) {
    auto s = defaultSettings();
    s.max_obstacles = 1;
    s.mpc.obstacle_sigma = default_sigma;
    ThorCore core;
    core.configure(readUrdf(), s);
    core.reset(kHome, Eigen::VectorXd());
    Obstacle o;
    o.position = core.sphereCenters().back() + Eigen::Vector3d(0.0, 0.6, 0.0);
    o.sigma = obstacle_sigma;
    core.setObstacles({o});
    const CoreOutput & out = core.step(constantVelocityInput(core, kHome, 0.0, 0.0));
    EXPECT_EQ(out.status, CoreOutput::Status::ok);
    return std::make_pair(out.cbf.h_now, out.cbf.distance);
  };
  const Eigen::Vector3d none = Eigen::Vector3d::Zero();
  const Eigen::Vector3d position(sigma_p, 0.0, 0.0);
  const Eigen::Vector3d use_default = Eigen::Vector3d::Constant(-1.0);

  const auto exact = h_now(none, use_default);
  const auto own = h_now(none, position);
  const auto from_default = h_now(position, use_default);
  const auto overridden = h_now(position, Eigen::Vector3d(0.0, -1.0, -1.0));

  EXPECT_NEAR(own.first, exact.first - 3.0 * sigma_p, 1e-9);  // sigma_bound = 3
  EXPECT_NEAR(from_default.first, own.first, 1e-12);
  EXPECT_NEAR(overridden.first, exact.first, 1e-12);
  // The diagnostics report the measured distance.
  EXPECT_NEAR(own.second, exact.second, 1e-12);
}

TEST(ThorCore, BrakesWhenThereIsNoSolution)
{
  // The obstacle below is inside the tool: without this, the protective stop
  // would take over (see ProtectiveStop*).
  auto s = defaultSettings();
  s.protective_stop.enabled = false;
  ThorCore core;
  core.configure(readUrdf(), s);
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(6);
  dq(0) = 2.0;
  core.reset(kHome, dq);
  // The tool, moving fast, is already inside an obstacle.
  Obstacle o;
  o.position = core.sphereCenters().back();
  o.radius = 0.2;
  core.setObstacles({o});
  const CoreOutput & out = core.step(constantVelocityInput(core, kHome, 0.0, 2.0));
  ASSERT_EQ(out.status, CoreOutput::Status::braking);
  EXPECT_EQ(out.scaling, 0.0);
  EXPECT_NEAR(out.ddq(0), -10.0, 1e-12);
  EXPECT_NEAR(out.dq(0), 2.0 - 10.0 * kDt, 1e-12);
  EXPECT_EQ(out.ddq.tail(5).cwiseAbs().maxCoeff(), 0.0);
}

// Moves the base joint at speed v for `steps` cycles; returns the trajectory time.
double moveBase(ThorCore & core, double v, int steps, double tau = 0.0)
{
  for (int k = 0; k < steps; ++k) tau += core.step(constantVelocityInput(core, kHome, tau, v)).scaling * kDt;
  return tau;
}

// An obstacle `gap` from the surface of the tool sphere (radius 0.05), beyond
// the tool along the wrist-tool direction, so that the tool is the closest
// robot sphere.
Obstacle beyondTheTool(ThorCore & core, double gap)
{
  const auto centers = core.sphereCenters();
  const Eigen::Vector3d outward = (centers[3] - centers[2]).normalized();
  Obstacle o;
  o.position = centers[3] + (0.05 + gap) * outward;
  return o;
}

// ISO 10218 protective stop: below the stop distance the robot brakes at the
// acceleration limits, all the joints together, and holds still. It does so
// even when it is not moving towards the obstacle (here the tool moves around
// the base and the obstacle is beyond the tool, across its motion).
TEST(ThorCore, ProtectiveStopBrakesTogetherAndHolds)
{
  ThorCore core;
  core.configure(readUrdf(), defaultSettings());  // stop distance = C = 0.05 m
  core.reset(kHome, Eigen::VectorXd());
  const double tau = moveBase(core, 1.0, 250);
  const Eigen::VectorXd dq0 = core.state().tail(6);
  ASSERT_GT(dq0(0), 0.5);

  core.setObstacles({beyondTheTool(core, 0.03)});
  const CoreOutput & first = core.step(constantVelocityInput(core, kHome, tau, 1.0));
  ASSERT_EQ(first.status, CoreOutput::Status::protective_stop);
  EXPECT_NEAR(first.guard_distance, 0.03, 1e-3);
  EXPECT_EQ(first.scaling, 0.0);
  // Synchronized: the acceleration is opposite to the velocity, at the limit
  // for the slowest joint and within it for the others.
  const double ratio = first.ddq(0) / dq0(0);
  EXPECT_LT(ratio, 0.0);
  for (Eigen::Index j = 0; j < 6; ++j) {
    EXPECT_NEAR(first.ddq(j), ratio * dq0(j), 1e-9) << "joint " << j;
    EXPECT_LE(std::abs(first.ddq(j)), 10.0 + 1e-9);
  }
  EXPECT_NEAR(first.ddq.cwiseAbs().maxCoeff(), 10.0, 1e-9);

  // Still within the braking time, then held.
  const double braking_time = dq0.cwiseAbs().maxCoeff() / 10.0;
  int k = 1;
  for (; k < 1000 && core.state().tail(6).cwiseAbs().maxCoeff() > 0.0; ++k)
    EXPECT_EQ(core.step(constantVelocityInput(core, kHome, tau, 1.0)).status, CoreOutput::Status::protective_stop);
  EXPECT_LE(k * kDt, braking_time + 2 * kDt);
  const Eigen::VectorXd held = core.state().head(6);
  for (int i = 0; i < 200; ++i) {
    const CoreOutput & out = core.step(constantVelocityInput(core, kHome, tau, 1.0));
    EXPECT_EQ(out.status, CoreOutput::Status::protective_stop);
    EXPECT_EQ(out.dq.cwiseAbs().maxCoeff(), 0.0);
  }
  EXPECT_EQ((core.state().head(6) - held).cwiseAbs().maxCoeff(), 0.0);
}

// The stop ends only with the distance restored beyond the hysteresis for the
// resume delay; then the trajectory continues.
TEST(ThorCore, ProtectiveStopResumesAfterTheDelay)
{
  auto s = defaultSettings();
  s.protective_stop.resume_margin = 0.05;
  s.protective_stop.resume_delay = 0.5;
  ThorCore core;
  core.configure(readUrdf(), s);
  core.reset(kHome, Eigen::VectorXd());
  const Obstacle near = beyondTheTool(core, 0.02);
  core.setObstacles({near});
  const CoreInput still = constantVelocityInput(core, kHome, 0.0, 0.0);
  ASSERT_EQ(core.step(still).status, CoreOutput::Status::protective_stop);

  // Within the hysteresis (0.05 to 0.10 m): still stopped.
  core.setObstacles({beyondTheTool(core, 0.08)});
  for (int i = 0; i < 500; ++i) ASSERT_EQ(core.step(still).status, CoreOutput::Status::protective_stop);

  // Restored: stopped for the delay, then running.
  core.setObstacles({beyondTheTool(core, 0.2)});
  int cycles = 0;
  while (cycles < 1000 && core.step(still).status == CoreOutput::Status::protective_stop) ++cycles;
  EXPECT_NEAR(cycles * kDt, 0.5, 2 * kDt);
  const double tau = moveBase(core, 0.5, 200);
  EXPECT_GT(tau, 0.1);

  // Invalid settings.
  s.protective_stop.resume_delay = -1.0;
  EXPECT_THROW(ThorCore().configure(readUrdf(), s), std::invalid_argument);
}

// The stop distance is the worst case over the perception errors.
TEST(ThorCore, ProtectiveStopUsesPerceptionErrors)
{
  auto stopped = [](const Obstacle & o) {
    ThorCore core;
    core.configure(readUrdf(), defaultSettings());
    core.reset(kHome, Eigen::VectorXd());
    Obstacle placed = o;
    placed.position = beyondTheTool(core, 0.08).position;
    core.setObstacles({placed});
    return core.step(constantVelocityInput(core, kHome, 0.0, 0.0)).status == CoreOutput::Status::protective_stop;
  };
  Obstacle o;
  EXPECT_FALSE(stopped(o));  // 0.08 m > 0.05 m
  o.delta_h = 0.04;          // 0.08 - 0.04 < 0.05
  EXPECT_TRUE(stopped(o));
  o.delta_h = 0.0;
  o.sigma = Eigen::Vector3d(0.011, -1.0, -1.0);  // 0.08 - 3 * 0.011 < 0.05
  EXPECT_TRUE(stopped(o));
}

// Tube recovery (Eqs. 25-27 of the paper): the robot stops outside the tube
// (its braking distance is longer than the tube while the trajectory time is
// stopped); when it may move again, it goes back to the reference with the
// trajectory time still stopped, then the trajectory resumes.
TEST(ThorCore, RecoversIntoTheTubeAfterAStop)
{
  auto s = defaultSettings();
  s.mpc.tube_bound = Eigen::VectorXd::Constant(6, 0.05);
  ThorCore core;
  core.configure(readUrdf(), s);
  core.reset(kHome, Eigen::VectorXd());
  double tau = moveBase(core, 1.5, 300);

  // Stop: braking from 1.5 rad/s at 10 rad/s^2 takes ~0.11 rad.
  core.setObstacles({beyondTheTool(core, 0.02)});
  for (int i = 0; i < 300; ++i) tau += core.step(constantVelocityInput(core, kHome, tau, 1.5)).scaling * kDt;
  const CoreOutput & stopped = core.step(constantVelocityInput(core, kHome, tau, 1.5));
  ASSERT_EQ(stopped.status, CoreOutput::Status::protective_stop);
  ASSERT_GT(stopped.tube_error, 0.05);

  core.setObstacles({});
  bool recovered = false;
  bool resumed = false;
  double last_error = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 2000 && !resumed; ++i) {
    const CoreOutput & out = core.step(constantVelocityInput(core, kHome, tau, 1.5));
    tau += out.scaling * kDt;
    if (out.status == CoreOutput::Status::recovery) {
      recovered = true;
      EXPECT_EQ(out.scaling, 0.0);
      EXPECT_LE(out.tube_error, last_error + 1e-9);  // back towards the reference
      last_error = out.tube_error;
    }
    if (recovered && out.status == CoreOutput::Status::ok && out.scaling > 0.5) {
      resumed = true;
      EXPECT_LE(out.tube_error, 0.05 + 1e-6);
    }
  }
  EXPECT_TRUE(recovered);
  EXPECT_TRUE(resumed);
}

// With a tube, the robot slows down along the path instead of deviating
// around the person. The tube holds whenever the MPC has a solution; a
// fallback braking may leave it, and the recovery brings the robot back.
TEST(ThorCore, TubeLimitsTheDeviationAroundAnObstacle)
{
  ThorCore free_core;
  free_core.configure(readUrdf(), defaultSettings(false));
  free_core.reset(kHome, Eigen::VectorXd());
  run(free_core, kHome, 0.8, 500);
  Obstacle person;
  person.position = free_core.sphereCenters().back();
  person.radius = 0.1;
  person.half_axis = Eigen::Vector3d(0.0, 0.0, 0.5);

  auto settings = defaultSettings(true);
  settings.protective_stop.enabled = false;  // only the MPC here
  ThorCore deviating;
  deviating.configure(readUrdf(), settings);
  deviating.reset(kHome, Eigen::VectorXd());
  deviating.setObstacles({person});
  const RunResult free_run = run(deviating, kHome, 0.8, 1500);

  constexpr double kTube = 0.05;
  settings.mpc.tube_bound = Eigen::VectorXd::Constant(6, kTube);
  ThorCore in_tube;
  in_tube.configure(readUrdf(), settings);
  in_tube.reset(kHome, Eigen::VectorXd());
  in_tube.setObstacles({person});
  double tau = 0.0, min_scaling = 1.0, min_distance = 1.0, max_ok_error = 0.0;
  int fallback = 0;
  for (int k = 0; k < 1500; ++k) {
    const CoreOutput & out = in_tube.step(constantVelocityInput(in_tube, kHome, tau, 0.8));
    tau += out.scaling * kDt;
    min_scaling = std::min(min_scaling, out.scaling);
    min_distance = std::min(min_distance, out.min_distance);
    if (out.status == CoreOutput::Status::ok)
      max_ok_error = std::max(max_ok_error, out.tube_error);
    else
      ++fallback;
  }
  std::cout << "without tube: deviation " << free_run.max_deviation << " rad, min scaling " << free_run.min_scaling
            << "; with tube: max error " << max_ok_error << " rad when solved, " << fallback
            << " braking or recovery cycles, min scaling " << min_scaling << "\n";
  EXPECT_LE(max_ok_error, kTube + 1e-6);
  EXPECT_GT(free_run.max_deviation, 3 * kTube);  // without the tube it goes around
  EXPECT_LT(min_scaling, 0.1);                   // with it, it stops in front of the person
  EXPECT_GT(min_distance, 0.0);
}

TEST(ThorCore, PositionFeedbackUsesMeasuredPosition)
{
  auto s = defaultSettings(false);
  s.state_feedback = thor_controllers::StateFeedback::position;
  ThorCore core;
  core.configure(readUrdf(), s);
  core.reset(kHome, Eigen::VectorXd());
  CoreInput in = constantVelocityInput(core, kHome, 0.0, 0.0);
  in.measured_q = kHome;
  in.measured_q(1) += 0.01;  // the robot is somewhere else
  in.next_target_q = in.measured_q;
  const CoreOutput & out = core.step(in);
  EXPECT_NEAR(out.q(1), in.measured_q(1), 1e-6);

  // Open loop ignores it.
  ThorCore open;
  open.configure(readUrdf(), defaultSettings(false));
  open.reset(kHome, Eigen::VectorXd());
  in.next_target_q = kHome;
  EXPECT_NEAR(open.step(in).q(1), kHome(1), 1e-6);
}


TEST(ThorCore, ZeroResumeDelayStillRequiresClearanceAndRest)
{
  auto s = defaultSettings(false);
  s.protective_stop.resume_delay = 0.0;
  ThorCore core;
  core.configure(readUrdf(), s);
  core.reset(kHome, Eigen::VectorXd::Constant(6, 0.5));
  auto in = constantVelocityInput(core, kHome, 0.0, 0.0);
  core.setObstacles({beyondTheTool(core, 0.02)});
  ASSERT_EQ(core.step(in).status, CoreOutput::Status::protective_stop);
  core.setObstacles({});
  EXPECT_EQ(core.step(in).status, CoreOutput::Status::protective_stop);
  core.reset(kHome, Eigen::VectorXd());
  core.setObstacles({beyondTheTool(core, 0.02)});
  ASSERT_EQ(core.step(in).status, CoreOutput::Status::protective_stop);
  core.setObstacles({beyondTheTool(core, 0.08)});
  EXPECT_EQ(core.step(in).status, CoreOutput::Status::protective_stop);
  core.setObstacles({});
  EXPECT_EQ(core.step(in).status, CoreOutput::Status::ok);
}

TEST(ThorCore, ResetClearsStopDelayAndRecovery)
{
  auto s = defaultSettings(false);
  s.mpc.tube_bound = Eigen::VectorXd::Constant(6, 0.01);
  ThorCore core;
  core.configure(readUrdf(), s);
  core.reset(kHome, Eigen::VectorXd());
  auto in = constantVelocityInput(core, kHome, 0.0, 0.0);
  in.next_target_q(0) += 0.2;
  ASSERT_EQ(core.step(in).status, CoreOutput::Status::braking);
  ASSERT_EQ(core.step(in).status, CoreOutput::Status::recovery);
  core.reset(kHome, Eigen::VectorXd());
  EXPECT_EQ(core.step(in).status, CoreOutput::Status::braking);

  in.next_target_q = kHome;
  core.setObstacles({beyondTheTool(core, 0.02)});
  ASSERT_EQ(core.step(in).status, CoreOutput::Status::protective_stop);
  core.setObstacles({});
  for (int i = 0; i < 100; ++i)
    ASSERT_EQ(core.step(in).status, CoreOutput::Status::protective_stop);
  core.reset(kHome, Eigen::VectorXd());
  EXPECT_EQ(core.step(in).status, CoreOutput::Status::ok);
  // A new stop must wait the full delay, including after a reset mid-delay.
  core.setObstacles({beyondTheTool(core, 0.02)});
  ASSERT_EQ(core.step(in).status, CoreOutput::Status::protective_stop);
  core.setObstacles({});
  for (int i = 0; i < 200; ++i)
    ASSERT_EQ(core.step(in).status, CoreOutput::Status::protective_stop);
}

TEST(ThorCore, InvalidObstaclesBrakeEvenWithoutCbfOrProtectiveStop)
{
  for (bool cbf : {false, true}) {
    auto s = defaultSettings(cbf);
    s.protective_stop.enabled = false;
    for (int field = 0; field < 7; ++field) {
      for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}) {
        ThorCore core;
        core.configure(readUrdf(), s);
        core.reset(kHome, Eigen::VectorXd::Constant(6, 0.5));
        auto in = constantVelocityInput(core, kHome, 0.0, 0.5);
        Obstacle o = beyondTheTool(core, 0.02);
        switch (field) {
          case 0: o.position.x() = bad; break;
          case 1: o.velocity.x() = bad; break;
          case 2: o.acceleration.x() = bad; break;
          case 3: o.half_axis.x() = bad; break;
          case 4: o.radius = bad; break;
          case 5: o.delta_h = bad; break;
          case 6: o.sigma.x() = bad; break;
        }
        core.setObstacles({o});
        for (int i = 0; i < 100; ++i) {
          const auto & out = core.step(in);
          ASSERT_EQ(out.status, CoreOutput::Status::protective_stop);
          EXPECT_EQ(out.scaling, 0.0);
          EXPECT_TRUE(out.q.allFinite());
          EXPECT_TRUE(out.ddq.allFinite());
          EXPECT_LE(out.ddq.cwiseAbs().maxCoeff(), 10.0 + 1e-9);
        }
        EXPECT_LT(core.state().tail(6).norm(), 1e-9);
        core.setObstacles({});
        EXPECT_EQ(core.step(in).status, CoreOutput::Status::ok);
      }
    }
  }
}

}  // namespace
