#include <gtest/gtest.h>

#include <thor_math/cbf.h>

#include <array>
#include <cmath>
#include <functional>

namespace thor::math::cbf {
namespace {

constexpr double kTr = 0.15;
constexpr double kAs = 2.5;
constexpr double kC = 0.1;
constexpr double kAtol = 1e-12;

using Psi = std::array<double, 4>;  // {d, v_r, v_o, a_o}

Eigen::Vector4d numericJacobian(const std::function<double(const Psi&)>& f, const Psi& psi)
{
  constexpr double eps = 1e-6;
  Eigen::Vector4d jac;
  for (std::size_t i = 0; i < 4; ++i) {
    Psi plus = psi, minus = psi;
    plus[i] += eps;
    minus[i] -= eps;
    jac(static_cast<Eigen::Index>(i)) = (f(plus) - f(minus)) / (2.0 * eps);
  }
  return jac;
}

double dmin(const Psi& p)
{
  Eigen::Vector4d jac;
  return minimumDistance(p[0], p[1], p[2], p[3], kTr, kAs, kAtol, jac);
}

double h(const Psi& p)
{
  Eigen::Vector4d jac;
  return barrier(p[0], p[1], p[2], p[3], kTr, kAs, kC, kAtol, jac);
}

TEST(CbfMinimumDistance, ApproachingRobotBrakesToStop)
{
  // t_dec = 0.2, t4 = 0.35: d_min = d + v_diff*t4 + a_s*t_dec^2/2.
  Eigen::Vector4d jac;
  const double value = minimumDistance(1.0, -0.5, 0.3, 0.0, kTr, kAs, kAtol, jac);
  EXPECT_NEAR(value, 0.77, 1e-12);
  // d(d_min)/dv_r = T_r - (v_r - v_o)/a_s
  EXPECT_TRUE(jac.isApprox(Eigen::Vector4d(1.0, 0.47, -0.35, -0.5 * 0.35 * 0.35), 1e-12));
}

TEST(CbfMinimumDistance, RecedingRobotKeepsCurrentDistance)
{
  Eigen::Vector4d jac;
  EXPECT_DOUBLE_EQ(minimumDistance(1.0, 0.4, 0.3, 0.0, kTr, kAs, kAtol, jac), 1.0);
  EXPECT_TRUE(jac.isApprox(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)));

  // Faster obstacle: the minimum is at the end of the reaction time.
  EXPECT_NEAR(minimumDistance(1.0, 0.1, 0.5, 0.0, kTr, kAs, kAtol, jac), 0.94, 1e-12);
  EXPECT_TRUE(jac.isApprox(Eigen::Vector4d(1.0, kTr, -kTr, -0.5 * kTr * kTr), 1e-12));
}

TEST(CbfMinimumDistance, RejectsNonPositiveDeceleration)
{
  Eigen::Vector4d jac;
  EXPECT_THROW(minimumDistance(1.0, -0.5, 0.0, 0.0, kTr, 0.0, kAtol, jac), std::invalid_argument);
}

TEST(CbfMinimumDistance, JacobianMatchesFiniteDifferences)
{
  const std::array<Psi, 7> cases = {{
    {1.0, -0.5, 0.3, 0.0},   // braking, minimum at t4
    {1.0, -0.5, 0.3, 1.2},   // obstacle accelerating
    {1.0, 0.4, 0.3, -2.0},   // receding, minimum at t0
    {1.0, 0.1, 0.2, -1.0},   // interior minimum t1 during the reaction
    {1.0, -2.0, -1.5, 0.0},  // interior minimum t3 during braking
    {0.12, -0.8, 0.5, 0.0},  // d_min below the margin
    {0.6, -1.0, -0.2, 3.0},
  }};
  for (const Psi& psi : cases) {
    SCOPED_TRACE(::testing::Message() << psi[0] << ", " << psi[1] << ", " << psi[2] << ", " << psi[3]);
    Eigen::Vector4d jac;
    minimumDistance(psi[0], psi[1], psi[2], psi[3], kTr, kAs, kAtol, jac);
    EXPECT_TRUE(jac.isApprox(numericJacobian(dmin, psi), 1e-6)) << jac.transpose();

    barrier(psi[0], psi[1], psi[2], psi[3], kTr, kAs, kC, kAtol, jac);
    EXPECT_TRUE(jac.isApprox(numericJacobian(h, psi), 1e-6)) << jac.transpose();
  }
}

TEST(CbfBarrier, AddsRecedingTermInsideTheMargin)
{
  Eigen::Vector4d jac;
  // Above the margin h = d_min - C.
  EXPECT_NEAR(barrier(1.0, -0.5, 0.3, 0.0, kTr, kAs, kC, kAtol, jac), 0.77 - kC, 1e-12);

  // Receding robot inside the margin: d_min = d, h = d - C + (C - d)/C*T_r*v_r.
  const double d = 0.05, v_r = 0.3;
  EXPECT_NEAR(barrier(d, v_r, 0.0, 0.0, kTr, kAs, kC, kAtol, jac),
              d - kC + (kC - d) / kC * kTr * v_r, 1e-12);
}

TEST(CbfBarrierTerms, ProjectsTheStateOnTheRobotObstacleDirection)
{
  const BarrierTerms b = barrierTerms(Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(-0.5, 0.2, 0.0),
                                      Eigen::Vector3d::Zero(), Eigen::Vector3d(0.3, 0.0, 0.1),
                                      Eigen::Vector3d::Zero(), kTr, kAs, kC);
  EXPECT_DOUBLE_EQ(b.d, 1.0);
  EXPECT_DOUBLE_EQ(b.v_r, -0.5);
  EXPECT_DOUBLE_EQ(b.v_o, 0.3);
  EXPECT_NEAR(b.h, 0.77 - kC, 1e-12);
  EXPECT_DOUBLE_EQ(b.h_chi, 1.0);
  EXPECT_TRUE(b.lie_g.isApprox(Eigen::Vector3d(0.47, 0.0, 0.0), 1e-12));
}

// dh/dt along the closed-loop motion must equal L_f h + L_g h * a_r.
TEST(CbfBarrierTerms, LieDerivativesMatchTimeDerivative)
{
  struct Case { Eigen::Vector3d p_r, v_r, a_r, p_o, v_o, a_o; };
  const std::array<Case, 4> cases = {{
    {{0.5, 0.2, 0.8}, {-0.3, 0.1, 0.05}, {1.0, -2.0, 0.5}, {1.2, 0.0, 0.9}, {-0.4, 0.2, 0.0}, {0.0, 0.0, 0.0}},
    {{0.5, 0.2, 0.8}, {-0.3, 0.1, 0.05}, {1.0, -2.0, 0.5}, {1.2, 0.0, 0.9}, {-0.4, 0.2, 0.0}, {0.5, -1.0, 0.2}},
    {{0.3, -0.1, 0.5}, {0.2, 0.3, 0.0}, {-3.0, 0.0, 1.0}, {0.4, 0.0, 0.5}, {0.1, 0.0, 0.0}, {0.0, 0.0, 0.0}},
    {{0.0, 0.0, 1.0}, {0.8, 0.0, -0.6}, {0.0, 4.0, 0.0}, {1.0, 0.5, 0.2}, {-1.0, -0.2, 0.3}, {0.3, 0.0, -0.4}},
  }};
  // Spheres too: the radii are constant, so the Lie derivatives stay exact.
  for (const double radius : {0.0, 0.05, 0.2}) {
    for (std::size_t index = 0; index < cases.size(); ++index) {
      SCOPED_TRACE(::testing::Message() << "radius " << radius << " case " << index);
      const Case& c = cases[index];
      auto h_at = [&](double t) {
        return barrierTerms(c.p_r + c.v_r * t + 0.5 * c.a_r * t * t, c.v_r + c.a_r * t,
                            c.p_o + c.v_o * t + 0.5 * c.a_o * t * t, c.v_o + c.a_o * t, c.a_o,
                            kTr, kAs, kC, radius).h;
      };
      constexpr double dt = 1e-6;
      const double numeric = (h_at(dt) - h_at(-dt)) / (2.0 * dt);
      const BarrierTerms b = barrierTerms(c.p_r, c.v_r, c.p_o, c.v_o, c.a_o, kTr, kAs, kC, radius);
      EXPECT_NEAR(b.lie_f + b.lie_g.dot(c.a_r), numeric, 1e-6);
    }
  }
}

// The gradients with respect to the robot center position (grad_p) and
// velocity (lie_g) linearize the barrier in the discrete-time CBF.
TEST(CbfBarrierTerms, StateGradientsMatchFiniteDifferences)
{
  struct Case { Eigen::Vector3d p_r, v_r, p_o, v_o, a_o; };
  const std::array<Case, 3> cases = {{
    {{0.5, 0.2, 0.8}, {-0.3, 0.1, 0.05}, {1.2, 0.0, 0.9}, {-0.4, 0.2, 0.0}, {0.5, -1.0, 0.2}},
    {{0.3, -0.1, 0.5}, {0.2, 0.3, 0.0}, {0.4, 0.0, 0.5}, {0.1, 0.0, 0.0}, {0.0, 0.0, 0.0}},
    {{0.0, 0.0, 1.0}, {0.8, 0.0, -0.6}, {1.0, 0.5, 0.2}, {-1.0, -0.2, 0.3}, {0.3, 0.0, -0.4}},
  }};
  constexpr double eps = 1e-6;
  for (const double radius : {0.0, 0.05, 0.2}) {
    for (std::size_t index = 0; index < cases.size(); ++index) {
      SCOPED_TRACE(::testing::Message() << "radius " << radius << " case " << index);
      const Case& c = cases[index];
      auto h_at = [&](const Eigen::Vector3d& p_r, const Eigen::Vector3d& v_r) {
        return barrierTerms(p_r, v_r, c.p_o, c.v_o, c.a_o, kTr, kAs, kC, radius).h;
      };
      Eigen::Vector3d grad_p, grad_v;
      for (Eigen::Index i = 0; i < 3; ++i) {
        const Eigen::Vector3d e = eps * Eigen::Vector3d::Unit(i);
        grad_p(i) = (h_at(c.p_r + e, c.v_r) - h_at(c.p_r - e, c.v_r)) / (2.0 * eps);
        grad_v(i) = (h_at(c.p_r, c.v_r + e) - h_at(c.p_r, c.v_r - e)) / (2.0 * eps);
      }
      const BarrierTerms b = barrierTerms(c.p_r, c.v_r, c.p_o, c.v_o, c.a_o, kTr, kAs, kC, radius);
      EXPECT_TRUE(b.grad_p.isApprox(grad_p, 1e-6)) << b.grad_p.transpose() << " vs " << grad_p.transpose();
      EXPECT_TRUE(b.lie_g.isApprox(grad_v, 1e-6)) << b.lie_g.transpose() << " vs " << grad_v.transpose();
    }
  }
}

// Spheres: the barrier sees the distance between the surfaces, while the
// projection direction is still the one between the centers.
TEST(CbfBarrierTerms, SphereRadiusShiftsTheDistance)
{
  const Eigen::Vector3d p_r(1.0, 0.0, 0.0), v_r(-0.5, 0.2, 0.0);
  const Eigen::Vector3d p_o = Eigen::Vector3d::Zero(), v_o(0.3, 0.0, 0.1);
  const double radius = 0.25;
  const BarrierTerms sphere = barrierTerms(p_r, v_r, p_o, v_o, Eigen::Vector3d::Zero(),
                                           kTr, kAs, kC, radius);
  EXPECT_DOUBLE_EQ(sphere.d, 1.0 - radius);
  EXPECT_DOUBLE_EQ(sphere.v_r, -0.5);
  EXPECT_DOUBLE_EQ(sphere.v_o, 0.3);

  Eigen::Vector4d jac;
  EXPECT_NEAR(sphere.h, barrier(1.0 - radius, -0.5, 0.3, 0.0, kTr, kAs, kC, kAtol, jac), 1e-12);
  EXPECT_DOUBLE_EQ(sphere.h_chi, jac(0));
  EXPECT_TRUE(sphere.lie_g.isApprox(jac(1) * Eigen::Vector3d::UnitX(), 1e-12));

  // Same pair as points: the radius lowers h by exactly the radius when
  // d_min stays above the margin.
  const BarrierTerms point = barrierTerms(p_r, v_r, p_o, v_o, Eigen::Vector3d::Zero(), kTr, kAs, kC);
  EXPECT_NEAR(point.h - sphere.h, radius, 1e-12);
  EXPECT_NEAR(point.lie_f, sphere.lie_f, 1e-12);
}

TEST(CbfBarrierTerms, OverlappingSpheresGiveNegativeDistance)
{
  const BarrierTerms b = barrierTerms(Eigen::Vector3d(0.1, 0.0, 0.0), Eigen::Vector3d::Zero(),
                                      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                      Eigen::Vector3d::Zero(), kTr, kAs, kC, 0.3);
  EXPECT_NEAR(b.d, -0.2, 1e-12);
  EXPECT_LT(b.h, 0.0);
}

// Reference values printed by cbf_python/controllers/kernels/ssm_cbf_acc.py
// (branch dynamic_parameters) with T_r = 0.15, a_s = 2.5, C = 0.1.
TEST(CbfPythonReference, MinimumDistanceAndBarrier)
{
  struct Row { double d, v_r, v_o, a_o, d_min; Psi jac_dmin; double h; Psi jac_h; };
  const std::array<Row, 10> rows = {{
    {1.0, -0.5, 0.3, 0.0, 0.77, {1.0, 0.47, -0.35, -0.06124999999999999}, 0.67, {1.0, 0.47, -0.35, -0.06124999999999999}},
    {1.0, -0.5, 0.3, 1.2, 0.6964999999999999, {1.0, 0.638, -0.35, -0.06124999999999999}, 0.5964999999999999, {1.0, 0.638, -0.35, -0.06124999999999999}},
    {1.0, 0.4, 0.3, -2.0, 1.0, {1.0, 0.0, 0.0, 0.0}, 0.9, {1.0, 0.0, 0.0, 0.0}},
    {1.0, 0.1, 0.2, -1.0, 0.995, {1.0, 0.1, -0.1, -0.005000000000000001}, 0.895, {1.0, 0.1, -0.1, -0.005000000000000001}},
    {1.0, -2.0, -1.5, 0.0, 0.875, {1.0, 0.35, -0.35, -0.06124999999999999}, 0.775, {1.0, 0.35, -0.35, -0.06124999999999999}},
    {0.12, -0.8, 0.5, 0.0, -0.36300000000000004, {1.0, 0.67, -0.47, -0.11044999999999999}, -1.0186000000000002, {2.2, 2.1685000000000003, -1.0339999999999998, -0.24298999999999998}},
    {0.6, -1.0, -0.2, 3.0, -0.09375000000000017, {1.0, 1.1300000000000001, -0.55, -0.15125000000000002}, -0.48437500000000044, {2.5, 3.1156250000000005, -1.375, -0.37812500000000004}},
    {0.05, 0.3, 0.0, 0.0, 0.05, {1.0, 0.0, 0.0, 0.0}, -0.027500000000000004, {0.55, 0.075, 0.0, 0.0}},
    {1.0, 0.0, 0.0, 0.0, 1.0, {1.0, 0.0, 0.0, 0.0}, 0.9, {1.0, 0.0, 0.0, 0.0}},
    {0.3, -0.4, 0.6, -1.5, 0.09407499999999999, {1.0, 0.364, -0.31, -0.04805}, -0.009480000000000021, {1.6, 0.5912875, -0.496, -0.07688}},
  }};
  for (std::size_t index = 0; index < rows.size(); ++index) {
    SCOPED_TRACE(index);
    const Row& r = rows[index];
    Eigen::Vector4d jac;
    EXPECT_NEAR(minimumDistance(r.d, r.v_r, r.v_o, r.a_o, kTr, kAs, kAtol, jac), r.d_min, 1e-12);
    for (Eigen::Index i = 0; i < 4; ++i)
      EXPECT_NEAR(jac(i), r.jac_dmin[static_cast<std::size_t>(i)], 1e-12);
    EXPECT_NEAR(barrier(r.d, r.v_r, r.v_o, r.a_o, kTr, kAs, kC, kAtol, jac), r.h, 1e-12);
    for (Eigen::Index i = 0; i < 4; ++i)
      EXPECT_NEAR(jac(i), r.jac_h[static_cast<std::size_t>(i)], 1e-12);
  }
}

TEST(CbfPythonReference, BarrierTerms)
{
  struct Row
  {
    Eigen::Vector3d p_r, v_r, p_o, v_o, a_o;
    double h, lie_f;
    Eigen::Vector3d lie_g;
    double h_chi, d, vr, vo;
  };
  const std::array<Row, 5> rows = {{
    {{0.5, 0.2, 0.8}, {-0.3, 0.1, 0.05}, {1.2, 0.0, 0.9}, {-0.4, 0.2, 0.0}, {0.0, 0.0, 0.0}, 0.6154551290379199, -0.12809735132494596, {-0.14288690166235204, 0.04082482904638631, -0.020412414523193145}, 1.0, 0.7348469228349533, 0.3061862178478973, 0.43546484316145395},
    {{0.5, 0.2, 0.8}, {-0.3, 0.1, 0.05}, {1.2, 0.0, 0.9}, {-0.4, 0.2, 0.0}, {0.5, -1.0, 0.2}, 0.624181436246585, -0.012660894609929698, {-0.14288690166235204, 0.04082482904638631, -0.020412414523193145}, 1.0, 0.7348469228349533, 0.3061862178478973, 0.43546484316145395},
    {{0.3, -0.1, 0.5}, {0.2, 0.3, 0.0}, {0.4, 0.0, 0.5}, {0.1, 0.0, 0.0}, {0.0, 0.0, 0.0}, -0.02602334059711293, -0.37409056967213805, {-0.3027790039756342, -0.30277900397563406, 0.0}, 1.5303300858899105, 0.14142135623730953, -0.35355339059327373, -0.07071067811865477},
    {{0.0, 0.0, 1.0}, {0.8, 0.0, -0.6}, {1.0, 0.5, 0.2}, {-1.0, -0.2, 0.3}, {0.3, 0.0, -0.4}, 0.5140693713677185, -1.4559120034921902, {-0.5950552681680749, -0.29752763408403743, 0.47604421453445983}, 1.0, 1.374772708486752, -0.9310629983402341, 0.9747065763874326},
    {{0.0, 0.0, 0.5}, {-0.2, 0.0, 0.0}, {0.05, 0.0, 0.5}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, -0.035, 0.13999999999999999, {-0.075, 0.0, 0.0}, 0.7, 0.05, 0.2, 0.0},
  }};
  for (std::size_t index = 0; index < rows.size(); ++index) {
    SCOPED_TRACE(index);
    const Row& r = rows[index];
    const BarrierTerms b = barrierTerms(r.p_r, r.v_r, r.p_o, r.v_o, r.a_o, kTr, kAs, kC);
    EXPECT_NEAR(b.h, r.h, 1e-12);
    EXPECT_NEAR(b.lie_f, r.lie_f, 1e-12);
    EXPECT_TRUE(b.lie_g.isApprox(r.lie_g, 1e-12)) << b.lie_g.transpose();
    EXPECT_NEAR(b.h_chi, r.h_chi, 1e-12);
    EXPECT_NEAR(b.d, r.d, 1e-12);
    EXPECT_NEAR(b.v_r, r.vr, 1e-12);
    EXPECT_NEAR(b.v_o, r.vo, 1e-12);
  }
}

TEST(CbfPerceptionMargin, MatchesEquation9)
{
  EXPECT_NEAR(perceptionMargin(3.291, 0.015, 0.072, 0.196, 0.3),
              3.291 * (0.015 + 0.072 * 0.3 + 0.5 * 0.196 * 0.09), 1e-15);
}

}  // namespace
}  // namespace thor::math::cbf
