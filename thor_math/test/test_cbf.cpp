#include <gtest/gtest.h>

#include <thor_math/cbf.h>

#include <array>
#include <cmath>
#include <functional>
#include <limits>

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

// Capsules: beyond the ends they are spheres centered at the ends, along the
// side the distance is measured from the axis.
TEST(CbfCapsuleBarrierTerms, ReducesToSpheresAndToTheAxisDistance)
{
  const Eigen::Vector3d p_o(0.0, 0.0, 1.0), half_axis(0.0, 0.0, 0.5);
  const Eigen::Vector3d v_o(0.3, 0.0, 0.2), a_o(0.0, 0.1, -0.4);
  const Eigen::Vector3d v_r(-0.5, 0.1, 0.7);
  const double radius = 0.2;

  // Above the top end: the sphere at the top end.
  const Eigen::Vector3d above(0.3, 0.2, 1.8);
  const BarrierTerms top = capsuleBarrierTerms(above, v_r, p_o, half_axis, v_o, a_o, kTr, kAs, kC, radius);
  const BarrierTerms top_sphere = barrierTerms(above, v_r, p_o + half_axis, v_o, a_o, kTr, kAs, kC, radius);
  EXPECT_DOUBLE_EQ(top.h, top_sphere.h);
  EXPECT_TRUE(top.lie_g.isApprox(top_sphere.lie_g));

  // Along the side: the distance from the axis, and the axial components of
  // the velocities do not matter.
  const Eigen::Vector3d side(1.0, 0.0, 1.3);
  const BarrierTerms b = capsuleBarrierTerms(side, v_r, p_o, half_axis, v_o, a_o, kTr, kAs, kC, radius);
  EXPECT_NEAR(b.d, 1.0 - radius, 1e-12);
  EXPECT_DOUBLE_EQ(b.v_r, -0.5);
  EXPECT_DOUBLE_EQ(b.v_o, 0.3);
  EXPECT_NEAR(b.lie_g.z(), 0.0, 1e-12);
  EXPECT_NEAR(b.grad_p.z(), 0.0, 1e-12);
  const Eigen::Vector3d axial(0.0, 0.0, 1.0);
  const BarrierTerms shifted = capsuleBarrierTerms(side, v_r + axial, p_o, half_axis, v_o - axial, a_o + axial,
                                                   kTr, kAs, kC, radius);
  EXPECT_NEAR(shifted.h, b.h, 1e-12);
  EXPECT_NEAR(shifted.lie_f, b.lie_f, 1e-12);

  // Zero axis: a sphere.
  const BarrierTerms sphere = capsuleBarrierTerms(side, v_r, p_o, Eigen::Vector3d::Zero(), v_o, a_o, kTr, kAs, kC, radius);
  EXPECT_DOUBLE_EQ(sphere.h, barrierTerms(side, v_r, p_o, v_o, a_o, kTr, kAs, kC, radius).h);

  // Robot center on the axis: u is orthogonal to the axis.
  const BarrierTerms inside = capsuleBarrierTerms(p_o, v_r, p_o, half_axis, v_o, a_o, kTr, kAs, kC, radius);
  EXPECT_NEAR(inside.d, -radius, 1e-9);
  EXPECT_NEAR(inside.lie_g.dot(half_axis), 0.0, 1e-12);
  EXPECT_TRUE(inside.lie_g.allFinite());
}

// As for spheres, the Lie derivatives and the state gradients must match finite
// differences, on the side and beyond the ends, with a tilted axis.
TEST(CbfCapsuleBarrierTerms, DerivativesMatchFiniteDifferences)
{
  struct Case { Eigen::Vector3d p_r, v_r, a_r, p_o, half_axis, v_o, a_o; };
  const std::array<Case, 4> cases = {{
    // side
    {{0.5, 0.2, 0.8}, {-0.3, 0.1, 0.05}, {1.0, -2.0, 0.5}, {1.2, 0.0, 0.9}, {0.1, 0.2, 0.6}, {-0.4, 0.2, 0.0}, {0.5, -1.0, 0.2}},
    {{0.3, -0.1, 0.5}, {0.2, 0.3, 0.0}, {-3.0, 0.0, 1.0}, {0.4, 0.0, 0.5}, {0.0, 0.0, 0.8}, {0.1, 0.0, 0.0}, {0.0, 0.0, 0.0}},
    // beyond the top end
    {{0.0, 0.0, 1.9}, {0.8, 0.0, -0.6}, {0.0, 4.0, 0.0}, {0.5, 0.3, 0.8}, {0.0, 0.1, 0.5}, {-1.0, -0.2, 0.3}, {0.3, 0.0, -0.4}},
    // beyond the bottom end
    {{1.0, 0.4, -0.6}, {-0.2, 0.0, 0.4}, {0.5, 0.5, -1.0}, {0.8, 0.2, 0.5}, {-0.1, 0.0, 0.6}, {0.0, 0.3, 0.0}, {0.0, 0.0, 0.0}},
  }};
  constexpr double eps = 1e-6;
  for (const double radius : {0.0, 0.1}) {
    for (std::size_t index = 0; index < cases.size(); ++index) {
      SCOPED_TRACE(::testing::Message() << "radius " << radius << " case " << index);
      const Case& c = cases[index];
      auto terms = [&](const Eigen::Vector3d& p_r, const Eigen::Vector3d& v_r,
                       const Eigen::Vector3d& p_o, const Eigen::Vector3d& v_o) {
        return capsuleBarrierTerms(p_r, v_r, p_o, c.half_axis, v_o, c.a_o, kTr, kAs, kC, radius);
      };
      const BarrierTerms b = terms(c.p_r, c.v_r, c.p_o, c.v_o);

      auto h_at = [&](double t) {
        return terms(c.p_r + c.v_r * t + 0.5 * c.a_r * t * t, c.v_r + c.a_r * t,
                     c.p_o + c.v_o * t + 0.5 * c.a_o * t * t, c.v_o + c.a_o * t).h;
      };
      EXPECT_NEAR(b.lie_f + b.lie_g.dot(c.a_r), (h_at(eps) - h_at(-eps)) / (2.0 * eps), 1e-6);

      Eigen::Vector3d grad_p, grad_v;
      for (Eigen::Index i = 0; i < 3; ++i) {
        const Eigen::Vector3d e = eps * Eigen::Vector3d::Unit(i);
        grad_p(i) = (terms(c.p_r + e, c.v_r, c.p_o, c.v_o).h - terms(c.p_r - e, c.v_r, c.p_o, c.v_o).h) / (2.0 * eps);
        grad_v(i) = (terms(c.p_r, c.v_r + e, c.p_o, c.v_o).h - terms(c.p_r, c.v_r - e, c.p_o, c.v_o).h) / (2.0 * eps);
      }
      EXPECT_TRUE(b.grad_p.isApprox(grad_p, 1e-6)) << b.grad_p.transpose() << " vs " << grad_p.transpose();
      EXPECT_TRUE(b.lie_g.isApprox(grad_v, 1e-6)) << b.lie_g.transpose() << " vs " << grad_v.transpose();
    }
  }
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

// The worst corner of the perception box relies on d_min being monotone in
// every component of psi: non-decreasing in d and v_r, non-increasing in v_o
// and a_o (docs/worst_case_h_design.md).
TEST(CbfMinimumDistance, IsMonotoneInEveryComponent)
{
  constexpr double step = 0.07;
  const std::array<double, 4> direction = {1.0, 1.0, -1.0, -1.0};
  for (const double d : {0.3, 0.8})
    for (const double v_r : {-1.2, -0.5, -0.05, 0.0, 0.05, 0.4})
      for (const double v_o : {-0.5, 0.0, 0.6})
        for (const double a_o : {-3.0, 0.0, 2.5})
          for (std::size_t i = 0; i < 4; ++i) {
            const Psi psi = {d, v_r, v_o, a_o};
            Psi moved = psi;
            moved[i] += step * direction[i];
            EXPECT_GE(dmin(moved), dmin(psi) - 1e-12)
              << "component " << i << " at " << d << " " << v_r << " " << v_o << " " << a_o;
          }
}

// The escape term starts at d_min = C; C_escape only changes its slope.
TEST(CbfBarrier, EscapeScaleSetsOnlyTheSlope)
{
  Eigen::Vector4d jac;
  // Receding robot inside the margin: d_min = d = 0.05.
  const double C_e = 0.4;
  EXPECT_NEAR(barrier(0.05, 0.3, 0.0, 0.0, kTr, kAs, kC, kAtol, jac, C_e),
              0.05 - kC + (kC - 0.05) / C_e * kTr * 0.3, 1e-15);
  EXPECT_DOUBLE_EQ(barrier(0.05, 0.3, 0.0, 0.0, kTr, kAs, kC, kAtol, jac, kC),
                   barrier(0.05, 0.3, 0.0, 0.0, kTr, kAs, kC, kAtol, jac));
  // Outside the margin the escape term is off.
  EXPECT_DOUBLE_EQ(barrier(0.5, 0.3, 0.0, 0.0, kTr, kAs, kC, kAtol, jac, C_e),
                   barrier(0.5, 0.3, 0.0, 0.0, kTr, kAs, kC, kAtol, jac));
  EXPECT_THROW(barrier(0.5, 0.3, 0.0, 0.0, kTr, kAs, kC, kAtol, jac, -0.1), std::invalid_argument);

  auto h_escape = [&](const Psi& p) {
    Eigen::Vector4d unused;
    return barrier(p[0], p[1], p[2], p[3], kTr, kAs, kC, kAtol, unused, C_e);
  };
  for (const Psi psi : {Psi{0.05, 0.3, 0.1, 0.5}, Psi{0.02, -0.2, 0.3, 0.0}, Psi{0.08, 1.0, -0.2, -1.0}}) {
    barrier(psi[0], psi[1], psi[2], psi[3], kTr, kAs, kC, kAtol, jac, C_e);
    EXPECT_TRUE(jac.isApprox(numericJacobian(h_escape, psi), 1e-6)) << jac.transpose();
  }

  // dh/dd = 1 - T_r v_r / C_e inside the margin: negative above v_r = C_e / T_r.
  const double v_r = 1.0;  // above kC / kTr, below C_e / kTr
  barrier(0.05, v_r, 0.0, 0.0, kTr, kAs, kC, kAtol, jac);
  EXPECT_LT(jac(0), 0.0);
  barrier(0.05, v_r, 0.0, 0.0, kTr, kAs, kC, kAtol, jac, C_e);
  EXPECT_GT(jac(0), 0.0);
}

// Cases for the perception bounds: approaching, receding and accelerating
// pairs, all with v_r below C / T_r over the box.
struct BoundsCase { Eigen::Vector3d p_r, v_r, p_o, v_o, a_o; };
const std::array<BoundsCase, 3> kBoundsCases = {{
  {{0.5, 0.2, 0.8}, {-0.3, 0.1, 0.05}, {1.2, 0.0, 0.9}, {-0.4, 0.2, 0.0}, {0.5, -1.0, 0.2}},
  {{0.3, -0.1, 0.5}, {0.2, 0.3, 0.0}, {0.4, 0.0, 0.5}, {0.1, 0.0, 0.0}, {0.0, 0.0, 0.0}},
  {{0.0, 0.0, 1.0}, {0.8, 0.0, -0.6}, {1.0, 0.5, 0.2}, {-1.0, -0.2, 0.3}, {0.3, 0.0, -0.4}},
}};

void expectSameTerms(const BarrierTerms& a, const BarrierTerms& b)
{
  EXPECT_NEAR(a.h, b.h, 1e-12);
  EXPECT_NEAR(a.lie_f, b.lie_f, 1e-12);
  EXPECT_TRUE(a.lie_g.isApprox(b.lie_g, 1e-12)) << a.lie_g.transpose() << " vs " << b.lie_g.transpose();
  EXPECT_TRUE(a.grad_p.isApprox(b.grad_p, 1e-12)) << a.grad_p.transpose() << " vs " << b.grad_p.transpose();
}

// Velocity and acceleration bounds do not rotate u: the corner is the state
// with the obstacle moving faster towards the robot, and the terms are those
// of that state.
TEST(CbfPerceptionBounds, VelocityBoundsMoveTheObstacleTowardsTheRobot)
{
  const PerceptionBounds bounds{0.0, 0.2, 1.5};
  for (std::size_t index = 0; index < kBoundsCases.size(); ++index) {
    SCOPED_TRACE(::testing::Message() << "case " << index);
    const BoundsCase& c = kBoundsCases[index];
    const Eigen::Vector3d u = (c.p_r - c.p_o).normalized();
    const BarrierTerms robust = barrierTerms(c.p_r, c.v_r, c.p_o, c.v_o, c.a_o, kTr, kAs, kC, 0.05, kAtol, bounds);
    const BarrierTerms corner = barrierTerms(c.p_r, c.v_r, c.p_o, c.v_o + bounds.v * u, c.a_o + bounds.a * u,
                                             kTr, kAs, kC, 0.05);
    expectSameTerms(robust, corner);
    EXPECT_TRUE(robust.psi_bound.isApprox(Eigen::Vector4d(0.0, 0.0, bounds.v, bounds.a)));
    EXPECT_LE(robust.h, barrierTerms(c.p_r, c.v_r, c.p_o, c.v_o, c.a_o, kTr, kAs, kC, 0.05).h);
  }
}

// With a radial motion the position bound only changes the distance: it is
// the same as inflating the obstacle, i.e. the delta_H of the paper.
TEST(CbfPerceptionBounds, RadialPositionBoundInflatesTheRadius)
{
  const Eigen::Vector3d p_r(0.2, 0.4, 0.9), p_o(0.9, 0.1, 0.6);
  const Eigen::Vector3d u = (p_r - p_o).normalized();
  const PerceptionBounds bounds{0.04, 0.0, 0.0};
  const BarrierTerms robust = barrierTerms(p_r, -0.3 * u, p_o, 0.5 * u, 1.2 * u, kTr, kAs, kC, 0.1, kAtol, bounds);
  const BarrierTerms inflated = barrierTerms(p_r, -0.3 * u, p_o, 0.5 * u, 1.2 * u, kTr, kAs, kC, 0.1 + bounds.p);
  expectSameTerms(robust, inflated);
  EXPECT_TRUE(robust.psi_bound.isApprox(Eigen::Vector4d(bounds.p, 0.0, 0.0, 0.0)));
}

// Deterministic samples in [-1, 1].
class Samples
{
public:
  double next()
  {
    state_ = state_ * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<double>(state_ >> 11) / static_cast<double>(1ULL << 53) * 2.0 - 1.0;
  }
  Eigen::Vector3d ball()
  {
    Eigen::Vector3d x;
    do { x << next(), next(), next(); } while (x.squaredNorm() > 1.0);
    return x;
  }
private:
  unsigned long long state_ = 42;
};

// The barrier at the corner is not above the barrier of any obstacle state
// in the box (up to second order in the position bound, which rotates u).
TEST(CbfPerceptionBounds, CornerIsTheMinimumOverTheBox)
{
  const PerceptionBounds bounds{0.01, 0.05, 0.3};
  Samples samples;
  for (const double radius : {0.0, 0.05}) {
    for (std::size_t index = 0; index < kBoundsCases.size(); ++index) {
      SCOPED_TRACE(::testing::Message() << "radius " << radius << " case " << index);
      const BoundsCase& c = kBoundsCases[index];
      const BarrierTerms robust = barrierTerms(c.p_r, c.v_r, c.p_o, c.v_o, c.a_o, kTr, kAs, kC, radius, kAtol, bounds);
      double h_lowest = std::numeric_limits<double>::infinity();
      for (int k = 0; k < 2000; ++k) {
        const double h_true = barrierTerms(c.p_r, c.v_r, c.p_o + bounds.p * samples.ball(),
                                           c.v_o + bounds.v * samples.ball(), c.a_o + bounds.a * samples.ball(),
                                           kTr, kAs, kC, radius).h;
        h_lowest = std::min(h_lowest, h_true);
      }
      EXPECT_GE(h_lowest, robust.h - 2e-4);
      // The box is not much larger than the reachable set.
      EXPECT_LT(h_lowest - robust.h, 0.05);
    }
  }
}

// rate_bound covers the error of dh/dt when the true obstacle state is
// elsewhere in the box: the robust barrier is h at the corner, so its rate is
// dh/dpsi(corner) . dpsi/dt(true state).
TEST(CbfPerceptionBounds, RateBoundCoversTheTrueRate)
{
  const PerceptionBounds bounds{0.02, 0.1, 0.5};
  const Eigen::Vector3d a_r(1.0, -2.0, 0.5);
  // psi of a robot point and an obstacle moving with constant accelerations.
  auto psi_at = [](const Eigen::Vector3d& p_r, const Eigen::Vector3d& v_r, const Eigen::Vector3d& a_r,
                   const Eigen::Vector3d& p_o, const Eigen::Vector3d& v_o, const Eigen::Vector3d& a_o,
                   double radius, double t) {
    const Eigen::Vector3d r = (p_r + v_r * t + 0.5 * a_r * t * t) - (p_o + v_o * t + 0.5 * a_o * t * t);
    const Eigen::Vector3d u = r.normalized();
    return Eigen::Vector4d(r.norm() - radius, u.dot(v_r + a_r * t), u.dot(v_o + a_o * t), u.dot(a_o));
  };
  Samples samples;
  // The receding fast robot (C_e = C) has a negative escape slope: there the
  // velocity bounds count too.
  struct Scenario { BoundsCase c; double radius; };
  const std::array<Scenario, 4> scenarios = {{
    {kBoundsCases[0], 0.05}, {kBoundsCases[1], 0.0}, {kBoundsCases[2], 0.05},
    {{{0.3, 0.0, 0.5}, {1.2, 0.0, 0.0}, {0.2, 0.0, 0.5}, {0.6, 0.0, 0.0}, {0.0, 0.0, 0.0}}, 0.03},
  }};
  for (std::size_t index = 0; index < scenarios.size(); ++index) {
    SCOPED_TRACE(::testing::Message() << "scenario " << index);
    const BoundsCase& c = scenarios[index].c;
    const double radius = scenarios[index].radius;
    const BarrierTerms corner = barrierTerms(c.p_r, c.v_r, c.p_o, c.v_o, c.a_o, kTr, kAs, kC, radius, kAtol, bounds);
    const double rate_corner = corner.lie_f + corner.lie_g.dot(a_r);
    if (index == 3) {
      EXPECT_LT(corner.dh_dpsi(0), 0.0);
      EXPECT_GT(corner.rate_bound, 0.0);
    }
    const Eigen::Vector3d u = (c.p_r - c.p_o).normalized();
    for (int k = 0; k < 500; ++k) {
      // True states that keep u: errors along u, the position one included.
      const Eigen::Vector3d p_o = c.p_o + bounds.p * samples.next() * u;
      const Eigen::Vector3d v_o = c.v_o + bounds.v * samples.next() * u;
      const Eigen::Vector3d a_o = c.a_o + bounds.a * samples.next() * u;
      constexpr double dt = 1e-6;
      const Eigen::Vector4d psi_dot = (psi_at(c.p_r, c.v_r, a_r, p_o, v_o, a_o, radius, dt) -
                                       psi_at(c.p_r, c.v_r, a_r, p_o, v_o, a_o, radius, -dt)) / (2.0 * dt);
      const double rate_true = corner.dh_dpsi.dot(psi_dot);
      EXPECT_GE(rate_true, rate_corner - corner.rate_bound - 1e-6);
    }
    // Only the position error, with the true velocity and acceleration at the
    // corner: the velocity errors above are favourable and would hide it.
    for (const double s : {-1.0, -0.5, 0.5, 1.0}) {
      const Eigen::Vector3d p_o = c.p_o + bounds.p * s * u;
      const Eigen::Vector3d v_o = c.v_o + corner.psi_bound(2) * u;
      const Eigen::Vector3d a_o = c.a_o + corner.psi_bound(3) * u;
      const Eigen::Vector3d v_r = c.v_r - corner.psi_bound(1) * u;
      constexpr double dt = 1e-6;
      const Eigen::Vector4d psi_dot = (psi_at(c.p_r, v_r, a_r, p_o, v_o, a_o, radius, dt) -
                                       psi_at(c.p_r, v_r, a_r, p_o, v_o, a_o, radius, -dt)) / (2.0 * dt);
      EXPECT_GE(corner.dh_dpsi.dot(psi_dot), rate_corner - corner.rate_bound - 1e-6) << "position error " << s;
    }
  }
  // No bounds, no rate bound.
  const BoundsCase& c = kBoundsCases[0];
  EXPECT_EQ(barrierTerms(c.p_r, c.v_r, c.p_o, c.v_o, c.a_o, kTr, kAs, kC).rate_bound, 0.0);
}

TEST(CbfPerceptionMargin, MatchesEquation28)
{
  EXPECT_NEAR(perceptionMargin(3.291, 0.015, 0.072, 0.196, 0.3),
              3.291 * (0.015 + 0.072 * 0.3 + 0.5 * 0.196 * 0.09), 1e-15);
}

}  // namespace
}  // namespace thor::math::cbf
