#include <thor_math/cbf.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace thor
{
namespace math
{
namespace cbf
{

  double minimumDistance ( double d, double v_r, double v_o, double a_o,
                           double T_r, double a_s, double atol,
                           Eigen::Vector4d& jacobian )
  {
    if (!(a_s > 0.0))
      throw std::invalid_argument("a_s must be positive");

    // Phases: [t0, t2] reaction, [t2, t4] braking until the robot stops.
    const double t0 = 0.0;
    const double t2 = T_r;
    const double t_dec = std::max(0.0, -v_r / a_s);
    const double t4 = t2 + t_dec;
    const double m = a_o - a_s;
    const double v_diff = v_r - v_o;

    // Stationary points of the distance inside each phase.
    bool has_t1 = false;
    double t1 = 0.0;
    if (std::abs(a_o) > atol)
    {
      const double t = v_diff / a_o;
      if (t0 + atol < t && t < t2 - atol)
      {
        t1 = t;
        has_t1 = true;
      }
    }
    bool has_t3 = false;
    double t3 = 0.0;
    if (std::abs(m) > atol && (t4 - t2) > atol)
    {
      const double t = (v_diff - a_s * t2) / m;
      if (t2 + atol < t && t < t4 - atol)
      {
        t3 = t;
        has_t3 = true;
      }
    }

    std::array<double, 4> candidates;
    std::size_t n = 0;
    auto add = [&](double t)
    {
      for (std::size_t k = 0; k < n; ++k)
        if (std::abs(t - candidates[k]) <= atol)
          return;
      candidates[n++] = t;
    };
    if (v_r < 0.0)
    {
      add(t0);
      add(t4);
      if (has_t1) add(t1);
      if (has_t3) add(t3);
    }
    else
    {
      add(t0);
      add(t2);
      if (has_t1) add(t1);
    }

    auto distance_at = [&](double t)
    {
      double value = d + v_diff * t - 0.5 * a_o * t * t;
      if (t > t2 + atol)
        value += 0.5 * a_s * (t - t2) * (t - t2);
      return value;
    };
    double t_star = candidates[0];
    double d_min = distance_at(t_star);
    for (std::size_t k = 1; k < n; ++k)
    {
      const double value = distance_at(candidates[k]);
      if (value < d_min)
      {
        d_min = value;
        t_star = candidates[k];
      }
    }

    // Envelope theorem, plus the dependence of t4 on v_r.
    jacobian << 1.0, t_star, -t_star, -0.5 * t_star * t_star;
    const bool at_other = std::abs(t_star - t0) <= atol ||
                          (has_t1 && std::abs(t_star - t1) <= atol) ||
                          std::abs(t_star - t2) <= atol ||
                          (has_t3 && std::abs(t_star - t3) <= atol);
    if (!at_other && std::abs(t_star - t4) <= atol && t_dec > atol)
    {
      const double dotd_t4 = v_diff - a_s * t2 - m * t4;
      jacobian(1) -= dotd_t4 / a_s;
    }
    return d_min;
  }

  double barrier ( double d, double v_r, double v_o, double a_o,
                   double T_r, double a_s, double C, double atol,
                   Eigen::Vector4d& jacobian, double C_escape )
  {
    if (!(C_escape >= 0.0))
      throw std::invalid_argument("C_escape must not be negative");
    Eigen::Vector4d dist_jac;
    const double d_min = minimumDistance(d, v_r, v_o, a_o, T_r, a_s, atol, dist_jac);

    double h = d_min - C;
    jacobian = dist_jac;
    if (d_min < C)
    {
      // Inside the margin: a receding robot (v_r > 0) increases h. The term
      // starts at d_min = C; C_e only sets its slope.
      const double C_e = C_escape > 0.0 ? C_escape : C;
      const double factor = (T_r / C_e) * v_r;
      h += ((C - d_min) / C_e) * T_r * v_r;
      jacobian -= factor * dist_jac;
      jacobian(1) += (C - d_min) * T_r / C_e;
    }
    return h;
  }

  namespace
  {
    struct Parameters
    {
      double T_r, a_s, C, C_escape, atol;
    };

    // barrierTerms() with the direction fallback_u used when the centers coincide.
    BarrierTerms pointTerms ( const Eigen::Vector3d& p_r, const Eigen::Vector3d& v_r_measured,
                              const Eigen::Vector3d& p_o, const Eigen::Vector3d& v_o_measured,
                              const Eigen::Vector3d& a_o_measured,
                              const Parameters& par, double radius,
                              const PerceptionBounds& bounds, const Eigen::Vector3d& fallback_u )
    {
      const double T_r = par.T_r, a_s = par.a_s, C = par.C, atol = par.atol;
      BarrierTerms out;
      const Eigen::Vector3d r = p_r - p_o;
      double center_distance = r.norm();
      Eigen::Vector3d u;
      if (center_distance <= atol)
      {
        u = fallback_u;
        center_distance = std::max(center_distance, atol);
      }
      else
      {
        u = r / center_distance;
      }

      // Box around the measured psi: an obstacle position error moves the
      // distance and rotates u, which changes every projection.
      const double d_psi = std::max(center_distance, 1e-3);
      auto normal = [&](const Eigen::Vector3d& w) { return (w - u * u.dot(w)).norm(); };
      out.psi_bound << bounds.p,
                       bounds.p * normal(v_r_measured) / d_psi,
                       bounds.v + bounds.p * normal(v_o_measured) / d_psi,
                       bounds.a + bounds.p * normal(a_o_measured) / d_psi;
      // d_min is monotone in every component of psi: its minimum over the box
      // is at this corner. Moving the velocities along u changes only their
      // projections.
      const Eigen::Vector4d shift(-out.psi_bound(0), -out.psi_bound(1), out.psi_bound(2), out.psi_bound(3));
      const Eigen::Vector3d v_r = v_r_measured + shift(1) * u;
      const Eigen::Vector3d v_o = v_o_measured + shift(2) * u;
      const Eigen::Vector3d a_o = a_o_measured + shift(3) * u;

      // The barrier sees the distance between the sphere surfaces; u and its
      // rotation rate depend on the centers only.
      out.d   = center_distance - radius + shift(0);
      out.v_r = u.dot(v_r);
      out.v_o = u.dot(v_o);
      out.a_o = u.dot(a_o);

      Eigen::Vector4d dh_dpsi;
      out.h = barrier(out.d, out.v_r, out.v_o, out.a_o, T_r, a_s, C, atol, dh_dpsi, par.C_escape);

      // Time derivative of psi along the drift: the projections change because
      // u rotates, with du/dt = (I - u u^T)(v_r - v_o)/|p_r - p_o|.
      const Eigen::Vector3d v_diff = v_r - v_o;
      Eigen::Vector4d psi_f;
      psi_f(0) = u.dot(v_diff);
      psi_f(1) = ((v_r - u * out.v_r) / d_psi).dot(v_diff);
      psi_f(2) = ((v_o - u * out.v_o) / d_psi).dot(v_diff) + out.a_o;
      psi_f(3) = ((a_o - u * out.a_o) / d_psi).dot(v_diff);

      out.lie_f = dh_dpsi.dot(psi_f);

      // Error of dh/dt = dh/dpsi(corner) . dpsi/dt when the true psi is
      // elsewhere in the box (frozen errors). The control part (dh/dv_r) u.a_r
      // does not change, since u does not. From the corner the true velocities
      // and acceleration differ by [0, 2 b] in the favourable direction, and
      // the rotation terms (~ 1/|p_r - p_o|) by +-b_d through the distance.
      const double rotation = dh_dpsi(1) * psi_f(1) + dh_dpsi(2) * (psi_f(2) - out.a_o) + dh_dpsi(3) * psi_f(3);
      out.rate_bound = std::abs(rotation) / d_psi * out.psi_bound(0)
                     + 2.0 * std::max(0.0, -dh_dpsi(0)) * (out.psi_bound(1) + out.psi_bound(2))
                     + 2.0 * std::max(0.0, dh_dpsi(2)) * out.psi_bound(3);
      // Only v_r depends on the robot point acceleration: d(v_r)/d(acc) = u.
      out.lie_g = dh_dpsi(1) * u;
      out.h_chi = dh_dpsi(0);
      out.dh_dpsi = dh_dpsi;
      // Gradient with respect to the robot center position, obstacle fixed:
      // d(d)/dp_r = u, d(u.w)/dp_r = (I - u u^T) w / |p_r - p_o|.
      out.grad_p = dh_dpsi(0) * u
                 + dh_dpsi(1) * (v_r - u * out.v_r) / d_psi
                 + dh_dpsi(2) * (v_o - u * out.v_o) / d_psi
                 + dh_dpsi(3) * (a_o - u * out.a_o) / d_psi;
      return out;
    }

  }  // namespace

  BarrierTerms barrierTerms ( const Eigen::Vector3d& p_r, const Eigen::Vector3d& v_r,
                              const Eigen::Vector3d& p_o, const Eigen::Vector3d& v_o,
                              const Eigen::Vector3d& a_o,
                              double T_r, double a_s, double C, double radius, double atol,
                              const PerceptionBounds& bounds, double C_escape )
  {
    return pointTerms(p_r, v_r, p_o, v_o, a_o, {T_r, a_s, C, C_escape, atol}, radius, bounds,
                      Eigen::Vector3d::UnitX());
  }

  BarrierTerms capsuleBarrierTerms ( const Eigen::Vector3d& p_r, const Eigen::Vector3d& v_r,
                                     const Eigen::Vector3d& p_o, const Eigen::Vector3d& half_axis,
                                     const Eigen::Vector3d& v_o, const Eigen::Vector3d& a_o,
                                     double T_r, double a_s, double C, double radius, double atol,
                                     const PerceptionBounds& bounds, double C_escape )
  {
    const double half_length = half_axis.norm();
    if (half_length <= atol)
      return barrierTerms(p_r, v_r, p_o, v_o, a_o, T_r, a_s, C, radius, atol, bounds, C_escape);
    const Eigen::Vector3d axis = half_axis / half_length;

    // Beyond the ends the closest point of the segment is an end, which moves
    // with the obstacle: a sphere.
    const double s = axis.dot(p_r - p_o);
    if (s >= half_length)
      return barrierTerms(p_r, v_r, p_o + half_axis, v_o, a_o, T_r, a_s, C, radius, atol, bounds, C_escape);
    if (s <= -half_length)
      return barrierTerms(p_r, v_r, p_o - half_axis, v_o, a_o, T_r, a_s, C, radius, atol, bounds, C_escape);

    // Along the side the closest point slides on the axis and the motion along
    // it does not change the distance: the pair is a pair of points in the
    // plane orthogonal to the axis. The axis is fixed, so the projections of
    // the derivatives are the derivatives of the projections, and the
    // gradients, which lie in the plane, are also those of the 3D state.
    const Eigen::Matrix3d P = Eigen::Matrix3d::Identity() - axis * axis.transpose();
    // If the robot center is on the axis, any direction in the plane will do.
    Eigen::Index least_aligned;
    axis.cwiseAbs().minCoeff(&least_aligned);
    const Eigen::Vector3d fallback_u = (P * Eigen::Vector3d::Unit(least_aligned)).normalized();
    return pointTerms(P * p_r, P * v_r, P * p_o, P * v_o, P * a_o, {T_r, a_s, C, C_escape, atol}, radius, bounds,
                      fallback_u);
  }

  double perceptionMargin ( double beta, double sigma_p, double sigma_v, double sigma_a, double T_f_bar )
  {
    return beta * (sigma_p + sigma_v * T_f_bar + 0.5 * sigma_a * T_f_bar * T_f_bar);
  }

}  // namespace cbf
}  // namespace math
}  // namespace thor
