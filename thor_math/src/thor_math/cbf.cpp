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
                   Eigen::Vector4d& jacobian )
  {
    Eigen::Vector4d dist_jac;
    const double d_min = minimumDistance(d, v_r, v_o, a_o, T_r, a_s, atol, dist_jac);

    double h = d_min - C;
    jacobian = dist_jac;
    if (d_min < C)
    {
      // Inside the margin: a receding robot (v_r > 0) increases h.
      const double factor = (T_r / C) * v_r;
      h += ((C - d_min) / C) * T_r * v_r;
      jacobian -= factor * dist_jac;
      jacobian(1) += (C - d_min) * T_r / C;
    }
    return h;
  }

  BarrierTerms barrierTerms ( const Eigen::Vector3d& p_r, const Eigen::Vector3d& v_r,
                              const Eigen::Vector3d& p_o, const Eigen::Vector3d& v_o,
                              const Eigen::Vector3d& a_o,
                              double T_r, double a_s, double C, double radius, double atol )
  {
    BarrierTerms out;
    const Eigen::Vector3d r = p_r - p_o;
    double center_distance = r.norm();
    Eigen::Vector3d u;
    if (center_distance <= atol)
    {
      u = Eigen::Vector3d::UnitX();
      center_distance = std::max(center_distance, atol);
    }
    else
    {
      u = r / center_distance;
    }

    // The barrier sees the distance between the sphere surfaces; u and its
    // rotation rate depend on the centers only.
    out.d   = center_distance - radius;
    out.v_r = u.dot(v_r);
    out.v_o = u.dot(v_o);
    out.a_o = u.dot(a_o);

    Eigen::Vector4d dh_dpsi;
    out.h = barrier(out.d, out.v_r, out.v_o, out.a_o, T_r, a_s, C, atol, dh_dpsi);

    // Time derivative of psi along the drift: the projections change because
    // u rotates, with du/dt = (I - u u^T)(v_r - v_o)/|p_r - p_o|.
    const double d_psi = std::max(center_distance, 1e-3);
    const Eigen::Vector3d v_diff = v_r - v_o;
    Eigen::Vector4d psi_f;
    psi_f(0) = u.dot(v_diff);
    psi_f(1) = ((v_r - u * out.v_r) / d_psi).dot(v_diff);
    psi_f(2) = ((v_o - u * out.v_o) / d_psi).dot(v_diff) + out.a_o;
    psi_f(3) = ((a_o - u * out.a_o) / d_psi).dot(v_diff);

    out.lie_f = dh_dpsi.dot(psi_f);
    // Only v_r depends on the robot point acceleration: d(v_r)/d(acc) = u.
    out.lie_g = dh_dpsi(1) * u;
    out.h_chi = dh_dpsi(0);
    // Gradient with respect to the robot center position, obstacle fixed:
    // d(d)/dp_r = u, d(u.w)/dp_r = (I - u u^T) w / |p_r - p_o|.
    out.grad_p = dh_dpsi(0) * u
               + dh_dpsi(1) * (v_r - u * out.v_r) / d_psi
               + dh_dpsi(2) * (v_o - u * out.v_o) / d_psi
               + dh_dpsi(3) * (a_o - u * out.a_o) / d_psi;
    return out;
  }

  double perceptionMargin ( double beta, double sigma_p, double sigma_v, double sigma_a, double T_f_bar )
  {
    return beta * (sigma_p + sigma_v * T_f_bar + 0.5 * sigma_a * T_f_bar * T_f_bar);
  }

}  // namespace cbf
}  // namespace math
}  // namespace thor
