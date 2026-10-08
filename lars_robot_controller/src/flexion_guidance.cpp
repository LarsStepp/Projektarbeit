#include "flexion_guidance.hpp"

#include <algorithm>
#include <cmath>

namespace neutral_axis_controllers {

namespace {
constexpr double kMinRadius = 1e-3;  // m, darunter ist die Tangente nicht definiert
}

void FlexionGuidance::configure(const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
                                double direction, double omega,
                                double d_lin_along, double d_lin_across,
                                double d_rot_about, double d_rot_across, double ramp_time) {
  c_ = center;
  n_ = axis.normalized();
  dir_ = direction;
  omega_ = omega;
  d_lin_along_ = d_lin_along;
  d_lin_across_ = d_lin_across;
  d_rot_about_ = d_rot_about;
  d_rot_across_ = d_rot_across;
  ramp_time_ = std::max(ramp_time, 1e-3);
}

Eigen::Vector3d FlexionGuidance::perp(const Eigen::Vector3d& p) const {
  const Eigen::Vector3d r = p - c_;
  return r - n_ * n_.dot(r);
}

void FlexionGuidance::start(const Eigen::Vector3d& p0) {
  r0_perp_ = perp(p0);
}

GuidanceOutput FlexionGuidance::compute(const Eigen::Vector3d& p, double t_since_start) const {
  GuidanceOutput out;

  // Rotatorischer Dämpfungsanteil (Achse fest)
  const Eigen::Matrix3d nnT = n_ * n_.transpose();
  out.D.setZero();
  out.D.bottomRightCorner<3, 3>() =
      d_rot_across_ * Eigen::Matrix3d::Identity() + (d_rot_about_ - d_rot_across_) * nnT;
  out.D.topLeftCorner<3, 3>() = d_lin_across_ * Eigen::Matrix3d::Identity();

  const Eigen::Vector3d r_perp = perp(p);
  if (r_perp.norm() < kMinRadius) {
    return out;  // degeneriert: isotrop gedämpft, keine Referenzgeschwindigkeit
  }

  const double s = std::clamp(t_since_start / ramp_time_, 0.0, 1.0);
  const double ramp = s * s * (3.0 - 2.0 * s);  // smoothstep

  const Eigen::Vector3d tang = n_.cross(r_perp);  // |tang| = |r_perp|
  out.t = dir_ * tang.normalized();

  out.D.topLeftCorner<3, 3>() += (d_lin_along_ - d_lin_across_) * out.t * out.t.transpose();

  out.v_ref.head<3>() = ramp * dir_ * omega_ * tang;
  out.v_ref.tail<3>() = ramp * dir_ * omega_ * n_;
  return out;
}

double FlexionGuidance::sweepAngle(const Eigen::Vector3d& p) const {
  const Eigen::Vector3d r = perp(p);
  if (r.norm() < kMinRadius || r0_perp_.norm() < kMinRadius) return 0.0;
  return dir_ * std::atan2(n_.dot(r0_perp_.cross(r)), r0_perp_.dot(r));
}

}  // namespace neutral_axis_controllers
