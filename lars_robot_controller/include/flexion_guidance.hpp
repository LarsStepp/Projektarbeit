#pragma once

#include "types.hpp"

namespace neutral_axis_controllers {

struct GuidanceOutput {
  Vector6d v_ref = Vector6d::Zero();       // Referenz-Twist (Basis-Frame)
  Matrix6d D = Matrix6d::Identity();       // anisotrope Dämpfung (Basis-Frame)
  Eigen::Vector3d t = Eigen::Vector3d::Zero();  // aktuelle Tangente (Einheitsvektor)
};

// Weiche Führung um eine Knieachse (Kniezentrum c, Achse n, beide im Basis-Frame, fest,
// da Oberschenkel fixiert). Wird in JEDEM Admittanz-Schritt aus der aktuellen EE-Position
// neu berechnet:
//   r_perp = (p - c) - n (n.(p - c))
//   t      = dir * (n x r_perp) / |n x r_perp|
//   v_lin  = dir * omega * (n x r_perp)     w_ref = dir * omega * n
//   D_lin  = d_across * I + (d_along - d_across) * t t^T
//   D_rot  = d_rot_across * I + (d_rot_about - d_rot_across) * n n^T
// Die Bias-Kraft entlang t ist in der Größenordnung D_along * |v_lin| (bekannt, klein halten).
class FlexionGuidance {
 public:
  void configure(const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
                 double direction,          // +1 / -1: Drehsinn um n, der "Extension" entspricht
                 double omega,              // rad/s Knie-Winkelgeschwindigkeit der Führung
                 double d_lin_along, double d_lin_across,
                 double d_rot_about, double d_rot_across,
                 double ramp_time);

  // Startpose der Exploration merken (Referenz für den überstrichenen Winkel)
  void start(const Eigen::Vector3d& p0);

  GuidanceOutput compute(const Eigen::Vector3d& p, double t_since_start) const;

  // überstrichener Winkel um n seit start(), positiv in Führungsrichtung (rad)
  double sweepAngle(const Eigen::Vector3d& p) const;

 private:
  Eigen::Vector3d perp(const Eigen::Vector3d& p) const;

  Eigen::Vector3d c_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d n_ = Eigen::Vector3d::UnitY();
  Eigen::Vector3d r0_perp_ = Eigen::Vector3d::UnitX();
  double dir_ = 1.0;
  double omega_ = 0.05;
  double d_lin_along_ = 50.0, d_lin_across_ = 150.0;
  double d_rot_about_ = 1.0, d_rot_across_ = 5.0;
  double ramp_time_ = 5.0;
};

}  // namespace neutral_axis_controllers
