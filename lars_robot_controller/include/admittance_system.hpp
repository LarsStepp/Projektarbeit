#pragma once

#include "types.hpp"

namespace neutral_axis_controllers {

// Virtuelles Masse-Dämpfer-System mit Referenzgeschwindigkeit (keine Feder!):
//   M * ddx_d + D * (dx_d - v_ref) = F_explore + F_interaction
// D (6x6, anisotrop) und v_ref kommen aus der Führung und werden vor jedem Update gesetzt.
// F_interaction = Kraft, die das Knie/Bein auf den Roboter ausübt (gravitationskompensiert).
// Integration implizit (Backward Euler) -> stabil auch bei hoher Dämpfung/großem dt.
class AdmittanceSystem {
 public:
  void configure(const Vector6d& mass_diag, double max_lin_vel, double max_ang_vel);

  void setGuidance(const Matrix6d& D, const Vector6d& v_ref);

  void initialize(const Pose& x0);

  void update(const Wrench& F_explore, const Wrench& F_interaction, double dt);

  const Pose& pose() const { return x_d_; }
  const Vector6d& velocity() const { return dx_d_; }

 private:
  Vector6d M_diag_ = Vector6d::Ones();
  Matrix6d D_ = Matrix6d::Identity();
  Vector6d v_ref_ = Vector6d::Zero();
  double max_lin_vel_ = 0.1;  // m/s
  double max_ang_vel_ = 0.3;  // rad/s

  Pose x_d_;
  Vector6d dx_d_ = Vector6d::Zero();
  Vector6d ddx_d_ = Vector6d::Zero();
};

}  // namespace neutral_axis_controllers
