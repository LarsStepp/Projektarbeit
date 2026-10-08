#include "admittance_system.hpp"

namespace neutral_axis_controllers {

void AdmittanceSystem::configure(const Vector6d& mass_diag, double max_lin_vel,
                                 double max_ang_vel) {
  M_diag_ = mass_diag;
  max_lin_vel_ = max_lin_vel;
  max_ang_vel_ = max_ang_vel;
}

void AdmittanceSystem::setGuidance(const Matrix6d& D, const Vector6d& v_ref) {
  D_ = D;
  v_ref_ = v_ref;
}

void AdmittanceSystem::initialize(const Pose& x0) {
  x_d_ = x0;
  dx_d_.setZero();
  ddx_d_.setZero();
}

void AdmittanceSystem::update(const Wrench& F_explore, const Wrench& F_interaction, double dt) {
  if (dt <= 0.0) return;

  // Vorzeichen: F_interaction ist die Kraft AUF den Roboter -> Roboter weicht in ihre Richtung aus.
  const Vector6d F = F_explore + F_interaction;

  // Implizit: (M + dt*D) dx_new = M dx + dt (F + D v_ref)
  const Matrix6d A = Matrix6d(M_diag_.asDiagonal()) + dt * D_;
  const Vector6d rhs = M_diag_.cwiseProduct(dx_d_) + dt * (F + D_ * v_ref_);
  const Vector6d dx_new = A.ldlt().solve(rhs);
  ddx_d_ = (dx_new - dx_d_) / dt;
  dx_d_ = dx_new;

  // Geschwindigkeitsbegrenzung (Patientensicherheit)
  const double v = dx_d_.head<3>().norm();
  if (v > max_lin_vel_) dx_d_.head<3>() *= (max_lin_vel_ / v);
  const double w = dx_d_.tail<3>().norm();
  if (w > max_ang_vel_) dx_d_.tail<3>() *= (max_ang_vel_ / w);

  // Position integrieren
  x_d_.p += dx_d_.head<3>() * dt;

  // Orientierung integrieren (Winkelgeschwindigkeit im Basis-Frame)
  const Eigen::Vector3d omega = dx_d_.tail<3>();
  const double angle = omega.norm() * dt;
  if (angle > 1e-12) {
    x_d_.q = Eigen::Quaterniond(Eigen::AngleAxisd(angle, omega.normalized())) * x_d_.q;
    x_d_.q.normalize();
  }
}

}  // namespace neutral_axis_controllers
