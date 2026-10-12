#include "impedance_controller.hpp"

namespace neutral_axis_controllers {

void CartesianImpedanceController::configure(const Matrix6d& K, const Matrix6d& D) {
  K_ = K;
  D_ = D;
}

Wrench CartesianImpedanceController::compute_wrench(const Pose& x_d, const Vector6d& dx_d,
                                                    const Pose& x, const Vector6d& dx) const {
  Vector6d e;
  e.head<3>() = x_d.p - x.p;

  // Orientierungsfehler als Rotationsvektor (kürzester Weg), im Basis-Frame
  Eigen::Quaterniond dq = x_d.q * x.q.inverse();
  if (dq.w() < 0.0) {
    dq.coeffs() *= -1.0;
  }
  const Eigen::AngleAxisd aa(dq);
  e.tail<3>() = aa.angle() * aa.axis();

  return K_ * e + D_ * (dx_d - dx);
}

}  // namespace neutral_axis_controllers