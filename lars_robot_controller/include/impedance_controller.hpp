#pragma once
 
#include "types.hpp"
 
namespace neutral_axis_controllers {
 
// Kartesischer Impedanzregler (innere Schleife):
//   F_cmd = K * e(x_d, x) + D * (dx_d - dx)
class CartesianImpedanceController {
 public:
  void configure(const Matrix6d& K, const Matrix6d& D);
 
  // Rückgabe: Wrench im Basis-Frame am Endeffektor
  Wrench compute_wrench(const Pose& x_d, const Vector6d& dx_d,
                        const Pose& x, const Vector6d& dx) const;
 
 private:
  Matrix6d K_ = Matrix6d::Zero();
  Matrix6d D_ = Matrix6d::Zero();
};
 
}  // namespace neutral_axis_controllers
