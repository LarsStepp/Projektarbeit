#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace neutral_axis_controllers {

// Eigen hat kein Vector6d -> eigene Aliase
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Wrench = Vector6d;  // [fx fy fz tx ty tz], im Basis-Frame

// Kartesische Pose. Orientierung als Quaternion, weil ein 6D-Vektor
// [pos; euler] bei Fehlerberechnung/Integration Probleme macht.
struct Pose {
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

}  // namespace neutral_axis_controllers
