#pragma once
 
#include "types.hpp"
 
namespace neutral_axis_controllers {
 
// Multisine-Anregung: pro Achse eine Sinuskraft mit eigener (inkommensurabler) Frequenz.
//   F_explore_i(t) = ramp(t) * A_i * sin(2*pi*f_i*t + phi_i)
// Die Frequenzen sind so gewählt, dass sich die Achsen nicht synchronisieren,
// und der Ramp (smoothstep) verhindert einen Kraftsprung beim Start.
class ExplorationGenerator {
 public:
  // amplitude: [N N N Nm Nm Nm], freq_hz: je Achse, ramp_time: Einblendzeit in s
  void configure(const Vector6d& amplitude, const Vector6d& freq_hz, double ramp_time);
 
  // t = Zeit seit Start der Exploration (NICHT absolute ROS-Zeit)
  Wrench excite(double t) const;
 
 private:
  Vector6d A_ = Vector6d::Zero();
  Vector6d omega_ = Vector6d::Zero();  // rad/s
  Vector6d phi_ = Vector6d::Zero();    // feste Phasenversätze
  double ramp_time_ = 5.0;
};
 
}  // namespace neutral_axis_controllers
