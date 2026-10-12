#include "exploration_generator.hpp"

#include <algorithm>
#include <cmath>

namespace neutral_axis_controllers {

void ExplorationGenerator::configure(const Vector6d& amplitude,
                                     const Vector6d& freq_hz,
                                     double ramp_time) {
  A_ = amplitude;
  omega_ = 2.0 * M_PI * freq_hz;
  ramp_time_ = std::max(ramp_time, 1e-3);
  // Feste, unterschiedliche Phasen, damit nicht alle Achsen gleichzeitig bei 0 starten
  phi_ << 0.0, 1.1, 2.3, 0.7, 1.9, 2.9;
}

Wrench ExplorationGenerator::excite(double t) const {
  const double s = std::clamp(t / ramp_time_, 0.0, 1.0);
  const double ramp = s * s * (3.0 - 2.0 * s);  // smoothstep

  Wrench F_explore;
  for (int i = 0; i < 6; ++i) {
    F_explore(i) = ramp * A_(i) * std::sin(omega_(i) * t + phi_(i));
  }
  return F_explore;
}

}  // namespace neutral_axis_controllers