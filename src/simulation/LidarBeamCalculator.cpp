#include "simulation/LidarBeamCalculator.h"

#include <mp-units/systems/si/unit_symbols.h>
#include <cmath>
#include <numbers>

namespace dmap {

namespace su = mp_units::si::unit_symbols;

LidarBeamCalculator::LidarBeamCalculator(LidarConfig cfg) : cfg_(cfg) {}

// Returns unit-vector beam directions in the lidar's LOCAL frame:
//   +x = forward (center beam)
//   +y = right
//   +z = up
// LidarMock::scan rotates these into world space using the drone's heading.
std::vector<UnitRay3> LidarBeamCalculator::unitDirectionsForScan() const {
  const double z_min_cm = cfg_.z_min.numerical_value_in(su::cm);
  const double d_cm     = cfg_.circle_spacing_d.numerical_value_in(su::cm);

  std::vector<UnitRay3> rays;

  for (int n = 0; n < cfg_.fov_circles; ++n) {
    // Elevation angle from the forward axis for this circle.
    // Circle 0 is always exactly forward (theta = 0).
    // For outer circles, theta = atan(n*D / Z_min).
    // If Z_min is zero (degenerate config), outer beams go perpendicular (pi/2).
    const double theta = (n == 0)        ? 0.0
                       : (z_min_cm > 0.0) ? std::atan2(static_cast<double>(n) * d_cm, z_min_cm)
                                          : std::numbers::pi / 2.0;

    // Circle n has 4^n beams = 2^(2n), computed as a left-shift.
    const long long num_beams = 1LL << (2 * n);

    for (long long k = 0; k < num_beams; ++k) {
      // Azimuth angle around the forward axis, evenly distributed.
      const double phi = static_cast<double>(k) *
                         (2.0 * std::numbers::pi / static_cast<double>(num_beams));

      // Spherical → Cartesian, polar axis = +x.
      rays.push_back({
          std::cos(theta),                  // x: forward component
          std::sin(theta) * std::cos(phi),  // y: right component
          std::sin(theta) * std::sin(phi),  // z: up component
      });
    }
  }

  return rays;
}

}  // namespace dmap
