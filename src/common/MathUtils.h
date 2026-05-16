// MathUtils.h
// Small numeric helpers shared across simulation, config, and mapping layers.
// All functions are inline so no accompanying .cpp is needed.

#pragma once

#include "common/Point3D.h"
#include "sensors/IPositionSensor.h"
#include "sensors/LidarTypes.h"

#include <mp-units/systems/si/unit_symbols.h>
#include <cmath>
#include <cstdint>
#include <numbers>

namespace dmap {

// Converts degrees to radians.
inline double toRad(double deg) {
  return deg * (std::numbers::pi / 180.0);
}

// Converts a decimal-places count to the corresponding step size.
// e.g. places=2 → 0.01, places=0 → 1.0, places=-1 → 10.0.
// Used to turn MissionConfig::xy_decimal_places / height_decimal_places
// into the grid cell size in cm.
inline double decimalPlacesToStep(std::int32_t places) {
  return std::pow(10.0, -static_cast<double>(places));
}

/// World-space point on the beam described by `hit`, using the same azimuth/elevation
/// convention as LidarMock (azimuth_xy = atan2(wy,wx), elevation = atan2(wz, hypot(wx,wy))).
/// @param origin Drone centre in cm (scan origin).
/// @param hit Beam direction in degrees and range in cm; distance 0 returns `origin` (too-close hits).
///            Do not call with the no-return sentinel (-1).
/// @return Hit position in cm.
inline Point3D hitToWorldPoint(const DronePosition& origin, const LidarHit& hit) {
  namespace su = mp_units::si::unit_symbols;
  const double az = toRad(hit.azimuth_xy.numerical_value_in(su::deg));
  const double el = toRad(hit.elevation.numerical_value_in(su::deg));
  const double d  = hit.distance.numerical_value_in(su::cm);

  const double ux = std::cos(el) * std::cos(az);
  const double uy = std::cos(el) * std::sin(az);
  const double uz = std::sin(el);

  const double ox = origin.x.numerical_value_in(su::cm);
  const double oy = origin.y.numerical_value_in(su::cm);
  const double oz = origin.height.numerical_value_in(su::cm);

  return {(ox + d * ux) * su::cm, (oy + d * uy) * su::cm, (oz + d * uz) * su::cm};
}

}  // namespace dmap
