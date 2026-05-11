// MathUtils.h
// Small numeric helpers shared across the simulation and config layers.
// All functions are inline so no accompanying .cpp is needed.

#pragma once

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

}  // namespace dmap
