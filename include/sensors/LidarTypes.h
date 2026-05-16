#pragma once

#include "common/Types.h"

#include <mp-units/systems/si/unit_symbols.h>
#include <vector>

namespace dmap {

/// Distance sentinel (cm) when a beam finds no occupied cell within Z_max.
inline constexpr double kLidarNoReturnDistanceCm = -1.0;

struct LidarHit {
  AngleDeg azimuth_xy{};
  AngleDeg elevation{};
  /// Range to first hit in cm; 0 = closer than Z_min (unmeasurable); -1 = no return in range.
  LengthCm distance{};
};

using LidarScanResult = std::vector<LidarHit>;

/// @return True when `hit.distance` is the no-return sentinel (-1 cm).
inline bool lidarHitIsMiss(const LidarHit& hit) {
  namespace su = mp_units::si::unit_symbols;
  return hit.distance.numerical_value_in(su::cm) < 0.0;
}

/// No-return distance value for beams that miss within Z_max.
inline LengthCm lidarNoReturnDistance() {
  namespace su = mp_units::si::unit_symbols;
  return kLidarNoReturnDistanceCm * su::cm;
}

}  // namespace dmap
