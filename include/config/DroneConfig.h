#pragma once

#include "common/Types.h"

#include <cstdint>

namespace dmap {

struct LidarConfig {
  LengthCm z_min{};
  LengthCm z_max{};
  LengthCm circle_spacing_d{};
  std::int32_t fov_circles{1};
};

struct DroneConfig {
  /// Radius of the sphere used for collision detection (cm).
  /// The drone is modelled as a perfect ball of this radius so that
  /// its collision footprint is independent of heading — no oriented-box
  /// geometry is needed and rotation never changes the occupied cells.
  LengthCm min_passable_radius{};
  AngleDeg max_rotate_per_command{};
  LengthCm max_advance_per_command{};
  LengthCm max_elevate_per_command{};
  LidarConfig lidar{};
};

}  // namespace dmap
