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
  LengthCm min_passable_width{};
  LengthCm min_passable_length{};
  LengthCm min_passable_height{};
  AngleDeg max_rotate_per_command{};
  LengthCm max_advance_per_command{};
  LengthCm max_elevate_per_command{};
  LidarConfig lidar{};
};

}  // namespace dmap
