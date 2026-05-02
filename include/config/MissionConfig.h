#pragma once

#include "common/Types.h"
#include "common/Point3D.h"

#include <cstdint>
#include <vector>

namespace dmap {

struct MissionConfig {
  LengthCm min_x{};
  LengthCm max_x{};
  LengthCm min_y{};
  LengthCm max_y{};
  LengthCm min_height{};
  LengthCm max_height{};
  std::int32_t xy_decimal_places{2};
  std::int32_t height_decimal_places{2};
  std::vector<Point3D> recharge_positions;
};

}  // namespace dmap
