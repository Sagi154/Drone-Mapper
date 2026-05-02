#pragma once

#include "common/Types.h"

#include <vector>

namespace dmap {

struct LidarHit {
  AngleDeg azimuth_xy{};
  AngleDeg elevation{};
  LengthCm distance{};
};

using LidarScanResult = std::vector<LidarHit>;

}  // namespace dmap
