#pragma once

#include "sensors/LidarTypes.h"

#include <optional>

namespace dmap {

class ILidarSensor {
 public:
  virtual ~ILidarSensor() = default;

  virtual LidarScanResult scan(std::optional<AngleDeg> xy_offset = std::nullopt,
                               std::optional<AngleDeg> height_angle = std::nullopt) = 0;
};

}  // namespace dmap
