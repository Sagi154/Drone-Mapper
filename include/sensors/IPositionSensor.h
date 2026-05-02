#pragma once

#include "common/Types.h"

namespace dmap {

struct DronePosition {
  LengthCm x{};
  LengthCm y{};
  LengthCm height{};
  AngleDeg xy_angle{};
};

class IPositionSensor {
 public:
  virtual ~IPositionSensor() = default;

  virtual DronePosition getPosition() const = 0;
};

}  // namespace dmap
