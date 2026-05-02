#pragma once

#include "common/Types.h"

namespace dmap {

enum class TurnDirection { Left, Right };

class IMovementDriver {
 public:
  virtual ~IMovementDriver() = default;

  virtual void rotate(TurnDirection direction, AngleDeg angle) = 0;
  virtual void advance(LengthCm distance) = 0;
  virtual void elevate(LengthCm distance) = 0;
};

}  // namespace dmap
