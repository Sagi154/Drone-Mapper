#pragma once

#include "drivers/IMovementDriver.h"
#include "simulation/SimulationState.h"

namespace dmap {

class MovementMock final : public IMovementDriver {
 public:
  explicit MovementMock(SimulationState& state);

  void rotate(TurnDirection direction, AngleDeg angle) override;
  void advance(LengthCm distance) override;
  void elevate(LengthCm distance) override;

 private:
  SimulationState& state_;
};

}  // namespace dmap
