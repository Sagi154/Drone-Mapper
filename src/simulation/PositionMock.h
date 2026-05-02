#pragma once

#include "sensors/IPositionSensor.h"
#include "simulation/SimulationState.h"

namespace dmap {

class PositionMock final : public IPositionSensor {
 public:
  explicit PositionMock(const SimulationState& state);

  DronePosition getPosition() const override;

 private:
  const SimulationState& state_;
};

}  // namespace dmap
