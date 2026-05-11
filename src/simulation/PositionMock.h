// PositionMock.h
// Simulated GPS / position sensor.  Simply reads the drone's current
// position straight from SimulationState, giving the algorithm a perfect,
// noise-free position fix at every call.

#pragma once

#include "sensors/IPositionSensor.h"
#include "simulation/SimulationState.h"

namespace dmap {

// Simulated position sensor that exposes the drone's exact ground-truth
// position.  Implements IPositionSensor so the algorithm treats it
// identically to a real sensor.
class PositionMock final : public IPositionSensor {
 public:
  // Binds the mock to the shared SimulationState it will read from.
  explicit PositionMock(const SimulationState& state);

  // Returns the drone's current position (x, y, height, heading angle)
  // as stored in SimulationState — always exact, never noisy.
  DronePosition getPosition() const override;

 private:
  const SimulationState& state_;
};

}  // namespace dmap
