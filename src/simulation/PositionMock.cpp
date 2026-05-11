// PositionMock.cpp
// Thin wrapper — delegates entirely to SimulationState::dronePosition().

#include "simulation/PositionMock.h"

namespace dmap {

PositionMock::PositionMock(const SimulationState& state) : state_(state) {}

DronePosition PositionMock::getPosition() const { return state_.dronePosition(); }

}  // namespace dmap
