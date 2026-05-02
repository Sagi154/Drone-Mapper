#include "simulation/MovementMock.h"

#include <mp-units/systems/si/unit_symbols.h>

namespace dmap {

namespace su = mp_units::si::unit_symbols;

MovementMock::MovementMock(SimulationState& state) : state_(state) {}

void MovementMock::rotate(TurnDirection direction, AngleDeg angle) {
  auto p = state_.dronePosition();
  const double sign = (direction == TurnDirection::Left) ? 1.0 : -1.0;
  const double delta = sign * static_cast<double>(angle.numerical_value_in(su::deg));
  const double cur = static_cast<double>(p.xy_angle.numerical_value_in(su::deg));
  p.xy_angle = (cur + delta) * su::deg;
  state_.setDronePosition(p);
}

void MovementMock::advance(LengthCm distance) {
  (void)distance;
}

void MovementMock::elevate(LengthCm distance) {
  (void)distance;
}

}  // namespace dmap
