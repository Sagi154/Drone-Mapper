#include "simulation/PositionMock.h"
#include "simulation/SimulationState.h"

#include <mp-units/systems/si/unit_symbols.h>
#include <gtest/gtest.h>

namespace su = mp_units::si::unit_symbols;
using namespace dmap;

TEST(PositionMock, ReturnsDefaultPosition) {
  SimulationState state;
  PositionMock pos(state);
  const auto p = pos.getPosition();
  EXPECT_DOUBLE_EQ(p.x.numerical_value_in(su::cm),        0.0);
  EXPECT_DOUBLE_EQ(p.y.numerical_value_in(su::cm),        0.0);
  EXPECT_DOUBLE_EQ(p.height.numerical_value_in(su::cm),   0.0);
  EXPECT_DOUBLE_EQ(p.xy_angle.numerical_value_in(su::deg), 0.0);
}

TEST(PositionMock, ReflectsStateAfterSetDronePosition) {
  SimulationState state;
  PositionMock pos(state);
  DronePosition expected{42.0 * su::cm, -7.5 * su::cm, 33.0 * su::cm, 180.0 * su::deg};
  state.setDronePosition(expected);
  const auto got = pos.getPosition();
  EXPECT_DOUBLE_EQ(got.x.numerical_value_in(su::cm),       42.0);
  EXPECT_DOUBLE_EQ(got.y.numerical_value_in(su::cm),       -7.5);
  EXPECT_DOUBLE_EQ(got.height.numerical_value_in(su::cm),  33.0);
  EXPECT_DOUBLE_EQ(got.xy_angle.numerical_value_in(su::deg), 180.0);
}
