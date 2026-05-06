#include "simulation/SimulationState.h"

#include "common/Point3D.h"
#include "mapping/MapTypes.h"

#include <mp-units/systems/si/unit_symbols.h>
#include <gtest/gtest.h>

namespace su = mp_units::si::unit_symbols;
using namespace dmap;

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

static Point3D pt(double x, double y, double h) {
  return {x * su::cm, y * su::cm, h * su::cm};
}

static MapBounds makeBounds(double x0, double x1, double y0, double y1,
                             double h0, double h1,
                             int xy_prec = 4, int h_prec = 4) {
  MapBounds b;
  b.min_x = x0 * su::cm;   b.max_x = x1 * su::cm;
  b.min_y = y0 * su::cm;   b.max_y = y1 * su::cm;
  b.min_height = h0 * su::cm; b.max_height = h1 * su::cm;
  b.xy_decimal_places     = xy_prec;
  b.height_decimal_places = h_prec;
  return b;
}

// -----------------------------------------------------------------------
// DronePosition
// -----------------------------------------------------------------------

TEST(SimulationState, DronePosition_DefaultIsZero) {
  SimulationState state;
  const auto p = state.dronePosition();
  EXPECT_DOUBLE_EQ(p.x.numerical_value_in(su::cm),       0.0);
  EXPECT_DOUBLE_EQ(p.y.numerical_value_in(su::cm),       0.0);
  EXPECT_DOUBLE_EQ(p.height.numerical_value_in(su::cm),  0.0);
  EXPECT_DOUBLE_EQ(p.xy_angle.numerical_value_in(su::deg), 0.0);
}

TEST(SimulationState, DronePosition_Roundtrip) {
  SimulationState state;
  DronePosition p{12.5 * su::cm, -3.0 * su::cm, 55.0 * su::cm, 45.0 * su::deg};
  state.setDronePosition(p);
  const auto got = state.dronePosition();
  EXPECT_DOUBLE_EQ(got.x.numerical_value_in(su::cm),      12.5);
  EXPECT_DOUBLE_EQ(got.y.numerical_value_in(su::cm),      -3.0);
  EXPECT_DOUBLE_EQ(got.height.numerical_value_in(su::cm), 55.0);
  EXPECT_DOUBLE_EQ(got.xy_angle.numerical_value_in(su::deg), 45.0);
}

// -----------------------------------------------------------------------
// hasBounds / setMapBounds
// -----------------------------------------------------------------------

TEST(SimulationState, HasBounds_FalseByDefault) {
  SimulationState state;
  EXPECT_FALSE(state.hasBounds());
}

TEST(SimulationState, HasBounds_TrueAfterSetMapBounds) {
  SimulationState state;
  state.setMapBounds(makeBounds(0, 100, 0, 100, 0, 100));
  EXPECT_TRUE(state.hasBounds());
}

// -----------------------------------------------------------------------
// truthValue — no bounds set
// -----------------------------------------------------------------------

TEST(SimulationState, TruthValue_NoBounds_UnknownCell_ReturnsEmpty) {
  SimulationState state;
  // When no bounds are configured, every position is treated as in-bounds
  // and an un-stored cell is Empty (not OutOfBounds).
  EXPECT_EQ(state.truthValue(pt(999, 999, 999)), MapValue::Empty);
}

TEST(SimulationState, TruthValue_NoBounds_OccupiedCell_ReturnsOccupied) {
  SimulationState state;
  state.setTruthCell(pt(10, 20, 5), MapValue::Occupied);
  EXPECT_EQ(state.truthValue(pt(10, 20, 5)), MapValue::Occupied);
}

TEST(SimulationState, TruthValue_NoBounds_ExplicitEmpty_ReturnsEmpty) {
  SimulationState state;
  state.setTruthCell(pt(10, 20, 5), MapValue::Empty);
  EXPECT_EQ(state.truthValue(pt(10, 20, 5)), MapValue::Empty);
}

// -----------------------------------------------------------------------
// truthValue — with bounds set
// -----------------------------------------------------------------------

TEST(SimulationState, TruthValue_WithBounds_InsideUnstoredCell_ReturnsEmpty) {
  SimulationState state;
  state.setMapBounds(makeBounds(0, 100, 0, 100, 0, 100));
  // Inside the flight volume but no cell set there → Empty.
  EXPECT_EQ(state.truthValue(pt(50, 50, 50)), MapValue::Empty);
}

TEST(SimulationState, TruthValue_WithBounds_OutsideCell_ReturnsOutOfBounds) {
  SimulationState state;
  state.setMapBounds(makeBounds(0, 100, 0, 100, 0, 100));
  EXPECT_EQ(state.truthValue(pt(200, 200, 200)), MapValue::OutOfBounds);
}

TEST(SimulationState, TruthValue_WithBounds_OccupiedInside_ReturnsOccupied) {
  SimulationState state;
  state.setMapBounds(makeBounds(0, 200, 0, 200, 0, 200));
  state.setTruthCell(pt(100, 100, 50), MapValue::Occupied);
  EXPECT_EQ(state.truthValue(pt(100, 100, 50)), MapValue::Occupied);
}

// -----------------------------------------------------------------------
// truthKey — floating-point drift
// -----------------------------------------------------------------------

TEST(SimulationState, TruthKey_FloatDrift_SameGridCell) {
  // Two coordinates that differ by less than one grid step (10^-4 cm)
  // should map to the same cell.
  SimulationState state;
  state.setMapBounds(makeBounds(0, 200, 0, 200, 0, 200, /*xy*/4, /*h*/4));
  state.setTruthCell(pt(100.0, 100.0, 50.0), MapValue::Occupied);
  // Tiny floating-point perturbation that stays within the same 0.0001 cm cell.
  EXPECT_EQ(state.truthValue(pt(100.000001, 100.0, 50.0)), MapValue::Occupied);
  EXPECT_EQ(state.truthValue(pt(100.0, 99.999999, 50.0)),  MapValue::Occupied);
}

// -----------------------------------------------------------------------
// clearGroundTruth
// -----------------------------------------------------------------------

TEST(SimulationState, ClearGroundTruth_RemovesAllCells) {
  SimulationState state;
  state.setTruthCell(pt(10, 10, 10), MapValue::Occupied);
  state.setTruthCell(pt(20, 20, 20), MapValue::Occupied);
  state.clearGroundTruth();
  EXPECT_EQ(state.truthValue(pt(10, 10, 10)), MapValue::Empty);
  EXPECT_EQ(state.truthValue(pt(20, 20, 20)), MapValue::Empty);
}

TEST(SimulationState, ClearGroundTruth_PreservesBounds) {
  SimulationState state;
  state.setMapBounds(makeBounds(0, 100, 0, 100, 0, 100));
  state.clearGroundTruth();
  EXPECT_TRUE(state.hasBounds());
  // Out-of-bounds check still works after clear.
  EXPECT_EQ(state.truthValue(pt(200, 200, 200)), MapValue::OutOfBounds);
}
