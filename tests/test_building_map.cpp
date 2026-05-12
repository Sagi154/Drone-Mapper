// test_building_map.cpp
// Unit tests for BuildingMap: default state, set/get round-trips, bounds
// enforcement, quantization behavior, allCells() enumeration, bounds()
// accessor, and MapFileWriter round-trip against the map file reader.

#include "mapping/BuildingMap.h"
#include "common/Point3D.h"
#include "io/ErrorLogger.h"
#include "io/MapFileReader.h"
#include "io/MapFileWriter.h"
#include "simulation/SimulationState.h"

#include <mp-units/systems/si/unit_symbols.h>
#include <gtest/gtest.h>
#include <filesystem>

namespace su = mp_units::si::unit_symbols;
using namespace dmap;

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

// Bounded 1-cm-grid mission covering [-100, 100] cm in XY, [0, 200] cm in Z.
static MissionConfig makeMission() {
  MissionConfig m;
  m.min_x      = -100.0 * su::cm;
  m.max_x      =  100.0 * su::cm;
  m.min_y      = -100.0 * su::cm;
  m.max_y      =  100.0 * su::cm;
  m.min_height =    0.0 * su::cm;
  m.max_height =  200.0 * su::cm;
  m.xy_decimal_places     = 0;  // 1 cm resolution
  m.height_decimal_places = 0;
  return m;
}

static Point3D pt(double x, double y, double h) {
  return {x * su::cm, y * su::cm, h * su::cm};
}

// -----------------------------------------------------------------------
// Default state
// -----------------------------------------------------------------------

// What: freshly constructed map with a bounded mission.
// Expected: any in-bounds point returns NotMapped (-1), not Empty (0).
TEST(BuildingMap, NotMappedByDefault) {
  BuildingMap map(makeMission());
  EXPECT_EQ(map.get(pt(0, 0, 100)), MapValue::NotMapped);
}

// -----------------------------------------------------------------------
// Set / Get round-trips
// -----------------------------------------------------------------------

// What: set a cell to Occupied then read it back at the same position.
// Expected: Occupied (1) is returned; the cell is distinguishable from
//           NotMapped and from Empty.
TEST(BuildingMap, SetGet_OccupiedRoundTrip) {
  BuildingMap map(makeMission());
  map.set(pt(10, 20, 50), MapValue::Occupied);
  EXPECT_EQ(map.get(pt(10, 20, 50)), MapValue::Occupied);
}

// What: set a cell to Empty then read it back.
// Expected: Empty (0) is returned, which is distinct from the default
//           NotMapped (-1) for cells that were never touched.
TEST(BuildingMap, SetGet_EmptyRoundTrip) {
  BuildingMap map(makeMission());
  map.set(pt(-5, 30, 100), MapValue::Empty);
  EXPECT_EQ(map.get(pt(-5, 30, 100)), MapValue::Empty);
}

// -----------------------------------------------------------------------
// Bounds enforcement
// -----------------------------------------------------------------------

// What: query a point outside the mission's max_x boundary.
// Expected: OutOfBounds (-2), not NotMapped (-1), to match the spec value.
TEST(BuildingMap, OutOfBounds_PointOutsideMission) {
  BuildingMap map(makeMission());
  EXPECT_EQ(map.get(pt(200, 0, 100)), MapValue::OutOfBounds);
}

// What: set on an out-of-bounds point must be silently discarded.
// Expected: subsequent get at that position still returns OutOfBounds,
//           confirming the write was a no-op.
TEST(BuildingMap, SetOutOfBounds_IsNoOp) {
  BuildingMap map(makeMission());
  map.set(pt(200, 0, 100), MapValue::Occupied);
  EXPECT_EQ(map.get(pt(200, 0, 100)), MapValue::OutOfBounds);
}

// -----------------------------------------------------------------------
// Quantization
// -----------------------------------------------------------------------

// What: two positions that differ by less than one grid cell (1 cm with
//       xy_decimal_places=0) map to the same cell after quantization.
// Expected: setting p1=10.0 cm and reading p2=10.4 cm returns the stored
//           value because both round to the same integer centimetre.
TEST(BuildingMap, Quantization_TwoNearbyPointsSameCell) {
  BuildingMap map(makeMission());
  map.set(pt(10.0, 0, 50), MapValue::Occupied);
  // 10.4 cm rounds to 10 with 0 decimal places — falls in the same cell.
  EXPECT_EQ(map.get(pt(10.4, 0, 50)), MapValue::Occupied);
}

// -----------------------------------------------------------------------
// allCells()
// -----------------------------------------------------------------------

// What: set N cells explicitly; allCells() must enumerate exactly N entries.
// Expected: size == 3, confirming that both Occupied and Empty cells are
//           counted and that no phantom cells are created.
TEST(BuildingMap, AllCells_CountsStoredCells) {
  BuildingMap map(makeMission());
  map.set(pt(10, 0, 50),  MapValue::Occupied);
  map.set(pt(20, 0, 50),  MapValue::Empty);
  map.set(pt(30, 0, 50),  MapValue::Occupied);
  EXPECT_EQ(map.allCells().size(), 3u);
}

// -----------------------------------------------------------------------
// bounds()
// -----------------------------------------------------------------------

// What: bounds() must reflect the MissionConfig supplied at construction.
// Expected: every min/max coordinate and both decimal-places values match
//           the mission, confirming the writer and bounds-checker use the
//           same numbers.
TEST(BuildingMap, Bounds_MatchMissionConfig) {
  const auto m = makeMission();
  BuildingMap map(m);
  const MapBounds b = map.bounds();
  EXPECT_DOUBLE_EQ(b.min_x.numerical_value_in(su::cm),
                   m.min_x.numerical_value_in(su::cm));
  EXPECT_DOUBLE_EQ(b.max_x.numerical_value_in(su::cm),
                   m.max_x.numerical_value_in(su::cm));
  EXPECT_DOUBLE_EQ(b.min_y.numerical_value_in(su::cm),
                   m.min_y.numerical_value_in(su::cm));
  EXPECT_DOUBLE_EQ(b.max_y.numerical_value_in(su::cm),
                   m.max_y.numerical_value_in(su::cm));
  EXPECT_DOUBLE_EQ(b.min_height.numerical_value_in(su::cm),
                   m.min_height.numerical_value_in(su::cm));
  EXPECT_DOUBLE_EQ(b.max_height.numerical_value_in(su::cm),
                   m.max_height.numerical_value_in(su::cm));
  EXPECT_EQ(b.xy_decimal_places,     m.xy_decimal_places);
  EXPECT_EQ(b.height_decimal_places, m.height_decimal_places);
}

// -----------------------------------------------------------------------
// MapFileWriter round-trip
// -----------------------------------------------------------------------

// What: write a sparse BuildingMap to disk and reload it with the same
//       grammar as map_input (loadGroundTruthMap).
// Expected: bounds are set and the three explicit cells round-trip to the
//           same Occupied / Empty values in SimulationState.
TEST(BuildingMap, MapFileWriter_RoundTrip) {
  BuildingMap map(makeMission());
  map.set(pt(10, 0, 50), MapValue::Occupied);
  map.set(pt(20, 0, 50), MapValue::Empty);
  map.set(pt(30, 0, 50), MapValue::Occupied);

  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "dm_building_map_roundtrip_test.txt";
  std::error_code ec;
  std::filesystem::remove(path, ec);

  ASSERT_TRUE(writeBuildingMap(path, map));

  SimulationState state;
  ErrorLogger logger;
  ASSERT_TRUE(loadGroundTruthMap(path, state, logger));
  ASSERT_TRUE(state.hasBounds());

  EXPECT_EQ(state.truthValue(pt(10, 0, 50)), MapValue::Occupied);
  EXPECT_EQ(state.truthValue(pt(20, 0, 50)), MapValue::Empty);
  EXPECT_EQ(state.truthValue(pt(30, 0, 50)), MapValue::Occupied);

  std::filesystem::remove(path, ec);
}
