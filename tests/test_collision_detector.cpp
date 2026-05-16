#include "simulation/CollisionDetector.h"
#include "simulation/SimulationState.h"

#include "common/Point3D.h"
#include "config/DroneConfig.h"
#include "mapping/MapTypes.h"
#include "sensors/IPositionSensor.h"

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

static DronePosition pos(double x, double y, double h, double angle_deg = 0.0) {
  return {x * su::cm, y * su::cm, h * su::cm, angle_deg * su::deg};
}

// Build a DroneConfig with a spherical body of the given radius (cm).
static DroneConfig makeCfg(double radius_cm) {
  DroneConfig cfg;
  cfg.min_passable_radius = radius_cm * su::cm;
  return cfg;
}

// -----------------------------------------------------------------------
// intersectsOccupied — single-point checks
// -----------------------------------------------------------------------

TEST(CollisionDetector, SinglePoint_EmptyStateReturnsFalse) {
  SimulationState state;
  CollisionDetector det(state);
  EXPECT_FALSE(det.intersectsOccupied(pt(0, 0, 0)));
}

TEST(CollisionDetector, SinglePoint_OccupiedCellReturnsTrue) {
  SimulationState state;
  state.setTruthCell(pt(10, 20, 5), MapValue::Occupied);
  CollisionDetector det(state);
  EXPECT_TRUE(det.intersectsOccupied(pt(10, 20, 5)));
}

TEST(CollisionDetector, SinglePoint_ExplicitEmptyCellReturnsFalse) {
  SimulationState state;
  state.setTruthCell(pt(10, 20, 5), MapValue::Empty);
  CollisionDetector det(state);
  EXPECT_FALSE(det.intersectsOccupied(pt(10, 20, 5)));
}

TEST(CollisionDetector, SinglePoint_OutOfBoundsReturnsFalse) {
  SimulationState state;
  MapBounds bounds;
  bounds.min_x = 0.0 * su::cm;    bounds.max_x = 100.0 * su::cm;
  bounds.min_y = 0.0 * su::cm;    bounds.max_y = 100.0 * su::cm;
  bounds.min_height = 0.0 * su::cm; bounds.max_height = 100.0 * su::cm;
  state.setMapBounds(bounds);
  CollisionDetector det(state);
  // OutOfBounds != Occupied, so should return false.
  EXPECT_FALSE(det.intersectsOccupied(pt(200, 200, 200)));
}

TEST(CollisionDetector, SinglePoint_AdjacentCellNotAffected) {
  SimulationState state;
  state.setTruthCell(pt(10, 20, 5), MapValue::Occupied);
  CollisionDetector det(state);
  EXPECT_FALSE(det.intersectsOccupied(pt(11, 20, 5)));
  EXPECT_FALSE(det.intersectsOccupied(pt(10, 21, 5)));
  EXPECT_FALSE(det.intersectsOccupied(pt(10, 20, 6)));
}

// -----------------------------------------------------------------------
// intersectsFootprint — sphere checks
// -----------------------------------------------------------------------

TEST(CollisionDetector, Footprint_OpenSpaceReturnsFalse) {
  // Scenario: drone (radius 10 cm) placed in a completely empty space.
  // Expected: no occupied cell within the sphere → false.
  SimulationState state;
  auto cfg = makeCfg(10.0);
  CollisionDetector det(state, cfg, 1.0, 1.0);
  EXPECT_FALSE(det.intersectsFootprint(pos(100, 100, 50, 0)));
}

TEST(CollisionDetector, Footprint_WallAtCenterReturnsTrue) {
  // Scenario: wall exactly at the sphere centre.
  // Expected: distance = 0 ≤ radius → true.
  SimulationState state;
  state.setTruthCell(pt(100, 100, 50), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(10.0), 1.0, 1.0);
  EXPECT_TRUE(det.intersectsFootprint(pos(100, 100, 50, 0)));
}

TEST(CollisionDetector, Footprint_WallInsideRadiusReturnsTrue) {
  // Scenario: wall 8 cm to the left of centre, radius = 10 cm.
  // Expected: distance = 8 ≤ 10 → true.
  SimulationState state;
  state.setTruthCell(pt(100, 108, 50), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(10.0), 1.0, 1.0);
  EXPECT_TRUE(det.intersectsFootprint(pos(100, 100, 50, 0)));
}

TEST(CollisionDetector, Footprint_WallOutsideRadiusReturnsFalse) {
  // Scenario: wall 15 cm to the left, radius = 10 cm.
  // Expected: distance = 15 > 10 → false.
  SimulationState state;
  state.setTruthCell(pt(100, 115, 50), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(10.0), 1.0, 1.0);
  EXPECT_FALSE(det.intersectsFootprint(pos(100, 100, 50, 0)));
}

TEST(CollisionDetector, Footprint_WallInsideRadiusVerticalReturnsTrue) {
  // Scenario: wall 8 cm above the centre, radius = 10 cm.
  // Expected: distance = 8 ≤ 10 → true.
  SimulationState state;
  state.setTruthCell(pt(100, 100, 58), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(10.0), 1.0, 1.0);
  EXPECT_TRUE(det.intersectsFootprint(pos(100, 100, 50, 0)));
}

TEST(CollisionDetector, Footprint_WallOutsideRadiusVerticalReturnsFalse) {
  // Scenario: wall 15 cm above the centre, radius = 10 cm.
  // Expected: distance = 15 > 10 → false.
  SimulationState state;
  state.setTruthCell(pt(100, 100, 65), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(10.0), 1.0, 1.0);
  EXPECT_FALSE(det.intersectsFootprint(pos(100, 100, 50, 0)));
}

TEST(CollisionDetector, Footprint_WallInsideRadiusForwardReturnsTrue) {
  // Scenario: wall 8 cm ahead along the heading, radius = 10 cm.
  // Expected: distance = 8 ≤ 10 → true regardless of heading.
  SimulationState state;
  state.setTruthCell(pt(108, 100, 50), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(10.0), 1.0, 1.0);
  EXPECT_TRUE(det.intersectsFootprint(pos(100, 100, 50, 0)));
}

TEST(CollisionDetector, Footprint_SphereIsHeadingIndependent) {
  // Scenario: wall 8 cm to the west; drone at heading 0° and 90°.
  // Expected: sphere footprint is identical for both headings → true in both cases.
  SimulationState state;
  state.setTruthCell(pt(92, 100, 50), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(10.0), 1.0, 1.0);
  EXPECT_TRUE(det.intersectsFootprint(pos(100, 100, 50, 0)));
  EXPECT_TRUE(det.intersectsFootprint(pos(100, 100, 50, 90)));
}

TEST(CollisionDetector, Footprint_DefaultConfig_BehavesLikeSinglePoint) {
  SimulationState state;
  state.setTruthCell(pt(10, 10, 10), MapValue::Occupied);
  // Simple constructor → zero dimensions → footprint degenerates to centre point.
  CollisionDetector det(state);
  EXPECT_TRUE(det.intersectsFootprint(pos(10, 10, 10, 0)));
  EXPECT_FALSE(det.intersectsFootprint(pos(11, 10, 10, 0)));
}

// -----------------------------------------------------------------------
// intersectsForwardFace  (forward hemisphere of the sphere)
// -----------------------------------------------------------------------

TEST(CollisionDetector, ForwardFace_WallInFront_ReturnsTrue) {
  // Scenario: drone at (100,100,50) facing east (0°), radius=10cm.
  // Wall at (110,100,50) is exactly 10cm ahead — on the sphere surface and
  // in the forward hemisphere.
  // Expected: true.
  SimulationState state;
  state.setTruthCell(pt(110, 100, 50), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(10.0), 1.0, 1.0);
  EXPECT_TRUE(det.intersectsForwardFace(pos(100, 100, 50, 0)));
}

TEST(CollisionDetector, ForwardFace_WallBehind_ReturnsFalse) {
  // Scenario: wall 10 cm BEHIND centre (x=90) — in the rear hemisphere.
  // Expected: rear hemisphere is excluded → false.
  SimulationState state;
  state.setTruthCell(pt(90, 100, 50), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(10.0), 1.0, 1.0);
  EXPECT_FALSE(det.intersectsForwardFace(pos(100, 100, 50, 0)));
}

// -----------------------------------------------------------------------
// intersectsElevateFace  (top / bottom hemisphere of the sphere)
// -----------------------------------------------------------------------

TEST(CollisionDetector, ElevateFace_WallAbove_UpwardCheck_ReturnsTrue) {
  // Scenario: drone at (100,100,50), radius=10cm.  Top of sphere at h=60.
  // Wall exactly at h=60 — on the top hemisphere surface.
  // Expected: true.
  SimulationState state;
  state.setTruthCell(pt(100, 100, 60), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(10.0), 1.0, 1.0);
  EXPECT_TRUE(det.intersectsElevateFace(pos(100, 100, 50, 0), /*upward=*/true));
}

TEST(CollisionDetector, ElevateFace_WallBelow_DownwardCheck_ReturnsTrue) {
  // Scenario: bottom of sphere at h=40.  Wall at h=40.
  // Expected: true.
  SimulationState state;
  state.setTruthCell(pt(100, 100, 40), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(10.0), 1.0, 1.0);
  EXPECT_TRUE(det.intersectsElevateFace(pos(100, 100, 50, 0), /*upward=*/false));
}

TEST(CollisionDetector, ElevateFace_WallAbove_DownwardCheck_ReturnsFalse) {
  // Scenario: checking the BOTTOM hemisphere (going down).
  // Wall above at h=60 is in the top hemisphere → excluded.
  // Expected: false.
  SimulationState state;
  state.setTruthCell(pt(100, 100, 60), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(10.0), 1.0, 1.0);
  EXPECT_FALSE(det.intersectsElevateFace(pos(100, 100, 50, 0), /*upward=*/false));
}
