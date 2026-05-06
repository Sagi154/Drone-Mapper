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

static DroneConfig makeCfg(double width_cm, double length_cm, double height_cm) {
  DroneConfig cfg;
  cfg.min_passable_width  = width_cm  * su::cm;
  cfg.min_passable_length = length_cm * su::cm;
  cfg.min_passable_height = height_cm * su::cm;
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
// intersectsFootprint — bounding box checks
// -----------------------------------------------------------------------

TEST(CollisionDetector, Footprint_OpenSpaceReturnsFalse) {
  SimulationState state;
  // 20cm wide, 20cm long, 20cm tall drone in a completely open space.
  auto cfg = makeCfg(20.0, 20.0, 20.0);
  CollisionDetector det(state, cfg, 1.0, 1.0);
  EXPECT_FALSE(det.intersectsFootprint(pos(100, 100, 50, 0)));
}

TEST(CollisionDetector, Footprint_WallAtCenterReturnsTrue) {
  SimulationState state;
  state.setTruthCell(pt(100, 100, 50), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(20.0, 20.0, 20.0), 1.0, 1.0);
  EXPECT_TRUE(det.intersectsFootprint(pos(100, 100, 50, 0)));
}

TEST(CollisionDetector, Footprint_WallInsideWidthReturnsTrue) {
  SimulationState state;
  // Drone faces east (0°). Left direction is +y. Half-width = 10cm.
  // Wall 8cm to the left → inside footprint.
  state.setTruthCell(pt(100, 108, 50), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(20.0, 20.0, 20.0), 1.0, 1.0);
  EXPECT_TRUE(det.intersectsFootprint(pos(100, 100, 50, 0)));
}

TEST(CollisionDetector, Footprint_WallOutsideWidthReturnsFalse) {
  SimulationState state;
  // Half-width = 10cm. Wall 15cm to the left → outside footprint.
  state.setTruthCell(pt(100, 115, 50), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(20.0, 20.0, 20.0), 1.0, 1.0);
  EXPECT_FALSE(det.intersectsFootprint(pos(100, 100, 50, 0)));
}

TEST(CollisionDetector, Footprint_WallInsideHeightReturnsTrue) {
  SimulationState state;
  // Half-height = 10cm. Wall 8cm above drone center → inside footprint.
  state.setTruthCell(pt(100, 100, 58), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(20.0, 20.0, 20.0), 1.0, 1.0);
  EXPECT_TRUE(det.intersectsFootprint(pos(100, 100, 50, 0)));
}

TEST(CollisionDetector, Footprint_WallOutsideHeightReturnsFalse) {
  SimulationState state;
  // Half-height = 10cm. Wall 15cm above drone center → outside footprint.
  state.setTruthCell(pt(100, 100, 65), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(20.0, 20.0, 20.0), 1.0, 1.0);
  EXPECT_FALSE(det.intersectsFootprint(pos(100, 100, 50, 0)));
}

TEST(CollisionDetector, Footprint_WallInsideLengthReturnsTrue) {
  SimulationState state;
  // Drone faces east (0°). Forward direction is +x. Half-length = 10cm.
  // Wall 8cm ahead of center → inside footprint.
  state.setTruthCell(pt(108, 100, 50), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(20.0, 20.0, 20.0), 1.0, 1.0);
  EXPECT_TRUE(det.intersectsFootprint(pos(100, 100, 50, 0)));
}

TEST(CollisionDetector, Footprint_RotatedHeading_WallOnWidthAxis) {
  SimulationState state;
  // Drone faces north (90°). Width axis is now east-west.
  // Left when facing north = west = -x direction.
  // Wall 8cm to the west of drone center → inside footprint.
  state.setTruthCell(pt(92, 100, 50), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(20.0, 20.0, 20.0), 1.0, 1.0);
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
// intersectsForwardFace
// -----------------------------------------------------------------------

TEST(CollisionDetector, ForwardFace_WallInFront_ReturnsTrue) {
  SimulationState state;
  // Drone at (100, 100, 50) facing east (0°), half_length=10cm.
  // Forward face centre at (110, 100, 50).  Wall exactly there.
  state.setTruthCell(pt(110, 100, 50), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(20.0, 20.0, 20.0), 1.0, 1.0);
  EXPECT_TRUE(det.intersectsForwardFace(pos(100, 100, 50, 0)));
}

TEST(CollisionDetector, ForwardFace_WallBehind_ReturnsFalse) {
  SimulationState state;
  // Wall 10cm BEHIND center (at x=90) should not be on the forward face.
  state.setTruthCell(pt(90, 100, 50), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(20.0, 20.0, 20.0), 1.0, 1.0);
  EXPECT_FALSE(det.intersectsForwardFace(pos(100, 100, 50, 0)));
}

// -----------------------------------------------------------------------
// intersectsElevateFace
// -----------------------------------------------------------------------

TEST(CollisionDetector, ElevateFace_WallAbove_UpwardCheck_ReturnsTrue) {
  SimulationState state;
  // Drone at (100, 100, 50), half_height=10cm.  Top face at h=60.
  state.setTruthCell(pt(100, 100, 60), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(20.0, 20.0, 20.0), 1.0, 1.0);
  EXPECT_TRUE(det.intersectsElevateFace(pos(100, 100, 50, 0), /*upward=*/true));
}

TEST(CollisionDetector, ElevateFace_WallBelow_DownwardCheck_ReturnsTrue) {
  SimulationState state;
  // Bottom face at h=40.  Wall at h=40.
  state.setTruthCell(pt(100, 100, 40), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(20.0, 20.0, 20.0), 1.0, 1.0);
  EXPECT_TRUE(det.intersectsElevateFace(pos(100, 100, 50, 0), /*upward=*/false));
}

TEST(CollisionDetector, ElevateFace_WallAbove_DownwardCheck_ReturnsFalse) {
  SimulationState state;
  // Checking the BOTTOM face (going down).  Wall above at h=60 → not on bottom face.
  state.setTruthCell(pt(100, 100, 60), MapValue::Occupied);
  CollisionDetector det(state, makeCfg(20.0, 20.0, 20.0), 1.0, 1.0);
  EXPECT_FALSE(det.intersectsElevateFace(pos(100, 100, 50, 0), /*upward=*/false));
}
