#include "simulation/MovementMock.h"
#include "simulation/SimulationState.h"

#include "config/DroneConfig.h"
#include "config/MissionConfig.h"
#include "drivers/IMovementDriver.h"
#include "mapping/MapTypes.h"

#include <mp-units/systems/si/unit_symbols.h>
#include <gtest/gtest.h>
#include <cmath>

namespace su = mp_units::si::unit_symbols;
using namespace dmap;

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

// DroneConfig with generous limits and a 20x20x20 cm body.
static DroneConfig makeDroneCfg() {
  DroneConfig cfg;
  cfg.min_passable_width   = 20.0 * su::cm;
  cfg.min_passable_length  = 20.0 * su::cm;
  cfg.min_passable_height  = 20.0 * su::cm;
  cfg.max_rotate_per_command  = 90.0 * su::deg;
  cfg.max_advance_per_command = 100.0 * su::cm;
  cfg.max_elevate_per_command = 100.0 * su::cm;
  return cfg;
}

// MissionConfig with 1 cm resolution (xy_decimal_places=0 → step=1cm).
static MissionConfig makeMissionCfg() {
  MissionConfig m;
  m.xy_decimal_places     = 0;
  m.height_decimal_places = 0;
  return m;
}

static Point3D pt(double x, double y, double h) {
  return {x * su::cm, y * su::cm, h * su::cm};
}

// -----------------------------------------------------------------------
// Fixture
// -----------------------------------------------------------------------

class MovementMockTest : public ::testing::Test {
 protected:
  SimulationState state;
  DroneConfig     drone_cfg  = makeDroneCfg();
  MissionConfig   mission    = makeMissionCfg();

  MovementMock makeMove() { return MovementMock(state, drone_cfg, mission); }

  double x()   { return state.dronePosition().x.numerical_value_in(su::cm); }
  double y()   { return state.dronePosition().y.numerical_value_in(su::cm); }
  double h()   { return state.dronePosition().height.numerical_value_in(su::cm); }
  double ang() { return state.dronePosition().xy_angle.numerical_value_in(su::deg); }
};

// -----------------------------------------------------------------------
// rotate
// -----------------------------------------------------------------------

TEST_F(MovementMockTest, Rotate_Left_IncreasesAngle) {
  auto move = makeMove();
  move.rotate(TurnDirection::Left, 30.0 * su::deg);
  EXPECT_DOUBLE_EQ(ang(), 30.0);
}

TEST_F(MovementMockTest, Rotate_Right_DecreasesAngle) {
  // Start at 45°, turn right 30° → 15°.
  DronePosition start{0 * su::cm, 0 * su::cm, 0 * su::cm, 45.0 * su::deg};
  state.setDronePosition(start);
  auto move = makeMove();
  move.rotate(TurnDirection::Right, 30.0 * su::deg);
  EXPECT_DOUBLE_EQ(ang(), 15.0);
}

TEST_F(MovementMockTest, Rotate_ZeroAngle_NoChange) {
  auto move = makeMove();
  move.rotate(TurnDirection::Left, 0.0 * su::deg);
  EXPECT_DOUBLE_EQ(ang(), 0.0);
}

TEST_F(MovementMockTest, Rotate_ExceedsMax_Rejected) {
  auto move = makeMove();
  // max is 90°, so 91° is rejected.
  move.rotate(TurnDirection::Left, 91.0 * su::deg);
  EXPECT_DOUBLE_EQ(ang(), 0.0);  // unchanged
}

TEST_F(MovementMockTest, Rotate_NegativeAngle_Rejected) {
  auto move = makeMove();
  move.rotate(TurnDirection::Left, -1.0 * su::deg);
  EXPECT_DOUBLE_EQ(ang(), 0.0);  // unchanged
}

// -----------------------------------------------------------------------
// advance
// -----------------------------------------------------------------------

TEST_F(MovementMockTest, Advance_East_UpdatesX) {
  // Drone facing east (0°): advance 50cm → x increases by 50.
  DronePosition start{100.0 * su::cm, 100.0 * su::cm, 50.0 * su::cm, 0.0 * su::deg};
  state.setDronePosition(start);
  auto move = makeMove();
  move.advance(50.0 * su::cm);
  EXPECT_NEAR(x(), 150.0, 1e-6);
  EXPECT_NEAR(y(), 100.0, 1e-6);
}

TEST_F(MovementMockTest, Advance_North_UpdatesY) {
  // Drone facing north (90°): advance 50cm → y increases by 50.
  DronePosition start{100.0 * su::cm, 100.0 * su::cm, 50.0 * su::cm, 90.0 * su::deg};
  state.setDronePosition(start);
  auto move = makeMove();
  move.advance(50.0 * su::cm);
  EXPECT_NEAR(x(), 100.0, 1e-6);
  EXPECT_NEAR(y(), 150.0, 1e-6);
}

TEST_F(MovementMockTest, Advance_NegativeDistance_Rejected) {
  DronePosition start{100.0 * su::cm, 100.0 * su::cm, 50.0 * su::cm, 0.0 * su::deg};
  state.setDronePosition(start);
  auto move = makeMove();
  move.advance(-10.0 * su::cm);
  EXPECT_NEAR(x(), 100.0, 1e-6);  // unchanged
}

TEST_F(MovementMockTest, Advance_ExceedsMax_Rejected) {
  DronePosition start{100.0 * su::cm, 100.0 * su::cm, 50.0 * su::cm, 0.0 * su::deg};
  state.setDronePosition(start);
  auto move = makeMove();
  move.advance(101.0 * su::cm);  // max is 100
  EXPECT_NEAR(x(), 100.0, 1e-6);  // unchanged
}

TEST_F(MovementMockTest, Advance_WallDirectlyAhead_Blocked) {
  // Drone at (100, 100, 50) facing east. Body is 20cm long (half=10cm).
  // Wall at (150, 100, 50) — exactly on the forward face when dest=(140,100,50).
  // Actually the forward face is at dest.x + half_length = 140 + 10 = 150.
  DronePosition start{100.0 * su::cm, 100.0 * su::cm, 50.0 * su::cm, 0.0 * su::deg};
  state.setDronePosition(start);
  state.setTruthCell(pt(150, 100, 50), MapValue::Occupied);
  auto move = makeMove();
  move.advance(40.0 * su::cm);  // dest=(140,100,50); forward face hits wall at 150
  EXPECT_NEAR(x(), 100.0, 1e-6);  // blocked — stays
}

TEST_F(MovementMockTest, Advance_WallAtSide_NotBlocked) {
  // Wall 30cm to the side (outside half-width=10cm) — should not block eastward advance.
  DronePosition start{100.0 * su::cm, 100.0 * su::cm, 50.0 * su::cm, 0.0 * su::deg};
  state.setDronePosition(start);
  state.setTruthCell(pt(140, 130, 50), MapValue::Occupied);  // 30cm to the north
  auto move = makeMove();
  move.advance(40.0 * su::cm);
  EXPECT_NEAR(x(), 140.0, 1e-6);  // should succeed
}

TEST_F(MovementMockTest, Advance_ThinWallInPath_Blocked) {
  // Wall at (120, 100, 50) is mid-path between start (100) and dest (150).
  // Forward face passes through x=130 (drone at 120, face at 120+10=130) during march.
  // Wall actually needs to be within the forward face range.
  // Drone is at (100,100,50), half_length=10. At travelled=10, drone=(110,100,50),
  // forward face at x=120. Wall at (120,100,50) → blocked by ray stepping.
  DronePosition start{100.0 * su::cm, 100.0 * su::cm, 50.0 * su::cm, 0.0 * su::deg};
  state.setDronePosition(start);
  state.setTruthCell(pt(120, 100, 50), MapValue::Occupied);
  auto move = makeMove();
  move.advance(50.0 * su::cm);  // dest would be at 150, but wall at 120
  EXPECT_NEAR(x(), 100.0, 1e-6);  // blocked by thin wall mid-path
}

// -----------------------------------------------------------------------
// elevate
// -----------------------------------------------------------------------

TEST_F(MovementMockTest, Elevate_Positive_MovesUp) {
  DronePosition start{100.0 * su::cm, 100.0 * su::cm, 50.0 * su::cm, 0.0 * su::deg};
  state.setDronePosition(start);
  auto move = makeMove();
  move.elevate(30.0 * su::cm);
  EXPECT_NEAR(h(), 80.0, 1e-6);
}

TEST_F(MovementMockTest, Elevate_Negative_MovesDown) {
  DronePosition start{100.0 * su::cm, 100.0 * su::cm, 50.0 * su::cm, 0.0 * su::deg};
  state.setDronePosition(start);
  auto move = makeMove();
  move.elevate(-20.0 * su::cm);
  EXPECT_NEAR(h(), 30.0, 1e-6);
}

TEST_F(MovementMockTest, Elevate_ExceedsMax_Rejected) {
  DronePosition start{100.0 * su::cm, 100.0 * su::cm, 50.0 * su::cm, 0.0 * su::deg};
  state.setDronePosition(start);
  auto move = makeMove();
  move.elevate(101.0 * su::cm);  // max is 100
  EXPECT_NEAR(h(), 50.0, 1e-6);  // unchanged
}

TEST_F(MovementMockTest, Elevate_WallAbove_GoingUp_Blocked) {
  // Drone at h=50, half_height=10cm → top face at h=60.
  // Wall at h=60. Elevating 20cm would put top face at h=80. But wall at h=60
  // is hit during the march (at travelled=0, dest top face = 50+20+10=80; but
  // actually at travelled=step the top face = 50+1+10=61... that doesn't hit 60.
  // Let's think: elevate(20), top face sweeps from 60 to 80.
  // At travelled=1, height=51, top face=61. No hit at 60.
  // Wait — the check is intersectsElevateFace which checks ONLY the top face.
  // Top face at height h means checking the plane h+half_height.
  // At dest h=70, top face = 70+10=80. Wall at 80 → blocked.
  DronePosition start{100.0 * su::cm, 100.0 * su::cm, 50.0 * su::cm, 0.0 * su::deg};
  state.setDronePosition(start);
  state.setTruthCell(pt(100, 100, 80), MapValue::Occupied);  // top face of dest
  auto move = makeMove();
  move.elevate(20.0 * su::cm);  // dest h=70, top face = 80 → blocked
  EXPECT_NEAR(h(), 50.0, 1e-6);  // unchanged
}

TEST_F(MovementMockTest, Elevate_WallBelow_GoingDown_Blocked) {
  // Drone at h=50, half_height=10cm → bottom face at h=40.
  // Elevate down 20cm: dest h=30, bottom face at h=20.
  // Wall at h=20 → blocked.
  DronePosition start{100.0 * su::cm, 100.0 * su::cm, 50.0 * su::cm, 0.0 * su::deg};
  state.setDronePosition(start);
  state.setTruthCell(pt(100, 100, 20), MapValue::Occupied);
  auto move = makeMove();
  move.elevate(-20.0 * su::cm);
  EXPECT_NEAR(h(), 50.0, 1e-6);  // unchanged
}

TEST_F(MovementMockTest, Elevate_WallBelow_GoingUp_NotBlocked) {
  // Going UP. Only the top face is checked. Wall below should not block.
  DronePosition start{100.0 * su::cm, 100.0 * su::cm, 50.0 * su::cm, 0.0 * su::deg};
  state.setDronePosition(start);
  state.setTruthCell(pt(100, 100, 30), MapValue::Occupied);  // below current pos
  auto move = makeMove();
  move.elevate(20.0 * su::cm);
  EXPECT_NEAR(h(), 70.0, 1e-6);  // should succeed
}
