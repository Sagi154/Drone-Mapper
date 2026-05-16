// Integration tests: multiple components working together.
//
// These tests exercise real wiring between SimulationState, MovementMock,
// CollisionDetector, PositionMock, and LidarMock — the same wiring used in
// the full drone pipeline.

#include "simulation/LidarMock.h"
#include "simulation/MovementMock.h"
#include "simulation/PositionMock.h"
#include "simulation/SimulationState.h"

#include "config/DroneConfig.h"
#include "config/MissionConfig.h"
#include "mapping/MapTypes.h"
#include "sensors/LidarTypes.h"

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

static DroneConfig makeMoveDroneCfg() {
  DroneConfig cfg;
  cfg.min_passable_width   = 10.0 * su::cm;
  cfg.min_passable_length  = 10.0 * su::cm;
  cfg.min_passable_height  = 10.0 * su::cm;
  cfg.max_rotate_per_command  = 180.0 * su::deg;
  cfg.max_advance_per_command = 200.0 * su::cm;
  cfg.max_elevate_per_command = 200.0 * su::cm;
  return cfg;
}

static DroneConfig makeLidarDroneCfg() {
  DroneConfig cfg;
  cfg.lidar.fov_circles      = 1;
  cfg.lidar.z_min            = 5.0   * su::cm;
  cfg.lidar.z_max            = 150.0 * su::cm;
  cfg.lidar.circle_spacing_d = 2.5   * su::cm;
  return cfg;
}

static MissionConfig makeMission() {
  MissionConfig m;
  m.xy_decimal_places     = 0;  // 1 cm step
  m.height_decimal_places = 0;
  return m;
}

// -----------------------------------------------------------------------
// Integration test 1: corridor navigation
//   Drone moves through an open corridor; collision stops it before walls.
// -----------------------------------------------------------------------

TEST(Integration, CorridorNavigation_DroneStopsBeforeWall) {
  SimulationState state;
  DroneConfig drone_cfg = makeMoveDroneCfg();
  MissionConfig mission = makeMission();

  // Drone starts at (100, 100, 50) facing east.
  state.setDronePosition({100.0 * su::cm, 100.0 * su::cm, 50.0 * su::cm, 0.0 * su::deg});

  // Wall at x=160 (within max_advance=200, but forward face would hit it).
  // Drone body half_length=5cm → forward face at dest x + 5.
  // Advance 60cm: dest=(160,100,50), forward face at x=165. No wall at 165.
  // Advance 70cm: dest=(170,100,50), forward face at x=175. No wall at 175.
  // Put wall at forward-face position: x=165, which blocks advance(60).
  state.setTruthCell(pt(165, 100, 50), MapValue::Occupied);

  MovementMock move(state, drone_cfg, mission);
  move.advance(60.0 * su::cm);  // forward face would reach 165 → blocked

  const auto pos = state.dronePosition();
  EXPECT_NEAR(pos.x.numerical_value_in(su::cm), 100.0, 1e-6);  // did not move

  // Now advance only 50cm: forward face at x=155 → no wall → should succeed.
  state.setDronePosition({100.0 * su::cm, 100.0 * su::cm, 50.0 * su::cm, 0.0 * su::deg});
  MovementMock move2(state, drone_cfg, mission);
  move2.advance(50.0 * su::cm);
  EXPECT_NEAR(state.dronePosition().x.numerical_value_in(su::cm), 150.0, 1e-6);
}

// -----------------------------------------------------------------------
// Integration test 2: scan a room — lidar detects walls in the right direction
// -----------------------------------------------------------------------

TEST(Integration, LidarScanRoom_DetectsWallAhead) {
  SimulationState state;
  DroneConfig lidar_cfg = makeLidarDroneCfg();

  // Drone at origin facing east. Wall at (80, 0, 0).
  state.setDronePosition({0.0 * su::cm, 0.0 * su::cm, 0.0 * su::cm, 0.0 * su::deg});
  state.setTruthCell(pt(80, 0, 0), MapValue::Occupied);

  LidarMock lidar(state, lidar_cfg);
  const auto result = lidar.scan();
  ASSERT_EQ(result.size(), 1u);

  // Hit azimuth should be ≈ 0° (east).
  const double az = result[0].azimuth_xy.numerical_value_in(su::deg);
  EXPECT_NEAR(az, 0.0, 5.0);  // within 5° of east

  // Hit distance should be ≈ 80cm.
  const double dist = result[0].distance.numerical_value_in(su::cm);
  EXPECT_NEAR(dist, 80.0, 2.0);
}

// -----------------------------------------------------------------------
// Integration test 3: move then scan — lidar sees new walls after moving
// -----------------------------------------------------------------------

TEST(Integration, MoveAndScan_LidarSeesWallAfterMoving) {
  SimulationState state;
  DroneConfig drone_cfg = makeMoveDroneCfg();
  DroneConfig lidar_cfg = makeLidarDroneCfg();
  MissionConfig mission = makeMission();

  // Drone at (0, 0, 0), wall at (200, 0, 0) — beyond z_max=150 initially.
  state.setDronePosition({0.0 * su::cm, 0.0 * su::cm, 0.0 * su::cm, 0.0 * su::deg});
  state.setTruthCell(pt(200, 0, 0), MapValue::Occupied);

  LidarMock lidar(state, lidar_cfg);
  // Before moving: wall is 200cm away, beyond z_max=150 → no return.
  const auto before = lidar.scan();
  ASSERT_EQ(before.size(), 1u);
  EXPECT_TRUE(lidarHitIsMiss(before[0]));

  // Move 100cm east → wall is now 100cm away → within z_max → hit.
  MovementMock move(state, drone_cfg, mission);
  move.advance(100.0 * su::cm);
  EXPECT_NEAR(state.dronePosition().x.numerical_value_in(su::cm), 100.0, 1e-6);

  const auto result = lidar.scan();
  ASSERT_EQ(result.size(), 1u);
  ASSERT_FALSE(lidarHitIsMiss(result[0]));
  const double dist = result[0].distance.numerical_value_in(su::cm);
  EXPECT_NEAR(dist, 100.0, 2.0);
}
