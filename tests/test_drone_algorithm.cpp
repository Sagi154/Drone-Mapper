// test_drone_algorithm.cpp
// Tests for DroneAlgorithm: scan fusion, state-machine exploration, and a
// small enclosed room where the drone should eventually finish.

#include "algorithm/DroneAlgorithm.h"

#include <gtest/gtest.h>

#include "config/DroneConfig.h"
#include "config/MissionConfig.h"
#include "mapping/BuildingMap.h"
#include "mapping/MapTypes.h"
#include "simulation/LidarMock.h"
#include "simulation/MovementMock.h"
#include "simulation/PositionMock.h"
#include "simulation/SimulationState.h"

#include "config/DroneConfigParser.h"
#include "config/MissionConfigParser.h"
#include "io/ErrorLogger.h"

#include <mp-units/systems/si/unit_symbols.h>

namespace su = mp_units::si::unit_symbols;

namespace {

dmap::Point3D pt(double x, double y, double h) {
  return {x * su::cm, y * su::cm, h * su::cm};
}

dmap::MissionConfig makeMissionForScanMapTest() {
  dmap::MissionConfig m;
  m.min_x      = -200.0 * su::cm;
  m.max_x      =  200.0 * su::cm;
  m.min_y      = -200.0 * su::cm;
  m.max_y      =  200.0 * su::cm;
  m.min_height =    0.0 * su::cm;
  m.max_height =  200.0 * su::cm;
  m.xy_decimal_places     = 0;
  m.height_decimal_places = 0;
  return m;
}

dmap::DroneConfig makeDroneForScanMapTest() {
  dmap::DroneConfig cfg;
  cfg.lidar.fov_circles      = 1;
  cfg.lidar.z_min            = 5.0   * su::cm;
  cfg.lidar.z_max            = 120.0 * su::cm;
  cfg.lidar.circle_spacing_d = 2.5   * su::cm;
  cfg.min_passable_radius    = 5.0   * su::cm;
  cfg.max_rotate_per_command  = 180.0 * su::deg;
  cfg.max_advance_per_command = 200.0 * su::cm;
  cfg.max_elevate_per_command = 200.0 * su::cm;
  return cfg;
}

void setBoundsFromMission(dmap::SimulationState& state, const dmap::MissionConfig& mission) {
  dmap::MapBounds bounds{};
  bounds.min_x              = mission.min_x;
  bounds.max_x              = mission.max_x;
  bounds.min_y              = mission.min_y;
  bounds.max_y              = mission.max_y;
  bounds.min_height         = mission.min_height;
  bounds.max_height         = mission.max_height;
  bounds.xy_decimal_places     = mission.xy_decimal_places;
  bounds.height_decimal_places = mission.height_decimal_places;
  state.setMapBounds(bounds);
}

// Small 1 cm grid room for end-to-end exploration (interior roughly -4..4).
dmap::MissionConfig makeSmallRoomMission() {
  dmap::MissionConfig m;
  m.min_x      = -20.0 * su::cm;
  m.max_x      =  20.0 * su::cm;
  m.min_y      = -20.0 * su::cm;
  m.max_y      =  20.0 * su::cm;
  m.min_height =  40.0 * su::cm;
  m.max_height =  60.0 * su::cm;
  m.xy_decimal_places     = 0;
  m.height_decimal_places = 0;
  m.initial_position = {0.0 * su::cm, 0.0 * su::cm, 50.0 * su::cm, 0.0 * su::deg};
  return m;
}

dmap::DroneConfig makeSmallRoomDrone() {
  dmap::DroneConfig cfg;
  cfg.lidar.fov_circles      = 1;
  cfg.lidar.z_min            = 5.0  * su::cm;
  cfg.lidar.z_max            = 30.0 * su::cm;
  cfg.lidar.circle_spacing_d = 2.5  * su::cm;
  cfg.min_passable_radius    = 2.0  * su::cm;
  cfg.max_rotate_per_command  = 180.0 * su::deg;
  cfg.max_advance_per_command = 10.0  * su::cm;
  cfg.max_elevate_per_command = 10.0  * su::cm;
  return cfg;
}

void buildSmallRoomWalls(dmap::SimulationState& state) {
  for (int x = -5; x <= 5; ++x) {
    state.setTruthCell(pt(x, -5, 50), dmap::MapValue::Occupied);
    state.setTruthCell(pt(x,  5, 50), dmap::MapValue::Occupied);
  }
  for (int y = -4; y <= 4; ++y) {
    state.setTruthCell(pt(-5, y, 50), dmap::MapValue::Occupied);
    state.setTruthCell(pt( 5, y, 50), dmap::MapValue::Occupied);
  }
}

}  // namespace

// What: default configs, empty map; first tick runs fullScan then planning.
// Expected: no throw; exploration ends once no passable frontier exists
//           (typically after scan + plan when start is not sphere-safe).
TEST(DroneAlgorithm, TickDoesNotThrow) {
  dmap::SimulationState state;
  dmap::ErrorLogger logger;
  const auto drone_cfg = dmap::parseDroneConfig("nonexistent_drone_config.txt", logger);
  const auto mission = dmap::parseMissionConfig("nonexistent_mission_config.txt", logger);
  dmap::BuildingMap map(mission);
  dmap::LidarMock lidar(state, drone_cfg);
  dmap::PositionMock pos(state);
  dmap::MovementMock move(state, drone_cfg, mission);
  dmap::DroneAlgorithm algo(lidar, pos, move, map, drone_cfg);

  int ticks = 0;
  constexpr int kCap = 20;
  while (!algo.isFinished() && ticks < kCap) {
    EXPECT_NO_THROW(algo.tick());
    ++ticks;
  }
  EXPECT_TRUE(algo.isFinished());
  EXPECT_LT(ticks, kCap);
}

// What: one tick (scanning phase) with a wall in ground truth ahead of the drone.
// Expected: fullScan marks beam cells Empty and the hit cell Occupied.
TEST(DroneAlgorithm, Tick_ScanUpdatesMap_OccupiedCellMarked) {
  dmap::SimulationState state;
  const auto mission = makeMissionForScanMapTest();
  setBoundsFromMission(state, mission);

  state.setDronePosition({0.0 * su::cm, 0.0 * su::cm, 50.0 * su::cm, 0.0 * su::deg});
  state.setTruthCell(pt(80, 0, 50), dmap::MapValue::Occupied);

  dmap::BuildingMap map(mission);
  const auto drone_cfg = makeDroneForScanMapTest();
  dmap::LidarMock lidar(state, drone_cfg);
  dmap::PositionMock pos(state);
  dmap::MovementMock move(state, drone_cfg, mission);
  dmap::DroneAlgorithm algo(lidar, pos, move, map, drone_cfg);

  algo.tick();

  EXPECT_EQ(map.get(pt(80, 0, 50)), dmap::MapValue::Occupied);
  EXPECT_EQ(map.get(pt(40, 0, 50)), dmap::MapValue::Empty);
}

// What: enclosed box room in ground truth; drone explores via frontier BFS.
// Expected: algorithm finishes within tick budget and records wall cells.
TEST(DroneAlgorithm, SmallRoom_ExploresAndFinishes) {
  const auto mission = makeSmallRoomMission();
  dmap::SimulationState state;
  setBoundsFromMission(state, mission);
  buildSmallRoomWalls(state);
  state.setDronePosition(mission.initial_position);

  dmap::BuildingMap map(mission);
  const auto drone_cfg = makeSmallRoomDrone();
  dmap::LidarMock lidar(state, drone_cfg);
  dmap::PositionMock pos(state);
  dmap::MovementMock move(state, drone_cfg, mission);
  dmap::DroneAlgorithm algo(lidar, pos, move, map, drone_cfg);

  int ticks = 0;
  constexpr int kMaxTicks = 500'000;
  while (!algo.isFinished() && ticks < kMaxTicks) {
    algo.tick();
    ++ticks;
  }

  EXPECT_TRUE(algo.isFinished());
  EXPECT_LT(ticks, kMaxTicks);
  EXPECT_EQ(map.get(pt(5, 0, 50)), dmap::MapValue::Occupied);
  EXPECT_EQ(map.get(pt(0, 0, 50)), dmap::MapValue::Empty);
}
