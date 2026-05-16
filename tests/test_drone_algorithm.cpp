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

// Flight volume and 1 cm grid; large enough for scan + short advance from origin.
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

// Lidar sees the wall at 80 cm; movement limits allow a small forward advance.
dmap::DroneConfig makeDroneForScanMapTest() {
  dmap::DroneConfig cfg;
  cfg.lidar.fov_circles      = 1;
  cfg.lidar.z_min            = 5.0   * su::cm;
  cfg.lidar.z_max            = 120.0 * su::cm;
  cfg.lidar.circle_spacing_d = 2.5   * su::cm;
  cfg.min_passable_width     = 10.0  * su::cm;
  cfg.min_passable_length    = 10.0  * su::cm;
  cfg.min_passable_height    = 10.0  * su::cm;
  cfg.max_rotate_per_command  = 180.0 * su::deg;
  cfg.max_advance_per_command = 200.0 * su::cm;
  cfg.max_elevate_per_command = 200.0 * su::cm;
  return cfg;
}

}  // namespace

// Default configs with zero advance step: every advance is a no-op at the same pose, so four
// ticks trigger four blocked advances and the algorithm reports finished without needing a map.
TEST(DroneAlgorithm, TickDoesNotThrow) {
  dmap::SimulationState state;
  dmap::ErrorLogger logger;
  const auto drone_cfg = dmap::parseDroneConfig("nonexistent_drone_config.txt", logger);
  const auto mission = dmap::parseMissionConfig("nonexistent_mission_config.txt", logger);
  dmap::BuildingMap map(mission);
  dmap::LidarMock lidar(state, drone_cfg);
  dmap::PositionMock pos(state);
  dmap::MovementMock move(state, drone_cfg, mission);
  dmap::DroneAlgorithm algo(lidar, pos, move, map, 0.0 * su::cm, drone_cfg.lidar.z_max);

  int ticks = 0;
  constexpr int kCap = 10;
  while (!algo.isFinished() && ticks < kCap) {
    EXPECT_NO_THROW(algo.tick());
    ++ticks;
  }
  EXPECT_TRUE(algo.isFinished());
  EXPECT_LT(ticks, kCap);
}

// What: one tick with a wall in ground truth in front of the drone.
// Expected: the drone map marks cells along the beam as Empty and the hit
//           cell as Occupied; a short advance still runs so this is not
//           confused with advance(0) blocked-move counting.
TEST(DroneAlgorithm, Tick_ScanUpdatesMap_OccupiedCellMarked) {
  dmap::SimulationState state;
  const auto mission = makeMissionForScanMapTest();
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

  state.setDronePosition({0.0 * su::cm, 0.0 * su::cm, 50.0 * su::cm, 0.0 * su::deg});
  state.setTruthCell(pt(80, 0, 50), dmap::MapValue::Occupied);

  dmap::BuildingMap map(mission);
  const auto drone_cfg = makeDroneForScanMapTest();
  dmap::LidarMock lidar(state, drone_cfg);
  dmap::PositionMock pos(state);
  dmap::MovementMock move(state, drone_cfg, mission);
  dmap::DroneAlgorithm algo(lidar, pos, move, map, 10.0 * su::cm, drone_cfg.lidar.z_max);

  algo.tick();

  EXPECT_EQ(map.get(pt(80, 0, 50)), dmap::MapValue::Occupied);
  EXPECT_EQ(map.get(pt(40, 0, 50)), dmap::MapValue::Empty);
}
