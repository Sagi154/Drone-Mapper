#include "algorithm/DroneAlgorithm.h"

#include <gtest/gtest.h>

#include "mapping/BuildingMap.h"
#include "simulation/LidarMock.h"
#include "simulation/MovementMock.h"
#include "simulation/PositionMock.h"
#include "simulation/SimulationState.h"

#include "config/DroneConfigParser.h"
#include "config/MissionConfigParser.h"
#include "io/ErrorLogger.h"

#include <mp-units/systems/si/unit_symbols.h>

namespace su = mp_units::si::unit_symbols;

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
  dmap::DroneAlgorithm algo(lidar, pos, move, map, 0.0 * su::cm);

  int ticks = 0;
  constexpr int kCap = 10;
  while (!algo.isFinished() && ticks < kCap) {
    EXPECT_NO_THROW(algo.tick());
    ++ticks;
  }
  EXPECT_TRUE(algo.isFinished());
  EXPECT_LT(ticks, kCap);
}
