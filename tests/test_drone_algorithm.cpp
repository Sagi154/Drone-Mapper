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

TEST(DroneAlgorithm, TickDoesNotThrow) {
  dmap::SimulationState state;
  dmap::ErrorLogger logger;
  const auto drone_cfg = dmap::parseDroneConfig("nonexistent_drone_config.txt", logger);
  const auto mission = dmap::parseMissionConfig("nonexistent_mission_config.txt", logger);
  dmap::BuildingMap map(mission);
  dmap::LidarMock lidar(state, drone_cfg);
  dmap::PositionMock pos(state);
  dmap::MovementMock move(state, drone_cfg, mission);
  dmap::DroneAlgorithm algo(lidar, pos, move, map);
  EXPECT_NO_THROW(algo.tick());
}
