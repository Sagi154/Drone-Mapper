#include "algorithm/DroneAlgorithm.h"
#include "common/Logger.h"
#include "config/DroneConfigParser.h"
#include "config/MissionConfigParser.h"
#include "io/MapFileReader.h"
#include "io/MapFileWriter.h"
#include "mapping/BuildingMap.h"
#include "simulation/LidarMock.h"
#include "simulation/MovementMock.h"
#include "simulation/PositionMock.h"
#include "simulation/SimulationState.h"

#include <filesystem>
#include <iostream>

int main(int argc, char** argv) {
  std::filesystem::path root = std::filesystem::current_path();
  if (argc > 1 && argv[1] != nullptr) {
    root = argv[1];
  }

  dmap::log::info("drone_mapper scaffold starting");

  const auto drone_cfg = dmap::parseDroneConfig(root / "drone_config.txt");
  const auto mission = dmap::parseMissionConfig(root / "mission_config.txt");

  dmap::SimulationState state;
  (void)dmap::loadGroundTruthMap(root / "map_input.txt", state);
  state.setDronePosition(mission.initial_position);

  dmap::BuildingMap map(mission);
  dmap::LidarMock lidar(state, drone_cfg);
  dmap::PositionMock pos(state);
  dmap::MovementMock move(state, drone_cfg, mission);
  dmap::DroneAlgorithm algo(lidar, pos, move, map);
  algo.tick();

  if (!dmap::writeBuildingMap(root / "map_output.txt", map)) {
    std::cerr << "failed to write map_output.txt\n";
    return 1;
  }

  dmap::log::info("drone_mapper scaffold finished");
  return 0;
}
