#pragma once

#include "mapping/MapTypes.h"
#include "simulation/SimulationState.h"

#include <filesystem>

namespace dmap {

bool loadGroundTruthMap(const std::filesystem::path& path, SimulationState& state);

}  // namespace dmap
