#include "io/MapFileReader.h"

#include <fstream>

namespace dmap {

bool loadGroundTruthMap(const std::filesystem::path& path, SimulationState& state) {
  state.clearGroundTruth();
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  return true;
}

}  // namespace dmap
