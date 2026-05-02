#include "config/DroneConfigParser.h"

#include <fstream>

namespace dmap {

DroneConfig parseDroneConfig(const std::filesystem::path& path) {
  DroneConfig cfg{};
  std::ifstream in(path);
  (void)in;
  return cfg;
}

}  // namespace dmap
