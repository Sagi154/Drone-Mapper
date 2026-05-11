// DroneConfigParser.cpp
// Parses key/value drone capability settings from drone_config.txt into
// strong-typed DroneConfig fields (cm/deg units).

#include "config/DroneConfigParser.h"

#include "config/ConfigParseUtil.h"

#include <fstream>
#include <mp-units/systems/si/unit_symbols.h>

namespace dmap {

namespace su = mp_units::si::unit_symbols;

DroneConfig parseDroneConfig(const std::filesystem::path& path) {
  DroneConfig cfg{};
  std::ifstream in(path);
  if (!in) {
    // Missing file is recoverable: return defaults and let callers continue.
    return cfg;
  }

  std::string line;
  while (std::getline(in, line)) {
    // Accept "key = value" lines with optional whitespace and inline comments.
    const auto kv = config_parse::parseKeyValueLine(line);
    if (!kv.has_value()) {
      continue;
    }

    const auto& [key, value] = *kv;
    try {
      if (key == "min_passable_width") {
        cfg.min_passable_width = std::stod(value) * su::cm;
      } else if (key == "min_passable_length") {
        cfg.min_passable_length = std::stod(value) * su::cm;
      } else if (key == "min_passable_height") {
        cfg.min_passable_height = std::stod(value) * su::cm;
      } else if (key == "max_rotate") {
        cfg.max_rotate_per_command = std::stod(value) * su::deg;
      } else if (key == "max_advance") {
        cfg.max_advance_per_command = std::stod(value) * su::cm;
      } else if (key == "max_elevate") {
        cfg.max_elevate_per_command = std::stod(value) * su::cm;
      } else if (key == "lidar_z_min") {
        cfg.lidar.z_min = std::stod(value) * su::cm;
      } else if (key == "lidar_z_max") {
        cfg.lidar.z_max = std::stod(value) * su::cm;
      } else if (key == "lidar_d") {
        cfg.lidar.circle_spacing_d = std::stod(value) * su::cm;
      } else if (key == "lidar_fov_circles") {
        cfg.lidar.fov_circles = std::stoi(value);
      }
    } catch (...) {
      // Malformed value is recoverable: keep prior/default field value.
    }
  }

  return cfg;
}

}  // namespace dmap
