// DroneConfigParser.cpp
// Parses key/value drone capability settings from drone_config.txt into
// strong-typed DroneConfig fields (cm/deg units).

#include "config/DroneConfigParser.h"

#include "config/ConfigParseUtil.h"

#include <fstream>
#include <mp-units/systems/si/unit_symbols.h>
#include <sstream>

namespace dmap {

namespace su = mp_units::si::unit_symbols;

DroneConfig parseDroneConfig(const std::filesystem::path& path, ErrorLogger& logger) {
  DroneConfig cfg{};
  std::ifstream in(path);
  if (!in) {
    // Missing file is recoverable: return defaults and let callers continue.
    std::ostringstream msg;
    msg << "[drone_config] could not open file \"" << path.string()
        << "\" — all fields use defaults";
    logger.add(msg.str());
    return cfg;
  }

  std::string line;
  int line_num = 0;
  while (std::getline(in, line)) {
    ++line_num;
    // Accept "key = value" lines with optional whitespace and inline comments.
    const auto kv = config_parse::parseKeyValueLine(line);
    if (!kv.has_value()) {
      // Blank/comment-only lines are valid to ignore. Non-empty malformed lines are recovered.
      const std::string cleaned = config_parse::stripCommentAndTrim(line);
      if (!cleaned.empty()) {
        std::ostringstream msg;
        msg << "[drone_config] line " << line_num << ": bad key/value format — skipped";
        logger.add(msg.str());
      }
      continue;
    }

    const auto& [key, value] = *kv;
    try {
      if (key == "min_passable_radius") {
        cfg.min_passable_radius = std::stod(value) * su::cm;
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
      } else {
        std::ostringstream msg;
        msg << "[drone_config] line " << line_num << ": unknown key \"" << key << "\" — skipped";
        logger.add(msg.str());
      }
    } catch (...) {
      // Malformed value is recoverable: keep prior/default field value.
      std::ostringstream msg;
      msg << "[drone_config] line " << line_num << ": bad value \"" << value << "\" for key \""
          << key << "\" — field keeps default";
      logger.add(msg.str());
    }
  }

  return cfg;
}

}  // namespace dmap
