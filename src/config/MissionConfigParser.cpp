// MissionConfigParser.cpp
// Parses mission boundaries, initial drone pose, and resolution settings from
// mission_config.txt into strong-typed MissionConfig fields.

#include "config/MissionConfigParser.h"

#include "config/ConfigParseUtil.h"

#include <fstream>
#include <mp-units/systems/si/unit_symbols.h>

namespace dmap {

namespace su = mp_units::si::unit_symbols;

MissionConfig parseMissionConfig(const std::filesystem::path& path) {
  MissionConfig cfg{};
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
      if (key == "min_x") {
        cfg.min_x = std::stod(value) * su::cm;
      } else if (key == "max_x") {
        cfg.max_x = std::stod(value) * su::cm;
      } else if (key == "min_y") {
        cfg.min_y = std::stod(value) * su::cm;
      } else if (key == "max_y") {
        cfg.max_y = std::stod(value) * su::cm;
      } else if (key == "min_height") {
        cfg.min_height = std::stod(value) * su::cm;
      } else if (key == "max_height") {
        cfg.max_height = std::stod(value) * su::cm;
      } else if (key == "start_x") {
        cfg.initial_position.x = std::stod(value) * su::cm;
      } else if (key == "start_y") {
        cfg.initial_position.y = std::stod(value) * su::cm;
      } else if (key == "start_height") {
        cfg.initial_position.height = std::stod(value) * su::cm;
      } else if (key == "start_angle") {
        cfg.initial_position.xy_angle = std::stod(value) * su::deg;
      } else if (key == "xy_decimal_places") {
        cfg.xy_decimal_places = std::stoi(value);
      } else if (key == "height_decimal_places") {
        cfg.height_decimal_places = std::stoi(value);
      }
    } catch (...) {
      // Malformed value is recoverable: keep prior/default field value.
    }
  }

  return cfg;
}

}  // namespace dmap
