#include "config/MissionConfigParser.h"

#include <fstream>
#include <mp-units/systems/si/unit_symbols.h>

namespace dmap {

namespace su = mp_units::si::unit_symbols;

MissionConfig parseMissionConfig(const std::filesystem::path& path) {
  MissionConfig cfg{};
  std::ifstream in(path);
  (void)in;

  cfg.min_x = -1000.0 * su::cm;
  cfg.max_x = 1000.0 * su::cm;
  cfg.min_y = -1000.0 * su::cm;
  cfg.max_y = 1000.0 * su::cm;
  cfg.min_height = 0.0 * su::cm;
  cfg.max_height = 500.0 * su::cm;
  cfg.xy_decimal_places = 2;
  cfg.height_decimal_places = 2;
  return cfg;
}

}  // namespace dmap
