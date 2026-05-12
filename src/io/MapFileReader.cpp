// MapFileReader.cpp
// Loads the simulator's ground-truth map: bounds metadata and occupied / empty cells.

#include "io/MapFileReader.h"

#include "config/ConfigParseUtil.h"

#include <fstream>
#include <mp-units/systems/si/unit_symbols.h>
#include <sstream>

namespace dmap {

namespace su = mp_units::si::unit_symbols;

bool loadGroundTruthMap(const std::filesystem::path& path, SimulationState& state,
                        ErrorLogger& logger) {
  state.clearGroundTruth();
  std::ifstream in(path);
  if (!in) {
    // Missing input map is unrecoverable for the loader contract.
    std::ostringstream msg;
    msg << "[map_input] could not open file \"" << path.string() << "\" — map is empty";
    logger.add(msg.str());
    return false;
  }

  std::string line;
  int line_num = 0;
  while (std::getline(in, line)) {
    ++line_num;
    const std::string cleaned = config_parse::stripCommentAndTrim(line);
    if (cleaned.empty()) {
      continue;
    }

    const auto tokens = config_parse::splitWhitespace(cleaned);
    if (tokens.empty()) {
      continue;
    }

    try {
      // bounds min_x max_x min_y max_y min_h max_h xy_dp h_dp
      if (tokens[0] == "bounds") {
        if (tokens.size() != 9) {
          std::ostringstream msg;
          msg << "[map_input] line " << line_num << ": bad \"bounds\" record (expected 9 tokens, got "
              << tokens.size() << ") — skipped";
          logger.add(msg.str());
          continue;
        }
        MapBounds bounds{};
        bounds.min_x = std::stod(tokens[1]) * su::cm;
        bounds.max_x = std::stod(tokens[2]) * su::cm;
        bounds.min_y = std::stod(tokens[3]) * su::cm;
        bounds.max_y = std::stod(tokens[4]) * su::cm;
        bounds.min_height = std::stod(tokens[5]) * su::cm;
        bounds.max_height = std::stod(tokens[6]) * su::cm;
        bounds.xy_decimal_places = std::stoi(tokens[7]);
        bounds.height_decimal_places = std::stoi(tokens[8]);
        state.setMapBounds(bounds);
      } else if (tokens[0] == "occupied") {
        if (tokens.size() != 4) {
          std::ostringstream msg;
          msg << "[map_input] line " << line_num
              << ": bad \"occupied\" record (expected 4 tokens, got " << tokens.size()
              << ") — skipped";
          logger.add(msg.str());
          continue;
        }
        // occupied x y height
        state.setTruthCell(Point3D{std::stod(tokens[1]) * su::cm,
                                   std::stod(tokens[2]) * su::cm,
                                   std::stod(tokens[3]) * su::cm},
                           MapValue::Occupied);
      } else if (tokens[0] == "empty") {
        if (tokens.size() != 4) {
          std::ostringstream msg;
          msg << "[map_input] line " << line_num
              << ": bad \"empty\" record (expected 4 tokens, got " << tokens.size()
              << ") — skipped";
          logger.add(msg.str());
          continue;
        }
        state.setTruthCell(Point3D{std::stod(tokens[1]) * su::cm,
                                   std::stod(tokens[2]) * su::cm,
                                   std::stod(tokens[3]) * su::cm},
                           MapValue::Empty);
      } else {
        std::ostringstream msg;
        msg << "[map_input] line " << line_num << ": unknown record type \"" << tokens[0]
            << "\" — skipped";
        logger.add(msg.str());
      }
    } catch (...) {
      // Ignore malformed lines and keep loading remaining content.
      std::ostringstream msg;
      msg << "[map_input] line " << line_num << ": bad numeric value — skipped";
      logger.add(msg.str());
    }
  }
  return true;
}

}  // namespace dmap
