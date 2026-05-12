// MapFileWriter.cpp
// Writes drone BuildingMap to map_output.txt using the same line grammar as map_input:
//   bounds min_x max_x min_y max_y min_h max_h xy_dp h_dp
//   occupied x y height
//   empty x y height
// Coordinates are in cm; bounds decimals follow mission resolution for readability.

#include "io/MapFileWriter.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <mp-units/systems/si/unit_symbols.h>

namespace dmap {

namespace su = mp_units::si::unit_symbols;

bool writeBuildingMap(const std::filesystem::path& path, const IBuildingMap& map) {
  std::ofstream out(path, std::ios::trunc);
  if (!out) {
    return false;
  }

  const MapBounds b = map.bounds();

  {
    const int prec_bounds =
        std::max(6, std::max(b.xy_decimal_places, b.height_decimal_places) + 2);
    out << std::fixed << std::setprecision(prec_bounds);
    out << "bounds " << b.min_x.numerical_value_in(su::cm) << ' '
        << b.max_x.numerical_value_in(su::cm) << ' ' << b.min_y.numerical_value_in(su::cm) << ' '
        << b.max_y.numerical_value_in(su::cm) << ' '
        << b.min_height.numerical_value_in(su::cm) << ' '
        << b.max_height.numerical_value_in(su::cm) << ' ' << b.xy_decimal_places << ' '
        << b.height_decimal_places << '\n';
  }

  for (const CellEntry& cell : map.allCells()) {
    if (cell.value != MapValue::Occupied && cell.value != MapValue::Empty) {
      continue;
    }
    const char* tag = (cell.value == MapValue::Occupied) ? "occupied" : "empty";
    out << tag << ' ' << std::fixed << std::setprecision(b.xy_decimal_places)
        << cell.position.x.numerical_value_in(su::cm) << ' '
        << cell.position.y.numerical_value_in(su::cm) << ' '
        << std::setprecision(b.height_decimal_places)
        << cell.position.height.numerical_value_in(su::cm) << '\n';
  }

  return static_cast<bool>(out);
}

}  // namespace dmap
