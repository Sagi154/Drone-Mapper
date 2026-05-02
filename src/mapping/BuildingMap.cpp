#include "mapping/BuildingMap.h"

#include <mp-units/systems/si/unit_symbols.h>

namespace dmap {

namespace su = mp_units::si::unit_symbols;

BuildingMap::BuildingMap(MissionConfig mission) : mission_(mission) {}

bool BuildingMap::inBounds(const Point3D& p) const {
  const double x = p.x.numerical_value_in(su::cm);
  const double y = p.y.numerical_value_in(su::cm);
  const double h = p.height.numerical_value_in(su::cm);
  const double minx = mission_.min_x.numerical_value_in(su::cm);
  const double maxx = mission_.max_x.numerical_value_in(su::cm);
  const double miny = mission_.min_y.numerical_value_in(su::cm);
  const double maxy = mission_.max_y.numerical_value_in(su::cm);
  const double minh = mission_.min_height.numerical_value_in(su::cm);
  const double maxh = mission_.max_height.numerical_value_in(su::cm);
  return x >= minx && x <= maxx && y >= miny && y <= maxy && h >= minh && h <= maxh;
}

MapValue BuildingMap::get(const Point3D& p) const {
  if (!inBounds(p)) {
    return MapValue::OutOfBounds;
  }
  const SparseKey k =
      quantizePoint(p, mission_.xy_decimal_places, mission_.height_decimal_places);
  const auto it = cells_.find(k);
  if (it == cells_.end()) {
    return MapValue::NotMapped;
  }
  return it->second;
}

void BuildingMap::set(const Point3D& p, MapValue v) {
  if (!inBounds(p)) {
    return;
  }
  const SparseKey k =
      quantizePoint(p, mission_.xy_decimal_places, mission_.height_decimal_places);
  cells_[k] = v;
}

}  // namespace dmap
