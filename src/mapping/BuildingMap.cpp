// BuildingMap: sparse occupancy grid keyed by quantized mission resolution.

#include "mapping/BuildingMap.h"

#include <mp-units/systems/si/unit_symbols.h>
#include <cmath>

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

MapBounds BuildingMap::bounds() const {
  MapBounds b;
  b.min_x              = mission_.min_x;
  b.max_x              = mission_.max_x;
  b.min_y              = mission_.min_y;
  b.max_y              = mission_.max_y;
  b.min_height         = mission_.min_height;
  b.max_height         = mission_.max_height;
  b.xy_decimal_places     = mission_.xy_decimal_places;
  b.height_decimal_places = mission_.height_decimal_places;
  return b;
}

static double dequantizeCm(std::int64_t q, std::int32_t decimal_places) {
  const double scale = std::pow(10.0, static_cast<double>(decimal_places));
  return static_cast<double>(q) / scale;
}

std::vector<CellEntry> BuildingMap::allCells() const {
  std::vector<CellEntry> out;
  out.reserve(cells_.size());
  const auto xy_dp = mission_.xy_decimal_places;
  const auto h_dp  = mission_.height_decimal_places;
  for (const auto& [key, value] : cells_) {
    const double x_cm = dequantizeCm(key.qx, xy_dp);
    const double y_cm = dequantizeCm(key.qy, xy_dp);
    const double h_cm = dequantizeCm(key.qh, h_dp);
    out.push_back(CellEntry{
        .position = {x_cm * su::cm, y_cm * su::cm, h_cm * su::cm},
        .value    = value,
    });
  }
  return out;
}

}  // namespace dmap
