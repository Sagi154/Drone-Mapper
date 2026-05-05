#include "simulation/SimulationState.h"

#include <mp-units/systems/si/unit_symbols.h>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace dmap {

namespace su = mp_units::si::unit_symbols;

// --- Map bounds ---

void SimulationState::setMapBounds(const MapBounds& bounds) {
  bounds_ = bounds;
  has_bounds_ = true;
}

// --- Ground-truth cells ---

void SimulationState::clearGroundTruth() {
  truth_.clear();
  // Bounds are intentionally preserved: the flight volume does not change
  // just because we reload the cell data.
}

void SimulationState::setTruthCell(const Point3D& p, MapValue v) {
  truth_[truthKey(p)] = v;
}

MapValue SimulationState::truthValue(const Point3D& p) const {
  if (has_bounds_ && !isInBounds(p)) {
    return MapValue::OutOfBounds;
  }
  auto it = truth_.find(truthKey(p));
  if (it == truth_.end()) {
    return MapValue::Empty;
  }
  return it->second;
}

// --- Private helpers ---

bool SimulationState::isInBounds(const Point3D& p) const {
  return p.x >= bounds_.min_x && p.x <= bounds_.max_x &&
         p.y >= bounds_.min_y && p.y <= bounds_.max_y &&
         p.height >= bounds_.min_height && p.height <= bounds_.max_height;
}

std::string SimulationState::truthKey(const Point3D& p) const {
  // Round each coordinate to the nearest grid cell before hashing.
  // This absorbs floating-point drift so that positions within the same
  // physical cell always produce the same key.
  const int xy_prec = has_bounds_ ? bounds_.xy_decimal_places : 4;
  const int h_prec  = has_bounds_ ? bounds_.height_decimal_places : 4;
  const double xy_scale = std::pow(10.0, xy_prec);
  const double h_scale  = std::pow(10.0, h_prec);

  const double rx = std::round(p.x.numerical_value_in(su::cm) * xy_scale) / xy_scale;
  const double ry = std::round(p.y.numerical_value_in(su::cm) * xy_scale) / xy_scale;
  const double rh = std::round(p.height.numerical_value_in(su::cm) * h_scale) / h_scale;

  std::ostringstream os;
  os << std::fixed
     << std::setprecision(xy_prec) << rx << ',' << ry << ','
     << std::setprecision(h_prec)  << rh;
  return os.str();
}

}  // namespace dmap
