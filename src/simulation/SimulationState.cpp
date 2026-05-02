#include "simulation/SimulationState.h"

#include <mp-units/ostream.h>
#include <sstream>

namespace dmap {

void SimulationState::clearGroundTruth() { truth_.clear(); }

void SimulationState::setTruthCell(const Point3D& p, MapValue v) { truth_[truthKey(p)] = v; }

MapValue SimulationState::truthValue(const Point3D& p) const {
  auto it = truth_.find(truthKey(p));
  if (it == truth_.end()) {
    return MapValue::Empty;
  }
  return it->second;
}

std::string SimulationState::truthKey(const Point3D& p) {
  std::ostringstream os;
  os << p.x << ',' << p.y << ',' << p.height;
  return os.str();
}

}  // namespace dmap
