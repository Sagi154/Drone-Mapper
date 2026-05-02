#include "simulation/CollisionDetector.h"
#include "simulation/SimulationState.h"

#include "mapping/MapTypes.h"

namespace dmap {

CollisionDetector::CollisionDetector(const SimulationState& state) : state_(state) {}

bool CollisionDetector::intersectsOccupied(const Point3D& at) const {
  return state_.truthValue(at) == MapValue::Occupied;
}

}  // namespace dmap
