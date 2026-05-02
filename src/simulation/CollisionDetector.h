#pragma once

#include "common/Point3D.h"

namespace dmap {

class SimulationState;

class CollisionDetector {
 public:
  explicit CollisionDetector(const SimulationState& state);

  bool intersectsOccupied(const Point3D& at) const;

 private:
  const SimulationState& state_;
};

}  // namespace dmap
