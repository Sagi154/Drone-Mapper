// DroneAlgorithm.cpp
// Milestone 2: projects lidar hits into the sparse BuildingMap using the same beam
// geometry as LidarMock (via hitToWorldPoint). For each hit with measurable range,
// marks Empty along the ray in grid-sized steps, then Occupied at the hit cell.
// Movement is unused until the sweep loop lands.

#include "algorithm/DroneAlgorithm.h"

#include "common/MathUtils.h"
#include "mapping/MapTypes.h"

#include <algorithm>
#include <mp-units/systems/si/unit_symbols.h>

namespace dmap {

DroneAlgorithm::DroneAlgorithm(ILidarSensor& lidar, IPositionSensor& pos, IMovementDriver& move,
                                 IBuildingMap& map)
    : lidar_(lidar), pos_(pos), move_(move), map_(map) {}

void DroneAlgorithm::tick() {
  if (finished_) {
    return;
  }
  (void)move_;

  const DronePosition here = pos_.getPosition();
  const LidarScanResult hits = lidar_.scan();

  using su = mp_units::si::unit_symbols;
  const MapBounds b = map_.bounds();
  const double xy_step = decimalPlacesToStep(b.xy_decimal_places);
  const double h_step  = decimalPlacesToStep(b.height_decimal_places);
  const double ray_step = std::min(xy_step, h_step);

  for (const LidarHit& h : hits) {
    const double d_cm = h.distance.numerical_value_in(su::cm);
    if (d_cm > 0.0) {
      for (double t_cm = ray_step; t_cm < d_cm; t_cm += ray_step) {
        LidarHit slice = h;
        slice.distance = t_cm * su::cm;
        map_.set(hitToWorldPoint(here, slice), MapValue::Empty);
      }
    }
    map_.set(hitToWorldPoint(here, h), MapValue::Occupied);
  }

  finished_ = true;
}

}  // namespace dmap
