// DroneAlgorithm.cpp
// Frontier-based BFS exploration algorithm.
// Each tick runs one phase of the state machine: SCANNING (fuse lidar into
// map), PLANNING (BFS to find nearest frontier and path), or MOVING (execute
// the next waypoint in the current path).

#include "algorithm/DroneAlgorithm.h"

#include "common/MathUtils.h"
#include "mapping/MapTypes.h"
#include "sensors/LidarTypes.h"

#include <algorithm>
#include <cmath>
#include <mp-units/systems/si/unit_symbols.h>

namespace dmap {

namespace {

namespace su = mp_units::si::unit_symbols;

bool nearlySameCm(LengthCm a, LengthCm b) {
  return std::abs(a.numerical_value_in(su::cm) - b.numerical_value_in(su::cm)) < 1e-6;
}

bool same_xy_height(const DronePosition& a, const DronePosition& b) {
  return nearlySameCm(a.x, b.x) && nearlySameCm(a.y, b.y) && nearlySameCm(a.height, b.height);
}

/// Marks cells along each lidar beam as Empty (up to hit or z_max) and the
/// hit cell as Occupied. Miss beams (distance == -1) mark the full ray empty.
/// Distance == 0 (too close) marks only the hit cell Occupied, no ray cells.
void applyLidarHitsToMap(IBuildingMap& map, const DronePosition& here,
                         const LidarScanResult& hits, LengthCm lidar_z_max) {
  const MapBounds b = map.bounds();
  const double xy_step  = decimalPlacesToStep(b.xy_decimal_places);
  const double h_step   = decimalPlacesToStep(b.height_decimal_places);
  const double ray_step = std::min(xy_step, h_step);
  const double z_max_cm = lidar_z_max.numerical_value_in(su::cm);

  for (const LidarHit& h : hits) {
    if (lidarHitIsMiss(h)) {
      for (double t_cm = ray_step; t_cm <= z_max_cm; t_cm += ray_step) {
        LidarHit slice = h;
        slice.distance = t_cm * su::cm;
        map.set(hitToWorldPoint(here, slice), MapValue::Empty);
      }
      continue;
    }

    const double d_cm = h.distance.numerical_value_in(su::cm);
    if (d_cm > 0.0) {
      for (double t_cm = ray_step; t_cm < d_cm; t_cm += ray_step) {
        LidarHit slice = h;
        slice.distance = t_cm * su::cm;
        map.set(hitToWorldPoint(here, slice), MapValue::Empty);
      }
    }
    map.set(hitToWorldPoint(here, h), MapValue::Occupied);
  }
}

}  // namespace

DroneAlgorithm::DroneAlgorithm(ILidarSensor& lidar, IPositionSensor& pos, IMovementDriver& move,
                               IBuildingMap& map, const DroneConfig& cfg)
    : lidar_(lidar),
      pos_(pos),
      move_(move),
      map_(map),
      cfg_(cfg) {}

void DroneAlgorithm::tick() {
  if (finished_) {
    return;
  }

  const DronePosition before = pos_.getPosition();
  applyLidarHitsToMap(map_, before, lidar_.scan(), cfg_.lidar.z_max);

  move_.advance(cfg_.max_advance_per_command);
  const DronePosition after = pos_.getPosition();

  if (same_xy_height(before, after)) {
    ++blocked_advances_;
    if (blocked_advances_ >= 4) {
      finished_ = true;
      return;
    }
    move_.rotate(TurnDirection::Right, 90.0 * su::deg);
  } else {
    blocked_advances_ = 0;
  }
}

}  // namespace dmap
