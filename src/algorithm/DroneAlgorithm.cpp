// DroneAlgorithm.cpp
// Frontier-based BFS exploration algorithm.
// Three-phase state machine per tick:
//   SCANNING — fullScan() fires a full spherical lidar sweep and fuses results
//              into the drone's map so all obstacles nearby are confirmed.
//   PLANNING — ExplorationFrontier::findPath() runs BFS through sphere-safe
//              empty cells to find the nearest frontier and the path to it.
//   MOVING   — executeNextStep() rotates to face the next waypoint, then
//              advances or elevates one grid step toward it.
// The algorithm is deterministic: scan pattern and BFS neighbour order are fixed.

#include "algorithm/DroneAlgorithm.h"

#include "common/MathUtils.h"
#include "mapping/MapTypes.h"
#include "sensors/LidarTypes.h"

#include <algorithm>
#include <cmath>
#include <numbers>
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

void DroneAlgorithm::fullScan() {
  // Full spherical sweep from the current position (position fixed for all shots).
  //
  // Step sizing (so no grid cell is left between adjacent aim directions at z_min):
  //   cell_cm  — smallest map cell edge (cm), min of XY and height quantization
  //   denom    — z_min in cm, or 1 if z_min is zero (avoids divide-by-zero)
  //   el_step  — base angular step (deg): atan(cell_cm / denom)
  //
  // Per elevation tier (el from -90° to +90°):
  //   el_clamped — elevation sent to scan(); last tier clamped to ±90°
  //   cos_el     — cos(el); latitude circle radius shrinks toward the poles
  //   az_step    — azimuth step at this tier: el_step/cos_el, or 360° at a pole
  const DronePosition here = pos_.getPosition();
  const MapBounds b = map_.bounds();

  const double cell_cm = std::min(decimalPlacesToStep(b.xy_decimal_places),
                                  decimalPlacesToStep(b.height_decimal_places));
  const double z_min_cm = cfg_.lidar.z_min.numerical_value_in(su::cm);
  const double denom = (z_min_cm > 0.0) ? z_min_cm : 1.0;
  const double el_step = std::atan(cell_cm / denom) * (180.0 / std::numbers::pi);

  // Full spherical sweep: outer loop over elevation (-90° to +90°), inner
  // loop over azimuth.  Near the poles the latitude circle shrinks, so the
  // azimuth step is widened proportionally (az_step = el_step / cos(el)).
  // When cos(el) ≈ 0 (within el_step of a pole) one azimuth scan suffices.
  for (double el = -90.0; el <= 90.0 + 1e-9; el += el_step) {
    const double el_clamped = std::clamp(el, -90.0, 90.0);
    const double cos_el = std::cos(el_clamped * (std::numbers::pi / 180.0));
    const double az_step = (cos_el > 1e-6) ? (el_step / cos_el) : 360.0;

    for (double az = 0.0; az < 360.0; az += az_step) {
      applyLidarHitsToMap(map_, here,
                          lidar_.scan(az * su::deg, el_clamped * su::deg),
                          cfg_.lidar.z_max);
    }
  }
}

void DroneAlgorithm::tick() {
  if (finished_) {
    return;
  }

  const DronePosition before = pos_.getPosition();
  fullScan();

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
