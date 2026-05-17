// SphericalScanner.cpp
// Implements the full spherical lidar sweep used by DroneAlgorithm's Scanning
// phase.  Two responsibilities live here:
//   1. applyLidarHitsToMap — translates one scan result into Empty/Occupied
//      cells in the drone's map by ray-marching along each beam.
//   2. SphericalScanner::scan — sweeps all elevation tiers and azimuth steps,
//      calling applyLidarHitsToMap for each shot.

#include "algorithm/SphericalScanner.h"

#include "common/MathUtils.h"
#include "mapping/MapTypes.h"
#include "sensors/LidarTypes.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>
#include <mp-units/systems/si/unit_symbols.h>

namespace dmap {

namespace {

namespace su = mp_units::si::unit_symbols;

/// Marks cells along each lidar beam as Empty (up to hit or z_max) and the
/// hit cell as Occupied. Miss beams (distance == -1) mark the full ray Empty.
/// Distance == 0 (too close to measure) marks only the hit cell Occupied.
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

SphericalScanner::SphericalScanner(ILidarSensor& lidar, IPositionSensor& pos,
                                   const LidarConfig& lidar_cfg)
    : lidar_(lidar), pos_(pos), lidar_cfg_(lidar_cfg) {}

void SphericalScanner::scan(IBuildingMap& map) const {
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
  const MapBounds b = map.bounds();

  const double cell_cm = std::min(decimalPlacesToStep(b.xy_decimal_places),
                                  decimalPlacesToStep(b.height_decimal_places));
  const double z_min_cm = lidar_cfg_.z_min.numerical_value_in(su::cm);
  const double denom = (z_min_cm > 0.0) ? z_min_cm : 1.0;
  const double el_step = std::atan(cell_cm / denom) * (180.0 / std::numbers::pi);

  // Include exact horizontal so thin walls at the drone's height are not skipped
  // between coarse elevation tiers.
  std::vector<double> elevations;
  elevations.reserve(static_cast<std::size_t>((180.0 / el_step) + 3.0));
  for (double el = -90.0; el <= 90.0 + 1e-9; el += el_step) {
    elevations.push_back(std::clamp(el, -90.0, 90.0));
  }
  if (std::find_if(elevations.begin(), elevations.end(),
                   [](double el) { return std::abs(el) < 1e-6; }) == elevations.end()) {
    elevations.push_back(0.0);
  }
  std::sort(elevations.begin(), elevations.end());

  for (const double el_clamped : elevations) {
    const double cos_el = std::cos(el_clamped * (std::numbers::pi / 180.0));
    const double az_step = (cos_el > 1e-6) ? (el_step / cos_el) : 360.0;

    for (double az = 0.0; az < 360.0; az += az_step) {
      applyLidarHitsToMap(map, here,
                          lidar_.scan(az * su::deg, el_clamped * su::deg),
                          lidar_cfg_.z_max);
    }
  }
}

}  // namespace dmap
