#include "simulation/LidarMock.h"

#include "common/Point3D.h"
#include "simulation/CollisionDetector.h"

#include <mp-units/systems/si/unit_symbols.h>
#include <cmath>
#include <numbers>

namespace dmap {

namespace su = mp_units::si::unit_symbols;

LidarMock::LidarMock(SimulationState& state, DroneConfig drone_cfg)
    : state_(state), drone_cfg_(drone_cfg), calc_(drone_cfg.lidar) {}

LidarScanResult LidarMock::scan(std::optional<AngleDeg> xy_offset,
                                std::optional<AngleDeg> height_angle) {
  const auto pos = state_.dronePosition();

  // Effective horizontal heading = drone heading + optional offset.
  const double offset_deg =
      xy_offset.has_value() ? xy_offset->numerical_value_in(su::deg) : 0.0;
  const double h_rad =
      (pos.xy_angle.numerical_value_in(su::deg) + offset_deg) *
      (std::numbers::pi / 180.0);

  // Optional vertical tilt applied to the entire cone before heading rotation.
  // Positive = tilts beams upward.  Rotates around the local +y axis.
  const double tilt_rad =
      height_angle.has_value()
          ? height_angle->numerical_value_in(su::deg) * (std::numbers::pi / 180.0)
          : 0.0;

  const double ox = pos.x.numerical_value_in(su::cm);
  const double oy = pos.y.numerical_value_in(su::cm);
  const double oz = pos.height.numerical_value_in(su::cm);

  const double z_max_cm = drone_cfg_.lidar.z_max.numerical_value_in(su::cm);
  const double step      = stepCm();

  CollisionDetector det(state_);
  LidarScanResult result;

  for (const auto& ray : calc_.unitDirectionsForScan()) {
    // 1. Apply vertical tilt (rotation around local +y, positive = beam nose up).
    //    lx' = lx*cos(tilt) - lz*sin(tilt)
    //    ly' = ly               (unchanged)
    //    lz' = lx*sin(tilt) + lz*cos(tilt)
    const double tx = ray.x * std::cos(tilt_rad) - ray.z * std::sin(tilt_rad);
    const double ty = ray.y;
    const double tz = ray.x * std::sin(tilt_rad) + ray.z * std::cos(tilt_rad);

    // 2. Rotate from local frame to world frame (rotation around Z by heading).
    //    local +x maps to world (cos h, sin h, 0)  — drone's forward direction
    //    local +y maps to world (-sin h, cos h, 0) — drone's left direction
    const double wx = tx * std::cos(h_rad) - ty * std::sin(h_rad);
    const double wy = tx * std::sin(h_rad) + ty * std::cos(h_rad);
    const double wz = tz;

    // World-space azimuth and elevation of this beam (used in the hit record).
    const double azimuth_deg =
        std::atan2(wy, wx) * (180.0 / std::numbers::pi);
    const double elevation_deg =
        std::atan2(wz, std::sqrt(wx * wx + wy * wy)) * (180.0 / std::numbers::pi);

    // 3. March along the ray, first-hit-only, up to Z_max.
    for (double t = step; t <= z_max_cm; t += step) {
      const Point3D sample{(ox + t * wx) * su::cm,
                           (oy + t * wy) * su::cm,
                           (oz + t * wz) * su::cm};
      if (det.intersectsOccupied(sample)) {
        result.push_back({azimuth_deg * su::deg,
                          elevation_deg * su::deg,
                          t * su::cm});
        break;  // first hit only — beam stops at the wall
      }
    }
  }

  return result;
}

double LidarMock::stepCm() const {
  if (!state_.hasBounds()) return 1.0;
  const auto& b = state_.mapBounds();
  const double xy_step = std::pow(10.0, -static_cast<double>(b.xy_decimal_places));
  const double h_step  = std::pow(10.0, -static_cast<double>(b.height_decimal_places));
  return std::min(xy_step, h_step);
}

}  // namespace dmap
