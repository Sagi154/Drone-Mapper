// CollisionDetector.cpp
// Implements footprint sampling: the drone is treated as a perfect sphere of
// radius min_passable_radius.  A bounding cube of side 2r is subdivided into
// a grid of sample points at the configured cell resolution; only points whose
// Euclidean distance from the centre is ≤ r are tested against the map.
// Because a sphere has no preferred orientation the heading is never needed for
// the full-footprint check.  For the directional half-sphere checks
// (forward-face, elevate-face) the heading is still used to select which half
// of the sphere to test.

#include "simulation/CollisionDetector.h"
#include "simulation/SimulationState.h"

#include "common/MathUtils.h"
#include "mapping/MapTypes.h"

#include <mp-units/systems/si/unit_symbols.h>
#include <cmath>

namespace dmap {

namespace su = mp_units::si::unit_symbols;

CollisionDetector::CollisionDetector(const SimulationState& state)
    : state_(state) {}

CollisionDetector::CollisionDetector(const SimulationState& state,
                                     DroneConfig drone_cfg,
                                     double step_xy_cm,
                                     double step_height_cm)
    : state_(state),
      drone_cfg_(drone_cfg),
      step_xy_cm_(step_xy_cm),
      step_height_cm_(step_height_cm) {}

bool CollisionDetector::intersectsOccupied(const Point3D& at) const {
  return state_.truthValue(at) == MapValue::Occupied;
}

// Shared helpers used by all three footprint-check methods.
namespace {

// Pre-computed sphere geometry for the drone at a given pose.
// cx/cy/ch = sphere centre in world coordinates.
// fwd_x/fwd_y = heading unit vector (needed for half-sphere direction tests).
// radius = min_passable_radius from DroneConfig.
struct SphereGeom {
  double cx, cy, ch;
  double fwd_x, fwd_y;
  double radius;
};

// Builds SphereGeom from the drone's current position and config.
SphereGeom makeGeom(const dmap::DronePosition& pos,
                    const dmap::DroneConfig& cfg) {
  namespace su = mp_units::si::unit_symbols;
  const double angle_rad = toRad(pos.xy_angle.numerical_value_in(su::deg));
  return {
      pos.x.numerical_value_in(su::cm),
      pos.y.numerical_value_in(su::cm),
      pos.height.numerical_value_in(su::cm),
      std::cos(angle_rad),
      std::sin(angle_rad),
      cfg.min_passable_radius.numerical_value_in(su::cm),
  };
}

// Number of sample points needed to cover [-r, +r] in steps of `step`.
// ceil ensures the span is always rounded OUTWARD: the last sample lands at or
// beyond +r, so the outermost grid cell the drone physically overlaps is
// never skipped.  Using round instead could truncate the span inward and miss
// a wall cell that the drone's surface actually touches.
// Each offset is computed from the index (not by repeated addition) to avoid
// floating-point accumulation.
int nSteps(double r, double step) {
  return static_cast<int>(std::ceil(2.0 * r / step)) + 1;
}

// i-th offset value in [-r, +r], computed exactly from the index.
double offset(int i, double r, double step) {
  return -r + i * step;
}

}  // namespace

bool CollisionDetector::intersectsFootprint(const DronePosition& pos) const {
  const auto g = makeGeom(pos, drone_cfg_);
  const double r2 = g.radius * g.radius;
  const int nxy = nSteps(g.radius, step_xy_cm_);
  const int nh  = nSteps(g.radius, step_height_cm_);
  for (int ix = 0; ix < nxy; ++ix) {
    const double dx = offset(ix, g.radius, step_xy_cm_);
    for (int iy = 0; iy < nxy; ++iy) {
      const double dy = offset(iy, g.radius, step_xy_cm_);
      for (int iz = 0; iz < nh; ++iz) {
        const double dz = offset(iz, g.radius, step_height_cm_);
        // Sphere test: skip corners of the bounding cube outside the sphere.
        if (dx * dx + dy * dy + dz * dz > r2) continue;
        const Point3D sample{(g.cx + dx) * su::cm,
                             (g.cy + dy) * su::cm,
                             (g.ch + dz) * su::cm};
        if (intersectsOccupied(sample)) return true;
      }
    }
  }
  return false;
}

bool CollisionDetector::intersectsForwardFace(const DronePosition& pos) const {
  const auto g = makeGeom(pos, drone_cfg_);
  const double r2 = g.radius * g.radius;
  const int nxy = nSteps(g.radius, step_xy_cm_);
  const int nh  = nSteps(g.radius, step_height_cm_);
  for (int ix = 0; ix < nxy; ++ix) {
    const double dx = offset(ix, g.radius, step_xy_cm_);
    for (int iy = 0; iy < nxy; ++iy) {
      const double dy = offset(iy, g.radius, step_xy_cm_);
      // Forward hemisphere: dot product of (dx, dy) with heading must be ≥ 0.
      if (dx * g.fwd_x + dy * g.fwd_y < 0.0) continue;
      for (int iz = 0; iz < nh; ++iz) {
        const double dz = offset(iz, g.radius, step_height_cm_);
        if (dx * dx + dy * dy + dz * dz > r2) continue;
        const Point3D sample{(g.cx + dx) * su::cm,
                             (g.cy + dy) * su::cm,
                             (g.ch + dz) * su::cm};
        if (intersectsOccupied(sample)) return true;
      }
    }
  }
  return false;
}

bool CollisionDetector::intersectsElevateFace(const DronePosition& pos,
                                              bool upward) const {
  const auto g = makeGeom(pos, drone_cfg_);
  const double r2 = g.radius * g.radius;
  const int nxy = nSteps(g.radius, step_xy_cm_);
  const int nh  = nSteps(g.radius, step_height_cm_);
  for (int ix = 0; ix < nxy; ++ix) {
    const double dx = offset(ix, g.radius, step_xy_cm_);
    for (int iy = 0; iy < nxy; ++iy) {
      const double dy = offset(iy, g.radius, step_xy_cm_);
      for (int iz = 0; iz < nh; ++iz) {
        const double dz = offset(iz, g.radius, step_height_cm_);
        // Leading hemisphere: top cap (dz ≥ 0) when rising, bottom (dz ≤ 0) when sinking.
        if (upward ? (dz < 0.0) : (dz > 0.0)) continue;
        if (dx * dx + dy * dy + dz * dz > r2) continue;
        const Point3D sample{(g.cx + dx) * su::cm,
                             (g.cy + dy) * su::cm,
                             (g.ch + dz) * su::cm};
        if (intersectsOccupied(sample)) return true;
      }
    }
  }
  return false;
}

}  // namespace dmap
