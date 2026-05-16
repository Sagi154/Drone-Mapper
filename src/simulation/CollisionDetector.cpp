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
#include <algorithm>
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

// True iff offset (dx,dy,dz) from the sphere centre lies inside the sphere
// (distance ≤ r) and the corresponding world cell is Occupied in the truth map.
bool mapHitInsideSphere(const SimulationState& state, const SphereGeom& g,
                        double dx, double dy, double dz, double r2) {
  if (dx * dx + dy * dy + dz * dz > r2) return false;
  const Point3D sample{(g.cx + dx) * su::cm, (g.cy + dy) * su::cm,
                       (g.ch + dz) * su::cm};
  return state.truthValue(sample) == MapValue::Occupied;
}

// Computes the first and last z-indices whose offset dz = -r + iz*step_h satisfies
// dz >= 0 (upward) or dz <= 0 (downward).  Returns the full [0, nh) range when
// called with upward=std::nullopt — used so all three public methods share one loop.
std::pair<int,int> zBounds(double r, double step_h, int nh, bool upward, bool fullRange) {
  if (fullRange) return {0, nh};
  if (upward) {
    // dz >= 0  ⟺  iz >= r/step_h
    const int iz0 = std::clamp(static_cast<int>(std::ceil(r / step_h - 1e-12)), 0, nh);
    return {iz0, nh};
  } else {
    // dz <= 0  ⟺  iz <= floor(r/step_h)
    const int iz1 = std::clamp(static_cast<int>(std::floor(r / step_h + 1e-12)) + 1, 0, nh);
    return {0, iz1};
  }
}

// Single shared loop used by all three public footprint-check methods.
//
// `iz0`/`iz1`   — z-index range to iterate; pass [0, nh) for a full sweep,
//                 or a tighter range to restrict to a vertical hemisphere.
// `xySkip(dx,dy)` — called once per (dx,dy) column, before the z-loop.
//                   Return true to skip the entire column (used for the forward
//                   hemisphere filter).  Pass `[]{return false;}` to disable.
//
// For each (dx, dy) column that passes xySkip, the z-loop runs from iz0 to iz1.
// For each sample in that range, the sphere distance is checked and then the
// truth map is queried.  Returns true on the first Occupied hit.
template <typename XYFilter>
bool sampleSphere(const SimulationState& state, const SphereGeom& g,
                  double step_xy_cm, double step_height_cm,
                  int iz0, int iz1,
                  XYFilter xySkip) {
  // r² — compare squared distances to avoid sqrt per sample.
  const double r2 = g.radius * g.radius;
  // Grid count along x and y: same step size in both horizontal axes.
  const int nxy = nSteps(g.radius, step_xy_cm);

  // Outer loop: x index from first sample (-r) through last sample (+r).
  for (int ix = 0; ix < nxy; ++ix) {
    // dx = x offset from sphere centre in centimetres.
    const double dx = offset(ix, g.radius, step_xy_cm);
    for (int iy = 0; iy < nxy; ++iy) {
      const double dy = offset(iy, g.radius, step_xy_cm);
      // XY filter: skip the entire vertical column if the caller says to.
      // For intersectsForwardFace this discards rear-hemisphere columns
      // without paying for any z iterations.
      if (xySkip(dx, dy)) continue;
      // Z loop runs only over iz0..iz1 (full range or one vertical hemisphere).
      for (int iz = iz0; iz < iz1; ++iz) {
        const double dz = offset(iz, g.radius, step_height_cm);
        // Sphere distance check + truth map lookup; returns true only if Occupied.
        if (mapHitInsideSphere(state, g, dx, dy, dz, r2)) return true;
      }
    }
  }
  return false;
}

}  // namespace

bool CollisionDetector::intersectsFootprint(const DronePosition& pos) const {
  const auto g  = makeGeom(pos, drone_cfg_);
  const int  nh = nSteps(g.radius, step_height_cm_);
  const auto [iz0, iz1] = zBounds(g.radius, step_height_cm_, nh, false, true);
  return sampleSphere(state_, g, step_xy_cm_, step_height_cm_, iz0, iz1,
                      [](double, double) { return false; });
}

bool CollisionDetector::intersectsForwardFace(const DronePosition& pos) const {
  const auto g  = makeGeom(pos, drone_cfg_);
  const int  nh = nSteps(g.radius, step_height_cm_);
  const auto [iz0, iz1] = zBounds(g.radius, step_height_cm_, nh, false, true);
  // XY filter: skip rear-hemisphere columns (dot product < 0) before the z-loop.
  return sampleSphere(state_, g, step_xy_cm_, step_height_cm_, iz0, iz1,
                      [&g](double dx, double dy) {
                        return dx * g.fwd_x + dy * g.fwd_y < 0.0;
                      });
}

bool CollisionDetector::intersectsElevateFace(const DronePosition& pos,
                                              bool upward) const {
  const auto g  = makeGeom(pos, drone_cfg_);
  const int  nh = nSteps(g.radius, step_height_cm_);
  // z-bounds restrict the loop to the leading vertical hemisphere.
  const auto [iz0, iz1] = zBounds(g.radius, step_height_cm_, nh, upward, false);
  return sampleSphere(state_, g, step_xy_cm_, step_height_cm_, iz0, iz1,
                      [](double, double) { return false; });
}

}  // namespace dmap
