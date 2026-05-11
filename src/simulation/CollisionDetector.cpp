// CollisionDetector.cpp
// Implements footprint sampling: the drone's oriented bounding box is
// subdivided into a grid of sample points at the configured cell resolution,
// and each point is tested against the ground-truth map.

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

// Pre-computed geometry of the drone's bounding box in world coordinates.
// cx/cy/ch = drone centre; fwd/rgt = unit vectors along the heading and
// its perpendicular; half_l/w/h = half the drone dimensions in each axis.
struct FootprintGeom {
  double cx, cy, ch;
  double fwd_x, fwd_y;
  double rgt_x, rgt_y;
  double half_l, half_w, half_h;
};

// Builds FootprintGeom from the drone's current position and config.
FootprintGeom makeGeom(const dmap::DronePosition& pos,
                       const dmap::DroneConfig& cfg) {
  namespace su = mp_units::si::unit_symbols;
  const double angle_rad = toRad(pos.xy_angle.numerical_value_in(su::deg));
  return {
      pos.x.numerical_value_in(su::cm),
      pos.y.numerical_value_in(su::cm),
      pos.height.numerical_value_in(su::cm),
      std::cos(angle_rad),   std::sin(angle_rad),
      std::sin(angle_rad),  -std::cos(angle_rad),
      cfg.min_passable_length.numerical_value_in(su::cm) / 2.0,
      cfg.min_passable_width.numerical_value_in(su::cm)  / 2.0,
      cfg.min_passable_height.numerical_value_in(su::cm) / 2.0,
  };
}

// Number of sample points needed to cover [-half, +half] in steps of `step`.
// ceil ensures the span is always rounded OUTWARD: the last sample lands at or
// beyond +half, so the outermost grid cell the drone physically overlaps is
// never skipped.  Using round instead could truncate the span inward and miss
// a wall cell that the drone's edge actually touches.
// Each offset is computed from the index (not by repeated addition) to avoid
// floating-point accumulation.
int nSteps(double half, double step) {
  return static_cast<int>(std::ceil(2.0 * half / step)) + 1;
}

// i-th offset value in [-half, +half], computed exactly from the index.
double offset(int i, double half, double step) {
  return -half + i * step;
}

}  // namespace

bool CollisionDetector::intersectsFootprint(const DronePosition& pos) const {
  const auto g = makeGeom(pos, drone_cfg_);
  const int nl = nSteps(g.half_l, step_xy_cm_);
  const int nw = nSteps(g.half_w, step_xy_cm_);
  const int nh = nSteps(g.half_h, step_height_cm_);
  for (int il = 0; il < nl; ++il) {
    const double dl = offset(il, g.half_l, step_xy_cm_);
    for (int iw = 0; iw < nw; ++iw) {
      const double dw = offset(iw, g.half_w, step_xy_cm_);
      for (int ih = 0; ih < nh; ++ih) {
        const double dh = offset(ih, g.half_h, step_height_cm_);
        const Point3D sample{(g.cx + dl * g.fwd_x + dw * g.rgt_x) * su::cm,
                             (g.cy + dl * g.fwd_y + dw * g.rgt_y) * su::cm,
                             (g.ch + dh) * su::cm};
        if (intersectsOccupied(sample)) return true;
      }
    }
  }
  return false;
}

bool CollisionDetector::intersectsForwardFace(const DronePosition& pos) const {
  const auto g = makeGeom(pos, drone_cfg_);
  const double dl = g.half_l;  // front face only
  const int nw = nSteps(g.half_w, step_xy_cm_);
  const int nh = nSteps(g.half_h, step_height_cm_);
  for (int iw = 0; iw < nw; ++iw) {
    const double dw = offset(iw, g.half_w, step_xy_cm_);
    for (int ih = 0; ih < nh; ++ih) {
      const double dh = offset(ih, g.half_h, step_height_cm_);
      const Point3D sample{(g.cx + dl * g.fwd_x + dw * g.rgt_x) * su::cm,
                           (g.cy + dl * g.fwd_y + dw * g.rgt_y) * su::cm,
                           (g.ch + dh) * su::cm};
      if (intersectsOccupied(sample)) return true;
    }
  }
  return false;
}

bool CollisionDetector::intersectsElevateFace(const DronePosition& pos,
                                              bool upward) const {
  const auto g = makeGeom(pos, drone_cfg_);
  const double dh = upward ? g.half_h : -g.half_h;  // top or bottom face only
  const int nl = nSteps(g.half_l, step_xy_cm_);
  const int nw = nSteps(g.half_w, step_xy_cm_);
  for (int il = 0; il < nl; ++il) {
    const double dl = offset(il, g.half_l, step_xy_cm_);
    for (int iw = 0; iw < nw; ++iw) {
      const double dw = offset(iw, g.half_w, step_xy_cm_);
      const Point3D sample{(g.cx + dl * g.fwd_x + dw * g.rgt_x) * su::cm,
                           (g.cy + dl * g.fwd_y + dw * g.rgt_y) * su::cm,
                           (g.ch + dh) * su::cm};
      if (intersectsOccupied(sample)) return true;
    }
  }
  return false;
}

}  // namespace dmap
