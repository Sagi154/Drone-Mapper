#include "simulation/CollisionDetector.h"
#include "simulation/SimulationState.h"

#include "mapping/MapTypes.h"

#include <mp-units/systems/si/unit_symbols.h>
#include <cmath>
#include <numbers>

namespace dmap {

namespace su = mp_units::si::unit_symbols;

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

// Shared helpers for all three footprint checks.
namespace {

struct FootprintGeom {
  double cx, cy, ch;
  double fwd_x, fwd_y;
  double rgt_x, rgt_y;
  double half_l, half_w, half_h;
};

FootprintGeom makeGeom(const dmap::DronePosition& pos,
                       const dmap::DroneConfig& cfg) {
  namespace su = mp_units::si::unit_symbols;
  const double angle_rad =
      pos.xy_angle.numerical_value_in(su::deg) * (std::numbers::pi / 180.0);
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
// Using round avoids floating-point accumulation: each offset is computed as
// (-half + i * step), not by repeated addition.
int nSteps(double half, double step) {
  return static_cast<int>(std::round(2.0 * half / step)) + 1;
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
