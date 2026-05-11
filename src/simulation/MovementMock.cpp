// MovementMock.cpp
// Each movement method validates limits then ray-marches along the intended
// path, checking only the leading face of the drone's bounding box at each
// step.  The interior is not re-checked because it was already clear at the
// previous step.

#include "simulation/MovementMock.h"

#include "common/Logger.h"
#include "common/MathUtils.h"
#include "common/Point3D.h"

#include <mp-units/systems/si/unit_symbols.h>
#include <cmath>
#include <functional>

namespace dmap {

namespace su = mp_units::si::unit_symbols;

namespace {

// Shared ray-march logic used by both advance() and elevate().
//
// Steps from |signed_step| to |signed_dist| (exclusive), calling is_blocked()
// at each intermediate position, then checks the final destination.
// If any check returns true the move is logged and rejected (returns false).
// Otherwise the destination is committed to state and true is returned.
//
// signed_step and signed_dist must have the same sign.  This sign encodes
// direction: positive = forward/up, negative = backward/down.
//
// make_pos(d) returns the DronePosition when the drone has travelled d cm
// along the path from its starting position (d is signed).
bool marchAndMove(
    SimulationState& state,
    double signed_step, double signed_dist,
    const std::function<DronePosition(double)>& make_pos,
    const std::function<bool(const DronePosition&)>& is_blocked,
    const char* path_blocked_msg,
    const char* dest_blocked_msg) {
  for (double t = signed_step; std::abs(t) < std::abs(signed_dist); t += signed_step) {
    if (is_blocked(make_pos(t))) {
      log::info(path_blocked_msg);
      return false;
    }
  }
  const DronePosition dest = make_pos(signed_dist);
  if (is_blocked(dest)) {
    log::info(dest_blocked_msg);
    return false;
  }
  state.setDronePosition(dest);
  return true;
}

}  // namespace

MovementMock::MovementMock(SimulationState& state, DroneConfig drone_cfg,
                           const MissionConfig& mission_cfg)
    : state_(state),
      cfg_(drone_cfg),
      step_xy_cm_(decimalPlacesToStep(mission_cfg.xy_decimal_places)),
      step_height_cm_(decimalPlacesToStep(mission_cfg.height_decimal_places)),
      detector_(state, drone_cfg, step_xy_cm_, step_height_cm_) {}

void MovementMock::rotate(TurnDirection direction, AngleDeg angle) {
  const double deg = angle.numerical_value_in(su::deg);
  // The spec allows negative angles: rotate(Left, -30°) == rotate(Right, 30°).
  if (std::abs(deg) > cfg_.max_rotate_per_command.numerical_value_in(su::deg)) {
    log::info("rotate: angle exceeds max_rotate_per_command, move rejected");
    return;
  }
  auto p = state_.dronePosition();
  // Spec angle convention: 0=east, 90=south, 180=west, 270=north (clockwise).
  // Right (clockwise) increases the angle; Left (counterclockwise) decreases it.
  const double sign = (direction == TurnDirection::Right) ? 1.0 : -1.0;
  const double cur = p.xy_angle.numerical_value_in(su::deg);
  p.xy_angle = (cur + sign * deg) * su::deg;
  state_.setDronePosition(p);
}

void MovementMock::advance(LengthCm distance) {
  const double dist_cm = distance.numerical_value_in(su::cm);
  // The spec allows negative distance (backward movement).
  if (std::abs(dist_cm) > cfg_.max_advance_per_command.numerical_value_in(su::cm)) {
    log::info("advance: distance exceeds max_advance_per_command, move rejected");
    return;
  }

  const DronePosition p = state_.dronePosition();
  const double angle_rad = toRad(p.xy_angle.numerical_value_in(su::deg));
  const double dx = std::cos(angle_rad);
  const double dy = std::sin(angle_rad);
  const double x0 = p.x.numerical_value_in(su::cm);
  const double y0 = p.y.numerical_value_in(su::cm);

  // For backward movement the leading face is the rear of the drone.
  // intersectsForwardFace checks +half_length along the heading; flipping the
  // heading 180° makes it probe the opposite (rear) face instead.
  const bool backward = dist_cm < 0.0;
  const double check_angle_deg =
      p.xy_angle.numerical_value_in(su::deg) + (backward ? 180.0 : 0.0);
  const double signed_step = backward ? -step_xy_cm_ : step_xy_cm_;

  // At each step only the leading face (entering new territory) is checked.
  const bool moved = marchAndMove(
      state_, signed_step, dist_cm,
      [&](double d) {
        DronePosition pos = p;
        pos.x = (x0 + d * dx) * su::cm;
        pos.y = (y0 + d * dy) * su::cm;
        pos.xy_angle = check_angle_deg * su::deg;
        return pos;
      },
      [&](const DronePosition& pos) { return detector_.intersectsForwardFace(pos); },
      "advance: path is blocked, move rejected",
      "advance: destination is blocked, move rejected");

  // marchAndMove committed a position with the collision-check heading.
  // Restore the original heading — backward movement does not change facing.
  if (moved && backward) {
    DronePosition fixed = state_.dronePosition();
    fixed.xy_angle = p.xy_angle;
    state_.setDronePosition(fixed);
  }
}

void MovementMock::elevate(LengthCm distance) {
  const double dist_cm = distance.numerical_value_in(su::cm);
  if (std::abs(dist_cm) > cfg_.max_elevate_per_command.numerical_value_in(su::cm)) {
    log::info("elevate: distance exceeds max_elevate_per_command, move rejected");
    return;
  }

  const DronePosition p = state_.dronePosition();
  const double h0   = p.height.numerical_value_in(su::cm);
  const double sign = (dist_cm >= 0.0) ? 1.0 : -1.0;
  const bool upward = (dist_cm >= 0.0);

  // At each step only the leading face (top when going up, bottom when going
  // down) is checked. The interior was already clear at the previous step.
  marchAndMove(
      state_, sign * step_height_cm_, dist_cm,
      [&](double d) {
        DronePosition pos = p;
        pos.height = (h0 + d) * su::cm;
        return pos;
      },
      [&](const DronePosition& pos) { return detector_.intersectsElevateFace(pos, upward); },
      "elevate: path is blocked, move rejected",
      "elevate: destination is blocked, move rejected");
}

}  // namespace dmap
