#include "simulation/MovementMock.h"

#include "common/Logger.h"
#include "common/Point3D.h"

#include <mp-units/systems/si/unit_symbols.h>
#include <cmath>
#include <numbers>

namespace dmap {

namespace su = mp_units::si::unit_symbols;

MovementMock::MovementMock(SimulationState& state, DroneConfig drone_cfg,
                           const MissionConfig& mission_cfg)
    : state_(state),
      cfg_(drone_cfg),
      step_xy_cm_(std::pow(10.0, -static_cast<double>(mission_cfg.xy_decimal_places))),
      step_height_cm_(std::pow(10.0, -static_cast<double>(mission_cfg.height_decimal_places))),
      detector_(state, drone_cfg, step_xy_cm_, step_height_cm_) {}

void MovementMock::rotate(TurnDirection direction, AngleDeg angle) {
  const double deg = angle.numerical_value_in(su::deg);
  if (deg < 0.0 || deg > cfg_.max_rotate_per_command.numerical_value_in(su::deg)) {
    log::info("rotate: angle out of range, move rejected");
    return;
  }
  auto p = state_.dronePosition();
  const double sign = (direction == TurnDirection::Left) ? 1.0 : -1.0;
  const double cur = p.xy_angle.numerical_value_in(su::deg);
  p.xy_angle = (cur + sign * deg) * su::deg;
  state_.setDronePosition(p);
}

void MovementMock::advance(LengthCm distance) {
  const double dist_cm = distance.numerical_value_in(su::cm);
  if (dist_cm < 0.0) {
    log::info("advance: negative distance rejected");
    return;
  }
  if (dist_cm > cfg_.max_advance_per_command.numerical_value_in(su::cm)) {
    log::info("advance: distance exceeds max_advance_per_command, move rejected");
    return;
  }

  auto p = state_.dronePosition();
  const double angle_rad =
      p.xy_angle.numerical_value_in(su::deg) * (std::numbers::pi / 180.0);
  const double dx = std::cos(angle_rad);
  const double dy = std::sin(angle_rad);
  const double x0 = p.x.numerical_value_in(su::cm);
  const double y0 = p.y.numerical_value_in(su::cm);
  const double h   = p.height.numerical_value_in(su::cm);

  // At each step only the forward face (the slice entering new territory) is
  // checked. The interior of the box was already clear at the previous step.
  DronePosition sample = p;
  for (double travelled = step_xy_cm_; travelled < dist_cm; travelled += step_xy_cm_) {
    sample.x = (x0 + travelled * dx) * su::cm;
    sample.y = (y0 + travelled * dy) * su::cm;
    if (detector_.intersectsForwardFace(sample)) {
      log::info("advance: path is blocked, move rejected");
      return;
    }
  }
  DronePosition dest = p;
  dest.x = (x0 + dist_cm * dx) * su::cm;
  dest.y = (y0 + dist_cm * dy) * su::cm;
  if (detector_.intersectsForwardFace(dest)) {
    log::info("advance: destination is blocked, move rejected");
    return;
  }

  state_.setDronePosition(dest);
}

void MovementMock::elevate(LengthCm distance) {
  const double dist_cm = distance.numerical_value_in(su::cm);
  if (std::abs(dist_cm) > cfg_.max_elevate_per_command.numerical_value_in(su::cm)) {
    log::info("elevate: distance exceeds max_elevate_per_command, move rejected");
    return;
  }

  auto p = state_.dronePosition();
  const double x   = p.x.numerical_value_in(su::cm);
  const double y   = p.y.numerical_value_in(su::cm);
  const double h0  = p.height.numerical_value_in(su::cm);
  const double sign = (dist_cm >= 0.0) ? 1.0 : -1.0;
  const double abs_dist = std::abs(dist_cm);

  // At each step only the leading face (top when going up, bottom when going
  // down) is checked. The interior was already clear at the previous step.
  const bool upward = (dist_cm >= 0.0);
  DronePosition sample = p;
  for (double travelled = step_height_cm_; travelled < abs_dist; travelled += step_height_cm_) {
    sample.height = (h0 + sign * travelled) * su::cm;
    if (detector_.intersectsElevateFace(sample, upward)) {
      log::info("elevate: path is blocked, move rejected");
      return;
    }
  }
  DronePosition dest = p;
  dest.height = (h0 + dist_cm) * su::cm;
  if (detector_.intersectsElevateFace(dest, upward)) {
    log::info("elevate: destination is blocked, move rejected");
    return;
  }

  state_.setDronePosition(dest);
}

}  // namespace dmap
