// DroneAlgorithm.cpp
// Frontier-based BFS exploration algorithm.
// Three-phase state machine per tick:
//   SCANNING — SphericalScanner::scan() fires a full spherical lidar sweep and
//              fuses results into the drone's map.
//   PLANNING — ExplorationFrontier::findPath() runs BFS through sphere-safe
//              empty cells to find the nearest frontier and the path to it.
//   MOVING   — executeNextStep() rotates to face the next waypoint, then
//              advances or elevates one grid step toward it.
// The algorithm is deterministic: scan pattern and BFS neighbour order are fixed.

#include "algorithm/DroneAlgorithm.h"

#include "algorithm/ExplorationFrontier.h"
#include "common/MathUtils.h"
#include "mapping/MapTypes.h"

#include <cmath>
#include <numbers>
#include <mp-units/systems/si/unit_symbols.h>

namespace dmap {

namespace {

namespace su = mp_units::si::unit_symbols;

/// Returns true if the drone centre `pos` is within half a grid step of
/// `target` in every axis, meaning the waypoint has been reached.
bool reachedWaypoint(const DronePosition& pos, const Point3D& target,
                     double xy_step, double h_step) {
  const double dx = std::abs(pos.x.numerical_value_in(su::cm) -
                             target.x.numerical_value_in(su::cm));
  const double dy = std::abs(pos.y.numerical_value_in(su::cm) -
                             target.y.numerical_value_in(su::cm));
  const double dh = std::abs(pos.height.numerical_value_in(su::cm) -
                             target.height.numerical_value_in(su::cm));
  return dx <= xy_step * 0.5 && dy <= xy_step * 0.5 && dh <= h_step * 0.5;
}

}  // namespace

DroneAlgorithm::DroneAlgorithm(ILidarSensor& lidar, IPositionSensor& pos, IMovementDriver& move,
                               IBuildingMap& map, const DroneConfig& cfg)
    : pos_(pos),
      move_(move),
      map_(map),
      cfg_(cfg),
      scanner_(lidar, pos, cfg.lidar) {}

void DroneAlgorithm::tick() {
  if (finished_) {
    return;
  }

  const MapBounds b = map_.bounds();
  const double xy_step = decimalPlacesToStep(b.xy_decimal_places);
  const double h_step  = decimalPlacesToStep(b.height_decimal_places);

  switch (phase_) {
    case Phase::Scanning:
      scanner_.scan(map_);
      phase_ = Phase::Planning;
      break;

    case Phase::Planning: {
      const DronePosition p = pos_.getPosition();
      const Point3D here{p.x, p.y, p.height};
      const PathResult result = frontier_.findPath(map_, here, cfg_.min_passable_radius);
      if (!result.found) {
        finished_ = true;
        return;
      }
      current_path_ = result.path;
      path_index_   = 0;
      phase_        = Phase::Moving;
      break;
    }

    case Phase::Moving: {
      executeNextStep();

      // Check whether the drone has reached the current waypoint.
      const DronePosition pos = pos_.getPosition();
      if (reachedWaypoint(pos, current_path_[path_index_], xy_step, h_step)) {
        ++path_index_;
        if (path_index_ >= current_path_.size()) {
          // Arrived at frontier — scan from the new position.
          phase_ = Phase::Scanning;
        }
      }
      break;
    }
  }
}

void DroneAlgorithm::executeNextStep() {
  // Issues one movement command toward current_path_[path_index_].
  // Priority: elevate first (if height differs), then rotate to face the
  // target, then advance.  Because BFS steps are axis-aligned, each waypoint
  // differs in at most one axis, so at most two ticks reach a waypoint
  // (one rotate + one advance, or one elevate).

  const DronePosition pos = pos_.getPosition();
  const Point3D& target   = current_path_[path_index_];

  // --- Height difference: elevate toward target height ---
  const double dh = target.height.numerical_value_in(su::cm) -
                    pos.height.numerical_value_in(su::cm);
  if (std::abs(dh) > 1e-6) {
    const double limit = cfg_.max_elevate_per_command.numerical_value_in(su::cm);
    move_.elevate(std::clamp(dh, -limit, limit) * su::cm);
    return;
  }

  // --- Horizontal step: rotate to face target, then advance ---
  const double dx = target.x.numerical_value_in(su::cm) -
                    pos.x.numerical_value_in(su::cm);
  const double dy = target.y.numerical_value_in(su::cm) -
                    pos.y.numerical_value_in(su::cm);

  if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {
    return;
  }

  // Target heading in degrees; atan2 convention matches MovementMock
  // (angle 0 = +X east, 90 = +Y south, clockwise increasing).
  const double target_heading = std::atan2(dy, dx) * (180.0 / std::numbers::pi);
  const double current_heading = pos.xy_angle.numerical_value_in(su::deg);

  // Shortest angular delta, normalised to (-180°, 180°].
  double delta = std::fmod(target_heading - current_heading, 360.0);
  if (delta > 180.0)  delta -= 360.0;
  if (delta < -180.0) delta += 360.0;

  const double rot_limit = cfg_.max_rotate_per_command.numerical_value_in(su::deg);

  if (std::abs(delta) > 1e-6) {
    const TurnDirection dir = (delta > 0.0) ? TurnDirection::Right : TurnDirection::Left;
    move_.rotate(dir, std::min(std::abs(delta), rot_limit) * su::deg);
    return;
  }

  move_.advance(cfg_.max_advance_per_command);
}

}  // namespace dmap
