#pragma once

#include "algorithm/ExplorationFrontier.h"
#include "algorithm/SphericalScanner.h"
#include "common/Point3D.h"
#include "config/DroneConfig.h"
#include "drivers/IMovementDriver.h"
#include "mapping/IBuildingMap.h"
#include "sensors/IPositionSensor.h"

#include <cstddef>
#include <vector>

namespace dmap {

/// Autonomous frontier-based BFS exploration algorithm.
/// Each tick advances one phase of the state machine:
///   Scanning — full spherical lidar sweep fused into the drone's map.
///   Planning — BFS finds the nearest reachable frontier and the path to it.
///   Moving   — executes one waypoint step along the planned path.
/// Accepts the full DroneConfig for capability parameters (advance step,
/// lidar range, collision radius).
class DroneAlgorithm {
 public:
  DroneAlgorithm(ILidarSensor& lidar, IPositionSensor& pos, IMovementDriver& move,
                 IBuildingMap& map, const DroneConfig& cfg);

  /// Advances the exploration by one phase step (scan, plan, or move).
  void tick();

  /// True when exploration is complete (no reachable frontiers remain).
  [[nodiscard]] bool isFinished() const noexcept { return finished_; }

 private:
  IPositionSensor& pos_;
  IMovementDriver& move_;
  IBuildingMap& map_;
  DroneConfig cfg_{};

  enum class Phase { Scanning, Planning, Moving };

  Phase phase_{Phase::Scanning};
  SphericalScanner scanner_;             ///< Performs full spherical lidar sweeps.
  ExplorationFrontier frontier_{};
  std::vector<Point3D> current_path_{};  ///< Waypoints to the current frontier.
  std::size_t path_index_{0};            ///< Index of the next waypoint to reach.
  bool finished_{false};

  /// Issues one movement command toward the current waypoint
  /// (current_path_[path_index_]): rotates to face it, then advances or
  /// elevates by one grid step.
  void executeNextStep();
};

}  // namespace dmap
