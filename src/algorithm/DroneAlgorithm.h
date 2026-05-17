#pragma once

#include "algorithm/ExplorationFrontier.h"
#include "common/Point3D.h"
#include "config/DroneConfig.h"
#include "drivers/IMovementDriver.h"
#include "mapping/IBuildingMap.h"
#include "sensors/ILidarSensor.h"
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
  ILidarSensor& lidar_;
  IPositionSensor& pos_;
  IMovementDriver& move_;
  IBuildingMap& map_;
  DroneConfig cfg_{};

  enum class Phase { Scanning, Planning, Moving };

  Phase phase_{Phase::Scanning};
  ExplorationFrontier frontier_{};
  std::vector<Point3D> current_path_{};  ///< Waypoints to the current frontier.
  std::size_t path_index_{0};            ///< Index of the next waypoint to reach.
  bool finished_{false};

  /// Fires a full spherical sweep from the current position and fuses all
  /// results into the map.  Elevation steps from -90° to +90°; at each tier
  /// a full 360° azimuth sweep is performed with an adaptive step that widens
  /// near the poles (where latitude circles shrink).  The base step is chosen
  /// so that at z_min distance no grid cell falls in the gap between adjacent
  /// beam center directions.
  void fullScan();

  /// Issues one movement command toward the current waypoint
  /// (current_path_[path_index_]): rotates to face it, then advances or
  /// elevates by one grid step.
  void executeNextStep();
};

}  // namespace dmap
