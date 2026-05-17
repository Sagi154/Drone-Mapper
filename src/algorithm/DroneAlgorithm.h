#pragma once

#include "config/DroneConfig.h"
#include "drivers/IMovementDriver.h"
#include "mapping/IBuildingMap.h"
#include "sensors/ILidarSensor.h"
#include "sensors/IPositionSensor.h"

namespace dmap {

/// Autonomous drone exploration algorithm.
/// Accepts the full DroneConfig so it can use all capability parameters
/// (advance step, lidar range, collision radius) without callers having to
/// extract individual fields.
class DroneAlgorithm {
 public:
  DroneAlgorithm(ILidarSensor& lidar, IPositionSensor& pos, IMovementDriver& move,
                 IBuildingMap& map, const DroneConfig& cfg);

  /// One step of the exploration loop.
  void tick();

  /// True when exploration is complete (no reachable frontiers remain).
  [[nodiscard]] bool isFinished() const noexcept { return finished_; }

 private:
  ILidarSensor& lidar_;
  IPositionSensor& pos_;
  IMovementDriver& move_;
  IBuildingMap& map_;
  DroneConfig cfg_{};

  int blocked_advances_{0};
  bool finished_{false};

  /// Fires a full spherical sweep from the current position and fuses all
  /// results into the map.  Elevation steps from -90° to +90°; at each tier
  /// a full 360° azimuth sweep is performed with an adaptive step that widens
  /// near the poles (where latitude circles shrink).  The base step is chosen
  /// so that at z_min distance no grid cell falls in the gap between adjacent
  /// beam center directions.
  void fullScan();
};

}  // namespace dmap
