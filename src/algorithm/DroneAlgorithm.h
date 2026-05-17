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
};

}  // namespace dmap
