#pragma once

#include "common/Types.h"
#include "drivers/IMovementDriver.h"
#include "mapping/IBuildingMap.h"
#include "sensors/ILidarSensor.h"
#include "sensors/IPositionSensor.h"

namespace dmap {

class DroneAlgorithm {
 public:
  DroneAlgorithm(ILidarSensor& lidar, IPositionSensor& pos, IMovementDriver& move,
                 IBuildingMap& map, LengthCm advance_step);

  /// One step: lidar scan into map, then try advance; on block rotate right until four blocks end the run.
  void tick();

  /// True once four consecutive advances fail (surrounded on the XY plane at current height).
  [[nodiscard]] bool isFinished() const noexcept { return finished_; }

 private:
  ILidarSensor& lidar_;
  IPositionSensor& pos_;
  IMovementDriver& move_;
  IBuildingMap& map_;
  LengthCm advance_step_{};

  int blocked_advances_{0};
  bool finished_{false};
};

}  // namespace dmap
