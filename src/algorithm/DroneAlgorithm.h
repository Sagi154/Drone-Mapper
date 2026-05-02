#pragma once

#include "drivers/IMovementDriver.h"
#include "mapping/IBuildingMap.h"
#include "sensors/ILidarSensor.h"
#include "sensors/IPositionSensor.h"

namespace dmap {

class DroneAlgorithm {
 public:
  DroneAlgorithm(ILidarSensor& lidar, IPositionSensor& pos, IMovementDriver& move,
                 IBuildingMap& map);

  void tick();

 private:
  ILidarSensor& lidar_;
  IPositionSensor& pos_;
  IMovementDriver& move_;
  IBuildingMap& map_;
};

}  // namespace dmap
