#include "algorithm/DroneAlgorithm.h"

namespace dmap {

DroneAlgorithm::DroneAlgorithm(ILidarSensor& lidar, IPositionSensor& pos, IMovementDriver& move,
                               IBuildingMap& map)
    : lidar_(lidar), pos_(pos), move_(move), map_(map) {}

void DroneAlgorithm::tick() {
  (void)pos_;
  (void)move_;
  (void)map_;
  (void)lidar_.scan();
}

}  // namespace dmap
