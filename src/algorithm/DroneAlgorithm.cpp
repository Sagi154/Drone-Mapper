// DroneAlgorithm.cpp
// Milestone 2: projects lidar hits into the sparse BuildingMap using the same beam
// geometry as LidarMock (via hitToWorldPoint). Movement is unused until exploration.

#include "algorithm/DroneAlgorithm.h"

#include "common/MathUtils.h"
#include "mapping/MapTypes.h"

namespace dmap {

DroneAlgorithm::DroneAlgorithm(ILidarSensor& lidar, IPositionSensor& pos, IMovementDriver& move,
                                 IBuildingMap& map)
    : lidar_(lidar), pos_(pos), move_(move), map_(map) {}

void DroneAlgorithm::tick() {
  if (finished_) {
    return;
  }
  (void)move_;

  const DronePosition here = pos_.getPosition();
  const LidarScanResult hits = lidar_.scan();

  for (const LidarHit& h : hits) {
    map_.set(hitToWorldPoint(here, h), MapValue::Occupied);
  }

  finished_ = true;
}

}  // namespace dmap
