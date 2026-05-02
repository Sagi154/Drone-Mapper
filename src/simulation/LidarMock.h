#pragma once

#include "config/DroneConfig.h"
#include "sensors/ILidarSensor.h"
#include "simulation/SimulationState.h"

namespace dmap {

class LidarMock final : public ILidarSensor {
 public:
  LidarMock(SimulationState& state, DroneConfig drone_cfg);

  LidarScanResult scan(std::optional<AngleDeg> xy_offset,
                         std::optional<AngleDeg> height_angle) override;

 private:
  SimulationState& state_;
  DroneConfig drone_cfg_{};
};

}  // namespace dmap
