#pragma once

#include "config/DroneConfig.h"
#include "sensors/ILidarSensor.h"
#include "simulation/LidarBeamCalculator.h"
#include "simulation/SimulationState.h"

namespace dmap {

class LidarMock final : public ILidarSensor {
 public:
  LidarMock(SimulationState& state, DroneConfig drone_cfg);

  // Fires all configured lidar beams into the ground-truth map and returns
  // every first-hit within Z_max range.
  // xy_offset:    optional extra horizontal rotation added to the drone heading.
  // height_angle: optional vertical tilt of the entire lidar cone (positive = up).
  LidarScanResult scan(std::optional<AngleDeg> xy_offset = std::nullopt,
                       std::optional<AngleDeg> height_angle = std::nullopt) override;

 private:
  SimulationState& state_;
  DroneConfig drone_cfg_{};
  LidarBeamCalculator calc_;

  // Returns the ray-march step size in cm derived from the map resolution.
  // Uses the finer of XY and height cell sizes to guarantee no cell is skipped.
  double stepCm() const;
};

}  // namespace dmap
