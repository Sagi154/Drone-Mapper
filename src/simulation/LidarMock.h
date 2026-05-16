// LidarMock.h
// Simulated lidar sensor.  For each scan it fires all beams defined by
// LidarBeamCalculator into the ground-truth map using ray-marching and
// returns one LidarHit per beam (distance=-1 when nothing is hit within Z_max).

#pragma once

#include "config/DroneConfig.h"
#include "sensors/ILidarSensor.h"
#include "simulation/LidarBeamCalculator.h"
#include "simulation/SimulationState.h"

namespace dmap {

// Simulated lidar that ray-marches through the ground-truth map.
// Implements ILidarSensor so the drone algorithm treats it like a real sensor.
class LidarMock final : public ILidarSensor {
 public:
  // Binds the mock to the shared SimulationState and stores the drone config
  // (which includes the lidar's FOV parameters).
  LidarMock(SimulationState& state, DroneConfig drone_cfg);

  // Fires all configured beams from the drone's current position.
  // Returns one LidarHit per beam; distance=-1 when nothing is hit within Z_max.
  //
  // xy_offset:    optional extra heading rotation applied to the entire cone
  //               (useful when the lidar is mounted at an angle).
  // height_angle: optional vertical tilt of the cone (positive = nose up).
  LidarScanResult scan(std::optional<AngleDeg> xy_offset = std::nullopt,
                       std::optional<AngleDeg> height_angle = std::nullopt) override;

 private:
  SimulationState& state_;
  DroneConfig drone_cfg_{};
  LidarBeamCalculator calc_;  // pre-built beam directions for this sensor config

  // Returns the ray-march step size in cm.
  // Derived from the map's grid resolution (min of XY and height cell sizes)
  // so that no occupied cell can be skipped between two consecutive samples.
  double stepCm() const;
};

}  // namespace dmap
