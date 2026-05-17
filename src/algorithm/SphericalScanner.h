// SphericalScanner.h
// Performs a full spherical lidar sweep from the drone's current position and
// fuses all hits into the drone's building map.  Separating this from
// DroneAlgorithm allows the sweep logic to be tested and reasoned about
// independently of the exploration state machine.

#pragma once

#include "config/DroneConfig.h"
#include "mapping/IBuildingMap.h"
#include "sensors/ILidarSensor.h"
#include "sensors/IPositionSensor.h"

namespace dmap {

/// Fires a full spherical lidar sweep from the drone's current position and
/// fuses all scan results into the provided building map.
///
/// The sweep steps elevation from -90° to +90°.  At each tier a complete
/// 360° azimuth sweep is performed.  The angular step between adjacent aim
/// directions is chosen so that at z_min distance no grid cell falls in the
/// gap between beam paths.  Near the poles the azimuth step widens
/// proportionally (az_step = el_step / cos(el)) to maintain uniform surface
/// spacing.
class SphericalScanner {
 public:
  /// Binds the scanner to the lidar sensor, position sensor, and lidar config.
  /// All references must outlive the scanner.
  SphericalScanner(ILidarSensor& lidar, IPositionSensor& pos,
                   const LidarConfig& lidar_cfg);

  /// Executes the full spherical sweep from the current drone position and
  /// writes the results into `map`.  Position is sampled once at the start;
  /// the drone must not move during the sweep.
  /// @param map  Drone's discovered building map — updated in place.
  void scan(IBuildingMap& map) const;

 private:
  ILidarSensor& lidar_;
  IPositionSensor& pos_;
  LidarConfig lidar_cfg_{};
};

}  // namespace dmap
