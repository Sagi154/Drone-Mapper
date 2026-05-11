// LidarBeamCalculator.h
// Computes the unit-vector direction of every lidar beam for one scan,
// based on the sensor's FOV configuration (number of circles, spacing,
// Z-min).  The result is in the lidar's local frame (+x = forward);
// LidarMock rotates the vectors into world space before ray-marching.

#pragma once

#include "config/DroneConfig.h"

#include <vector>

namespace dmap {

// A unit direction vector in 3-D space (magnitude == 1).
// Expressed in the lidar's local frame:
//   +x = forward (the center beam points here)
//   +y = right
//   +z = up
struct UnitRay3 {
  double x{};
  double y{};
  double z{};
};

// Generates all beam directions for a single lidar scan according to the
// concentric-circle FOV model defined in LidarConfig.
// Circle 0 has exactly 1 beam (straight ahead); circle n has 4^n beams
// evenly spaced around the cone at the elevation angle for that circle.
class LidarBeamCalculator {
 public:
  // Stores the lidar configuration to use for every subsequent call.
  explicit LidarBeamCalculator(LidarConfig cfg);

  // Returns one unit vector per beam, in local frame, in circle order
  // (circle 0 first, then circle 1, ...).
  // The vectors are recomputed each call; cache the result if you call this
  // frequently in a tight loop.
  std::vector<UnitRay3> unitDirectionsForScan() const;

 private:
  LidarConfig cfg_{};
};

}  // namespace dmap
