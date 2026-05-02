#pragma once

#include "config/DroneConfig.h"

#include <vector>

namespace dmap {

struct UnitRay3 {
  double x{};
  double y{};
  double z{};
};

class LidarBeamCalculator {
 public:
  explicit LidarBeamCalculator(LidarConfig cfg);

  std::vector<UnitRay3> unitDirectionsForScan() const;

 private:
  LidarConfig cfg_{};
};

}  // namespace dmap
