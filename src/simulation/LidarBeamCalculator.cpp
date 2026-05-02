#include "simulation/LidarBeamCalculator.h"

namespace dmap {

LidarBeamCalculator::LidarBeamCalculator(LidarConfig cfg) : cfg_(cfg) {}

std::vector<UnitRay3> LidarBeamCalculator::unitDirectionsForScan() const {
  (void)cfg_;
  return {};
}

}  // namespace dmap
