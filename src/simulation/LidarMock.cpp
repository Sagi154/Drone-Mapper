#include "simulation/LidarMock.h"

namespace dmap {

LidarMock::LidarMock(SimulationState& state, DroneConfig drone_cfg)
    : state_(state), drone_cfg_(drone_cfg) {}

LidarScanResult LidarMock::scan(std::optional<AngleDeg> xy_offset,
                                std::optional<AngleDeg> height_angle) {
  (void)state_;
  (void)drone_cfg_;
  (void)xy_offset;
  (void)height_angle;
  return {};
}

}  // namespace dmap
