#include "simulation/LidarMock.h"
#include "simulation/SimulationState.h"

#include <gtest/gtest.h>

#include "config/DroneConfig.h"

TEST(LidarMock, ScanReturnsEmptyByDefault) {
  dmap::SimulationState state;
  dmap::DroneConfig cfg{};
  dmap::LidarMock lidar(state, cfg);
  const auto r = lidar.scan();
  EXPECT_TRUE(r.empty());
}
