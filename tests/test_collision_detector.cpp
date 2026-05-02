#include "simulation/CollisionDetector.h"
#include "simulation/SimulationState.h"

#include <gtest/gtest.h>

#include "common/Point3D.h"

TEST(CollisionDetector, NoOccupiedByDefault) {
  dmap::SimulationState state;
  dmap::CollisionDetector det(state);
  dmap::Point3D p{};
  EXPECT_FALSE(det.intersectsOccupied(p));
}
