// test_exploration_frontier.cpp
// Unit tests for ExplorationFrontier BFS: sphere-safe passability and path
// to the nearest frontier cell adjacent to NotMapped space.

#include "algorithm/ExplorationFrontier.h"

#include <gtest/gtest.h>

#include "config/MissionConfig.h"
#include "mapping/BuildingMap.h"
#include "mapping/MapTypes.h"

#include <mp-units/systems/si/unit_symbols.h>

namespace su = mp_units::si::unit_symbols;

namespace {

dmap::MissionConfig makeMission1cmGrid() {
  dmap::MissionConfig m;
  m.min_x      = -50.0 * su::cm;
  m.max_x      =  50.0 * su::cm;
  m.min_y      = -50.0 * su::cm;
  m.max_y      =  50.0 * su::cm;
  m.min_height =   0.0 * su::cm;
  m.max_height = 100.0 * su::cm;
  m.xy_decimal_places     = 0;
  m.height_decimal_places = 0;
  return m;
}

dmap::Point3D pt(double x, double y, double h) {
  return {x * su::cm, y * su::cm, h * su::cm};
}

/// Fills an axis-aligned box of Empty cells (inclusive bounds, cm grid indices).
void fillEmptyBox(dmap::BuildingMap& map, int x0, int x1, int y0, int y1, int z0,
                  int z1) {
  for (int x = x0; x <= x1; ++x) {
    for (int y = y0; y <= y1; ++y) {
      for (int z = z0; z <= z1; ++z) {
        map.set(pt(x, y, z), dmap::MapValue::Empty);
      }
    }
  }
}

/// Fills a cube of Empty cells centred at (cx,cy,ch) with half-edge `half` (cm).
void fillEmptyCube(dmap::BuildingMap& map, int cx, int cy, int ch, int half) {
  for (int x = cx - half; x <= cx + half; ++x) {
    for (int y = cy - half; y <= cy + half; ++y) {
      for (int z = ch - half; z <= ch + half; ++z) {
        map.set(pt(x, y, z), dmap::MapValue::Empty);
      }
    }
  }
}

}  // namespace

// What: start cell is Empty but a neighbour within drone_radius is NotMapped.
// Expected: BFS cannot treat start as passable; findPath returns found=false.
TEST(ExplorationFrontier, StartNotPassableWhenSphereHasNotMapped) {
  dmap::BuildingMap map(makeMission1cmGrid());
  map.set(pt(0, 0, 50), dmap::MapValue::Empty);
  // Neighbour within 5 cm radius is still NotMapped.

  dmap::ExplorationFrontier frontier;
  const dmap::PathResult result =
      frontier.findPath(map, pt(0, 0, 50), 5.0 * su::cm);

  EXPECT_FALSE(result.found);
  EXPECT_TRUE(result.path.empty());
}

// What: straight corridor of confirmed-empty cells ending next to unknown space.
// Expected: path to the nearest frontier (last empty cell before NotMapped).
// Uses zero radius so only each cell centre must be Empty.
TEST(ExplorationFrontier, FindsPathAlongEmptyCorridor) {
  dmap::BuildingMap map(makeMission1cmGrid());
  // 3x3 corridor along +X (including ±Y and ±height); only +X ends at unknown.
  fillEmptyBox(map, -1, 2, -1, 1, 49, 51);
  // (3,0,50) stays NotMapped -> (2,0,50) is a frontier.

  dmap::ExplorationFrontier frontier;
  const dmap::PathResult result =
      frontier.findPath(map, pt(0, 0, 50), 0.0 * su::cm);

  ASSERT_TRUE(result.found);
  ASSERT_EQ(result.path.size(), 2u);
  EXPECT_EQ(map.get(result.path[0]), dmap::MapValue::Empty);
  EXPECT_EQ(map.get(result.path[1]), dmap::MapValue::Empty);
  EXPECT_EQ(result.path[1].x.numerical_value_in(su::cm), 2.0);
  EXPECT_EQ(result.path[1].y.numerical_value_in(su::cm), 0.0);
}

// What: empty cube large enough for a 5 cm drone sphere at the centre.
// Expected: BFS reaches a frontier on the cube surface facing NotMapped outside.
TEST(ExplorationFrontier, FindsFrontierInsideEmptyCube) {
  dmap::BuildingMap map(makeMission1cmGrid());
  fillEmptyCube(map, 0, 0, 50, 8);

  dmap::ExplorationFrontier frontier;
  const dmap::PathResult result =
      frontier.findPath(map, pt(0, 0, 50), 5.0 * su::cm);

  EXPECT_TRUE(result.found);
  EXPECT_FALSE(result.path.empty());
}
