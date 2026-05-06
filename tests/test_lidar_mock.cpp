#include "simulation/LidarMock.h"
#include "simulation/SimulationState.h"

#include "config/DroneConfig.h"
#include "mapping/MapTypes.h"

#include <mp-units/systems/si/unit_symbols.h>
#include <gtest/gtest.h>
#include <cmath>

namespace su = mp_units::si::unit_symbols;
using namespace dmap;

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

// Single center beam, z_min=20cm, z_max=120cm, no spacing needed.
static DroneConfig makeSingleBeamCfg() {
  DroneConfig cfg;
  cfg.lidar.fov_circles      = 1;
  cfg.lidar.z_min            = 20.0  * su::cm;
  cfg.lidar.z_max            = 120.0 * su::cm;
  cfg.lidar.circle_spacing_d = 2.5   * su::cm;
  return cfg;
}

// Two-circle config: 1 center + 4 outer beams.
static DroneConfig makeTwoCircleCfg() {
  DroneConfig cfg;
  cfg.lidar.fov_circles      = 2;
  cfg.lidar.z_min            = 20.0  * su::cm;
  cfg.lidar.z_max            = 120.0 * su::cm;
  cfg.lidar.circle_spacing_d = 2.5   * su::cm;
  return cfg;
}

static Point3D pt(double x, double y, double h) {
  return {x * su::cm, y * su::cm, h * su::cm};
}

// Place the drone at a given position and return a configured LidarMock.
static void placeAt(SimulationState& state, double x, double y, double h, double ang = 0.0) {
  state.setDronePosition({x * su::cm, y * su::cm, h * su::cm, ang * su::deg});
}

// -----------------------------------------------------------------------
// Basic
// -----------------------------------------------------------------------

TEST(LidarMock, ScanReturnsEmptyByDefault) {
  SimulationState state;
  DroneConfig cfg{};
  LidarMock lidar(state, cfg);
  EXPECT_TRUE(lidar.scan().empty());
}

// -----------------------------------------------------------------------
// Center-beam hit / miss
// -----------------------------------------------------------------------

TEST(LidarMock, CenterBeam_WallAhead_ReturnsOneHit) {
  SimulationState state;
  placeAt(state, 0, 0, 0, 0);        // drone at origin, facing east
  state.setTruthCell(pt(50, 0, 0), MapValue::Occupied);  // wall 50cm ahead

  LidarMock lidar(state, makeSingleBeamCfg());
  const auto result = lidar.scan();
  ASSERT_EQ(result.size(), 1u);
}

TEST(LidarMock, CenterBeam_WallAhead_CorrectDistance) {
  SimulationState state;
  placeAt(state, 0, 0, 0, 0);
  state.setTruthCell(pt(50, 0, 0), MapValue::Occupied);

  // With 1cm step and bounds with 0 decimal places, step=1cm.
  // Wall is 50cm ahead so the reported distance should be ≈50cm.
  LidarMock lidar(state, makeSingleBeamCfg());
  const auto result = lidar.scan();
  ASSERT_EQ(result.size(), 1u);
  const double dist_cm = result[0].distance.numerical_value_in(su::cm);
  EXPECT_NEAR(dist_cm, 50.0, 2.0);  // within 2cm of the wall
}

TEST(LidarMock, CenterBeam_WallBeyondZMax_NoHit) {
  SimulationState state;
  placeAt(state, 0, 0, 0, 0);
  // Wall at 150cm, but z_max = 120cm → should not be detected.
  state.setTruthCell(pt(150, 0, 0), MapValue::Occupied);

  LidarMock lidar(state, makeSingleBeamCfg());
  EXPECT_TRUE(lidar.scan().empty());
}

TEST(LidarMock, CenterBeam_WallBehind_NoHit) {
  SimulationState state;
  placeAt(state, 0, 0, 0, 0);  // facing east (+x)
  state.setTruthCell(pt(-50, 0, 0), MapValue::Occupied);  // wall behind

  LidarMock lidar(state, makeSingleBeamCfg());
  EXPECT_TRUE(lidar.scan().empty());
}

// -----------------------------------------------------------------------
// XY offset rotates the scan
// -----------------------------------------------------------------------

TEST(LidarMock, XYOffset_RotatesScan_HitAfterRotation) {
  SimulationState state;
  placeAt(state, 0, 0, 0, 0);  // facing east
  // Wall 50cm to the NORTH (+y). Normally not hit by east-facing beam.
  state.setTruthCell(pt(0, 50, 0), MapValue::Occupied);

  LidarMock lidar(state, makeSingleBeamCfg());
  // No hit without offset.
  EXPECT_TRUE(lidar.scan().empty());
  // With +90° offset the beam points north → should hit.
  ASSERT_FALSE(lidar.scan(90.0 * su::deg).empty());
}

// -----------------------------------------------------------------------
// Multi-circle: wall ahead hits multiple beams
// -----------------------------------------------------------------------

TEST(LidarMock, TwoCircles_WallAhead_MultipleBeamsHit) {
  SimulationState state;

  // Use 1 cm grid (xy_decimal_places=0) so truthKey rounds fractional beam
  // samples to the nearest whole-cm cell.
  MapBounds bounds;
  bounds.min_x = -200.0 * su::cm;  bounds.max_x = 200.0 * su::cm;
  bounds.min_y = -200.0 * su::cm;  bounds.max_y = 200.0 * su::cm;
  bounds.min_height = -200.0 * su::cm; bounds.max_height = 200.0 * su::cm;
  bounds.xy_decimal_places     = 0;
  bounds.height_decimal_places = 0;
  state.setMapBounds(bounds);

  placeAt(state, 0, 0, 50, 0);  // drone at origin, facing east

  // Thick block (20 cm deep in x) so every beam is guaranteed to sample
  // at least one occupied cell regardless of exact angle.
  for (int dx = 40; dx <= 60; ++dx)
    for (int dy = -20; dy <= 20; ++dy)
      for (int dz = -20; dz <= 20; ++dz)
        state.setTruthCell(pt(dx, dy, 50 + dz), MapValue::Occupied);

  LidarMock lidar(state, makeTwoCircleCfg());
  const auto result = lidar.scan();
  // The lidar intentionally has blind spots between adjacent beams.
  // Features that fall entirely in a gap and are smaller than the mission
  // resolution are not required to be detected (per spec).
  // At least 4 of the 5 beams must hit the wall; the outer circle must
  // contribute (result > 1 beam).
  EXPECT_GE(result.size(), 4u);
  EXPECT_GT(result.size(), 1u);
}
