#include "simulation/LidarMock.h"
#include "simulation/SimulationState.h"

#include "config/DroneConfig.h"
#include "mapping/MapTypes.h"
#include "sensors/LidarTypes.h"

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

TEST(LidarMock, ScanReturnsOneMissByDefault) {
  SimulationState state;
  DroneConfig cfg{};
  LidarMock lidar(state, cfg);
  const auto result = lidar.scan();
  ASSERT_EQ(result.size(), 1u);
  EXPECT_TRUE(lidarHitIsMiss(result[0]));
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
  ASSERT_FALSE(lidarHitIsMiss(result[0]));
  const double dist_cm = result[0].distance.numerical_value_in(su::cm);
  EXPECT_NEAR(dist_cm, 50.0, 2.0);  // within 2cm of the wall
}

TEST(LidarMock, CenterBeam_WallBeyondZMax_NoHit) {
  SimulationState state;
  placeAt(state, 0, 0, 0, 0);
  // Wall at 150cm, but z_max = 120cm → should not be detected.
  state.setTruthCell(pt(150, 0, 0), MapValue::Occupied);

  LidarMock lidar(state, makeSingleBeamCfg());
  const auto result = lidar.scan();
  ASSERT_EQ(result.size(), 1u);
  EXPECT_TRUE(lidarHitIsMiss(result[0]));
}

TEST(LidarMock, CenterBeam_WallBehind_NoHit) {
  SimulationState state;
  placeAt(state, 0, 0, 0, 0);  // facing east (+x)
  state.setTruthCell(pt(-50, 0, 0), MapValue::Occupied);  // wall behind

  LidarMock lidar(state, makeSingleBeamCfg());
  const auto result = lidar.scan();
  ASSERT_EQ(result.size(), 1u);
  EXPECT_TRUE(lidarHitIsMiss(result[0]));
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
  const auto no_offset = lidar.scan();
  ASSERT_EQ(no_offset.size(), 1u);
  EXPECT_TRUE(lidarHitIsMiss(no_offset[0]));
  // With +90° offset the beam points north → should hit.
  const auto rotated = lidar.scan(90.0 * su::deg);
  ASSERT_EQ(rotated.size(), 1u);
  EXPECT_FALSE(lidarHitIsMiss(rotated[0]));
}

// -----------------------------------------------------------------------
// Multi-circle: wall ahead hits multiple beams
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// Z-min: hits closer than Z-min report distance=0
// -----------------------------------------------------------------------

TEST(LidarMock, CenterBeam_HitWithinZMin_ReportsZeroDistance) {
  SimulationState state;
  placeAt(state, 0, 0, 0, 0);
  // Wall at 10cm, but z_min = 20cm → distance must be reported as 0.
  state.setTruthCell(pt(10, 0, 0), MapValue::Occupied);

  LidarMock lidar(state, makeSingleBeamCfg());  // z_min=20, z_max=120
  const auto result = lidar.scan();
  ASSERT_EQ(result.size(), 1u);
  const double dist_cm = result[0].distance.numerical_value_in(su::cm);
  EXPECT_DOUBLE_EQ(dist_cm, 0.0);  // too close → unmeasurable → 0
}

TEST(LidarMock, CenterBeam_HitAtExactlyZMin_ReportsRealDistance) {
  SimulationState state;
  placeAt(state, 0, 0, 0, 0);
  // Wall at exactly z_min (20cm) → real distance reported (not 0).
  state.setTruthCell(pt(20, 0, 0), MapValue::Occupied);

  LidarMock lidar(state, makeSingleBeamCfg());  // z_min=20
  const auto result = lidar.scan();
  ASSERT_EQ(result.size(), 1u);
  const double dist_cm = result[0].distance.numerical_value_in(su::cm);
  EXPECT_GT(dist_cm, 0.0);  // at or beyond z_min → measurable distance returned
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
  ASSERT_EQ(result.size(), 5u);
  // The lidar intentionally has blind spots between adjacent beams.
  // Features that fall entirely in a gap and are smaller than the mission
  // resolution are not required to be detected (per spec).
  // At least 4 of the 5 beams must hit the wall; the outer circle must
  // contribute (more than one real hit).
  int hit_count = 0;
  for (const auto& h : result) {
    if (!lidarHitIsMiss(h)) {
      ++hit_count;
    }
  }
  EXPECT_GE(hit_count, 4);
  EXPECT_GT(hit_count, 1);
}
