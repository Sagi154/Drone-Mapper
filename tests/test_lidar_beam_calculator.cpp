#include "simulation/LidarBeamCalculator.h"

#include "config/DroneConfig.h"

#include <mp-units/systems/si/unit_symbols.h>
#include <gtest/gtest.h>
#include <cmath>

namespace su = mp_units::si::unit_symbols;
using namespace dmap;

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

static LidarConfig makeCfg(int circles, double z_min_cm, double d_cm) {
  LidarConfig c;
  c.fov_circles      = circles;
  c.z_min            = z_min_cm * su::cm;
  c.z_max            = 200.0   * su::cm;
  c.circle_spacing_d = d_cm    * su::cm;
  return c;
}

// -----------------------------------------------------------------------
// Beam count
// -----------------------------------------------------------------------

TEST(LidarBeamCalculator, FovCircles1_Returns1Beam) {
  LidarBeamCalculator calc(makeCfg(1, 20.0, 2.5));
  EXPECT_EQ(calc.unitDirectionsForScan().size(), 1u);
}

TEST(LidarBeamCalculator, FovCircles2_Returns5Beams) {
  // 1 (circle 0) + 4 (circle 1) = 5
  LidarBeamCalculator calc(makeCfg(2, 20.0, 2.5));
  EXPECT_EQ(calc.unitDirectionsForScan().size(), 5u);
}

TEST(LidarBeamCalculator, FovCircles3_Returns21Beams) {
  // 1 + 4 + 16 = 21
  LidarBeamCalculator calc(makeCfg(3, 20.0, 2.5));
  EXPECT_EQ(calc.unitDirectionsForScan().size(), 21u);
}

TEST(LidarBeamCalculator, FovCircles5_Returns341Beams) {
  // 1 + 4 + 16 + 64 + 256 = 341
  LidarBeamCalculator calc(makeCfg(5, 20.0, 2.5));
  EXPECT_EQ(calc.unitDirectionsForScan().size(), 341u);
}

// -----------------------------------------------------------------------
// Beam directions
// -----------------------------------------------------------------------

TEST(LidarBeamCalculator, CenterBeam_IsExactlyForward) {
  // Circle 0 (the only beam in fov_circles=1) must point exactly along +x.
  LidarBeamCalculator calc(makeCfg(1, 20.0, 2.5));
  const auto rays = calc.unitDirectionsForScan();
  ASSERT_EQ(rays.size(), 1u);
  EXPECT_NEAR(rays[0].x, 1.0, 1e-12);
  EXPECT_NEAR(rays[0].y, 0.0, 1e-12);
  EXPECT_NEAR(rays[0].z, 0.0, 1e-12);
}

TEST(LidarBeamCalculator, AllBeams_AreUnitLength) {
  LidarBeamCalculator calc(makeCfg(4, 20.0, 2.5));
  for (const auto& r : calc.unitDirectionsForScan()) {
    const double mag = std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
    EXPECT_NEAR(mag, 1.0, 1e-10) << "x=" << r.x << " y=" << r.y << " z=" << r.z;
  }
}

TEST(LidarBeamCalculator, Circle1Beams_CorrectElevationAngle) {
  // With z_min=20cm and D=2.5cm, circle-1 elevation = atan(1*2.5/20).
  const double expected_theta = std::atan2(2.5, 20.0);
  LidarBeamCalculator calc(makeCfg(2, 20.0, 2.5));
  const auto rays = calc.unitDirectionsForScan();
  // Rays[0] = center beam. Rays[1..4] = circle-1 beams.
  for (size_t i = 1; i <= 4; ++i) {
    // The elevation angle from the forward axis = acos(x) for a unit vector.
    const double theta = std::acos(rays[i].x);
    EXPECT_NEAR(theta, expected_theta, 1e-10)
        << "beam " << i << ": theta=" << theta << " expected=" << expected_theta;
  }
}
