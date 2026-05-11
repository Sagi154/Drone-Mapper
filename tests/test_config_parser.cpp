#include "config/DroneConfigParser.h"
#include "config/MissionConfigParser.h"

#include <filesystem>
#include <fstream>
#include <mp-units/systems/si/unit_symbols.h>
#include <gtest/gtest.h>

namespace su = mp_units::si::unit_symbols;

namespace {

std::filesystem::path writeTempFile(const std::string& file_name,
                                    const std::string& content) {
  const auto path = std::filesystem::temp_directory_path() / file_name;
  std::ofstream out(path, std::ios::trunc);
  out << content;
  return path;
}

}  // namespace

// Scenario: a full drone_config file contains all supported keys.
// Expected: every field is parsed into DroneConfig using the configured units.
// Why: validates the happy path for the new file format parser.
TEST(ConfigParser, DroneConfigParsesKnownKeys) {
  const auto path = writeTempFile(
      "dmap_drone_config_test.txt",
      "# drone config\n"
      "min_passable_width = 30\n"
      "min_passable_length=31\n"
      "min_passable_height = 22\n"
      "max_rotate = 91\n"
      "max_advance = 105\n"
      "max_elevate = 55\n"
      "lidar_z_min = 20\n"
      "lidar_z_max = 120\n"
      "lidar_d = 2.5\n"
      "lidar_fov_circles = 5\n");

  const auto cfg = dmap::parseDroneConfig(path);
  EXPECT_NEAR(cfg.min_passable_width.numerical_value_in(su::cm), 30.0, 1e-9);
  EXPECT_NEAR(cfg.min_passable_length.numerical_value_in(su::cm), 31.0, 1e-9);
  EXPECT_NEAR(cfg.min_passable_height.numerical_value_in(su::cm), 22.0, 1e-9);
  EXPECT_NEAR(cfg.max_rotate_per_command.numerical_value_in(su::deg), 91.0, 1e-9);
  EXPECT_NEAR(cfg.max_advance_per_command.numerical_value_in(su::cm), 105.0, 1e-9);
  EXPECT_NEAR(cfg.max_elevate_per_command.numerical_value_in(su::cm), 55.0, 1e-9);
  EXPECT_NEAR(cfg.lidar.z_min.numerical_value_in(su::cm), 20.0, 1e-9);
  EXPECT_NEAR(cfg.lidar.z_max.numerical_value_in(su::cm), 120.0, 1e-9);
  EXPECT_NEAR(cfg.lidar.circle_spacing_d.numerical_value_in(su::cm), 2.5, 1e-9);
  EXPECT_EQ(cfg.lidar.fov_circles, 5);
}

// Scenario: parser is called with a missing file path.
// Expected: parser does not throw and returns default-initialized values.
// Why: assignment requires graceful recovery from input problems.
TEST(ConfigParser, DroneConfigMissingFileReturnsDefaults) {
  const auto cfg = dmap::parseDroneConfig("nope.txt");
  EXPECT_NEAR(cfg.min_passable_width.numerical_value_in(su::cm), 0.0, 1e-9);
  EXPECT_EQ(cfg.lidar.fov_circles, 1);
}

// Scenario: config file contains only a subset of keys.
// Expected: present keys are parsed; missing keys keep defaults.
// Why: partial files should still be recoverable.
TEST(ConfigParser, DroneConfigMissingKeysKeepDefaults) {
  const auto path = writeTempFile("dmap_drone_config_partial_test.txt",
                                  "max_rotate = 80\n");
  const auto cfg = dmap::parseDroneConfig(path);
  EXPECT_NEAR(cfg.max_rotate_per_command.numerical_value_in(su::deg), 80.0, 1e-9);
  EXPECT_NEAR(cfg.max_advance_per_command.numerical_value_in(su::cm), 0.0, 1e-9);
}

// Scenario: mission parser smoke test for current stub behavior.
// Expected: parser call does not throw.
// Why: keeps baseline test coverage while mission parser is still pending.
TEST(ConfigParser, MissionConfigParses) {
  EXPECT_NO_THROW(dmap::parseMissionConfig("nope.txt"));
}
