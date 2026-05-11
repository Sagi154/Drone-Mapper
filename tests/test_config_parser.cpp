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

// Scenario: a full mission_config file contains boundaries, start pose, and resolution.
// Expected: every supported mission key is parsed into MissionConfig fields.
// Why: validates the happy path for mission parser and file format.
TEST(ConfigParser, MissionConfigParses) {
  const auto path = writeTempFile(
      "dmap_mission_config_test.txt",
      "# Mission boundaries\n"
      "min_x = 0\n"
      "max_x = 500\n"
      "min_y = 0\n"
      "max_y = 400\n"
      "min_height = 0\n"
      "max_height = 300\n"
      "start_x = 100\n"
      "start_y = 200\n"
      "start_height = 50\n"
      "start_angle = 15\n"
      "xy_decimal_places = 1\n"
      "height_decimal_places = 2\n");

  const auto cfg = dmap::parseMissionConfig(path);
  EXPECT_NEAR(cfg.min_x.numerical_value_in(su::cm), 0.0, 1e-9);
  EXPECT_NEAR(cfg.max_x.numerical_value_in(su::cm), 500.0, 1e-9);
  EXPECT_NEAR(cfg.min_y.numerical_value_in(su::cm), 0.0, 1e-9);
  EXPECT_NEAR(cfg.max_y.numerical_value_in(su::cm), 400.0, 1e-9);
  EXPECT_NEAR(cfg.min_height.numerical_value_in(su::cm), 0.0, 1e-9);
  EXPECT_NEAR(cfg.max_height.numerical_value_in(su::cm), 300.0, 1e-9);
  EXPECT_NEAR(cfg.initial_position.x.numerical_value_in(su::cm), 100.0, 1e-9);
  EXPECT_NEAR(cfg.initial_position.y.numerical_value_in(su::cm), 200.0, 1e-9);
  EXPECT_NEAR(cfg.initial_position.height.numerical_value_in(su::cm), 50.0, 1e-9);
  EXPECT_NEAR(cfg.initial_position.xy_angle.numerical_value_in(su::deg), 15.0, 1e-9);
  EXPECT_EQ(cfg.xy_decimal_places, 1);
  EXPECT_EQ(cfg.height_decimal_places, 2);
}

// Scenario: parser is called with a missing mission_config file.
// Expected: parser returns default values without throwing.
// Why: assignment requires graceful handling of missing/bad input files.
TEST(ConfigParser, MissionConfigMissingFileReturnsDefaults) {
  const auto cfg = dmap::parseMissionConfig("nope.txt");
  EXPECT_NEAR(cfg.min_x.numerical_value_in(su::cm), 0.0, 1e-9);
  EXPECT_NEAR(cfg.max_x.numerical_value_in(su::cm), 0.0, 1e-9);
  EXPECT_NEAR(cfg.initial_position.x.numerical_value_in(su::cm), 0.0, 1e-9);
  EXPECT_EQ(cfg.xy_decimal_places, 2);
  EXPECT_EQ(cfg.height_decimal_places, 2);
}
