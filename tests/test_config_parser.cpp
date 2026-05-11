#include "config/DroneConfigParser.h"
#include "config/MissionConfigParser.h"
#include "io/ErrorLogger.h"
#include "io/MapFileReader.h"
#include "mapping/MapTypes.h"
#include "simulation/SimulationState.h"

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

  dmap::ErrorLogger logger;
  const auto cfg = dmap::parseDroneConfig(path, logger);
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
  dmap::ErrorLogger logger;
  const auto cfg = dmap::parseDroneConfig("nope.txt", logger);
  EXPECT_NEAR(cfg.min_passable_width.numerical_value_in(su::cm), 0.0, 1e-9);
  EXPECT_EQ(cfg.lidar.fov_circles, 1);
}

// Scenario: config file contains only a subset of keys.
// Expected: present keys are parsed; missing keys keep defaults.
// Why: partial files should still be recoverable.
TEST(ConfigParser, DroneConfigMissingKeysKeepDefaults) {
  const auto path = writeTempFile("dmap_drone_config_partial_test.txt",
                                  "max_rotate = 80\n");
  dmap::ErrorLogger logger;
  const auto cfg = dmap::parseDroneConfig(path, logger);
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

  dmap::ErrorLogger logger;
  const auto cfg = dmap::parseMissionConfig(path, logger);
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
  dmap::ErrorLogger logger;
  const auto cfg = dmap::parseMissionConfig("nope.txt", logger);
  EXPECT_NEAR(cfg.min_x.numerical_value_in(su::cm), 0.0, 1e-9);
  EXPECT_NEAR(cfg.max_x.numerical_value_in(su::cm), 0.0, 1e-9);
  EXPECT_NEAR(cfg.initial_position.x.numerical_value_in(su::cm), 0.0, 1e-9);
  EXPECT_EQ(cfg.xy_decimal_places, 2);
  EXPECT_EQ(cfg.height_decimal_places, 2);
}

// Scenario: map_input contains bounds metadata and sparse occupied cells.
// Expected: bounds are set in SimulationState and listed occupied cells are loaded.
// Why: validates the main map_input parser contract used by simulation components.
TEST(ConfigParser, MapInputLoadsBoundsAndOccupiedCells) {
  const auto path = writeTempFile(
      "dmap_map_input_test.txt",
      "# bounds format: min_x max_x min_y max_y min_h max_h xy_dp h_dp\n"
      "bounds 0 500 0 400 0 300 1 1\n"
      "occupied 100.0 200.0 0.0\n"
      "occupied 100.0 201.0 0.0\n");

  dmap::SimulationState state;
  dmap::ErrorLogger logger;
  ASSERT_TRUE(dmap::loadGroundTruthMap(path, state, logger));
  ASSERT_TRUE(state.hasBounds());

  const auto bounds = state.mapBounds();
  EXPECT_NEAR(bounds.min_x.numerical_value_in(su::cm), 0.0, 1e-9);
  EXPECT_NEAR(bounds.max_x.numerical_value_in(su::cm), 500.0, 1e-9);
  EXPECT_NEAR(bounds.min_y.numerical_value_in(su::cm), 0.0, 1e-9);
  EXPECT_NEAR(bounds.max_y.numerical_value_in(su::cm), 400.0, 1e-9);
  EXPECT_EQ(bounds.xy_decimal_places, 1);
  EXPECT_EQ(bounds.height_decimal_places, 1);

  EXPECT_EQ(state.truthValue(dmap::Point3D{100.0 * su::cm, 200.0 * su::cm, 0.0 * su::cm}),
            dmap::MapValue::Occupied);
  EXPECT_EQ(state.truthValue(dmap::Point3D{100.0 * su::cm, 202.0 * su::cm, 0.0 * su::cm}),
            dmap::MapValue::Empty);
}

// Scenario: map_input file path does not exist.
// Expected: loader returns false without crashing.
// Why: caller uses this return value to detect missing required input.
TEST(ConfigParser, MapInputMissingFileReturnsFalse) {
  dmap::SimulationState state;
  dmap::ErrorLogger logger;
  EXPECT_FALSE(dmap::loadGroundTruthMap("nope.txt", state, logger));
}

// Scenario: drone config is fully valid.
// Expected: parser emits no ErrorLogger entries.
// Why: valid files should not generate input_errors output.
TEST(ConfigParser, ErrorLoggerDroneConfigCleanRunLogsNothing) {
  const auto path = writeTempFile(
      "dmap_drone_config_clean_errorlog_test.txt",
      "min_passable_width = 30\n"
      "lidar_fov_circles = 5\n");

  dmap::ErrorLogger logger;
  (void)dmap::parseDroneConfig(path, logger);
  EXPECT_TRUE(logger.empty());
}

// Scenario: drone config path is missing.
// Expected: parser logs one file-open failure entry.
// Why: missing files must be recoverable but reported.
TEST(ConfigParser, ErrorLoggerDroneConfigMissingFileLogged) {
  dmap::ErrorLogger logger;
  (void)dmap::parseDroneConfig("nope_errorlog.txt", logger);
  ASSERT_EQ(logger.size(), 1U);
  EXPECT_NE(logger.lines().front().find("[drone_config] could not open file"), std::string::npos);
}

// Scenario: drone config contains a non-numeric value for a numeric field.
// Expected: parser logs a bad-value entry and continues.
// Why: malformed values should be recoverable and diagnosable.
TEST(ConfigParser, ErrorLoggerDroneConfigBadValueLogged) {
  const auto path = writeTempFile("dmap_drone_config_bad_value_test.txt",
                                  "lidar_d = abc\n");

  dmap::ErrorLogger logger;
  (void)dmap::parseDroneConfig(path, logger);
  ASSERT_EQ(logger.size(), 1U);
  EXPECT_NE(logger.lines().front().find("bad value \"abc\""), std::string::npos);
}

// Scenario: drone config contains an unsupported key.
// Expected: parser logs an unknown-key entry.
// Why: typoed keys should not fail silently.
TEST(ConfigParser, ErrorLoggerDroneConfigUnknownKeyLogged) {
  const auto path = writeTempFile("dmap_drone_config_unknown_key_test.txt",
                                  "turbo_mode = 1\n");

  dmap::ErrorLogger logger;
  (void)dmap::parseDroneConfig(path, logger);
  ASSERT_EQ(logger.size(), 1U);
  EXPECT_NE(logger.lines().front().find("unknown key \"turbo_mode\""), std::string::npos);
}

// Scenario: map_input has an occupied record with wrong token count.
// Expected: loader logs a bad-record entry and skips the line.
// Why: malformed records should not stop the loader.
TEST(ConfigParser, ErrorLoggerMapInputBadRecordLogged) {
  const auto path = writeTempFile(
      "dmap_map_input_bad_record_test.txt",
      "bounds 0 500 0 400 0 300 1 1\n"
      "occupied 100.0 200.0\n");

  dmap::SimulationState state;
  dmap::ErrorLogger logger;
  ASSERT_TRUE(dmap::loadGroundTruthMap(path, state, logger));
  ASSERT_EQ(logger.size(), 1U);
  EXPECT_NE(logger.lines().front().find("bad \"occupied\" record"), std::string::npos);
}
