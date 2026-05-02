#include "config/DroneConfigParser.h"
#include "config/MissionConfigParser.h"

#include <gtest/gtest.h>

TEST(ConfigParser, DroneConfigParses) { EXPECT_NO_THROW(dmap::parseDroneConfig("nope.txt")); }

TEST(ConfigParser, MissionConfigParses) {
  EXPECT_NO_THROW(dmap::parseMissionConfig("nope.txt"));
}
