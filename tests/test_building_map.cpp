#include "common/Point3D.h"
#include "mapping/BuildingMap.h"

#include <gtest/gtest.h>

#include "config/MissionConfigParser.h"
#include "io/ErrorLogger.h"

TEST(BuildingMap, NotMappedByDefault) {
  dmap::ErrorLogger logger;
  const auto mission = dmap::parseMissionConfig("nonexistent_mission_config.txt", logger);
  dmap::BuildingMap map(mission);
  dmap::Point3D p{};
  EXPECT_EQ(map.get(p), dmap::MapValue::NotMapped);
}
