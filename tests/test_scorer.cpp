#include "mapping/Scorer.h"

#include <gtest/gtest.h>

#include "mapping/BuildingMap.h"

#include "config/MissionConfigParser.h"

TEST(Scorer, ReturnsStubValue) {
  const auto mission = dmap::parseMissionConfig("nope.txt");
  dmap::BuildingMap a(mission);
  dmap::BuildingMap b(mission);
  dmap::Scorer scorer;
  EXPECT_DOUBLE_EQ(scorer.scorePercent(a, b), 0.0);
}
