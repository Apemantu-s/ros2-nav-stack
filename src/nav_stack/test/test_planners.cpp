#include <gtest/gtest.h>
#include "nav_stack/planners.hpp"

using namespace nav_stack;

static CostMap wallMap() {
  GridInfo g;
  g.width = g.height = 200;
  g.resolution = 0.05;
  g.origin_x = g.origin_y = 0;
  std::vector<int8_t> occ(g.width * g.height, 0);
  for (int y = 0; y < 160; ++y)
    for (int x = 98; x < 102; ++x) occ[g.index(x, y)] = 100;
  return CostMap(g, occ, CostMap::Params{});
}

TEST(CostMap, DistanceGrowsAwayFromWall) {
  CostMap cm = wallMap();
  EXPECT_NEAR(cm.distance(101, 50), 0.0, 1e-6);
  EXPECT_NEAR(cm.distance(102, 50), 0.05, 1e-6);
  EXPECT_NEAR(cm.distance(103, 50), 0.10, 1e-6);
  EXPECT_NEAR(cm.distance(150, 50), 0.6, 1e-6);
}

TEST(CostMap, LethalAndSoftCost) {
  CostMap cm = wallMap();
  EXPECT_TRUE(cm.lethal(105, 50));
  EXPECT_FALSE(cm.lethal(106, 50));
  EXPECT_NEAR(cm.cost(106, 50), 3.0 * (0.6 - 0.25) / (0.6 - 0.24), 1e-4);
  EXPECT_GT(cm.cost(106, 50), cm.cost(107, 50));
  EXPECT_EQ(cm.cost(150, 50), 0.0f);
  EXPECT_TRUE(cm.lethal(-1, 50));
}

TEST(CostMap, SegmentFree) {
  CostMap cm = wallMap();
  EXPECT_FALSE(cm.segmentFree(2, 2, 8, 2));
  EXPECT_TRUE(cm.segmentFree(2, 2, 2, 8));
  EXPECT_TRUE(cm.segmentFree(2, 9, 8, 9));


TEST(AStar, GoesAroundWall) {
  CostMap cm = wallMap();
  auto r = planAStar(cm, {2, 2}, {8, 2});
  ASSERT_TRUE(r.success);
  // straight line = 6m, going around is way longer
  EXPECT_GT(pathLength(r.path), 12.0);
  for (size_t i = 1; i < r.path.size(); ++i)
    EXPECT_TRUE(cm.segmentFree(r.path[i - 1].x, r.path[i - 1].y, r.path[i].x, r.path[i].y));
}

TEST(AStar, GoalInWallFails) {
  CostMap cm = wallMap();
  EXPECT_FALSE(planAStar(cm, {2, 2}, {5, 2}).success);
}



}