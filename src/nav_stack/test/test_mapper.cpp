#include <gtest/gtest.h>
#include <cmath>
#include "nav_stack/grid.hpp"

using namespace nav_stack;

TEST(Mapper, WallInFrontIsOccupiedAndSpaceIsFree) {
  GridInfo g;
  g.width = g.height = 100;
  g.resolution = 0.05;
  g.origin_x = g.origin_y = -2.5;
  OccupancyMapper m(g, OccupancyMapper::Params{});
  std::vector<float> ranges = {2.0f};
  for (int i = 0; i < 5; ++i) m.addScan(0, 0, 0, 0.0, 0.01, ranges, 0.1);
  auto occ = m.toOccupancy();
  int cx, cy;
  g.worldToCell(2.0, 0.0, cx, cy);
  EXPECT_GT(occ[g.index(cx, cy)], 65);
  g.worldToCell(1.0, 0.0, cx, cy);
  EXPECT_LT(occ[g.index(cx, cy)], 35);
  g.worldToCell(0.0, 1.0, cx, cy);
  EXPECT_EQ(occ[g.index(cx, cy)], -1);
}

TEST(Mapper, CellsStopAtTheCap) {
  GridInfo g;
  g.width = g.height = 100;
  g.resolution = 0.05;
  g.origin_x = g.origin_y = -2.5;
  OccupancyMapper::Params p;
  OccupancyMapper m(g, p);
  std::vector<float> ranges = {2.0f};
  for (int i = 0; i < 100; ++i) m.addScan(0, 0, 0, 0.0, 0.01, ranges, 0.1);
  int cx, cy;
  g.worldToCell(2.0, 0.0, cx, cy);
  EXPECT_FLOAT_EQ(m.logOdds()[g.index(cx, cy)], p.l_max);
  g.worldToCell(1.0, 0.0, cx, cy);
  EXPECT_FLOAT_EQ(m.logOdds()[g.index(cx, cy)], p.l_min);
}

TEST(Mapper, NoHitClearsButNeverMarksAWall) {
  GridInfo g;
  g.width = g.height = 100;
  g.resolution = 0.05;
  g.origin_x = g.origin_y = -2.5;
  OccupancyMapper m(g, OccupancyMapper::Params{});
  std::vector<float> ranges = {std::numeric_limits<float>::infinity()};
  m.addScan(0, 0, 0, 0.0, 0.01, ranges, 0.1);
  auto occ = m.toOccupancy();
  int cx, cy;
  g.worldToCell(2.0, 0.0, cx, cy);
  EXPECT_LT(occ[g.index(cx, cy)], 50);
  for (int8_t v : occ) EXPECT_LT(v, 50);
}
