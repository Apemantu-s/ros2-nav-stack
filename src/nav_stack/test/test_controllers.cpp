#include <gtest/gtest.h>
#include <cmath>
#include "nav_stack/controllers.hpp"

using namespace nav_stack;

template <typename C>
double simulate(C& ctrl, const std::vector<Point>& path, Pose2D pose, bool& reached) {
  double worst = 0;
  reached = false;
  for (int i = 0; i < 2000; ++i) {
    Cmd c = ctrl.compute(path, pose);
    pose.x += c.v * std::cos(pose.theta) * 0.05;
    pose.y += c.v * std::sin(pose.theta) * 0.05;
    pose.theta += c.w * 0.05;
    worst = std::max(worst, crossTrackError(path, pose));
    if (std::hypot(pose.x - path.back().x, pose.y - path.back().y) < 0.2) {
      reached = true;
      break;
    }
  }
  return worst;
}

static std::vector<Point> lPath() {
  std::vector<Point> p;
  for (int i = 0; i <= 40; ++i) p.push_back({i * 0.1, 0});
  for (int i = 1; i <= 40; ++i) p.push_back({4, i * 0.1});
  return p;
}

TEST(CrossTrackError, DistanceToSegmentsNotPoints) {
  std::vector<Point> path = {{0, 0}, {2, 0}};
  EXPECT_NEAR(crossTrackError(path, {1.0, 0.3, 0}), 0.3, 1e-9);
  EXPECT_NEAR(crossTrackError(path, {3.0, 0.0, 0}), 1.0, 1e-9);
}

TEST(ClosestIndex, FindsNearestPoint) {
  auto p = lPath();
  EXPECT_EQ(closestIndex(p, {1.02, 0.1, 0}), 10u);
  EXPECT_EQ(closestIndex(p, {4.1, 2.0, 0}), 60u);
}

TEST(PurePursuit, FollowsLPath) {
  PurePursuit pp(PurePursuit::Params{}, ControlLimits{});
  bool reached;
  double worst = simulate(pp, lPath(), {0, 0.3, 0}, reached);
  EXPECT_TRUE(reached);
  EXPECT_LT(worst, 0.4);
}

TEST(PurePursuit, TurnsInPlaceWhenFacingAway) {
  PurePursuit pp(PurePursuit::Params{}, ControlLimits{});
  Cmd c = pp.compute(lPath(), {0, 0, M_PI});
  EXPECT_EQ(c.v, 0.0);
  EXPECT_NE(c.w, 0.0);
}

TEST(Mpc, FollowsLPath) {
  Mpc mpc(Mpc::Params{}, ControlLimits{});
  bool reached;
  double worst = simulate(mpc, lPath(), {0, 0.3, 0}, reached);
  EXPECT_TRUE(reached);
  EXPECT_LT(worst, 0.4);
}

TEST(Mpc, RespectsLimits) {
  ControlLimits lim;
  Mpc mpc(Mpc::Params{}, lim);
  Cmd c = mpc.compute(lPath(), {0, 0, M_PI});
  EXPECT_LE(c.v, lim.v_max + 1e-9);
  EXPECT_GE(c.v, 0.0);
  EXPECT_LE(std::abs(c.w), lim.w_max + 1e-9);
}

TEST(Mpc, DoesntCircleWhenHeadingCrossesPi) {
  std::vector<Point> path;
  for (int i = 0; i <= 40; ++i) path.push_back({-i * 0.1, 0});
  Mpc mpc(Mpc::Params{}, ControlLimits{});
  Pose2D pose{0, 0, -3.10};
  double turned = 0;
  for (int i = 0; i < 400; ++i) {
    Cmd c = mpc.compute(path, pose);
    turned += c.w * 0.05;
    pose.x += c.v * std::cos(pose.theta) * 0.05;
    pose.y += c.v * std::sin(pose.theta) * 0.05;
    pose.theta += c.w * 0.05;
  }
  EXPECT_LT(std::abs(turned), 0.5);
}


TEST(ResamplePath, AddsPointsButKeepsShape) {
  std::vector<Point> path = {{0, 0}, {1, 0}, {1, 0.5}};
  auto r = resamplePath(path, 0.1);
  EXPECT_EQ(r.size(), 16u);
  for (size_t i = 1; i < r.size(); ++i)
    EXPECT_LE(std::hypot(r[i].x - r[i - 1].x, r[i].y - r[i - 1].y), 0.1 + 1e-9);
  EXPECT_DOUBLE_EQ(r.back().x, 1.0);
  EXPECT_DOUBLE_EQ(r.back().y, 0.5);
}

TEST(Mpc, FollowsSparsePathAfterResampling) {
  std::vector<Point> sparse = {{0, 0}, {4, 0}, {4, 4}};
  Mpc mpc(Mpc::Params{}, ControlLimits{});
  bool reached;
  double err = simulate(mpc, resamplePath(sparse, 0.05), {0, 0, 0}, reached);
  EXPECT_TRUE(reached);
  EXPECT_LT(err, 0.15);
}