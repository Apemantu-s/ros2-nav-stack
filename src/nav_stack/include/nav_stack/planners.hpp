#pragma once
#include <cstdint>
#include <random>
#include <vector>
#include "nav_stack/grid.hpp"

namespace nav_stack {

struct Point {
  double x, y;
};

double pathLength(const std::vector<Point>& path);


class CostMap {
 public:
  struct Params {
    int occupied_threshold = 65;
    double lethal_radius = 0.24;
    double cost_radius = 0.6;
    double cost_scale = 3.0;
    bool unknown_is_free = true;
  };

  CostMap(const GridInfo& info, const std::vector<int8_t>& occ, const Params& p);

  const GridInfo& info() const { return info_; }
  bool lethal(int cx, int cy) const;
  float cost(int cx, int cy) const { return cost_[info_.index(cx, cy)]; }
  float distance(int cx, int cy) const { return dist_[info_.index(cx, cy)]; }
  bool segmentFree(double x0, double y0, double x1, double y1) const;
  bool pointFree(double x, double y) const;

 private:
  GridInfo info_;
  Params p_;
  std::vector<float> dist_;
  std::vector<float> cost_;
};

struct PlanResult {
  bool success = false;
  std::vector<Point> path;
  double time_ms = 0.0;
  int expansions = 0;
};

PlanResult planAStar(const CostMap& map, Point start, Point goal);

struct RrtStarParams {
  int max_iterations = 5000;
  double step = 0.3;
  double goal_bias = 0.1;
  double goal_tolerance = 0.2;
  double max_rewire_radius = 1.0;
  double time_budget_ms = 300.0;
  bool stop_at_first_solution = false;
};

PlanResult planRrtStar(const CostMap& map, Point start, Point goal, const RrtStarParams& p, std::mt19937& rng);

}