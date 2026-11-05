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


}
