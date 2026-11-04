#pragma once
#include <cstdint>
#include <vector>

namespace nav_stack {

struct GridInfo {
  int width = 0;
  int height = 0;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;

  bool inBounds(int cx, int cy) const { return cx >= 0 && cy >= 0 && cx < width && cy < height; }
  int index(int cx, int cy) const { return cy * width + cx; }
  void worldToCell(double x, double y, int& cx, int& cy) const;
  void cellToWorld(int cx, int cy, double& x, double& y) const;
};

class OccupancyMapper {
 public:
  struct Params {
    double l_hit = 0.85;
    double l_miss = -0.4;
    double l_min = -2.0;
    double l_max = 3.5;
    double max_range = 5.0;
    double free_range_for_max = 4.0;  
  };

  OccupancyMapper(const GridInfo& info, const Params& p);


  void addScan(double sensor_x, double sensor_y, double sensor_yaw, double angle_min,
               double angle_inc, const std::vector<float>& ranges, double range_min);

  std::vector<int8_t> toOccupancy() const;
  const GridInfo& info() const { return info_; }
  const std::vector<float>& logOdds() const { return lo_; }

 private:
  void traceRay(int x0, int y0, int x1, int y1, bool mark_end);
  GridInfo info_;
  Params p_;
  std::vector<float> lo_;
};

} 
