#pragma once
#include <vector>
#include "nav_stack/planners.hpp"

namespace nav_stack {

struct Pose2D {
  double x, y, theta;
};

struct Cmd {
  double v = 0.0;
  double w = 0.0;
};

struct ControlLimits {
  double v_max = 0.5;
  double w_max = 1.5;
};

// index of closest path point to the robot (starts looking at `from`)
size_t closestIndex(const std::vector<Point>& path, const Pose2D& pose, size_t from = 0);
// cross track error = distance from robot to the path line segments
double crossTrackError(const std::vector<Point>& path, const Pose2D& pose);

// follow the path by steering toward a point one lookahead ahead on it
class PurePursuit {
 public:
  struct Params {
    double lookahead = 0.5;
    double rotate_in_place_angle = 1.2;  // rad. if target is more than this off, just spin in place
    double slow_down_dist = 0.6;         // start slowing down this close to goal
  };
  PurePursuit(const Params& p, const ControlLimits& lim) : p_(p), lim_(lim) {}
  Cmd compute(const std::vector<Point>& path, const Pose2D& pose);

 private:
  Params p_;
  ControlLimits lim_;
};

}