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

size_t closestIndex(const std::vector<Point>& path, const Pose2D& pose, size_t from = 0);
double crossTrackError(const std::vector<Point>& path, const Pose2D& pose);

class PurePursuit {
 public:
  struct Params {
    double lookahead = 0.5;
    double rotate_in_place_angle = 1.2;
    double slow_down_dist = 0.6;
  };
  PurePursuit(const Params& p, const ControlLimits& lim) : p_(p), lim_(lim) {}
  Cmd compute(const std::vector<Point>& path, const Pose2D& pose);

 private:
  Params p_;
  ControlLimits lim_;
};

class Mpc {
 public:
  struct Params {
    int horizon = 15;
    double dt = 0.1;
    double w_pos = 10.0;
    double w_heading = 1.0;
    double w_v = 0.5;
    double w_dv = 2.0;
    double w_dw = 0.5;
    double w_w = 0.05;
    int iterations = 40;
    double slow_down_dist = 0.6;
  };
  Mpc(const Params& p, const ControlLimits& lim);
  Cmd compute(const std::vector<Point>& path, const Pose2D& pose);
  void reset();

 private:
  struct Ref {
    double x, y, theta, v;
  };
  std::vector<Ref> buildReference(const std::vector<Point>& path, const Pose2D& pose) const;
  double cost(const Pose2D& start, const std::vector<double>& v, const std::vector<double>& w,
              const std::vector<Ref>& ref) const;
  void clampControls(std::vector<double>& v, std::vector<double>& w) const;

  Params p_;
  ControlLimits lim_;
  std::vector<double> v_, w_;
  double last_v_ = 0.0, last_w_ = 0.0;
  double step_size_ = 0.01;
};

}