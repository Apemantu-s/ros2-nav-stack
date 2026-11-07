#include <algorithm>
#include <cmath>
#include <limits>
#include "nav_stack/controllers.hpp"

namespace nav_stack {

size_t closestIndex(const std::vector<Point>& path, const Pose2D& pose, size_t from) {
  size_t best = from;
  double best_d = std::numeric_limits<double>::max();
  for (size_t i = from; i < path.size(); ++i) {
    double d = std::hypot(path[i].x - pose.x, path[i].y - pose.y);
    if (d < best_d) { best_d = d; best = i; }
  }
  return best;
}

std::vector<Point> resamplePath(const std::vector<Point>& path, double spacing) {
  std::vector<Point> out;
  if (path.empty()) return out;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const Point& a = path[i];
    const Point& b = path[i + 1];
    int n = static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / spacing));
    if (n < 1) n = 1;
    for (int k = 0; k < n; ++k) {
      double t = static_cast<double>(k) / n;
      out.push_back({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)});
    }
  }
  out.push_back(path.back());
  return out;
}

double crossTrackError(const std::vector<Point>& path, const Pose2D& pose) {
  if (path.empty()) return 0.0;
  if (path.size() == 1) return std::hypot(path[0].x - pose.x, path[0].y - pose.y);
  double best = std::numeric_limits<double>::max();
  for (size_t i = 1; i < path.size(); ++i) {
    const double ax = path[i - 1].x, ay = path[i - 1].y;
    const double bx = path[i].x - ax, by = path[i].y - ay;
    const double L2 = bx * bx + by * by;
    double t = L2 > 1e-12 ? ((pose.x - ax) * bx + (pose.y - ay) * by) / L2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    best = std::min(best, std::hypot(ax + t * bx - pose.x, ay + t * by - pose.y));
  }
  return best;
}

Cmd PurePursuit::compute(const std::vector<Point>& path, const Pose2D& pose) {
  Cmd cmd;
  if (path.empty()) return cmd;
  const size_t ci = closestIndex(path, pose);

  size_t ti = ci;
  while (ti + 1 < path.size() &&
         std::hypot(path[ti].x - pose.x, path[ti].y - pose.y) < p_.lookahead)
    ++ti;
  const Point& target = path[ti];

  const double dx = target.x - pose.x, dy = target.y - pose.y;
  const double c = std::cos(pose.theta), s = std::sin(pose.theta);
  const double lx = c * dx + s * dy;
  const double ly = -s * dx + c * dy;
  const double alpha = std::atan2(ly, lx);

  if (std::abs(alpha) > p_.rotate_in_place_angle) {
    cmd.v = 0.0;
    cmd.w = std::copysign(lim_.w_max * 0.8, alpha);
    return cmd;
  }

  const double Ld2 = std::max(lx * lx + ly * ly, 1e-6);
  const double curvature = 2.0 * ly / Ld2;

  const double goal_dist = std::hypot(path.back().x - pose.x, path.back().y - pose.y);
  double v = lim_.v_max / (1.0 + 1.5 * std::abs(curvature));
  v = std::min(v, lim_.v_max * std::clamp(goal_dist / p_.slow_down_dist, 0.15, 1.0));

  double w = v * curvature;
  if (std::abs(w) > lim_.w_max) {
    v *= lim_.w_max / std::abs(w);
    w = std::copysign(lim_.w_max, w);
  }
  cmd.v = v;
  cmd.w = w;
  return cmd;
}

}