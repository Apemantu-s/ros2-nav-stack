#include "nav_stack/grid.hpp"
#include <algorithm>
#include <cmath>

namespace nav_stack {

void GridInfo::worldToCell(double x, double y, int& cx, int& cy) const {
  cx = static_cast<int>(std::floor((x - origin_x) / resolution));
  cy = static_cast<int>(std::floor((y - origin_y) / resolution));
}

void GridInfo::cellToWorld(int cx, int cy, double& x, double& y) const {
  x = origin_x + (cx + 0.5) * resolution;
  y = origin_y + (cy + 0.5) * resolution;
}

OccupancyMapper::OccupancyMapper(const GridInfo& info, const Params& p)
    : info_(info), p_(p), lo_(info.width * info.height, 0.0f) {}

void OccupancyMapper::traceRay(int x0, int y0, int x1, int y1, bool mark_end) {
  // bresenham. cells before the end are free
  int dx = std::abs(x1 - x0), dy = -std::abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  int x = x0, y = y0;
  while (!(x == x1 && y == y1)) {
    if (info_.inBounds(x, y)) {
      float& l = lo_[info_.index(x, y)];
      l = std::max<float>(p_.l_min, l + p_.l_miss);
    }
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x += sx; }
    if (e2 <= dx) { err += dx; y += sy; }
  }
  if (info_.inBounds(x1, y1)) {
    float& l = lo_[info_.index(x1, y1)];
    if (mark_end)
      l = std::min<float>(p_.l_max, l + p_.l_hit);
    else
      l = std::max<float>(p_.l_min, l + p_.l_miss);
  }
}

void OccupancyMapper::addScan(double sx, double sy, double syaw, double angle_min, double angle_inc,
                              const std::vector<float>& ranges, double range_min) {
  int cx0, cy0;
  info_.worldToCell(sx, sy, cx0, cy0);
  for (size_t i = 0; i < ranges.size(); ++i) {
    double r = ranges[i];
    bool hit = true;
    if (!std::isfinite(r) || r >= p_.max_range) {
      // no hit or a really long one. long ones can be the floor if the robot
      // pitches. only use them to clear space close by
      r = p_.free_range_for_max;
      hit = false;
    }
    if (r < range_min) continue;
    const double a = syaw + angle_min + i * angle_inc;
    int cx1, cy1;
    info_.worldToCell(sx + r * std::cos(a), sy + r * std::sin(a), cx1, cy1);
    traceRay(cx0, cy0, cx1, cy1, hit);
  }
}

std::vector<int8_t> OccupancyMapper::toOccupancy() const {
  std::vector<int8_t> out(lo_.size());
  for (size_t i = 0; i < lo_.size(); ++i) {
    if (lo_[i] == 0.0f) {
      out[i] = -1;
    } else {
      const double p = 1.0 - 1.0 / (1.0 + std::exp(lo_[i]));
      out[i] = static_cast<int8_t>(std::lround(p * 100.0));
    }
  }
  return out;
}

}  // namespace nav_stack
