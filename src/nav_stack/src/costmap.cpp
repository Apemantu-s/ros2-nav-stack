#include <algorithm>
#include <cmath>
#include "nav_stack/planners.hpp"

namespace nav_stack {

double pathLength(const std::vector<Point>& path) {
  double len = 0;
  for (size_t i = 1; i < path.size(); ++i) {
    len += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return len;
}

CostMap::CostMap(const GridInfo& info, const std::vector<int8_t>& occ, const Params& p)
    : info_(info), p_(p) {
  dist_.assign(occ.size(), p_.cost_radius);

  const int r_cells = static_cast<int>(std::ceil(p_.cost_radius / info_.resolution));
  for (int cy = 0; cy < info_.height; ++cy) {
    for (int cx = 0; cx < info_.width; ++cx) {
      int8_t value = occ[info_.index(cx, cy)];
      bool is_obstacle;
      if (value < 0) {
        is_obstacle = !p_.unknown_is_free;
      } else {
        is_obstacle = value >= p_.occupied_threshold;
      }
      if (!is_obstacle) continue;

      for (int dy = -r_cells; dy <= r_cells; ++dy) {
        for (int dx = -r_cells; dx <= r_cells; ++dx) {
          int nx = cx + dx;
          int ny = cy + dy;
          if (!info_.inBounds(nx, ny)) continue;
          float d = std::hypot(dx, dy) * info_.resolution;
          float& current = dist_[info_.index(nx, ny)];
          if (d < current) current = d;
        }
      }
    }
  }

  cost_.resize(dist_.size());
  for (size_t i = 0; i < dist_.size(); ++i) {
    float d = dist_[i];
    if (d <= p_.lethal_radius) {
      cost_[i] = 1e9f;
    } else if (d >= p_.cost_radius) {
      cost_[i] = 0.0f;
    } else {
      cost_[i] = p_.cost_scale * (p_.cost_radius - d) / (p_.cost_radius - p_.lethal_radius);
    }
  }
}

bool CostMap::lethal(int cx, int cy) const {
  if (!info_.inBounds(cx, cy)) return true;
  return dist_[info_.index(cx, cy)] <= p_.lethal_radius;
}

bool CostMap::pointFree(double x, double y) const {
  int cx, cy;
  info_.worldToCell(x, y, cx, cy);
  return !lethal(cx, cy);
}

bool CostMap::segmentFree(double x0, double y0, double x1, double y1) const {
  double len = std::hypot(x1 - x0, y1 - y0);
  int steps = static_cast<int>(std::ceil(len / (0.5 * info_.resolution)));
  if (steps < 1) steps = 1;
  for (int i = 0; i <= steps; ++i) {
    double t = static_cast<double>(i) / steps;
    if (!pointFree(x0 + t * (x1 - x0), y0 + t * (y1 - y0))) return false;
  }
  return true;
}

}
