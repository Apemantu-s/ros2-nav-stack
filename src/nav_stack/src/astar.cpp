#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include "nav_stack/planners.hpp"

namespace nav_stack {

static float heuristic(int x0, int y0, int x1, int y1) {
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int straight = std::abs(dx - dy);
  int diagonal = std::min(dx, dy);
  return straight + 1.41421356f * diagonal;
}

PlanResult planAStar(const CostMap& map, Point start, Point goal) {
  auto t0 = std::chrono::steady_clock::now();
  PlanResult res;
  const GridInfo& g = map.info();

  int sx, sy, gx, gy;
  g.worldToCell(start.x, start.y, sx, sy);
  g.worldToCell(goal.x, goal.y, gx, gy);
  if (!g.inBounds(sx, sy) || !g.inBounds(gx, gy) || map.lethal(gx, gy)) return res;

  const int n = g.width * g.height;
  std::vector<float> cost_so_far(n, 1e30f);
  std::vector<int> came_from(n, -1);
  std::vector<bool> done(n, false);

  using Entry = std::pair<float, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

  int start_idx = g.index(sx, sy);
  int goal_idx = g.index(gx, gy);
  cost_so_far[start_idx] = 0;
  open.push({heuristic(sx, sy, gx, gy), start_idx});

  bool start_lethal = map.lethal(sx, sy);

  // 8 neighbors
  int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!open.empty()) {
    int cur = open.top().second;
    open.pop();
    if (done[cur]) continue;
    done[cur] = true;
    res.expansions++;
    if (cur == goal_idx) break;

    int cx = cur % g.width;
    int cy = cur / g.width;
    bool cur_lethal = map.lethal(cx, cy);

    for (int k = 0; k < 8; ++k) {
      int nx = cx + dxs[k];
      int ny = cy + dys[k];
      if (!g.inBounds(nx, ny)) continue;
      int next = g.index(nx, ny);
      if (done[next]) continue;

      bool next_lethal = map.lethal(nx, ny);
      if (next_lethal && !(start_lethal && cur_lethal)) continue;

      float step = (k < 4) ? 1.0f : 1.41421356f;
      float cell_cost = next_lethal ? 50.0f : map.cost(nx, ny);
      float new_cost = cost_so_far[cur] + step * (1.0f + cell_cost);
      if (new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        came_from[next] = cur;
        open.push({new_cost + heuristic(nx, ny, gx, gy), next});
      }
    }
  }

  if (came_from[goal_idx] != -1 || goal_idx == start_idx) {
    std::vector<Point> path;
    int i = goal_idx;
    while (i != -1) {
      double wx, wy;
      g.cellToWorld(i % g.width, i / g.width, wx, wy);
      path.push_back({wx, wy});
      i = came_from[i];
    }
    std::reverse(path.begin(), path.end());
    path.front() = start;
    path.back() = goal;
    res.path = path;
    res.success = true;
  }
  res.time_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  return res;
}

}  // namespace nav_stack
