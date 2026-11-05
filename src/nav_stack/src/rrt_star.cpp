#include <algorithm>
#include <chrono>
#include <cmath>
#include "nav_stack/planners.hpp"

namespace nav_stack {

namespace {

struct Node {
  Point p;
  int parent;
};

double dist(const Point& a, const Point& b) { return std::hypot(a.x - b.x, a.y - b.y); }

double costOf(const std::vector<Node>& tree, int i) {
  double c = 0;
  while (tree[i].parent != -1) {
    c += dist(tree[i].p, tree[tree[i].parent].p);
    i = tree[i].parent;
  }
  return c;
}

bool edgeFree(const CostMap& map, const Point& a, const Point& b, bool from_lethal_start) {
  if (!from_lethal_start) return map.segmentFree(a.x, a.y, b.x, b.y);
  double d = dist(a, b);
  if (d < 0.15) return map.pointFree(b.x, b.y);
  double t = 0.15 / d;
  return map.segmentFree(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), b.x, b.y);
}

}  // namespace

PlanResult planRrtStar(const CostMap& map, Point start, Point goal, const RrtStarParams& p,
                       std::mt19937& rng) {
  auto t0 = std::chrono::steady_clock::now();
  auto elapsed_ms = [&] {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  };
  PlanResult res;
  if (!map.pointFree(goal.x, goal.y)) return res;

  const GridInfo& g = map.info();
  std::uniform_real_distribution<double> rand_x(g.origin_x, g.origin_x + g.width * g.resolution);
  std::uniform_real_distribution<double> rand_y(g.origin_y, g.origin_y + g.height * g.resolution);
  std::uniform_real_distribution<double> rand01(0.0, 1.0);
  bool start_lethal = !map.pointFree(start.x, start.y);

  const double area = g.width * g.resolution * g.height * g.resolution;
  const double gamma = 2.0 * std::sqrt(1.5 * area / M_PI);

  std::vector<Node> tree;
  tree.push_back({start, -1});
  std::vector<int> goal_nodes;

  for (int it = 0; it < p.max_iterations; ++it) {
    if (elapsed_ms() > p.time_budget_ms) break;
    if (p.stop_at_first_solution && !goal_nodes.empty()) break;

    Point sample;
    if (rand01(rng) < p.goal_bias) {
      sample = goal;
    } else {
      sample = {rand_x(rng), rand_y(rng)};
    }

    int nearest = 0;
    for (size_t i = 1; i < tree.size(); ++i) {
      if (dist(tree[i].p, sample) < dist(tree[nearest].p, sample)) nearest = i;
    }

    Point new_p = sample;
    double d = dist(tree[nearest].p, sample);
    if (d > p.step) {
      new_p.x = tree[nearest].p.x + (sample.x - tree[nearest].p.x) * p.step / d;
      new_p.y = tree[nearest].p.y + (sample.y - tree[nearest].p.y) * p.step / d;
    }
    if (!map.pointFree(new_p.x, new_p.y)) continue;
    if (!edgeFree(map, tree[nearest].p, new_p, nearest == 0 && start_lethal)) continue;

    double n = tree.size() + 1;
    double radius = std::min(p.max_rewire_radius, gamma * std::sqrt(std::log(n) / n));
    std::vector<int> near;
    for (size_t i = 0; i < tree.size(); ++i) {
      if (dist(tree[i].p, new_p) <= radius) near.push_back(i);
    }

    int best_parent = nearest;
    double best_cost = costOf(tree, nearest) + dist(tree[nearest].p, new_p);
    for (int i : near) {
      double c = costOf(tree, i) + dist(tree[i].p, new_p);
      if (c < best_cost && edgeFree(map, tree[i].p, new_p, i == 0 && start_lethal)) {
        best_cost = c;
        best_parent = i;
      }
    }
    tree.push_back({new_p, best_parent});
    int new_idx = tree.size() - 1;

    for (int i : near) {
      if (i == best_parent || i == 0) continue;
      double c = best_cost + dist(new_p, tree[i].p);
      if (c < costOf(tree, i) && map.segmentFree(new_p.x, new_p.y, tree[i].p.x, tree[i].p.y)) {
        tree[i].parent = new_idx;
      }
    }

    if (dist(new_p, goal) <= p.goal_tolerance) goal_nodes.push_back(new_idx);
  }

  res.expansions = tree.size();

  int best_goal = -1;
  for (int i : goal_nodes) {
    if (best_goal == -1 || costOf(tree, i) < costOf(tree, best_goal)) best_goal = i;
  }
  if (best_goal != -1) {
    std::vector<Point> path;
    for (int i = best_goal; i != -1; i = tree[i].parent) path.push_back(tree[i].p);
    std::reverse(path.begin(), path.end());
    path.push_back(goal);
    res.path = path;
    res.success = true;
  }
  res.time_ms = elapsed_ms();
  return res;
}

}