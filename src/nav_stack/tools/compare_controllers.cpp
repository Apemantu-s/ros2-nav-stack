#include <chrono>
#include <cmath>
#include <cstdio>
#include "nav_stack/controllers.hpp"

using namespace nav_stack;

static std::vector<Point> lPath() {
  std::vector<Point> p;
  for (int i = 0; i <= 40; ++i) p.push_back({i * 0.1, 0});
  for (int i = 1; i <= 40; ++i) p.push_back({4, i * 0.1});
  return p;
}

template <typename C>
void run(const char* name, C& ctrl) {
  const auto path = lPath();
  Pose2D pose{0, 0.3, 0};
  double worst = 0, steering = 0, prev_w = 0, total_ms = 0, max_ms = 0;
  int steps = 0;
  bool reached = false;
  for (int i = 0; i < 2000; ++i) {
    auto t0 = std::chrono::steady_clock::now();
    Cmd c = ctrl.compute(path, pose);
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    total_ms += ms;
    max_ms = std::max(max_ms, ms);

    steering += std::abs(c.w - prev_w);
    prev_w = c.w;

    pose.x += c.v * std::cos(pose.theta) * 0.05;
    pose.y += c.v * std::sin(pose.theta) * 0.05;
    pose.theta += c.w * 0.05;
    steps = i + 1;

    if (i > 40) worst = std::max(worst, crossTrackError(path, pose));
    if (std::hypot(pose.x - path.back().x, pose.y - path.back().y) < 0.2) {
      reached = true;
      break;
    }
  }
  std::printf("%-26s reached %d  time %5.1f s  worst %.3f m  steering %5.1f  %.2f ms/call (max %.2f)\n",
              name, reached, steps * 0.05, worst, steering, total_ms / steps, max_ms);
}

int main() {
  { PurePursuit c(PurePursuit::Params{}, ControlLimits{}); run("pure pursuit", c); }
  { Mpc c(Mpc::Params{}, ControlLimits{}); run("mpc", c); }
  { Mpc::Params p; p.w_heading = 0; Mpc c(p, ControlLimits{}); run("mpc, no heading term", c); }
  { Mpc::Params p; p.w_dv = 0; p.w_dw = 0; Mpc c(p, ControlLimits{}); run("mpc, no smoothness terms", c); }
  { Mpc::Params p; p.horizon = 5; Mpc c(p, ControlLimits{}); run("mpc, horizon 5", c); }
  { Mpc::Params p; p.horizon = 30; Mpc c(p, ControlLimits{}); run("mpc, horizon 30", c); }
}
