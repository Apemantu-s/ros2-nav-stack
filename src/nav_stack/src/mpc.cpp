#include <algorithm>
#include <cmath>
#include "nav_stack/controllers.hpp"

namespace nav_stack {

Mpc::Mpc(const Params& p, const ControlLimits& lim) : p_(p), lim_(lim) { reset(); }

void Mpc::reset() {
  v_.assign(p_.horizon, 0.0);
  w_.assign(p_.horizon, 0.0);
  last_v_ = 0.0;
  last_w_ = 0.0;
  step_size_ = 0.01;
}

static double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

static double remainingLength(const std::vector<Point>& path, size_t seg, double x, double y) {
  if (seg + 1 >= path.size()) return 0.0;
  double d = std::hypot(path[seg + 1].x - x, path[seg + 1].y - y);
  for (size_t i = seg + 2; i < path.size(); ++i) {
    d += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return d;
}

std::vector<Mpc::Ref> Mpc::buildReference(const std::vector<Point>& path, const Pose2D& pose) const {
  std::vector<Ref> ref;
  size_t seg = closestIndex(path, pose);
  double px = path[seg].x;
  double py = path[seg].y;

  for (int k = 0; k < p_.horizon; ++k) {
    double to_goal = remainingLength(path, seg, px, py);
    double v_ref = lim_.v_max * std::clamp(to_goal / p_.slow_down_dist, 0.0, 1.0);

    double step = v_ref * p_.dt;
    while (step > 1e-9 && seg + 1 < path.size()) {
      double d = std::hypot(path[seg + 1].x - px, path[seg + 1].y - py);
      if (d <= step) {
        step -= d;
        seg++;
        px = path[seg].x;
        py = path[seg].y;
      } else {
        px += step / d * (path[seg + 1].x - px);
        py += step / d * (path[seg + 1].y - py);
        step = 0;
      }
    }

    double th = pose.theta;
    if (seg + 1 < path.size()) {
      th = std::atan2(path[seg + 1].y - path[seg].y, path[seg + 1].x - path[seg].x);
    } else if (path.size() >= 2) {
      size_t n = path.size();
      th = std::atan2(path[n - 1].y - path[n - 2].y, path[n - 1].x - path[n - 2].x);
    }
    ref.push_back({px, py, th, v_ref});
  }
  return ref;
}

double Mpc::cost(const Pose2D& start, const std::vector<double>& v, const std::vector<double>& w,
                 const std::vector<Ref>& ref) const {
  double x = start.x, y = start.y, th = start.theta;
  double total = 0.0;
  for (int k = 0; k < p_.horizon; ++k) {
    double th_mid = th + 0.5 * w[k] * p_.dt;
    x += v[k] * std::cos(th_mid) * p_.dt;
    y += v[k] * std::sin(th_mid) * p_.dt;
    th += w[k] * p_.dt;

    double ex = x - ref[k].x;
    double ey = y - ref[k].y;
    double eth = wrapAngle(th - ref[k].theta);
    double prev_v = (k == 0) ? last_v_ : v[k - 1];
    double prev_w = (k == 0) ? last_w_ : w[k - 1];

    total += p_.w_pos * (ex * ex + ey * ey);
    total += p_.w_heading * eth * eth;
    total += p_.w_v * (v[k] - ref[k].v) * (v[k] - ref[k].v);
    total += p_.w_w * w[k] * w[k];
    total += p_.w_dv * (v[k] - prev_v) * (v[k] - prev_v);
    total += p_.w_dw * (w[k] - prev_w) * (w[k] - prev_w);
  }
  return total;
}

void Mpc::clampControls(std::vector<double>& v, std::vector<double>& w) const {
  for (int k = 0; k < p_.horizon; ++k) {
    v[k] = std::clamp(v[k], 0.0, lim_.v_max);
    w[k] = std::clamp(w[k], -lim_.w_max, lim_.w_max);
  }
}

Cmd Mpc::compute(const std::vector<Point>& path, const Pose2D& pose) {
  Cmd cmd;
  if (path.empty()) return cmd;
  std::vector<Ref> ref = buildReference(path, pose);

  std::vector<double> v(v_.begin() + 1, v_.end());
  std::vector<double> w(w_.begin() + 1, w_.end());
  v.push_back(v_.back());
  w.push_back(w_.back());
  clampControls(v, w);

  double current = cost(pose, v, w, ref);
  const double eps = 1e-5;

  for (int it = 0; it < p_.iterations; ++it) {
    std::vector<double> grad_v(p_.horizon), grad_w(p_.horizon);
    for (int k = 0; k < p_.horizon; ++k) {
      std::vector<double> v2 = v;
      v2[k] += eps;
      grad_v[k] = (cost(pose, v2, w, ref) - current) / eps;
      std::vector<double> w2 = w;
      w2[k] += eps;
      grad_w[k] = (cost(pose, v, w2, ref) - current) / eps;
    }

    bool improved = false;
    for (int tries = 0; tries < 10; ++tries) {
      std::vector<double> v_new = v, w_new = w;
      for (int k = 0; k < p_.horizon; ++k) {
        v_new[k] -= step_size_ * grad_v[k];
        w_new[k] -= step_size_ * grad_w[k];
      }
      clampControls(v_new, w_new);
      double c = cost(pose, v_new, w_new, ref);
      if (c < current) {
        v = v_new;
        w = w_new;
        current = c;
        step_size_ *= 1.5;
        improved = true;
        break;
      }
      step_size_ *= 0.5;
    }
    if (!improved) break;
  }
  if (step_size_ < 1e-6) step_size_ = 1e-6;

  v_ = v;
  w_ = w;
  cmd.v = v[0];
  cmd.w = w[0];
  last_v_ = cmd.v;
  last_w_ = cmd.w;
  return cmd;
}

}