#include <random>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "nav_stack/planners.hpp"

using nav_stack::Point;

class PlannerNode : public rclcpp::Node {
 public:
  PlannerNode() : Node("planner"), rng_(declare_parameter("seed", 1)) {
    algorithm_ = declare_parameter("algorithm", std::string("astar"));
    cm_params_.lethal_radius = declare_parameter("lethal_radius", 0.24);
    cm_params_.cost_radius = declare_parameter("cost_radius", 0.6);
    rrt_params_.time_budget_ms = declare_parameter("rrt_time_budget_ms", 300.0);
    rrt_params_.max_iterations = declare_parameter("rrt_max_iterations", 5000);
    periodic_replan_s_ = declare_parameter("periodic_replan_s", 2.0);

    path_pub_ = create_publisher<nav_msgs::msg::Path>("plan", rclcpp::QoS(1).transient_local());
    stats_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("planner/stats", 10);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) { map_ = m; });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "ekf/odom", 20, [this](nav_msgs::msg::Odometry::SharedPtr m) { odom_ = m; });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr g) {
          goal_ = Point{g->pose.position.x, g->pose.position.y};
          has_goal_ = true;
          path_.clear();
          RCLCPP_INFO(get_logger(), "new goal %.2f %.2f", goal_.x, goal_.y);
          tick();
        });
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(500), std::bind(&PlannerNode::tick, this));
  }

 private:
  Point robot() const { return {odom_->pose.pose.position.x, odom_->pose.pose.position.y}; }

  size_t closestPoint() const {
    Point r = robot();
    size_t best = 0;
    for (size_t i = 1; i < path_.size(); ++i) {
      if (std::hypot(path_[i].x - r.x, path_[i].y - r.y) < std::hypot(path_[best].x - r.x, path_[best].y - r.y))
        best = i;
    }
    return best;
  }

  bool pathStillValid(const nav_stack::CostMap& cm) const {
    if (path_.size() < 2) return false;
    size_t start = closestPoint();
    Point r = robot();
    if (std::hypot(path_[start].x - r.x, path_[start].y - r.y) > 1.0) return false;
    for (size_t i = start + 1; i < path_.size(); ++i) {
      if (!cm.segmentFree(path_[i - 1].x, path_[i - 1].y, path_[i].x, path_[i].y)) return false;
    }
    return true;
  }

  double remainingLength() const {
    size_t start = closestPoint();
    Point r = robot();
    double len = std::hypot(path_[start].x - r.x, path_[start].y - r.y);
    for (size_t i = start + 1; i < path_.size(); ++i) {
      len += std::hypot(path_[i].x - path_[i - 1].x, path_[i].y - path_[i - 1].y);
    }
    return len;
  }

  void tick() {
    if (!has_goal_ || !map_ || !odom_) return;

    nav_stack::GridInfo info;
    info.width = map_->info.width;
    info.height = map_->info.height;
    info.resolution = map_->info.resolution;
    info.origin_x = map_->info.origin.position.x;
    info.origin_y = map_->info.origin.position.y;
    nav_stack::CostMap cm(info, map_->data, cm_params_);

    bool valid = pathStillValid(cm);
    double t = now().seconds();
    bool time_for_periodic = t - last_plan_time_ > periodic_replan_s_;
    if (valid && !time_for_periodic) return;

    nav_stack::PlanResult res;
    if (algorithm_ == "rrtstar") {
      res = nav_stack::planRrtStar(cm, robot(), goal_, rrt_params_, rng_);
    } else {
      res = nav_stack::planAStar(cm, robot(), goal_);
    }
    last_plan_time_ = t;

    std_msgs::msg::Float64MultiArray stats;
    stats.data = {res.success ? 1.0 : 0.0, res.time_ms, nav_stack::pathLength(res.path),
                  static_cast<double>(res.expansions), valid ? 0.0 : 1.0};
    stats_pub_->publish(stats);

    if (valid) {
      if (res.success && nav_stack::pathLength(res.path) < remainingLength() - 0.2) {
        path_ = res.path;
        publishPath();
      }
      return;
    }

    if (res.success) {
      path_ = res.path;
    } else {
      Point r = robot();
      RCLCPP_WARN(get_logger(), "no path found (robot %.2f %.2f free=%d, goal free=%d)", r.x, r.y,
                  cm.pointFree(r.x, r.y), cm.pointFree(goal_.x, goal_.y));
      path_.clear();
    }
    publishPath();
  }

  void publishPath() {
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom";
    for (const auto& p : path_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.orientation.w = 1.0;
      msg.poses.push_back(ps);
    }
    path_pub_->publish(msg);
  }

  std::string algorithm_;
  nav_stack::CostMap::Params cm_params_;
  nav_stack::RrtStarParams rrt_params_;
  double periodic_replan_s_;
  double last_plan_time_ = -1e9;
  std::mt19937 rng_;
  bool has_goal_ = false;
  Point goal_{0, 0};
  std::vector<Point> path_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr stats_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
}