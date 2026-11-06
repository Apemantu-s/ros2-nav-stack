#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include "nav_stack/controllers.hpp"

using nav_stack::Point;

class ControllerNode : public rclcpp::Node {
 public:
  ControllerNode() : Node("controller") {
    type_ = declare_parameter("controller", std::string("pure_pursuit"));
    lim_.v_max = declare_parameter("v_max", 0.5);
    lim_.w_max = declare_parameter("w_max", 1.5);
    goal_tol_ = declare_parameter("goal_tolerance", 0.2);
    stop_dist_ = declare_parameter("stop_distance", 0.2);
    const double rate = declare_parameter("rate", 20.0);

    nav_stack::PurePursuit::Params pp;
    pp.lookahead = declare_parameter("lookahead", 0.5);
    pp_ = std::make_unique<nav_stack::PurePursuit>(pp, lim_);
    nav_stack::Mpc::Params mp;
    mp.horizon = declare_parameter("mpc_horizon", 15);
    mp.dt = declare_parameter("mpc_dt", 0.1);
    mpc_ = std::make_unique<nav_stack::Mpc>(mp, lim_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    err_pub_ = create_publisher<std_msgs::msg::Float64>("nav/tracking_error", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("nav/status", rclcpp::QoS(1).transient_local());
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "plan", rclcpp::QoS(1).transient_local(), [this](nav_msgs::msg::Path::SharedPtr m) {
          path_.clear();
          for (const auto& p : m->poses) path_.push_back({p.pose.position.x, p.pose.position.y});
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "ekf/odom", 20, [this](nav_msgs::msg::Odometry::SharedPtr m) { odom_ = m; });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr g) {
          goal_ = Point{g->pose.position.x, g->pose.position.y};
          active_ = true;
          path_.clear();
          mpc_->reset();
          goal_time_ = now();
          setStatus("driving");
        });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::SharedPtr s) { scan_ = s; });
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(1.0 / rate),
                                  std::bind(&ControllerNode::tick, this));
    setStatus("idle");
  }

 private:
  void setStatus(const std::string& s) {
    std_msgs::msg::String m;
    m.data = s;
    status_pub_->publish(m);
  }

  double frontClearance() const {
    if (!scan_) return 1e9;
    double best = 1e9;
    for (size_t i = 0; i < scan_->ranges.size(); ++i) {
      const double a = scan_->angle_min + i * scan_->angle_increment;
      if (std::abs(a) < 0.5 && std::isfinite(scan_->ranges[i]) && scan_->ranges[i] > scan_->range_min)
        best = std::min<double>(best, scan_->ranges[i]);
    }
    return best;
  }

  void tick() {
    geometry_msgs::msg::Twist cmd;
    if (!active_ || !odom_) {
      return;
    }
    const auto& q = odom_->pose.pose.orientation;
    nav_stack::Pose2D pose{odom_->pose.pose.position.x, odom_->pose.pose.position.y,
                           2 * std::atan2(q.z, q.w)};

    if (std::hypot(goal_.x - pose.x, goal_.y - pose.y) < goal_tol_) {
      active_ = false;
      cmd_pub_->publish(cmd);
      setStatus("reached");
      RCLCPP_INFO(get_logger(), "goal reached in %.1f s", (now() - goal_time_).seconds());
      return;
    }
    if (path_.size() < 2) {
      cmd_pub_->publish(cmd);
      return;
    }

    nav_stack::Cmd c = type_ == "mpc" ? mpc_->compute(path_, pose) : pp_->compute(path_, pose);

    if (c.v > 0 && frontClearance() < stop_dist_) c.v = 0.0;

    cmd.linear.x = c.v;
    cmd.angular.z = c.w;
    cmd_pub_->publish(cmd);

    std_msgs::msg::Float64 e;
    e.data = nav_stack::crossTrackError(path_, pose);
    err_pub_->publish(e);
  }

  std::string type_;
  nav_stack::ControlLimits lim_;
  double goal_tol_, stop_dist_;
  std::unique_ptr<nav_stack::PurePursuit> pp_;
  std::unique_ptr<nav_stack::Mpc> mpc_;
  bool active_ = false;
  Point goal_{0, 0};
  rclcpp::Time goal_time_;
  std::vector<Point> path_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
}