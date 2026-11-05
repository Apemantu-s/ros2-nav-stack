#include <chrono>
#include <cmath>
#include <deque>                                   // DAY 13
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "nav_stack/grid.hpp"

class MapperNode : public rclcpp::Node {
 public:
  MapperNode() : Node("grid_mapper") {
    nav_stack::GridInfo info;
    const double size = declare_parameter("map_size", 24.0);
    info.resolution = declare_parameter("resolution", 0.05);
    info.width = info.height = static_cast<int>(size / info.resolution);
    info.origin_x = info.origin_y = -size / 2;     // centred on the start
    lidar_x_ = declare_parameter("lidar_x", 0.0);
    frame_ = declare_parameter("frame_id", std::string("odom"));
    nav_stack::OccupancyMapper::Params mp;
    mp.max_range = declare_parameter("max_range", 5.0); 
    mapper_ = std::make_unique<nav_stack::OccupancyMapper>(info, mp);
    
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos);
    pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "ekf/odom", 20,
        [this](nav_msgs::msg::Odometry::SharedPtr m) {
          history_.push_back(m);
          if (history_.size() > 200) history_.pop_front();
        });
    max_turn_rate_ = declare_parameter("max_turn_rate", 0.6);   // DAY 13
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), std::bind(&MapperNode::onScan, this, std::placeholders::_1));
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(500),
                                  std::bind(&MapperNode::publishMap, this));
  }

 private:
  bool poseAt(const rclcpp::Time& t, double& x, double& y, double& yaw, double& w) const {
    if (history_.empty()) return false;
    size_t best = 0;
    double best_dt = 1e9;
    for (size_t i = 0; i < history_.size(); ++i) {
      double dt = std::abs((rclcpp::Time(history_[i]->header.stamp) - t).seconds());
      if (dt < best_dt) {
        best_dt = dt;
        best = i;
      }
    }
    if (best_dt > 0.05) return false;
    const auto& p = history_[best]->pose.pose;
    x = p.position.x;
    y = p.position.y;
    yaw = 2 * std::atan2(p.orientation.z, p.orientation.w);
    w = history_[best]->twist.twist.angular.z;
    return true;
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    double px, py, yaw, w;
    if (!poseAt(scan->header.stamp, px, py, yaw, w)) {
      skipped_++;
      return;
    }
    if (std::abs(w) > max_turn_rate_) return;
    const double sx = px + lidar_x_ * std::cos(yaw);
    const double sy = py + lidar_x_ * std::sin(yaw);
    mapper_->addScan(sx, sy, yaw, scan->angle_min, scan->angle_increment, scan->ranges,
                     scan->range_min);
    scans_++;
  }

  void publishMap() {
    if (scans_ == 0) return; 
    const auto& info = mapper_->info();
    nav_msgs::msg::OccupancyGrid g;
    g.header.stamp = now();
    g.header.frame_id = frame_;
    g.info.resolution = info.resolution;
    g.info.width = info.width;
    g.info.height = info.height;
    g.info.origin.position.x = info.origin_x;
    g.info.origin.position.y = info.origin_y;
    g.info.origin.orientation.w = 1.0;
    g.data = mapper_->toOccupancy();
    map_pub_->publish(g);
  }

  std::unique_ptr<nav_stack::OccupancyMapper> mapper_;
  double lidar_x_;
  std::string frame_;
  long scans_ = 0, skipped_ = 0;
  double max_turn_rate_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> history_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapperNode>());
  rclcpp::shutdown();
}