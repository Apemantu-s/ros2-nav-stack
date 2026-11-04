#include <chrono>
#include <cmath>
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
    mapper_ = std::make_unique<nav_stack::OccupancyMapper>(info, nav_stack::OccupancyMapper::Params{});

    auto qos = rclcpp::QoS(1).transient_local().reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos);
    pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "ekf/odom", 20, [this](nav_msgs::msg::Odometry::SharedPtr m) { pose_ = m; });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), std::bind(&MapperNode::onScan, this, std::placeholders::_1));
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(500),
                                  std::bind(&MapperNode::publishMap, this));
  }

 private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (!pose_) return;
    const auto& p = pose_->pose.pose;
    const double px = p.position.x;
    const double py = p.position.y;
    const double yaw = 2 * std::atan2(p.orientation.z, p.orientation.w);
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
  long scans_ = 0;
  nav_msgs::msg::Odometry::SharedPtr pose_;
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