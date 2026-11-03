#include <cmath>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class WheelOdomNode : public rclcpp::Node {
 public:
  WheelOdomNode() : Node("wheel_odom") {
    ticks_per_rev_ = declare_parameter("ticks_per_rev", 1024);
    r_left_ = declare_parameter("wheel_radius_left", 0.0503);
    r_right_ = declare_parameter("wheel_radius_right", 0.0499);
    track_ = declare_parameter("wheel_separation", 0.315);
    rate_ = declare_parameter("publish_rate", 50.0);
    left_name_ = declare_parameter("left_joint", std::string("left_wheel_joint"));
    right_name_ = declare_parameter("right_joint", std::string("right_wheel_joint"));

    pub_ = create_publisher<nav_msgs::msg::Odometry>("wheel/odom", 10);
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 50, std::bind(&WheelOdomNode::onJoints, this, std::placeholders::_1));
  }

 private:
  long toTicks(double angle) const {
    return static_cast<long>(std::floor(angle / (2 * M_PI) * ticks_per_rev_));
  }

  void onJoints(const sensor_msgs::msg::JointState::SharedPtr msg) {
    double ql = NAN, qr = NAN;
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      if (msg->name[i] == left_name_) ql = msg->position[i];
      if (msg->name[i] == right_name_) qr = msg->position[i];
    }
    if (std::isnan(ql) || std::isnan(qr)) return;
    const rclcpp::Time stamp(msg->header.stamp);
    const long tl = toTicks(ql), tr = toTicks(qr);

    if (!started_) {
      started_ = true;
      last_tl_ = tl;
      last_tr_ = tr;
      last_time_ = stamp;
      return;
    }
    const double dt = (stamp - last_time_).seconds();
    if (dt < 1.0 / rate_) return;  // publish at ~50hz

    const double tick_angle = 2 * M_PI / ticks_per_rev_;
    const double dl = (tl - last_tl_) * tick_angle * r_left_;
    const double dr = (tr - last_tr_) * tick_angle * r_right_;
    last_tl_ = tl;
    last_tr_ = tr;
    last_time_ = stamp;

    const double ds = 0.5 * (dl + dr);
    const double dth = (dr - dl) / track_;
    x_ += ds * std::cos(th_ + 0.5 * dth);
    y_ += ds * std::sin(th_ + 0.5 * dth);
    th_ += dth;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = std::sin(th_ / 2);
    odom.pose.pose.orientation.w = std::cos(th_ / 2);
    odom.twist.twist.linear.x = ds / dt;
    odom.twist.twist.angular.z = dth / dt;
    pub_->publish(odom);
  }

  int ticks_per_rev_;
  double r_left_, r_right_, track_, rate_;
  std::string left_name_, right_name_;
  bool started_ = false;
  long last_tl_ = 0, last_tr_ = 0;
  rclcpp::Time last_time_;
  double x_ = 0, y_ = 0, th_ = 0;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdomNode>());
  rclcpp::shutdown();
}
