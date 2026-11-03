
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "nav_stack/ekf.hpp"

class EkfNode : public rclcpp::Node {
 public:
  EkfNode() : Node("ekf") {
    nav_stack::Ekf::Params p;
    p.r_wheel_v = std::pow(declare_parameter("wheel_v_std", 0.02), 2);
    p.r_wheel_w = std::pow(declare_parameter("wheel_w_std", 0.2), 2);
    p.r_gyro = std::pow(declare_parameter("gyro_std", 0.01), 2);
    use_imu_ = declare_parameter("use_imu", true);
    publish_tf_ = declare_parameter("publish_tf", true);
    ekf_ = nav_stack::Ekf(p);

    pub_ = create_publisher<nav_msgs::msg::Odometry>("ekf/odom", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "wheel/odom", 20, std::bind(&EkfNode::onWheel, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu", 50, std::bind(&EkfNode::onImu, this, std::placeholders::_1));
  }

 private:
  void predictTo(const rclcpp::Time& t) {
    if (!have_time_) {
      last_ = t;
      have_time_ = true;
      return;
    }
    const double dt = (t - last_).seconds();
    if (dt <= 0) return;  // old message, just do the update
    ekf_.predict(dt);
    last_ = t;
  }

  void onWheel(const nav_msgs::msg::Odometry::SharedPtr msg) {
    predictTo(msg->header.stamp);
    const bool silent = msg->twist.twist.linear.x == 0.0 && msg->twist.twist.angular.z == 0.0;
    if (silent) {
      silent_count_++;
    } else {
      silent_count_ = 0;
    }
    still_ = silent_count_ >= 10;
    ekf_.updateWheel(msg->twist.twist.linear.x, msg->twist.twist.angular.z, still_);
    publish(msg->header.stamp);
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!use_imu_) return;
    double gyro = msg->angular_velocity.z;
    if (still_) {
      bias_count_++;
      bias_ += (gyro - bias_) / bias_count_;
    }
    predictTo(msg->header.stamp);
    ekf_.updateGyro(gyro - bias_);
  }

  void publish(const builtin_interfaces::msg::Time& stamp) {
    const auto& P = ekf_.cov();
    nav_msgs::msg::Odometry o;
    o.header.stamp = stamp;
    o.header.frame_id = "odom";
    o.child_frame_id = "base_link";
    o.pose.pose.position.x = ekf_.x();
    o.pose.pose.position.y = ekf_.y();
    o.pose.pose.orientation.z = std::sin(ekf_.theta() / 2);
    o.pose.pose.orientation.w = std::cos(ekf_.theta() / 2);
    o.pose.covariance[0] = P(0, 0);
    o.pose.covariance[7] = P(1, 1);
    o.pose.covariance[35] = P(2, 2);
    o.twist.twist.linear.x = ekf_.v();
    o.twist.twist.angular.z = ekf_.w();
    pub_->publish(o);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header = o.header;
      t.child_frame_id = "base_link";
      t.transform.translation.x = ekf_.x();
      t.transform.translation.y = ekf_.y();
      t.transform.rotation = o.pose.pose.orientation;
      tf_->sendTransform(t);
    }
  }

  nav_stack::Ekf ekf_;
  bool use_imu_, publish_tf_;
  bool have_time_ = false;
  int silent_count_ = 0;
  bool still_ = false;
  double bias_ = 0.0;
  long bias_count_ = 0;
  rclcpp::Time last_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfNode>());
  rclcpp::shutdown();
}
