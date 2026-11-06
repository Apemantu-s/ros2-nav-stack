#!/usr/bin/env python3
import json
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

w = json.load(open(sys.argv[1]))
start, goal = w["start"], w["goal"]
dx = goal["x"] - start["x"]
dy = goal["y"] - start["y"]
c, s = math.cos(-start["yaw"]), math.sin(-start["yaw"])
gx, gy = c * dx - s * dy, s * dx + c * dy

rclpy.init()
node = Node("send_goal")
pub = node.create_publisher(PoseStamped, "goal_pose", QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
msg = PoseStamped()
msg.header.frame_id = "odom"
msg.pose.position.x = gx
msg.pose.position.y = gy
msg.pose.orientation.w = 1.0

while pub.get_subscription_count() < 2:
    rclpy.spin_once(node, timeout_sec=0.1)
pub.publish(msg)
rclpy.spin_once(node, timeout_sec=0.5)
print(f"goal sent: odom ({gx:.2f}, {gy:.2f}), {math.hypot(gx, gy):.2f} m from the start in a straight line")
node.destroy_node()
rclpy.shutdown()