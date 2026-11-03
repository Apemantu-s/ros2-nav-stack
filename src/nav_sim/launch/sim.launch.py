import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("nav_sim")
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=os.path.join(share, "worlds", "demo.sdf")),
        DeclareLaunchArgument("headless", default_value="true"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(share, "models")),
        ExecuteProcess(cmd=["gz", "sim", "-s", "-r", "--headless-rendering", world],
                       output="screen", condition=IfCondition(headless)),
        ExecuteProcess(cmd=["gz", "sim", "-r", world], output="screen",
                       condition=UnlessCondition(headless)),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": os.path.join(share, "config", "bridge.yaml"),
                          "use_sim_time": True}]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["--z", "0.14", "--frame-id", "base_link",
                        "--child-frame-id", "lidar_link"],
             parameters=[{"use_sim_time": True}]),
    ])
