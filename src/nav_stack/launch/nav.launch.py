from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planner = LaunchConfiguration("planner")
    controller = LaunchConfiguration("controller")
    use_imu = LaunchConfiguration("use_imu")
    sim = {"use_sim_time": True}
    return LaunchDescription([
        DeclareLaunchArgument("planner", default_value="astar"),        # astar | rrtstar
        DeclareLaunchArgument("controller", default_value="mpc"),      # pure_pursuit | mpc
        DeclareLaunchArgument("use_imu", default_value="true"),
        Node(package="nav_stack", executable="wheel_odom_node", parameters=[sim]),
        Node(package="nav_stack", executable="ekf_node", parameters=[sim, {"use_imu": use_imu}]),
        Node(package="nav_stack", executable="mapper_node", parameters=[sim]),
        Node(package="nav_stack", executable="planner_node", parameters=[sim, {"algorithm": planner}]),
        Node(package="nav_stack", executable="controller_node",
             parameters=[sim, {"controller": controller}]),
    ])