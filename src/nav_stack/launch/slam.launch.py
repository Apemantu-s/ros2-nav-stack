
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    params = os.path.join(get_package_share_directory("nav_stack"), "config", "slam_toolbox.yaml")
    slam = LifecycleNode(package="slam_toolbox", executable="async_slam_toolbox_node",
                         name="slam_toolbox", namespace="", output="screen",
                         parameters=[params, {"use_sim_time": True, "use_lifecycle_manager": False}],
                         remappings=[("/map", "/slam_map"), ("/map_metadata", "/slam_map_metadata")])
    configure = EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                                            transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state="configuring", goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                                              transition_id=Transition.TRANSITION_ACTIVATE))]))
    return LaunchDescription([slam, configure, activate])