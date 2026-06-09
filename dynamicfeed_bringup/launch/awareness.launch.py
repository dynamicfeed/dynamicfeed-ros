import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("dynamicfeed_bringup"), "config", "feeds.yaml")
    return LaunchDescription([
        Node(
            package="dynamicfeed_awareness",
            executable="awareness_node",
            name="dynamicfeed_awareness",
            parameters=[cfg],
            output="screen",
        ),
    ])
