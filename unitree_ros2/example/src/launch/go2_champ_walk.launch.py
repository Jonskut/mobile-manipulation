import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("unitree_ros2_example"),
        "config",
        "go2_champ_gait.yaml",
    )
    return LaunchDescription(
        [
            Node(
                package="unitree_ros2_example",
                executable="go2_champ_walk_controller",
                name="go2_champ_walk_controller",
                output="screen",
                parameters=[config],
            ),
        ]
    )
