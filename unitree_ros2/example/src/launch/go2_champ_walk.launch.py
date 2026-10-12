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
            # ROS-2-only CHAMP controller: PUB /lowcmd, SUB /lowstate.
            # Single process — no DDS bridge, no loopback UDP (the old
            # gait_planner + dds_bridge split was removed; this controller
            # talks native ROS 2 topics, mapped to DDS rt/lowcmd|rt/lowstate
            # by rmw_cyclonedds_cpp on the same domain/interface).
            #
            # Requires:
            #   RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            #   ROS_DOMAIN_ID matching the sim/robot
            #   CYCLONEDDS_URI on the right interface (lo in sim)
            Node(
                package="unitree_ros2_example",
                executable="go2_champ_ros2_controller",
                name="go2_champ_walk_controller",
                output="screen",
                parameters=[config],
            ),
        ]
    )

