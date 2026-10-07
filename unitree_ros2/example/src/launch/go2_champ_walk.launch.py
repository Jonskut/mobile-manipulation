import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("unitree_ros2_example"),
        "config",
        "go2_champ_gait.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dds_domain_id",
                default_value="1",
                description="Unitree DDS domain (ChannelFactory::Init). Must match ROS_DOMAIN_ID.",
            ),
            DeclareLaunchArgument(
                "dds_interface",
                default_value="lo",
                description="Network interface for Unitree ChannelFactory (use 'lo' in sim).",
            ),
            DeclareLaunchArgument("kp", default_value="60.0"),
            DeclareLaunchArgument("kd", default_value="5.0"),
            # ROS-only gait planner (rclcpp). Takes gait/kp/kd/etc. from the yaml.
            Node(
                package="unitree_ros2_example",
                executable="go2_champ_gait_planner",
                name="go2_champ_walk_controller",
                output="screen",
                parameters=[config],
            ),
            # Pure-DDS bridge (Unitree ChannelFactory + raw rcl C API).
            # Must be its own process: ChannelFactory cannot share a process
            # with rclcpp on the same DDS domain. Positional args:
            #   <dds_domain_id> <dds_interface> <kp> <kd>
            # (anything after --ros-args is ignored by its arg parser).
            Node(
                package="unitree_ros2_example",
                executable="go2_champ_dds_bridge",
                name="go2_champ_dds_bridge",
                output="screen",
                arguments=[
                    LaunchConfiguration("dds_domain_id"),
                    LaunchConfiguration("dds_interface"),
                    LaunchConfiguration("kp"),
                    LaunchConfiguration("kd"),
                ],
            ),
        ]
    )
