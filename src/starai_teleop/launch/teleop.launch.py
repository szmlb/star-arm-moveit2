"""Bring up violin (leader) + viola (follower) drivers plus the teleop bridge.

Default ports point at the udev-created stable symlinks (see docs/starai-arm.md);
override with e.g. `viola_port:=/dev/ttyUSB1` if the udev rule isn't installed yet.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    viola_port = LaunchConfiguration("viola_port")
    violin_port = LaunchConfiguration("violin_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "viola_port",
                default_value="/dev/starai_viola",
                description="Serial port for the viola follower arm",
            ),
            DeclareLaunchArgument(
                "violin_port",
                default_value="/dev/starai_violin",
                description="Serial port for the violin leader arm",
            ),
            Node(
                package="robo_driver",
                executable="driver",
                name="robo_driver_node",
                namespace="viola",
                output="screen",
                parameters=[{"port": viola_port, "baudrate": 1000000, "lock": "enable"}],
            ),
            Node(
                package="robo_driver",
                executable="driver",
                name="robo_driver_node",
                namespace="violin",
                output="screen",
                parameters=[{"port": violin_port, "baudrate": 1000000, "lock": "disable"}],
            ),
            Node(
                package="starai_teleop",
                executable="teleop_bridge",
                name="starai_teleop_bridge",
                output="screen",
                parameters=[
                    {"leader_namespace": "violin", "follower_namespace": "viola"}
                ],
            ),
        ]
    )
