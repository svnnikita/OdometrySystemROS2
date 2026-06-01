import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = "trajectory_publisher",
            executable = "trajectory_publisher_node",
            name = "trajectory_publisher_node",
            output = "screen",

            parameters = [
                {
                    "counts2meters": 0.000532,
                    "baseline": 0.46,
                    "forward_offset": 0.072
                }
            ]
        ),

        Node(
            package = "dxdy_publisher",
            executable = "dxdy_publisher_node",
            name = "dxdy_publisher_node",
            output = "screen",
        )
    ])
