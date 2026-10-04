from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("sac_control_safety"), "config", "guardian.yaml"
    )
    return LaunchDescription([
        Node(
            package="sac_control_safety",
            executable="command_guardian",
            name="command_guardian",
            parameters=[config],
            output="screen",
        )
    ])
