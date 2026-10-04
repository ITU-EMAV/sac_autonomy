"""Map-free camera path and controller preview; never publishes to the vehicle command topic.

Start jetson_perception/perception.launch.py separately before this launch.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    planner = os.path.join(
        get_package_share_directory('planner'), 'launch',
        'planner_composable_launch.launch.py')
    return LaunchDescription([
        LogInfo(msg='Camera planning dry-run: no map localization, guardian, gateway or vehicle output.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planner),
            launch_arguments={
                'reactive_mode': 'true',
                'enable_traffic_behavior': 'true',
                # Current RGB-only bags cannot provide metric hazard distance.
                'enable_road_hazards': 'false',
                'controller_command_topic': '/planning/dry_run/controller_command',
                'traffic_odometry_topic': '',
                'launch_rviz': 'false',
            }.items(),
        ),
    ])
