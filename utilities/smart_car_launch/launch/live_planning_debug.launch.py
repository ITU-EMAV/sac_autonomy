"""Live perception and planning debug graph with vehicle actuation disconnected.

ZED2, Velodyne, and encoder publishers are expected to run separately.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _include(package, filename, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', filename)),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    speed_config = os.path.join(
        get_package_share_directory('sac_control_safety'), 'config')
    return LaunchDescription([
        _include('smart_car_launch', 'localization.launch.py', {
            'use_sim_time': 'false',
            'launch_rviz': 'true',
        }),
        _include('smart_car_launch', 'lidar_perception.launch.py', {
            'use_sim_time': 'false',
        }),
        _include('jetson_perception', 'perception.launch.py', {
            'use_sim_time': 'false',
            'enable_road_hazards': 'true',
            'hazard_auto_depth_preview': 'false',
        }),
        _include('planner', 'planner_composable_launch.launch.py', {
            'use_sim_time': 'false',
            'reactive_mode': 'false',
            'launch_rviz': 'false',
            'enable_traffic_behavior': 'true',
            'enable_road_hazards': 'true',
            'enable_dynamic_behavior': 'true',
        }),
        Node(
            package='sac_control_safety',
            executable='vehicle_speed_mux.py',
            name='vehicle_speed_mux',
            parameters=[os.path.join(speed_config, 'vehicle_speed.yaml'),
                        {'use_sim_time': False}],
            output='screen',
        ),
        Node(
            package='sac_control_safety',
            executable='longitudinal_controller.py',
            name='longitudinal_controller',
            parameters=[os.path.join(speed_config, 'longitudinal.yaml'),
                        {'use_sim_time': False}],
            output='screen',
        ),
    ])
