"""Bag-only perception and planning debug graph using one ROS clock.

Start the bag separately with ``ros2 bag play /smart_car_ws/new_bag --clock``.
No Guardian, UDP sender, or physical vehicle actuation is launched here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package, filename, arguments):
    path = os.path.join(get_package_share_directory(package), 'launch', filename)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    hazard_preview = LaunchConfiguration('enable_road_hazard_preview')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Bag oynatımında /clock kullan; tüm alt launchlara iletilir.'),
        DeclareLaunchArgument(
            'enable_road_hazard_preview', default_value='true',
            description='Depth olmayan bagde sadece RGB hazard önizlemesi.'),
        _include('smart_car_launch', 'localization.launch.py', {
            'use_sim_time': use_sim_time,
            'launch_rviz': 'true',
        }),
        _include('smart_car_launch', 'lidar_perception.launch.py', {
            'use_sim_time': use_sim_time,
        }),
        _include('jetson_perception', 'perception.launch.py', {
            'use_sim_time': use_sim_time,
            'enable_road_hazards': hazard_preview,
            'hazard_auto_depth_preview': 'true',
        }),
        # The current new_bag has no registered depth. Hazard slowdown and
        # metric object behavior stay disabled rather than using fake ranges.
        _include('planner', 'planner_composable_launch.launch.py', {
            'use_sim_time': use_sim_time,
            'reactive_mode': 'false',
            'launch_rviz': 'false',
            'enable_traffic_behavior': 'true',
            'traffic_camera_timeout': '1.5',
            'enable_road_hazards': 'false',
            'enable_dynamic_behavior': 'false',
        }),
        Node(
            package='sac_control_safety',
            executable='vehicle_speed_mux.py',
            name='vehicle_speed_mux',
            parameters=[
                os.path.join(get_package_share_directory('sac_control_safety'),
                             'config', 'vehicle_speed.yaml'),
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),
        Node(
            package='sac_control_safety',
            executable='longitudinal_controller.py',
            name='longitudinal_controller',
            parameters=[
                os.path.join(get_package_share_directory('sac_control_safety'),
                             'config', 'longitudinal.yaml'),
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),
    ])
