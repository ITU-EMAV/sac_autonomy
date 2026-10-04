import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    lidar_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lidar_localization_ros2'),
                'launch',
                'lidar_localization.launch.py'
            )
        )
    )

    # RViz'i belirli konfigürasyonla çalıştır
    rviz_config = os.path.join(
        get_package_share_directory('smart_car_launch'),
        'rviz',
        'localization.rviz'
    )

    rviz = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        lidar_localization_launch,
        rviz
    ])
