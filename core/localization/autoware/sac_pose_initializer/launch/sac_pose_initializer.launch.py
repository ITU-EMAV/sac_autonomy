"""sac_pose_initializer dugumunu config dosyasindaki parametrelerle baslatir.

Varsayilan config paketin kendi icinden gelir. Ust seviye bir launch
dosyasindan farkli bir config vermek icin:

    ros2 launch sac_pose_initializer sac_pose_initializer.launch.py \
        param_file:=/path/to/sac_pose_initializer.param.yaml
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def _default_use_sim_time():
    """Default for use_sim_time, from smart_car_launch/config/use_sim_time.yaml."""
    path = os.path.join(get_package_share_directory('smart_car_launch'),
                        'config', 'use_sim_time.yaml')
    with open(path, encoding='utf-8') as config_file:
        return str(bool((yaml.safe_load(config_file) or {})['use_sim_time'])).lower()

def generate_launch_description():
    default_param_file = os.path.join(
        get_package_share_directory('sac_pose_initializer'),
        'config',
        'sac_pose_initializer.param.yaml',
    )

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param_file,
        description='sac_pose_initializer parametre dosyasi',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=_default_use_sim_time(),
        description='Bag oynatirken true yapin',
    )

    # Config dosyasindaki degeri ezmek icin (or. lokal cerceveli haritada 0.0).
    # Bos birakilirsa param dosyasindaki deger kullanilir.
    map_z_arg = DeclareLaunchArgument(
        'map_z',
        default_value='',
        description="Harita zemin yuksekligi [m]; bos ise config'teki deger kullanilir",
    )

    def make_node(context, *_, **__):
        params = [
            LaunchConfiguration('param_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
        map_z = LaunchConfiguration('map_z').perform(context)
        if map_z != '':
            params.append({'map_z': float(map_z)})

        return [Node(
            package='sac_pose_initializer',
            executable='sac_pose_initializer_node',
            name='sac_pose_initializer',
            output='both',
            parameters=params,
        )]

    return LaunchDescription([
        param_file_arg,
        use_sim_time_arg,
        map_z_arg,
        OpaqueFunction(function=make_node),
    ])
