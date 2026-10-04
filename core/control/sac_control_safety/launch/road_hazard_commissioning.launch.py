"""Model-required bench/replay entry point; inherits disabled physical transport."""
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration



def _default_use_sim_time():
    """Default for use_sim_time, from smart_car_launch/config/use_sim_time.yaml."""
    path = os.path.join(get_package_share_directory('smart_car_launch'),
                        'config', 'use_sim_time.yaml')
    with open(path, encoding='utf-8') as config_file:
        return str(bool((yaml.safe_load(config_file) or {})['use_sim_time'])).lower()

def setup(context):
    model = LaunchConfiguration('hazard_model').perform(context)
    if not os.path.isfile(model) or os.path.splitext(model)[1] not in ('.pt', '.engine'):
        raise RuntimeError('hazard_model must point to an existing pothole/speed_bump .pt or .engine')
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('sac_control_safety'), 'launch', 'online_vehicle_safe.launch.py')),
        launch_arguments={
            'enable_road_hazards': 'true', 'hazard_model': model,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'replay_mode': LaunchConfiguration('replay_mode'),
            'camera_mode': LaunchConfiguration('camera_mode'),
            'enable_guardian': LaunchConfiguration('enable_guardian'),
            'serialize_gpu': 'true',
        }.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hazard_model', description='Local trained pothole/speed_bump model'),
        DeclareLaunchArgument('use_sim_time', default_value=_default_use_sim_time()),
        DeclareLaunchArgument('replay_mode', default_value='false'),
        DeclareLaunchArgument('camera_mode', default_value='false'),
        DeclareLaunchArgument('enable_guardian', default_value='true'),
        OpaqueFunction(function=setup),
    ])
