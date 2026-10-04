"""Launch point-cloud and Lanelet2 map providers."""

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

def _enabled(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def _setup(context, *_, **__):
    use_sim_time = LaunchConfiguration('use_sim_time')
    nodes = []

    if _enabled(context, 'launch_pointcloud_map'):
        map_path = LaunchConfiguration('pointcloud_map_path').perform(context)
        metadata_path = LaunchConfiguration('pointcloud_map_metadata_path').perform(context)
        nodes.append(Node(
            package='autoware_map_loader', executable='autoware_pointcloud_map_loader',
            name='pointcloud_map_loader', output='both',
            parameters=[{
                'enable_whole_load': True,
                'enable_downsampled_whole_load': True,
                'leaf_size': LaunchConfiguration('pointcloud_map_leaf_size'),
                'enable_partial_load': True,
                'enable_selected_load': False,
                'pcd_paths_or_directory': [map_path],
                'pcd_metadata_path': metadata_path,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('output/pointcloud_map', LaunchConfiguration('pointcloud_map_topic')),
                ('output/debug/downsampled_pointcloud_map',
                 LaunchConfiguration('downsampled_pointcloud_map_topic')),
                ('service/get_partial_pcd_map',
                 LaunchConfiguration('partial_pointcloud_map_service')),
                ('service/get_differential_pcd_map',
                 LaunchConfiguration('differential_pointcloud_map_service')),
            ],
        ))

    if _enabled(context, 'launch_lanelet_map'):
        nodes.append(Node(
            package='autoware_map_projection_loader',
            executable='autoware_map_projection_loader_node',
            name='map_projection_loader',
            output='both',
            parameters=[
                LaunchConfiguration('map_projection_loader_param_path'),
                {
                    'map_projector_info_path': LaunchConfiguration('map_projector_info_path'),
                    'lanelet2_map_path': LaunchConfiguration('lanelet2_map_path'),
                    'use_sim_time': use_sim_time,
                },
            ],
            remappings=[
                ('output/map_projector_info', LaunchConfiguration('map_projector_info_topic')),
            ],
        ))
        nodes.append(Node(
            package='autoware_map_loader', executable='autoware_lanelet2_map_loader',
            name='lanelet2_map_loader', output='both',
            parameters=[
                LaunchConfiguration('lanelet2_map_loader_param_path'),
                {
                    'lanelet2_map_path': LaunchConfiguration('lanelet2_map_path'),
                    'use_sim_time': use_sim_time,
                },
            ],
            remappings=[
                ('output/lanelet2_map', LaunchConfiguration('lanelet2_map_topic')),
                ('/map/vector_map', LaunchConfiguration('lanelet2_map_topic')),
            ],
        ))
        nodes.append(Node(
            package='autoware_lanelet2_map_visualizer',
            executable='autoware_lanelet2_map_visualizer',
            name='lanelet2_map_visualization',
            output='both',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('input/lanelet2_map', LaunchConfiguration('lanelet2_map_topic')),
                ('output/lanelet2_map_marker',
                 LaunchConfiguration('lanelet2_map_marker_topic')),
            ],
        ))

    return nodes


def generate_launch_description():
    share = get_package_share_directory('smart_car_launch')
    config_path = os.path.join(share, 'config', 'maps', 'map_config.yaml')
    with open(config_path, encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}

    def configured_path(key):
        value = config.get(key, '')
        if not value or os.path.isabs(value):
            return value
        return os.path.join(share, 'maps', value)

    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value=_default_use_sim_time()),
        DeclareLaunchArgument('launch_pointcloud_map', default_value='true'),
        DeclareLaunchArgument('launch_lanelet_map', default_value='true'),
        DeclareLaunchArgument(
            'pointcloud_map_path', default_value=configured_path('pointcloud_map_path')),
        DeclareLaunchArgument(
            'pointcloud_map_metadata_path',
            default_value=configured_path('pointcloud_map_metadata_path')),
        DeclareLaunchArgument('pointcloud_map_leaf_size', default_value='0.5'),
        DeclareLaunchArgument('pointcloud_map_topic', default_value='/map/pointcloud_map'),
        DeclareLaunchArgument(
            'downsampled_pointcloud_map_topic',
            default_value='/map/downsampled_pointcloud_map'),
        DeclareLaunchArgument(
            'partial_pointcloud_map_service', default_value='/map/get_partial_pointcloud_map'),
        DeclareLaunchArgument(
            'differential_pointcloud_map_service',
            default_value='/map/get_differential_pointcloud_map'),
        DeclareLaunchArgument(
            'lanelet2_map_path', default_value=configured_path('lanelet2_map_path')),
        DeclareLaunchArgument(
            'map_projector_info_path',
            default_value=configured_path('map_projector_info_path')),
        DeclareLaunchArgument(
            'lanelet2_map_loader_param_path',
            default_value=os.path.join(
                share, 'config', 'maps', 'lanelet2_map_loader.param.yaml')),
        DeclareLaunchArgument(
            'map_projection_loader_param_path',
            default_value=os.path.join(
                share, 'config', 'maps', 'map_projection_loader.param.yaml')),
        DeclareLaunchArgument(
            'map_projector_info_topic', default_value='/map/map_projector_info'),
        DeclareLaunchArgument('lanelet2_map_topic', default_value='/map/vector_map'),
        DeclareLaunchArgument(
            'lanelet2_map_marker_topic', default_value='/map/lanelet2_map_marker'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_setup)])
