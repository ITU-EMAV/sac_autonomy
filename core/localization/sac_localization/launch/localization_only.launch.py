from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('sac_localization').find('sac_localization')
    
    # Default config file
    default_config_file = os.path.join(pkg_share, 'params', 'localization.yaml')
    
    # Declare launch arguments
    declare_localization_mode = DeclareLaunchArgument(
        'localization_mode',
        default_value='gps',
        description='Localization mode: gps, lidar, or fusion'
    )
    
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to config file'
    )
    
    # Get launch configuration variables
    localization_mode = LaunchConfiguration('localization_mode')
    config_file = LaunchConfiguration('config_file')
    
    # Create the localization node
    localization_node = Node(
        package='sac_localization',
        executable='sac_localization_node',
        name='sac_localization',
        parameters=[
            config_file,
            {'localization_mode': localization_mode}
        ],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        declare_localization_mode,
        declare_config_file,
        localization_node
    ])
