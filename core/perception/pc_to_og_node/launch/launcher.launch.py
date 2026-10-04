from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('pc_to_og_node')
    config_file = os.path.join(pkg_share, 'config', 'config.yaml')
    return LaunchDescription([
        Node(
            package='pc_to_og_node',
            executable='pc_to_og_node',
            name='pc_to_og_node',
            parameters=[config_file]
        )
    ])
