import os
import launch  # noqa: E402
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition
from launch.actions import TimerAction


def generate_launch_description():
    # sac_trajectory_planning_dir = LaunchConfiguration('sac_trajectory_planning_dir', default=os.path.join(
    #     get_package_share_directory('sac_trajectory_planning'), 'launch'))
    pc2og_pkg_share = get_package_share_directory('pc_to_og_node')
    pc2og_config_file = os.path.join(pc2og_pkg_share, 'config', 'config.yaml')

    planning_group = GroupAction([
        Node(package='sac_trajectory_planning',
             executable='local_planner',
             name='local_planner',
             arguments=[]
             ),
        Node(package='sac_trajectory_planning',
             executable='global_planner',
             name='global_planner',
             arguments=[]
             ),
     #    Node(package='sac_trajectory_planning',
     #         executable='pc2og',
     #         name='pc2og',
     #         arguments=[]
     #         ),
        Node(package='pc_to_og_node',
             executable='pc_to_og_node',
             name='pc_to_og_node',
             parameters=[pc2og_config_file]
             ),

    ])

    ld = LaunchDescription()
    ld.add_action(planning_group)
    return ld
