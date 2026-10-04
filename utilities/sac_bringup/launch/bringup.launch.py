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
    foxglove_dir = LaunchConfiguration('foxglove_dir', default=os.path.join(
        get_package_share_directory('foxglove_bridge'), 'launch'))

    sac_localization_dir = LaunchConfiguration('sac_localization_dir', default=os.path.join(
        get_package_share_directory('sac_localization'), 'launch'))
    
    # sac_gnss_dir = LaunchConfiguration('sac_gnss_dir', default=os.path.join(
    #     get_package_share_directory('sac_gnss'), 'launch'))

    sac_imu_pkg_dir = LaunchConfiguration('sac_imu_pkg_dir', default=os.path.join(
        get_package_share_directory('bno055'), 'launch'))
    

    sac_group = GroupAction([
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                [foxglove_dir, "/foxglove_bridge_launch.xml"])
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [sac_imu_pkg_dir, "/bno055.launch.py"])
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [sac_gnss_dir, "/ublox_simple_rtk.launch.py"])
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [sac_localization_dir, "/localization.launch.py"])
        # ),
        Node(package='sac_network',
             executable='udp_sender',
             name='udp_sender',
             arguments=[],
             parameters=[{'legacy_cmd_vel_enabled': True}]
             ),
        Node(package='sac_network',
             executable='udp_receiver',
             name='udp_receiver',
             arguments=[]
             )

    ])

    ld = LaunchDescription()
    ld.add_action(sac_group)
    return ld
