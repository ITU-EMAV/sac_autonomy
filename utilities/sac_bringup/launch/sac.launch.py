"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False')

    ## ***** File paths ******
    pkg_share = FindPackageShare('smart_car_launch').find('smart_car_launch')
    cartographer_config_dir = os.path.join(pkg_share, 'config', 'cartographer')
    # urdf_file = os.path.join(urdf_dir, 'backpack_3d.urdf')
    # with open(urdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    # ## ***** Nodes *****
    # robot_state_publisher_node = Node(
    #     package = 'robot_state_publisher',
    #     executable = 'robot_state_publisher',
    #     parameters=[
    #         {'robot_description': robot_desc},
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    #     output = 'screen'
    #     )

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': False}],
        arguments = [
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'sac.lua'],
        remappings = [
            ('points2', 'velodyne_points'),
            ('echoes', 'scan'),
            ('imu', '/zed/zed_node/imu/data')],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': False},
            {'resolution': 0.05}],
        )
    
    tf1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','-0.25','0.0','0.0','base_link','velodyne']
        )
    tf2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','base_link','zed_imu_link']
        )

    scan = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            # Remap the input point cloud topic
            remappings=[
                ('cloud_in', '/velodyne_points') # Or your PointCloud2 topic
            ],
            parameters=[
                {
                    'target_frame': 'velodyne',  # The frame to which the sensor is attached
                    'transform_tolerance': 0.01,
                    'min_height': -0.1,          # Minimum height of points to consider
                    'max_height': 1.0,           # Maximum height of points to consider
                    'angle_min': -3.1415,      # Start angle of the scan [rad]
                    'angle_max': 3.1415,       # End angle of the scan [rad]
                    'angle_increment': 0.0007, # Angular resolution of the scan [rad]
                    'scan_time': 0.1,
                    'range_min': 0.45,           # Minimum range of the scan [m]
                    'range_max': 20.0,           # Maximum range of the scan [m]
                    'use_inf': True,
                    'inf_epsilon': 1.0
                }
            ]
        )
 


    return LaunchDescription([
        use_sim_time_arg,
        tf1,
        tf2,
        scan,
        # Nodes
        # robot_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])

