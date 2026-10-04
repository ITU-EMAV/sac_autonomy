# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch pointcloud processing nodes for rosbag playback - no driver needed."""

import os

import ament_index_python.packages
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # Input pointcloud topic from rosbag
    input_pointcloud_topic_arg = DeclareLaunchArgument(
        'input_pointcloud_topic',
        default_value='velodyne_points',  # Change this to match your rosbag topic name
        description='Input pointcloud topic name from rosbag'
    )

    cropbox_filter_param_path = os.path.join(
        ament_index_python.packages.get_package_share_directory('smart_car_launch'),
        'config',
        'lidar',
        'cropbox.param.yaml'
    )

    output_cropbox_filtered_points_topic_arg = DeclareLaunchArgument(
        'output_cropbox_filtered_points_topic',
        default_value='/sensing/lidar_top/points_cropbox_filtered',
        description='Output cropbox filtered points topic name'
    )

    distorsion_corrector_param_path = os.path.join(
        ament_index_python.packages.get_package_share_directory('smart_car_launch'),
        'config',
        'lidar',
        'distortion_corrector_node.param.yaml'
    )

    input_twist_topic_arg = DeclareLaunchArgument(
        'input_twist_topic',
        default_value='/gyro_twist_with_covariance',
        description='Input twist topic name'
    )
    input_imu_topic_arg = DeclareLaunchArgument(
        'input_imu_topic',
        default_value='/sbg/ros/imu/data_corrected',
        description='Input IMU topic name'
    )

    output_corrected_points_topic_arg = DeclareLaunchArgument(
        'output_corrected_points_topic',
        default_value='/sensing/lidar_top/points_corrected',
        description='Output corrected points topic name'
    )

    downsample_param_path = os.path.join(
        ament_index_python.packages.get_package_share_directory('smart_car_launch'),
        'config',
        'lidar',
        'downsample.param.yaml'
    )

    output_downsampled_points_topic_arg = DeclareLaunchArgument(
        'output_downsampled_points_topic',
        default_value='/sensing/lidar_top/points_downsampled',
        description='Output downsampled points topic name'
    )

    # ---------------------------------------------------------------- #
    # base_link -> velodyne static TF arguments
    # ---------------------------------------------------------------- #
    publish_velodyne_tf_arg = DeclareLaunchArgument(
        'publish_velodyne_tf',
        default_value='true',
        description='Publish base_link -> velodyne static TF. '
                    'Set false if /tf_static is already recorded in the rosbag.'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Parent frame of the static transform'
    )

    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value='velodyne',
        description='Child frame of the static transform (lidar frame_id)'
    )

    lidar_x_arg = DeclareLaunchArgument(
        'lidar_x', default_value='0.0', description='Lidar x offset [m]')
    lidar_y_arg = DeclareLaunchArgument(
        'lidar_y', default_value='0.0', description='Lidar y offset [m]')
    lidar_z_arg = DeclareLaunchArgument(
        'lidar_z', default_value='0.0', description='Lidar z offset [m]')
    lidar_yaw_arg = DeclareLaunchArgument(
        'lidar_yaw', default_value='0.0', description='Lidar yaw [rad]')
    lidar_pitch_arg = DeclareLaunchArgument(
        'lidar_pitch', default_value='0.0', description='Lidar pitch [rad]')
    lidar_roll_arg = DeclareLaunchArgument(
        'lidar_roll', default_value='0.0', description='Lidar roll [rad]')

    # args order: x y z yaw pitch roll parent_frame child_frame
    velodyne_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_velodyne',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_velodyne_tf')),
        arguments=[
            LaunchConfiguration('lidar_x'),
            LaunchConfiguration('lidar_y'),
            LaunchConfiguration('lidar_z'),
            LaunchConfiguration('lidar_yaw'),
            LaunchConfiguration('lidar_pitch'),
            LaunchConfiguration('lidar_roll'),
            LaunchConfiguration('base_frame'),
            LaunchConfiguration('lidar_frame'),
        ]
    )

    container = ComposableNodeContainer(
            name='pointcloud_processing_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Removed velodyne_driver and velodyne_transform nodes
                ComposableNode(
                    package='pointcloud_preprocessor',
                    plugin='pointcloud_preprocessor::CropBoxFilterComponent',
                    name='cropbox_filter',
                    parameters=[cropbox_filter_param_path],
                    remappings=[
                        ('input', LaunchConfiguration('input_pointcloud_topic')),  # Now uses rosbag topic
                        ('output', LaunchConfiguration('output_cropbox_filtered_points_topic'))
                        ]),
                ComposableNode(
                    package='pointcloud_preprocessor',
                    plugin='pointcloud_preprocessor::DistortionCorrectorComponent',
                    name='distortion_corrector',
                    parameters=[distorsion_corrector_param_path],
                    remappings=[
                        ('~/input/pointcloud', LaunchConfiguration('output_cropbox_filtered_points_topic')),
                        ('~/input/twist', LaunchConfiguration('input_twist_topic')),
                        ('~/input/imu', LaunchConfiguration('input_imu_topic')),
                        ('~/output/pointcloud', LaunchConfiguration('output_corrected_points_topic'))
                    ]),
                ComposableNode(
                    package='pointcloud_preprocessor',
                    plugin='pointcloud_preprocessor::VoxelGridDownsampleFilterComponent',
                    name='voxel_grid_downsample_filter',
                    parameters=[downsample_param_path],
                    remappings=[
                        ('input', LaunchConfiguration('output_corrected_points_topic')),
                        ('output', LaunchConfiguration('output_downsampled_points_topic'))
                    ]),
    ])

    return LaunchDescription([
                              input_pointcloud_topic_arg,
                              output_cropbox_filtered_points_topic_arg,
                              input_twist_topic_arg,
                              input_imu_topic_arg,
                              output_corrected_points_topic_arg,
                              output_downsampled_points_topic_arg,
                              publish_velodyne_tf_arg,
                              base_frame_arg,
                              lidar_frame_arg,
                              lidar_x_arg,
                              lidar_y_arg,
                              lidar_z_arg,
                              lidar_yaw_arg,
                              lidar_pitch_arg,
                              lidar_roll_arg,
                              velodyne_static_tf,
                              container
                             ])