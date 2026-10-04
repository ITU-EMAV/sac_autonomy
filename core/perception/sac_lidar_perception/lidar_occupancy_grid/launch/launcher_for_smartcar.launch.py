import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

flag_for_tf = True  # Set this to False to disable TF publishing

def generate_launch_description():
    # parameter files
    pkg = get_package_share_directory
    config_occupancy = os.path.join(pkg('lidar_occupancy_grid'), 'config', 'config.params.yaml')
    config_ground   = os.path.join(pkg('ground_segmentation'), 'config', 'ground_segmentation.param.yaml')
    config_cluster  = os.path.join(pkg('object_clustering'), 'config', 'object_clustering.param.yaml')
    config_filter   = os.path.join(pkg('lidar_occupancy_grid'), 'config', 'lidar_filters.params.yaml')

    # nodes
    nodes = [
        Node(
            package='lidar_occupancy_grid',
            executable='lidar_filter',
            output='screen',
            parameters=[config_filter]
        ),
        Node(
            package='lidar_occupancy_grid',
            executable='lidar_occupancy_grid',
            output='screen',
            parameters=[config_occupancy]
        ),
        Node(
            package='ground_segmentation',
            executable='travel_ground_filter_node',
            output='screen',
            parameters=[config_ground]
        ),
        Node(
            package='object_clustering',
            executable='travel_object_clustering_node',
            output='screen',
            parameters=[config_cluster]
        )
    ]

    # optionally add robot_state_publisher
    if flag_for_tf:
        xacro_file = os.path.join(pkg('lidar_occupancy_grid'), 'urdf', 'smart_car.urdf.xacro')
        doc = xacro.process_file(xacro_file)
        robot_description = {'robot_description': doc.toxml()}
        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[robot_description]
            )
        )

    return LaunchDescription(nodes)
