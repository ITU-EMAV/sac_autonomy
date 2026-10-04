import os
import launch  # noqa: E402
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
import json


def generate_launch_description():

    config_dir = os.path.join(
        get_package_share_directory('sac_perception_cpp'),
        'config',
        "zedc.json")
    # with open(config_dir, "r") as f:
    #     data = json.load(f)

    

    actionList = []

    # i = 0
    # for key, transform in data.items():
    #     world_frame = transform["world"]
    #     translation = world_frame["translation"]
    #     rotation = world_frame["rotation"]

    #     # convert Image coordinate to ROS coordinate
    #     actionList.append(
    #         Node(package='tf2_ros',
    #              executable='static_transform_publisher',
    #              name=f'static_tf_pub_zed_{i}',
    #              arguments=[
    #                         f"{float(translation[2])}",
    #                         f"{float(-translation[0])}",
    #                         f"{float(-translation[1])}",
    #                         f"{float(-rotation[1])}",
    #                         # no pitch movement. handled in safety code.
    #                         '0.0',
    #                         f"{float(rotation[2])}",
    #                         'base_link',
    #                         f"zed_{i}"
    #              ],
    #              ),
    #     )
    #     i += 1

    actionList.append(
        Node(package='sac_perception_cpp',
             executable='perception_pcl',
             name='perception_pcl',
             arguments=[config_dir],
             output='screen',
            )
    )

    bringup_cmd_group = GroupAction(actionList)

    ld = LaunchDescription()
    ld.add_action(bringup_cmd_group)

    return ld
