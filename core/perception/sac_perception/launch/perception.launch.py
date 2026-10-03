"""Perception: the local occupancy grid (/sac/perception/grid).

Arguments:
  use_sim_time:=true    in the simulation
  config:=sim           config/<config>.yaml
  map:=sparse_voxel     another map representation: config/map_<map>.yaml over it
                        (default: the config's, direct_projection)
  camera:=true          the front camera's depth too (config/camera.yaml)

  ros2 launch sac_perception perception.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    share = get_package_share_directory("sac_perception")
    config = LaunchConfiguration("config").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    parameters = [os.path.join(share, "config", f"{config}.yaml")]
    if LaunchConfiguration("camera").perform(context).lower() == "true":
        parameters.append(os.path.join(share, "config", "camera.yaml"))
    representation = LaunchConfiguration("map").perform(context)
    if representation:
        parameters.append(os.path.join(share, "config", f"map_{representation}.yaml"))
    return [
        Node(
            package="sac_perception",
            executable="perception_node",
            name="perception",
            output="screen",
            parameters=parameters + [{"use_sim_time": use_sim_time}],
            remappings=[
                ("~/grid", "/sac/perception/grid"),
                ("~/map", "/sac/perception/map"),
                ("~/objects", "/sac/perception/objects"),
                ("~/objects/markers", "/sac/perception/objects/markers"),
                ("~/timing", "/sac/perception/timing"),
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("config", default_value="sim"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("camera", default_value="false"),
            OpaqueFunction(function=nodes),
        ]
    )
