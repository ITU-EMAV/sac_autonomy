"""Local planner: the route around the perception grid's obstacles (/sac/planning/trajectory).

Arguments:
  use_sim_time:=true    in the simulation
  config:=sim           config/<config>.yaml
  max_speed:=8.0        [m/s] instead of the config's

  ros2 launch sac_local_planner local_planner.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    share = get_package_share_directory("sac_local_planner")
    config = LaunchConfiguration("config").perform(context)
    max_speed = LaunchConfiguration("max_speed").perform(context)
    overrides = {"use_sim_time": LaunchConfiguration("use_sim_time").perform(context).lower() == "true"}
    if max_speed:
        overrides["limits.max_speed"] = float(max_speed)
    return [
        Node(
            package="sac_local_planner",
            executable="local_planner_node",
            name="local_planner",
            output="screen",
            parameters=[os.path.join(share, "config", f"{config}.yaml"), overrides],
            remappings=[
                ("path", "/sac/planning/path"),
                ("grid", "/sac/perception/grid"),
                ("objects", "/sac/perception/objects"),
                ("~/trajectory", "/sac/planning/trajectory"),
                ("~/candidates", "/sac/planning/candidates"),
                ("~/timing", "/sac/planning/timing"),
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("config", default_value="sim"),
            DeclareLaunchArgument("max_speed", default_value=""),
            OpaqueFunction(function=nodes),
        ]
    )
