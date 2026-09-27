"""The autonomy chain of the SAC car: planning -> control, the same in the simulation and
on the real car. The car's side (drivers or gazebo_environment) must be running and give the
map -> base_footprint TF.

Arguments:
  use_sim_time:=true   in the simulation
  site:=sonoma         site config sac_planning/config/<site>.yaml (datum and default route)
  route:=<file>        another route of the site (a path, or a file in sac_planning/routes)

  ros2 launch sac_bringup autonomy.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    site = LaunchConfiguration("site").perform(context)
    route = LaunchConfiguration("route").perform(context)

    site_config = os.path.join(get_package_share_directory("sac_planning"), "config", f"{site}.yaml")
    planner_parameters = [site_config, {"use_sim_time": use_sim_time}]
    if route:
        planner_parameters.append({"route": route})

    return [
        Node(
            package="sac_planning",
            executable="route_planner",
            name="route_planner",
            output="screen",
            parameters=planner_parameters,
            remappings=[
                ("~/path", "/sac/planning/path"),
                ("~/geojson", "/sac/planning/route_geojson"),
            ],
        ),
        Node(
            package="sac_control",
            executable="pure_pursuit",
            name="pure_pursuit",
            output="screen",
            parameters=[
                os.path.join(get_package_share_directory("sac_control"), "config", "pure_pursuit.yaml"),
                {"use_sim_time": use_sim_time},
            ],
            remappings=[
                ("path", "/sac/planning/path"),
                ("cmd_vel", "/sac/actuators/cmd_vel"),
                ("~/lookahead", "/sac/control/lookahead"),
                ("~/cross_track_error", "/sac/control/cross_track_error"),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("site", default_value="sonoma"),
            DeclareLaunchArgument("route", default_value="", description="Route file (default: the site's)"),
            OpaqueFunction(function=nodes),
        ]
    )
