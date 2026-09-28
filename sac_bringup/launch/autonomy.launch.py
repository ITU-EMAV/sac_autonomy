"""The autonomy chain of the SAC car: planning -> control, the same in the simulation and
on the real car. The car's side (drivers or gazebo_environment) must be running and give the
map -> base_footprint TF.

Arguments:
  use_sim_time:=true   in the simulation
  localization:=true   also start sac_localization, which then gives map -> base_footprint
                       (run the simulation with ground_truth_tf:=false)
  perception:=true     also start sac_perception: the local occupancy grid
                       (/sac/perception/grid) from the lidars
  estimator:=, motion_model:=
                       passed to the localization (see sac_localization's launch file)
  site:=sonoma         site config sac_planning/config/<site>.yaml (datum and default route)
  route:=<file>        another route of the site (a path, or a file in sac_planning/routes)
  max_speed:=8.0       speed limit [m/s] instead of sac_control's config (e.g. first runs on the car)

  ros2 launch sac_bringup autonomy.launch.py use_sim_time:=true
  ros2 launch sac_bringup autonomy.launch.py use_sim_time:=true localization:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    site = LaunchConfiguration("site").perform(context)
    route = LaunchConfiguration("route").perform(context)
    max_speed = LaunchConfiguration("max_speed").perform(context)

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
            ]
            + ([{"max_speed": float(max_speed)}] if max_speed else []),
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
            DeclareLaunchArgument("max_speed", default_value="", description="[m/s] (default: sac_control's)"),
            DeclareLaunchArgument("localization", default_value="false"),
            DeclareLaunchArgument("estimator", default_value=""),
            DeclareLaunchArgument("motion_model", default_value=""),
            DeclareLaunchArgument("perception", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("sac_perception"), "launch", "perception.launch.py")
                ),
                launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
                condition=IfCondition(LaunchConfiguration("perception")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("sac_localization"), "launch", "localization.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "estimator": LaunchConfiguration("estimator"),
                    "motion_model": LaunchConfiguration("motion_model"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("localization")),
            ),
            OpaqueFunction(function=nodes),
        ]
    )
