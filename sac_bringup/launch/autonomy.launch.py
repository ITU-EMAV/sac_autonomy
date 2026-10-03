"""The autonomy chain of the SAC car: planning -> control, the same in the simulation and
on the real car. The car's side (drivers or gazebo_environment) must be running and give the
map -> base_footprint TF.

Arguments:
  use_sim_time:=true   in the simulation
  localization:=true   also start sac_localization, which then gives map -> base_footprint
                       (run the simulation with ground_truth_tf:=false)
  perception:=true     also start sac_perception: the local occupancy grid
                       (/sac/perception/grid) from the lidars
  perception_map:=sparse_voxel
                       the perception's map representation (default: direct_projection)
  perception_camera:=true
                       the perception uses the front camera's depth too
  local_planner:=true  drive around obstacles: sac_local_planner follows the route around
                       the grid's obstacles (or stops before them) and the controller drives
                       its trajectory; starts the perception too
  estimator:=, motion_model:=, lidar_odometry:=
                       passed to the localization (see sac_localization's launch file)
  site:=sonoma         site config sac_planning/config/<site>.yaml (datum and default route)
  route:=<file>        another route of the site (a path, or a file in sac_planning/routes)
  max_speed:=8.0       speed limit [m/s] instead of sac_control's config (e.g. first runs on the car)

  ros2 launch sac_bringup autonomy.launch.py use_sim_time:=true
  ros2 launch sac_bringup autonomy.launch.py use_sim_time:=true localization:=true
  ros2 launch sac_bringup autonomy.launch.py use_sim_time:=true local_planner:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, launch_file, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package), "launch", launch_file)),
        launch_arguments=arguments.items(),
    )


def nodes(context):
    def flag(name):
        return LaunchConfiguration(name).perform(context).lower() == "true"

    use_sim_time = flag("use_sim_time")
    site = LaunchConfiguration("site").perform(context)
    route = LaunchConfiguration("route").perform(context)
    max_speed = LaunchConfiguration("max_speed").perform(context)
    local_planner = flag("local_planner")
    sim_time = "true" if use_sim_time else "false"

    site_config = os.path.join(get_package_share_directory("sac_planning"), "config", f"{site}.yaml")
    planner_parameters = [site_config, {"use_sim_time": use_sim_time}]
    if route:
        planner_parameters.append({"route": route})

    actions = []
    if flag("perception") or local_planner:
        actions.append(
            include(
                "sac_perception",
                "perception.launch.py",
                {
                    "use_sim_time": sim_time,
                    "map": LaunchConfiguration("perception_map").perform(context),
                    "camera": LaunchConfiguration("perception_camera").perform(context),
                },
            )
        )
    if local_planner:
        actions.append(
            include("sac_local_planner", "local_planner.launch.py", {"use_sim_time": sim_time, "max_speed": max_speed})
        )
    controller_parameters = [
        os.path.join(get_package_share_directory("sac_control"), "config", "pure_pursuit.yaml"),
        {"use_sim_time": use_sim_time, "input": "trajectory" if local_planner else "path"},
    ] + ([{"max_speed": float(max_speed)}] if max_speed else [])

    return actions + [
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
            parameters=controller_parameters,
            remappings=[
                ("path", "/sac/planning/path"),
                ("trajectory", "/sac/planning/trajectory"),
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
            DeclareLaunchArgument("lidar_odometry", default_value=""),
            DeclareLaunchArgument("perception", default_value="false"),
            DeclareLaunchArgument("local_planner", default_value="false"),
            DeclareLaunchArgument("perception_map", default_value=""),
            DeclareLaunchArgument("perception_camera", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("sac_localization"), "launch", "localization.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "estimator": LaunchConfiguration("estimator"),
                    "motion_model": LaunchConfiguration("motion_model"),
                    "lidar_odometry": LaunchConfiguration("lidar_odometry"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("localization")),
            ),
            OpaqueFunction(function=nodes),
        ]
    )
