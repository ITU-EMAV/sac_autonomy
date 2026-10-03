"""Localization: the local (odom) and the global (map) filter.

Arguments:
  use_sim_time:=true      in the simulation
  config:=sim             config/<config>_local.yaml and config/<config>_global.yaml
  estimator:=ekf          engine for both filters (default: the config's): ekf | iekf | ukf
  motion_model:=...       constant_acceleration (the config's, default) | imu_driven |
                          kinematic_bicycle | dynamic_bicycle;
                          other models also load config/models/<model>.yaml
  lidar_odometry:=fast_lio
                          also run FAST-LIO2 (external/FAST_LIO_ROS2, GPL-2.0, its own process)
                          with config/fast_lio/<config>.yaml, its motion a velocity input of
                          the local filter (config/lio/<config>_local.yaml)

  ros2 launch sac_localization localization.launch.py use_sim_time:=true estimator:=ukf
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    share = get_package_share_directory("sac_localization")
    config = LaunchConfiguration("config").perform(context)
    estimator = LaunchConfiguration("estimator").perform(context)
    motion_model = LaunchConfiguration("motion_model").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    lidar_odometry = LaunchConfiguration("lidar_odometry").perform(context)

    overrides = {"use_sim_time": use_sim_time}
    if estimator:
        overrides["estimator.type"] = estimator
    overlay = []
    if motion_model and motion_model != "constant_acceleration":
        overlay = [os.path.join(share, "config", "models", f"{motion_model}.yaml")]

    extra = []
    if lidar_odometry == "fast_lio":
        overlay.append(os.path.join(share, "config", "lio", f"{config}_local.yaml"))
        extra.append(
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                name="fast_lio",
                output="screen",
                parameters=[os.path.join(share, "config", "fast_lio", f"{config}.yaml"), {"use_sim_time": use_sim_time}],
                remappings=[("/Odometry", "/sac/localization/lio/odometry"), ("/path", "/sac/localization/lio/path")],
            )
        )
    elif lidar_odometry:
        raise RuntimeError(f"lidar_odometry: fast_lio or nothing, not '{lidar_odometry}'")

    return extra + [
        Node(
            package="sac_localization",
            executable="localization_node",
            name=f"localization_{instance}",
            output="screen",
            parameters=[os.path.join(share, "config", f"{config}_{instance}.yaml")] + overlay + [overrides],
        )
        for instance in ("local", "global")
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("config", default_value="sim"),
            DeclareLaunchArgument("estimator", default_value=""),
            DeclareLaunchArgument("motion_model", default_value=""),
            DeclareLaunchArgument("lidar_odometry", default_value=""),
            OpaqueFunction(function=nodes),
        ]
    )
