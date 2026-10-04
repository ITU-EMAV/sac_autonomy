import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("smart_car_launch")
    lio_sam_share_dir = get_package_share_directory("lio_sam")
    params_file = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("rviz")
    robot_xacro = os.path.join(share_dir, "urdf", "smart_car.urdf.xacro")

    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"use_sim_time": True,
                         "robot_description": Command(["xacro ", robot_xacro])}],
            output="screen",
        ),
    ]

    for executable in (
        "lio_sam_imuPreintegration",
        "lio_sam_imageProjection",
        "lio_sam_featureExtraction",
        "lio_sam_mapOptimization",
    ):
        nodes.append(Node(package="lio_sam", executable=executable, name=executable,
                          parameters=[params_file], output="screen"))

    nodes.append(Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(lio_sam_share_dir, "config", "rviz2.rviz")],
        condition=IfCondition(use_rviz),
        output="screen",
    ))

    nodes.append(Node(
        package="smart_car_launch",
        executable="lio_sam_auto_save.py",
        parameters=[{
            "idle_seconds": 5.0,
            "resolution": 0.2,
            "destination":
                str(Path.home() / ".local/share/sac_autonomy/maps/lio_sam"),
        }],
        output="screen",
    ))

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                share_dir, "config", "localization", "lio_sam_without_mag.yaml"),
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        *nodes,
    ])
