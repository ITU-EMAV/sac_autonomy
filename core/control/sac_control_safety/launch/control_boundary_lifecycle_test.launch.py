"""Lightweight non-actuating lifecycle test for the complete ROS command boundary."""
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    planner_config = os.path.join(
        get_package_share_directory("planner"), "config", "config.yaml")
    with open(planner_config, encoding="utf-8") as config_file:
        controller = yaml.safe_load(config_file)["controller_exe"]["ros__parameters"]
    guardian = os.path.join(
        get_package_share_directory("sac_control_safety"), "config", "guardian.yaml")
    return LaunchDescription([
        Node(package="sac_control_safety", executable="replay_localization_adapter",
             parameters=[{"replay_only": True, "use_sim_time": False}]),
        Node(package="planner", executable="controller_node", name="controller_exe",
             parameters=[controller, {"use_sim_time": False}], output="log"),
        Node(package="sac_network", executable="udp_receiver", output="log"),
        Node(package="sac_control_safety", executable="command_guardian",
             parameters=[guardian, {"use_sim_time": False}], output="log"),
        Node(package="sac_network", executable="udp_sender", parameters=[{
            "use_sim_time": False, "transport_enabled": False,
            "gateway_supports_safety_contract": False}], output="log"),
    ])
