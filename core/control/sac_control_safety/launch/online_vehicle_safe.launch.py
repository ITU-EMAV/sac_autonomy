"""Fail-closed online graph. Mapping/NDT and real actuation are excluded."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node



def _default_use_sim_time():
    """Default for use_sim_time, from smart_car_launch/config/use_sim_time.yaml."""
    path = os.path.join(get_package_share_directory('smart_car_launch'),
                        'config', 'use_sim_time.yaml')
    with open(path, encoding='utf-8') as config_file:
        return str(bool((yaml.safe_load(config_file) or {})['use_sim_time'])).lower()


def _default_max_speed_mps():
    """Use the planner controller's configured speed ceiling for the full graph."""
    path = os.path.join(get_package_share_directory('planner'), 'config', 'config.yaml')
    with open(path, encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}
    return str(config['controller_exe']['ros__parameters']['linear_velocity'])

def include(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), "launch", launch_file)),
        launch_arguments=(arguments or {}).items())


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    replay_mode = LaunchConfiguration("replay_mode")
    lidar_worker_threads = LaunchConfiguration("lidar_worker_threads")
    lidar_debug = LaunchConfiguration("lidar_debug")
    serialize_gpu = LaunchConfiguration("serialize_gpu")
    camera_mode = LaunchConfiguration("camera_mode")
    camera_true = "'.lower() in ('true', '1', 'yes', 'on')"
    replay_true = "'.lower() in ('true', '1', 'yes', 'on')"
    mapped_localization = UnlessCondition(PythonExpression([
        "'", camera_mode, camera_true, " or '", replay_mode, replay_true]))
    replay_localization = IfCondition(PythonExpression([
        "'", replay_mode, replay_true, " and not ('", camera_mode, camera_true, ")"]))
    localization_config = os.path.join(
        get_package_share_directory("sac_localization"), "params", "localization.yaml")
    guardian_config = os.path.join(
        get_package_share_directory("sac_control_safety"), "config", "guardian.yaml")
    common = {"use_sim_time": use_sim_time}
    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_guardian", default_value="true",
            description="Disable Guardian only for command-generation debugging; UDP stays disabled"),
        DeclareLaunchArgument("enable_road_hazards", default_value="true"),
        DeclareLaunchArgument("enable_dynamic_behavior", default_value="true"),
        DeclareLaunchArgument("max_speed_mps", default_value=_default_max_speed_mps()),
        DeclareLaunchArgument("hazard_model", default_value=os.path.join(
            get_package_share_directory("jetson_perception"), "models",
            "road_hazard", "road_hazard_fp16.engine")),
        DeclareLaunchArgument("use_sim_time", default_value=_default_use_sim_time()),
        DeclareLaunchArgument(
            "replay_mode", default_value="false",
            description="Use recorded ZED odometry only after an operator /initialpose"),
        DeclareLaunchArgument(
            "lidar_worker_threads", default_value="2",
            description="Bounded LiDAR component-container executor workers"),
        DeclareLaunchArgument(
            "lidar_debug", default_value="false",
            description="Short replay profiling only; enables LiDAR stage timing logs"),
        DeclareLaunchArgument(
            "serialize_gpu", default_value="true",
            description="Serialize road/sign GPU inference while preserving latest-frame workers"),
        DeclareLaunchArgument(
            "camera_mode", default_value="false",
            description="Camera-only driving: the controller runs pure pursuit on the "
                        "base_link road-mask path instead of the Lanelet trajectory"),
        Node(package="sac_localization", executable="sac_localization_node",
             name="sac_localization", condition=mapped_localization,
             parameters=[localization_config, common, {
                 "localization_mode": "gps", "lidar.enabled": False,
                 "output.publish_frequency": 50.0}],
             remappings=[("odom", "/localization/online/odometry")], output="screen"),
        Node(package="sac_control_safety", executable="odometry_pose_bridge",
             name="online_localization_pose_bridge", condition=mapped_localization,
             parameters=[common], output="screen"),
        Node(package="sac_control_safety", executable="replay_localization_adapter",
             name="replay_localization_adapter", condition=replay_localization,
             parameters=[common, {"replay_only": True}], output="screen"),
        include("smart_car_launch", "lidar_perception.launch.py", {
            "use_sim_time": use_sim_time, "debug": lidar_debug,
            "worker_threads": lidar_worker_threads}),
        include("jetson_perception", "perception.launch.py", {
            "use_sim_time": use_sim_time, "publish_visualization": "false",
            "serialize_gpu": serialize_gpu,
            "enable_road_hazards": LaunchConfiguration("enable_road_hazards"),
            "hazard_model": LaunchConfiguration("hazard_model"),
            "hazard_auto_depth_preview": "false"}),
        include("planner", "planner_composable_launch.launch.py", {
            "reactive_mode": camera_mode, "launch_rviz": "false",
            "enable_road_hazards": LaunchConfiguration("enable_road_hazards"),
            "enable_dynamic_behavior": LaunchConfiguration("enable_dynamic_behavior"),
            "max_speed_mps": LaunchConfiguration("max_speed_mps"),
            "use_sim_time": use_sim_time}),
        # Do not force __node for this executable: it owns multiple deliberately
        # distinct receiver nodes, and a global rename would duplicate their names.
        Node(package="sac_network", executable="udp_receiver",
             parameters=[common], output="screen"),
        Node(package="sac_control_safety", executable="vehicle_speed_mux.py",
             name="vehicle_speed_mux", parameters=[
                 os.path.join(get_package_share_directory("sac_control_safety"),
                              "config", "vehicle_speed.yaml"), common], output="screen"),
        Node(package="sac_control_safety", executable="longitudinal_controller.py",
             name="longitudinal_controller", parameters=[
                 os.path.join(get_package_share_directory("sac_control_safety"),
                              "config", "longitudinal.yaml"), common], output="screen"),
        Node(package="sac_control_safety", executable="command_guardian",
             name="command_guardian", condition=IfCondition(LaunchConfiguration("enable_guardian")),
             parameters=[guardian_config, common, {
                 "controller_input_topic": "/control/longitudinal_command",
                 "max_target_speed_mps": LaunchConfiguration("max_speed_mps"),
                 "localization_topic": "/localization/ekf_localizer/pose_with_cov",
                 "require_localization": PythonExpression([
                     "'", camera_mode, "'.lower() not in ('true', '1', 'yes', 'on')"]),
                 "trajectory_topic": PythonExpression([
                     "'/reactive_planner/trajectory' if '", camera_mode,
                     "'.lower() in ('true', '1', 'yes', 'on') else '/trajectory_planner/trajectory'"])
             }], output="screen"),
        Node(package="sac_network", executable="udp_sender",
             name="udp_actuator_transport", parameters=[common, {
                 "transport_enabled": False,
                 "gateway_supports_safety_contract": False}], output="screen"),
    ])
