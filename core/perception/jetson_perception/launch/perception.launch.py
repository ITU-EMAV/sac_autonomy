import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def _default_use_sim_time():
    """Default for use_sim_time, from smart_car_launch/config/use_sim_time.yaml."""
    path = os.path.join(get_package_share_directory('smart_car_launch'),
                        'config', 'use_sim_time.yaml')
    with open(path, encoding='utf-8') as config_file:
        return str(bool((yaml.safe_load(config_file) or {})['use_sim_time'])).lower()

def generate_launch_description():
    model_dir = os.path.join(get_package_share_directory("jetson_perception"), "models")
    road_engine = os.path.join(model_dir, "yolopv2_road_masks_fp16.engine")
    road_checkpoint = os.path.join(model_dir, "yolopv2.pt")
    default_config = os.path.join(
        get_package_share_directory("jetson_perception"), "config", "perception.yaml"
    )
    config = LaunchConfiguration("config")
    image_topic = LaunchConfiguration("image_topic")
    road_weights = LaunchConfiguration("road_weights")
    sign_model = LaunchConfiguration("sign_model")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_visualization = LaunchConfiguration("publish_visualization")
    serialize_gpu = LaunchConfiguration("serialize_gpu")

    return LaunchDescription([
        DeclareLaunchArgument("enable_road_hazards", default_value="true"),
        DeclareLaunchArgument("hazard_rgb_only_preview", default_value="false"),
        # Direct bag replay can show RGB detections even without registered depth.
        # Metric hazard control is selected separately by the planner launch.
        DeclareLaunchArgument("hazard_auto_depth_preview", default_value="true"),
        DeclareLaunchArgument("hazard_model", default_value=os.path.join(model_dir, "road_hazard", "road_hazard_fp16.engine")),
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument(
            "image_topic", default_value="/zed/zed_node/left/image_rect_color"
        ),
        DeclareLaunchArgument(
            "road_weights",
            default_value=(road_engine if os.path.isfile(road_engine) and
                           os.path.getsize(road_engine) > 0 else road_checkpoint),
        ),
        DeclareLaunchArgument(
            "sign_model",
            default_value=os.path.join(model_dir, "traffic_sign_22cls.engine"),
        ),
        DeclareLaunchArgument("use_sim_time", default_value=_default_use_sim_time()),
        DeclareLaunchArgument("publish_visualization", default_value="true"),
        DeclareLaunchArgument("serialize_gpu", default_value="true"),
        Node(
            package="jetson_perception", executable="road_hazard_node", name="road_hazard_node",
            condition=IfCondition(LaunchConfiguration("enable_road_hazards")),
            parameters=[config, {"model_path": LaunchConfiguration("hazard_model"),
                                 "image_topic": image_topic, "use_sim_time": use_sim_time,
                                 "serialize_gpu": serialize_gpu,
                                 "rgb_only_preview": LaunchConfiguration("hazard_rgb_only_preview"),
                                 "auto_depth_preview": LaunchConfiguration("hazard_auto_depth_preview"),
                                 "publish_visualization": publish_visualization}], output="screen"),
        Node(
            package="jetson_perception",
            executable="yolopv2_road_node",
            name="yolopv2_road_node",
            output="screen",
            parameters=[config, {"image_topic": image_topic, "weights": road_weights,
                                 "use_sim_time": use_sim_time,
                                 "serialize_gpu": serialize_gpu,
                                 "publish_visualization": publish_visualization}],
        ),
        Node(
            package="jetson_perception",
            executable="traffic_sign_node",
            name="traffic_sign_node",
            output="screen",
            parameters=[config, {"image_topic": image_topic, "model_path": sign_model,
                                 "use_sim_time": use_sim_time,
                                 "serialize_gpu": serialize_gpu,
                                 "publish_visualization": publish_visualization}],
        ),
        Node(package="jetson_perception", executable="dynamic_object_tracker",
             name="dynamic_object_tracker", parameters=[{"use_sim_time": use_sim_time}],
             output="screen"),
    ])
