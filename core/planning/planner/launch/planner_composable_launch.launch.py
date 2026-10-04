import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node



def _default_use_sim_time():
    """Default for use_sim_time, from smart_car_launch/config/use_sim_time.yaml."""
    path = os.path.join(get_package_share_directory('smart_car_launch'),
                        'config', 'use_sim_time.yaml')
    with open(path, encoding='utf-8') as config_file:
        return str(bool((yaml.safe_load(config_file) or {})['use_sim_time'])).lower()

def _as_bool(value):
    return str(value).lower() in ('true', '1', 'yes', 'on')

def generate_launch_description():
    planner_share = get_package_share_directory('planner')
    map_share = get_package_share_directory('smart_car_launch')
    config_dir = os.path.join(planner_share, 'config', 'config.yaml')
    map_config_path = os.path.join(map_share, 'config', 'maps', 'map_config.yaml')

    with open(map_config_path, encoding='utf-8') as map_config_file:
        map_config = yaml.safe_load(map_config_file) or {}

    default_osm_path = map_config.get('lanelet2_map_path', '')
    if default_osm_path and not os.path.isabs(default_osm_path):
        default_osm_path = os.path.join(map_share, 'maps', default_osm_path)
    if not default_osm_path:
        raise RuntimeError(
            f"lanelet2_map_path is not configured in {map_config_path}"
        )
    osm_path = LaunchConfiguration('osm_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    common_parameters = {'osm_path': osm_path, 'use_sim_time': use_sim_time}

    with open(config_dir, encoding='utf-8') as config_file:
        launch_config = yaml.safe_load(config_file) or {}
    controller_parameters = launch_config.get(
        'controller_exe', {}).get('ros__parameters', {})
    traffic_parameters = launch_config.get(
        'traffic_speed_planner', {}).get('ros__parameters', {})
    hazard_parameters = launch_config.get(
        'road_hazard_speed_planner', {}).get('ros__parameters', {})
    behavior_parameters = launch_config.get(
        'behavior_manager', {}).get('ros__parameters', {})
    reactive_default = launch_config.get(
        'planner_launch', {}).get('ros__parameters', {}).get('reactive_mode', False)
    rviz_default = launch_config.get(
        'planner_launch', {}).get('ros__parameters', {}).get('launch_rviz', False)
    enforce_freshness = launch_config.get(
        'planner_launch', {}).get('ros__parameters', {}).get('enforce_freshness', True)

    osm_path_arg = DeclareLaunchArgument(
        'osm_path',
        default_value=default_osm_path,
        description='Absolute path to the Lanelet2 OSM map.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=_default_use_sim_time(),
        description='Use /clock when replaying a rosbag.',
    )
    reactive_mode_arg = DeclareLaunchArgument(
        'reactive_mode',
        default_value=str(reactive_default).lower(),
        description='Use a road-mask local trajectory instead of Lanelet/localization.',
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value=str(rviz_default).lower(),
        description='Launch RViz with the base_link reactive-planning view.',
    )
    global_planner = ComposableNode(
        package='planner',
        plugin='smart_car::GlobalPlanner',
        name='global_planner_exe',
        parameters=[config_dir, common_parameters],
    )

    mission_planner = ComposableNode(
        package='planner',
        plugin='smart_car::MissionPlanner',
        name='mission_planner_exe',
        parameters=[config_dir, common_parameters],
    )

    occupancy_grid = ComposableNode(
        package='planner',
        plugin='smart_car::OccupancyGrid',
        name='occupancy_grid_exe',
        parameters=[config_dir, common_parameters],
    )

    trajectory_planner = ComposableNode(
        package='planner',
        plugin='smart_car::TrajectoryPlanner',
        name='trajectory_planner_exe',
        parameters=[config_dir, common_parameters],
    )

    def launch_setup(context):
        max_speed = float(LaunchConfiguration('max_speed_mps').perform(context))
        if not 0.0 < max_speed <= 2.0:
            raise ValueError('max_speed_mps must be within (0, 2.0] for initial commissioning')
        reactive_mode = _as_bool(LaunchConfiguration('reactive_mode').perform(context))
        traffic_enabled = _as_bool(LaunchConfiguration('enable_traffic_behavior').perform(context))
        hazards_enabled = _as_bool(LaunchConfiguration('enable_road_hazards').perform(context))
        dynamic_enabled = _as_bool(LaunchConfiguration('enable_dynamic_behavior').perform(context))
        controller_overrides = {'use_sim_time': use_sim_time,
                                'enforce_freshness': enforce_freshness,
                                'linear_velocity': max_speed,
                                'require_hazard_speed_constraint': hazards_enabled,
                                'require_traffic_speed_constraint': traffic_enabled,
                                'require_map_speed_constraint': not reactive_mode,
                                'require_dynamic_speed_constraint': dynamic_enabled,
                                'command_topic': LaunchConfiguration('controller_command_topic')}
        if reactive_mode:
            controller_overrides.update({
                'trajectory_topic': '/reactive_planner/trajectory',
                'global_frame': 'base_link',
                'local_path_mode': True,
                # The reactive path is refreshed continuously; stop quickly if the mask is lost.
                'path_timeout': 0.5,
            })

        controller = Node(
            package='planner', executable='controller_node', name='controller_exe',
            parameters=[controller_parameters, controller_overrides], output='screen')

        if reactive_mode:
            descriptions = [
                ComposableNode(
                    package='planner',
                    plugin='smart_car::ReactiveRoadPlanner',
                    name='reactive_road_planner_exe',
                    parameters=[config_dir, {'use_sim_time': use_sim_time}],
                ),
            ]
        else:
            descriptions = [
                trajectory_planner,
                occupancy_grid,
                mission_planner,
                global_planner,
            ]

        actions = [ComposableNodeContainer(
            name='planner_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=descriptions,
            output='screen',
        ), controller]
        if traffic_enabled:
            traffic_config = dict(traffic_parameters)
            traffic_config['camera_timeout'] = float(
                LaunchConfiguration('traffic_camera_timeout').perform(context))
            actions.append(Node(
                package='planner', executable='traffic_speed_planner.py',
                name='traffic_speed_planner', output='screen',
            parameters=[traffic_config, {'use_sim_time': use_sim_time,
                    'enforce_freshness': enforce_freshness,
                    'max_speed_mps': max_speed,
                    'speed_topic': LaunchConfiguration('traffic_speed_topic')}]))
        if not reactive_mode:
            actions.append(Node(
                package='planner', executable='map_speed_constraint.py',
                name='map_speed_constraint', output='screen',
                parameters=[{'use_sim_time': use_sim_time,
                             'enforce_freshness': enforce_freshness}]))
        if dynamic_enabled:
            actions.append(Node(
                package='planner', executable='behavior_manager.py',
                name='behavior_manager', output='screen',
                parameters=[behavior_parameters, {'use_sim_time': use_sim_time,
                    'enforce_freshness': enforce_freshness,
                    'max_speed_mps': max_speed}]))
        if hazards_enabled:
            actions.append(Node(
                package='planner', executable='road_hazard_speed_planner.py',
                name='road_hazard_speed_planner', output='screen',
                parameters=[hazard_parameters, {'use_sim_time': use_sim_time,
                    'enforce_freshness': enforce_freshness,
                    'max_speed_mps': max_speed,
                    'path_topic': '/reactive_planner/trajectory' if reactive_mode else
                                  '/trajectory_planner/trajectory'}]))
        if reactive_mode and _as_bool(LaunchConfiguration('launch_rviz').perform(context)):
            actions.append(Node(
                package='rviz2',
                executable='rviz2',
                name='reactive_planning_rviz',
                arguments=['-d', os.path.join(os.path.dirname(config_dir), 'reactive_planning.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
                output='log',
            ))
        return actions

    return LaunchDescription([
        DeclareLaunchArgument(
            'max_speed_mps',
            default_value=str(controller_parameters['linear_velocity']),
            description='Speed ceiling from planner/config/config.yaml controller_exe.linear_velocity; at most 2 m/s'),
        DeclareLaunchArgument("enable_traffic_behavior", default_value="true"),
        DeclareLaunchArgument(
            'traffic_camera_timeout',
            default_value=str(traffic_parameters.get('camera_timeout', 0.75)),
            description='Traffic camera freshness in seconds; bag debug may need more margin.'),
        # The standalone launch also serves RGB-only bag replay. Full live
        # launches explicitly enable these metric-depth consumers.
        DeclareLaunchArgument("enable_road_hazards", default_value="false"),
        DeclareLaunchArgument("enable_dynamic_behavior", default_value="false"),
        DeclareLaunchArgument('controller_command_topic',
                              default_value='/control/controller_command'),
        DeclareLaunchArgument('traffic_speed_topic',
                              default_value='/vehicle/speed'),
        osm_path_arg,
        use_sim_time_arg,
        reactive_mode_arg,
        launch_rviz_arg,
        OpaqueFunction(function=launch_setup),
    ])
