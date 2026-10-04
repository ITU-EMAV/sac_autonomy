from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def configure_localization(context, *args, **kwargs):
    # Get package directory
    pkg_share = FindPackageShare('sac_localization').find('sac_localization')

    # Get launch configuration variables
    localization_mode = LaunchConfiguration('localization_mode').perform(context)
    run_lidarslam = LaunchConfiguration('run_lidarslam').perform(context)
    param_file_path = LaunchConfiguration('param_file').perform(context)

    # Determine which parameter file to use based on mode
    config_file = os.path.join(pkg_share, 'params', 'localization.yaml')
    if localization_mode == 'lidar':
        config_file = os.path.join(pkg_share, 'params', 'lidar_mode.yaml')
    elif localization_mode == 'fusion':
        config_file = os.path.join(pkg_share, 'params', 'fusion_mode.yaml')

    # If a custom param file is provided, use that instead
    if param_file_path and param_file_path != '':
        config_file = param_file_path

    nodes_to_launch = []

    # Try to find lidarslam package and include it if found
    try:
        # Find lidarslam package
        lidarslam_share = FindPackageShare('lidarslam').find('lidarslam')
        lidarslam_launch = os.path.join(lidarslam_share, 'launch', 'lidarslam.launch.py')

        # Include LiDAR SLAM launch file if it exists and run_lidarslam is true
        if os.path.exists(lidarslam_launch) and run_lidarslam == 'true':
            lidarslam_launch_include = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidarslam_launch)
            )
            nodes_to_launch.append(lidarslam_launch_include)
            print("LiDAR SLAM launch file found and will be included")
        else:
            print("LiDAR SLAM launch file found but will not be included (run_lidarslam=false)")
    except:
        print("LiDAR SLAM package not found. Only localization will be launched.")

    # Create the localization node
    localization_node = Node(
        package='sac_localization',
        executable='sac_localization_node',
        name='sac_localization',
        parameters=[
            config_file,
            {'localization_mode': localization_mode}
        ],
        output='screen'
    )
    nodes_to_launch.append(localization_node)

    # Print debug info
    print(f"Launching localization node with mode: {localization_mode}")
    print(f"Using config file: {config_file}")
    print(f"Config file contents:")

    try:
        with open(config_file, 'r') as f:
            for line in f.readlines()[:20]:  # Show first 20 lines
                print(f"  {line.strip()}")
    except Exception as e:
        print(f"Error reading config file: {e}")



    tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','-0.25','0.0','0.0','base_link','velodyne']
        )
    nodes_to_launch.append(tf)


    return nodes_to_launch

def generate_launch_description():
    # Declare launch arguments
    declare_localization_mode = DeclareLaunchArgument(
        'localization_mode',
        default_value='gps',
        description='Localization mode: gps, lidar, or fusion'
    )

    declare_run_lidarslam = DeclareLaunchArgument(
        'run_lidarslam',
        default_value='true',
        description='Whether to run LiDAR SLAM'
    )

    declare_param_file = DeclareLaunchArgument(
        'param_file',
        default_value='',
        description='Optional: Path to parameter file to override defaults'
    )

    # Return the launch description
    return LaunchDescription([
        declare_localization_mode,
        declare_run_lidarslam,
        declare_param_file,
        OpaqueFunction(function=configure_localization)
    ])
