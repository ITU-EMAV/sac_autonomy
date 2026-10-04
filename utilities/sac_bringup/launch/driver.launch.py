"""Launch the vehicle drivers and SAC bringup stack."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_file(package_name, filename, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(package_name), 'launch', filename]
            )
        ),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    zed_params = PathJoinSubstitution(
        [FindPackageShare('sac_bringup'), 'config', 'zed_odometry.yaml']
    )

    zed2 = _launch_file(
        'zed_wrapper',
        'zed_camera.launch.py',
        {
            'camera_model': 'zed2',
            'ros_params_override_path': zed_params,
            # Publish only the camera's internal, fixed URDF tree. Localization
            # remains the sole owner of the odom/map transforms above it.
            'publish_urdf': 'true',
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'publish_imu_tf': 'false',
        },
    )

    velodyne = _launch_file(
        'velodyne',
        'velodyne-all-nodes-VLP16-launch.py',
    )

    sac_bringup = _launch_file(
        'sac_bringup',
        'bringup.launch.py',
    )

    return LaunchDescription([
        zed2,
        velodyne,
        sac_bringup,
    ])
