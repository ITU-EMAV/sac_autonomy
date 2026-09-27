"""Shows the car in RViz without a simulation, with sliders for the joints.

Useful to check frames and sensor mounts after changing urdf/sac.urdf.xacro.
Needs rviz2 and joint_state_publisher_gui, which the car and the headless simulation images
do not install.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("sac_description")
    return LaunchDescription(
        [
            IncludeLaunchDescription(PathJoinSubstitution([share, "launch", "description.launch.py"])),
            Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([share, "rviz", "config.rviz"])],
            ),
        ]
    )
