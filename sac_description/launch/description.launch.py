"""Publishes the car's frames: robot_state_publisher with urdf/sac.urdf.xacro.

On the real car this gives the TF from base_footprint to every sensor frame. The wheel and
steering joints move with /joint_states when the vehicle interface publishes it.

Arguments:
  use_sim_time:=true   in a simulation or when playing a bag with --clock
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf = PathJoinSubstitution([FindPackageShare("sac_description"), "urdf", "sac.urdf.xacro"])
    # value_type=str: otherwise the URDF text is parsed as YAML, and a ":" in it breaks that
    robot_description = ParameterValue(Command(["xacro ", urdf]), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
        ]
    )
