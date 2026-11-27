"""Kohlfahrt: the stack (pilsbot.launch.py) and recording (log_kohl.launch.py)."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    included_teleop_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('pilsbot_bringup'), 'launch/pilsbot.launch.py'])])
    )

    # Arguments such as output_dir:=... and groups:=... reach log_kohl.launch.py.
    included_logging_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('pilsbot_bringup'), 'launch/log_kohl.launch.py'])])
    )

    ld = LaunchDescription()
    ld.add_action(included_teleop_camera_launch)
    ld.add_action(included_logging_launch)

    return ld
