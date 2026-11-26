"""
Cameras, for the camera container on the Nano (ADR 0003).

The OAK runs mjpeg_node (pilsbot_depthai): JPEG straight from the camera,
decoded on the Jetson only if enable_raw_decode:=true. The CSI camera runs
jetson_camera. Further mjpeg_node arguments (e.g. color_fps:=15) pass through.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nvdecoder = LaunchConfiguration('nvdecoder')
    enable_raw_decode = LaunchConfiguration('enable_raw_decode')
    csi_camera = LaunchConfiguration('csi_camera')

    return LaunchDescription([
        DeclareLaunchArgument(
            'nvdecoder', default_value='nvv4l2decoder',
            description='Jetson hardware JPEG decoder for enable_raw_decode (ADR 0004)'),
        DeclareLaunchArgument(
            'enable_raw_decode', default_value='false',
            description='Also publish the decoded colour image; recording and Foxglove '
                        'use the JPEG topics'),
        DeclareLaunchArgument(
            'csi_camera', default_value='true',
            description='Start the CSI camera (jetson_camera)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('pilsbot_depthai'), 'launch', 'mjpeg_node.launch.py'])),
            launch_arguments={'nvdecoder': nvdecoder,
                              'enable_raw_decode': enable_raw_decode}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('jetson_camera'), 'launch', 'jetson_camera_launch.py'])),
            condition=IfCondition(csi_camera),
        ),
    ])
