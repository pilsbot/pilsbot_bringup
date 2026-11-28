"""
Cameras, for the camera container on the Nano (ADR 0003).

The OAK runs mjpeg_node (pilsbot_depthai): JPEG straight from the camera,
decoded on the Jetson only if enable_raw_decode:=true. Further mjpeg_node
arguments (e.g. color_fps:=15) pass through.

The CSI camera runs jetson_camera and publishes /front_camera/image_raw (with
/compressed below it) and /front_camera/camera_info in
front_camera_optical_frame: an image carries an optical frame (z forward,
x right, y down; REP 103), not the x-forward front_camera_link it hangs
from (pilsbot_description's URDF). Its base topic is set with the node's
topic_name parameter rather than a remap: image_transport derives camera_info
and the transport topics from the base topic, while a remap of image_raw would
leave image_raw/compressed (the topic that is recorded) under its old name.
jetson_camera's launch file does not expose topic_name, so it is set with
SetParameter in the CSI camera's scope.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare

CSI_TOPIC = '/front_camera/image_raw'
CSI_FRAME = 'front_camera_optical_frame'


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
        GroupAction(
            condition=IfCondition(csi_camera),
            actions=[
                SetParameter('topic_name', CSI_TOPIC),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution(
                        [FindPackageShare('jetson_camera'), 'launch',
                         'jetson_camera_launch.py'])),
                    launch_arguments={'frame_id': CSI_FRAME}.items(),
                ),
            ]),
    ])
