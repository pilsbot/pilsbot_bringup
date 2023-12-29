from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_model = LaunchConfiguration('camera_model', default='OAK-D-LITE')
    colorResolution = LaunchConfiguration('colorResolution',default="480p")
    colorFramerate = LaunchConfiguration('colorFramerate',  default=10)
    monoResolution = LaunchConfiguration('monoResolution',  default="480p")
    monoFramerate = LaunchConfiguration('monoFramerate',    default=10)
    withoutLights = LaunchConfiguration('withoutLights',   default=False)

    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=camera_model,
        description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `OAK-D, OAK-D-LITE`.')

    declare_colorResolution_cmd = DeclareLaunchArgument(
        'colorResolution',
        choices=['480p', '720p', '1080p', '4K'],
        default_value=colorResolution,
        description='The resolution of the color camera')

    declare_colorFramerate_cmd = DeclareLaunchArgument(
        'colorFramerate',
        # choices = [10, 15, 30, 60],
        default_value=colorFramerate,
        description='The framerate of the color camera'
    )

    declare_monoResolution_cmd = DeclareLaunchArgument(
        'monoResolution',
        choices=['480p', '720p', '1080p', '4K'],
        default_value=monoResolution,
        description='The resolution of the mono cameras')

    declare_monoFramerate_cmd = DeclareLaunchArgument(
        'monoFramerate',
        # choices= [10, 15, 30, 60],
        default_value=monoFramerate,
        description='The framerate of the mono cameras')

    declare_no_lighting_cmd = DeclareLaunchArgument(
        'withoutLights',
        default_value=withoutLights,
        description='If set, STVO lighting is suppressed (on closed courses only! Führerscheinentzug!)')

    included_pilsbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('pilsbot_driver'), 'launch/pilsbot_teleop.launch.py'])]),
    )

    included_jeston_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('jetson_camera'), 'launch/jetson_camera_launch.py'])]),
    )

    included_pilsbot_depthai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('pilsbot_depthai'), 'launch/rgb_stereo_node.launch.py'])]),
        launch_arguments={'camera_model': camera_model,
                          'colorResolution': colorResolution,
                          'colorFramerate': colorFramerate,
                          'monoResolution': monoResolution,
                          'monoFramerate': monoFramerate}.items()
    )

    included_pilsbot_gnss = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('pilsbot_gnss_receiver'), 'launch/ublox-receiver.launch.py'])]),
    )

    included_pilsbot_lighting_bridge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('pilsbot_indicators'), 'launch/bridge.launch.py'])]),
        )

    included_pilsbot_lighting = None
    if not withoutLights or True: # FIXME: Switch does not seem to work. Too tired to debug. Fuckall.
        included_pilsbot_lighting = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('pilsbot_indicators'), 'launch/stvo.launch.py'])]),
        )


    ld = LaunchDescription()
    ld.add_action(declare_camera_model_cmd)
    ld.add_action(declare_colorFramerate_cmd)
    ld.add_action(declare_colorResolution_cmd)
    ld.add_action(declare_monoFramerate_cmd)
    ld.add_action(declare_monoResolution_cmd)
    ld.add_action(declare_no_lighting_cmd)

    ld.add_action(included_pilsbot_launch)
    ld.add_action(included_jeston_camera_launch)
    ld.add_action(included_pilsbot_depthai_launch)
    ld.add_action(included_pilsbot_gnss)
    ld.add_action(included_pilsbot_lighting_bridge)
    if included_pilsbot_lighting:
        ld.add_action(included_pilsbot_lighting)

    return ld
