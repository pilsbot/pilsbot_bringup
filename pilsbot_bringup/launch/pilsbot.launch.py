from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Stack container (ADR 0003): driver, teleop, GNSS, lighting, foxglove_bridge.
    # The cameras run in the camera container: camera.launch.py.
    withLights = LaunchConfiguration('withLights',   default=True)      # deactivate with withLights:=false
    robot_profile = LaunchConfiguration('robot_profile', default='pilsbot')
    foxglove = LaunchConfiguration('foxglove', default='true')
    foxglove_port = LaunchConfiguration('foxglove_port', default='8765')

    declare_foxglove_cmd = DeclareLaunchArgument(
        'foxglove',
        default_value=foxglove,
        description='Start foxglove_bridge; it only sends what a connected client subscribes to.')

    declare_foxglove_port_cmd = DeclareLaunchArgument(
        'foxglove_port',
        default_value=foxglove_port,
        description='WebSocket port of foxglove_bridge')

    declare_robot_profile_cmd = DeclareLaunchArgument(
        'robot_profile',
        default_value=robot_profile,
        description='Robot profile: a name shipped with pilsbot_description, or a path.')

    declare_lighting_cmd = DeclareLaunchArgument(
        'withLights',
        default_value=withLights,
        description='If set, STVO lighting is suppressed (on closed courses only! Führerscheinentzug!)')

    included_pilsbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('pilsbot_driver'), 'launch/pilsbot_teleop.launch.py'])]),
        launch_arguments={'robot_profile': robot_profile}.items(),
    )

    included_pilsbot_gnss = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('pilsbot_gnss_receiver'), 'launch/ublox-receiver.launch.py'])]),
    )

    included_pilsbot_lighting = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('pilsbot_indicators'), 'launch/stvo.launch.py'])
                ]),
        launch_arguments={'robot_profile': robot_profile}.items(),
        condition=IfCondition(withLights),
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{'port': foxglove_port}],
        condition=IfCondition(foxglove),
    )

    ld = LaunchDescription()
    ld.add_action(declare_lighting_cmd)
    ld.add_action(declare_robot_profile_cmd)
    ld.add_action(declare_foxglove_cmd)
    ld.add_action(declare_foxglove_port_cmd)

    ld.add_action(included_pilsbot_launch)
    ld.add_action(included_pilsbot_gnss)
    ld.add_action(included_pilsbot_lighting)
    ld.add_action(foxglove_bridge)

    return ld
