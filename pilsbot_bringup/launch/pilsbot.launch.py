from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# foxglove_bridge 3.x capabilities that only show data: no publishing, no
# service calls, no parameter access. The bridge has no authentication.
FOXGLOVE_READ_ONLY = ['connectionGraph', 'assets']
FOXGLOVE_ALL = ['clientPublish', 'parameters', 'parametersSubscribe', 'services',
                'connectionGraph', 'assets']
# Matches no topic: clients may not advertise topics even if clientPublish
# is enabled by mistake without a whitelist.
FOXGLOVE_NO_CLIENT_TOPICS = ['(?!)']


def foxglove_capabilities_from(arg):
    """Parse the foxglove_capabilities launch argument into a list."""
    if arg.strip() == 'all':
        return list(FOXGLOVE_ALL)
    capabilities = [c.strip() for c in arg.split(',') if c.strip()]
    unknown = [c for c in capabilities if c not in FOXGLOVE_ALL]
    if unknown:
        raise RuntimeError(
            f'Unknown foxglove capabilities {unknown}; known: {", ".join(FOXGLOVE_ALL)}')
    return capabilities


def foxglove_bridge_setup(context):
    capabilities = foxglove_capabilities_from(
        LaunchConfiguration('foxglove_capabilities').perform(context))
    parameters = {
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true',
        'port': int(LaunchConfiguration('foxglove_port').perform(context)),
        # An empty list would be dropped as an untyped parameter, so the bridge
        # would fall back to its defaults (all capabilities). '' is no capability.
        'capabilities': capabilities or [''],
    }
    if 'clientPublish' not in capabilities:
        parameters['client_topic_whitelist'] = FOXGLOVE_NO_CLIENT_TOPICS
    actions = []
    write = [c for c in capabilities if c not in FOXGLOVE_READ_ONLY]
    if write:
        actions.append(LogInfo(msg=(
            '\n' + '!' * 72 +
            f'\n!! foxglove_bridge allows {", ".join(write)} without authentication:'
            '\n!! anyone on the network can drive or reconfigure the robot.'
            '\n' + '!' * 72)))
    actions.append(Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[parameters],
    ))
    return actions


def devices_setup(context, gnss, lighting):
    """GNSS and lighting talk to devices that only the robot has."""
    if LaunchConfiguration('hardware').perform(context) == 'real':
        return [gnss, lighting]
    # The simulation has no GNSS receiver and no head MCU (lighting would also
    # fail to open its serial port).
    return [LogInfo(msg='hardware:=gazebo: GNSS and lighting are not started '
                        '(no devices in the simulation).')]


def generate_launch_description():
    # Stack container (ADR 0003): driver, teleop, GNSS, lighting, foxglove_bridge.
    # The cameras run in the camera container: camera.launch.py.
    # hardware:=gazebo (pilsbot_simulation) starts the same stack without GNSS
    # and lighting; the controller manager then runs inside Gazebo.
    withLights = LaunchConfiguration('withLights', default='true')
    robot_profile = LaunchConfiguration('robot_profile', default='pilsbot')
    foxglove = LaunchConfiguration('foxglove', default='true')
    foxglove_port = LaunchConfiguration('foxglove_port', default='8765')
    foxglove_capabilities = LaunchConfiguration(
        'foxglove_capabilities', default=','.join(FOXGLOVE_READ_ONLY))

    declare_foxglove_cmd = DeclareLaunchArgument(
        'foxglove',
        default_value=foxglove,
        description='Start foxglove_bridge; it only sends what a connected client subscribes to.')

    declare_foxglove_port_cmd = DeclareLaunchArgument(
        'foxglove_port',
        default_value=foxglove_port,
        description='WebSocket port of foxglove_bridge')

    declare_foxglove_capabilities_cmd = DeclareLaunchArgument(
        'foxglove_capabilities',
        default_value=foxglove_capabilities,
        description='Comma-separated foxglove_bridge capabilities. The default is read-only. '
                    'Development only, never at public outings: add clientPublish, '
                    'parameters, parametersSubscribe, services (or "all"); anyone on the '
                    'network can then drive and reconfigure the robot.')

    declare_hardware_cmd = DeclareLaunchArgument(
        'hardware', default_value='real', choices=['real', 'gazebo'],
        description='real: the robot\'s devices. gazebo: the simulation '
                    '(pilsbot_simulation); no GNSS and no lighting.')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', choices=['true', 'false'],
        description='Use /clock (true in simulation).')

    declare_robot_profile_cmd = DeclareLaunchArgument(
        'robot_profile',
        default_value=robot_profile,
        description='Robot profile: a name shipped with pilsbot_description, or a path.')

    declare_lighting_cmd = DeclareLaunchArgument(
        'withLights',
        default_value=withLights,
        description='Start the StVO lighting. withLights:=false only on closed courses '
                    '(Führerscheinentzug!)')

    included_pilsbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('pilsbot_driver'), 'launch/pilsbot_teleop.launch.py'])]),
        launch_arguments={
            'robot_profile': robot_profile,
            'hardware': LaunchConfiguration('hardware'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    included_pilsbot_gnss = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('pilsbot_gnss_receiver'), 'launch/ublox-receiver.launch.py'])]),
    )

    included_pilsbot_lighting = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('pilsbot_indicators'), 'launch/stvo.launch.py'])]),
        launch_arguments={'robot_profile': robot_profile}.items(),
        condition=IfCondition(withLights),
    )

    devices = OpaqueFunction(
        function=devices_setup, args=[included_pilsbot_gnss, included_pilsbot_lighting])

    foxglove_bridge = OpaqueFunction(
        function=foxglove_bridge_setup, condition=IfCondition(foxglove))

    ld = LaunchDescription()
    ld.add_action(declare_hardware_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_lighting_cmd)
    ld.add_action(declare_robot_profile_cmd)
    ld.add_action(declare_foxglove_cmd)
    ld.add_action(declare_foxglove_port_cmd)
    ld.add_action(declare_foxglove_capabilities_cmd)

    ld.add_action(included_pilsbot_launch)
    ld.add_action(devices)
    ld.add_action(foxglove_bridge)

    return ld
