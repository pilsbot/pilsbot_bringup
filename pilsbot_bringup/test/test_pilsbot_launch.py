# What pilsbot.launch.py starts for hardware:=real and hardware:=gazebo.
# The launch description is expanded without starting anything: the included
# launch files are identified by their package, not loaded.
import importlib.util
from pathlib import Path
import re

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'pilsbot.launch.py'


def expand(**arguments):
    """Declare the arguments and run the OpaqueFunctions, like `ros2 launch` does."""
    spec = importlib.util.spec_from_file_location('pilsbot_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    context = LaunchContext()
    context.launch_configurations.update(arguments)
    pending = list(module.generate_launch_description().entities)
    actions = []
    while pending:
        entity = pending.pop(0)
        if entity.condition is not None and not entity.condition.evaluate(context):
            continue
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(context)  # applies the default and checks the choices
        elif isinstance(entity, OpaqueFunction):
            pending[:0] = entity.visit(context) or []
        else:
            actions.append(entity)
    return context, actions


def evaluate(context, substitutions):
    return perform_substitutions(context, normalize_to_list_of_substitutions(substitutions))


def includes(context, actions):
    """{package of the included launch file: {argument: value}} of what is started."""
    started = {}
    for include in actions:
        if isinstance(include, IncludeLaunchDescription):
            location = include.launch_description_source.location
            # Humble's launch prints an unresolved substitution path as
            # "...FindPackageShare(pkg='<name>')..."; no public accessor.
            [package] = re.findall(r"FindPackageShare\(pkg='([^']+)'\)", location)
            started[package] = {evaluate(context, key): evaluate(context, value)
                                for key, value in include.launch_arguments}
    return started


def foxglove_parameters(context, actions):
    # Node._Node__parameters: Humble's launch_ros has no public accessor.
    [bridge] = [a for a in actions
                if isinstance(a, Node) and a.node_executable == 'foxglove_bridge']
    merged = {}
    for entry in evaluate_parameters(context, bridge._Node__parameters):
        merged.update(entry)
    return merged


def log_messages(context, actions):
    return [evaluate(context, a.msg) for a in actions if isinstance(a, LogInfo)]


# --- hardware:=real (the robot) ----------------------------------------------

def test_real_is_the_default_and_starts_driver_gnss_and_lighting():
    context, actions = expand()

    started = includes(context, actions)
    assert set(started) == {'pilsbot_driver', 'pilsbot_gnss_receiver', 'pilsbot_indicators'}
    assert started['pilsbot_driver']['hardware'] == 'real'
    assert started['pilsbot_driver']['use_sim_time'] == 'false'


def test_real_runs_foxglove_on_wall_time():
    context, actions = expand()

    assert foxglove_parameters(context, actions)['use_sim_time'] is False


def test_real_without_lights_still_starts_gnss():
    context, actions = expand(withLights='false')

    assert set(includes(context, actions)) == {'pilsbot_driver', 'pilsbot_gnss_receiver'}


def test_real_says_nothing_about_the_simulation():
    context, actions = expand()

    assert not any('gazebo' in m for m in log_messages(context, actions))


# --- hardware:=gazebo (simulation) -------------------------------------------

@pytest.mark.parametrize('with_lights', ['true', 'false'])
def test_gazebo_starts_no_gnss_and_no_lighting(with_lights):
    context, actions = expand(hardware='gazebo', use_sim_time='true', withLights=with_lights)

    assert set(includes(context, actions)) == {'pilsbot_driver'}


def test_gazebo_says_why_gnss_and_lighting_are_missing():
    context, actions = expand(hardware='gazebo', use_sim_time='true')

    [message] = log_messages(context, actions)
    assert 'hardware:=gazebo' in message
    assert 'GNSS' in message and 'lighting' in message


def test_hardware_and_use_sim_time_reach_the_driver():
    context, actions = expand(hardware='gazebo', use_sim_time='true', robot_profile='p')

    assert includes(context, actions)['pilsbot_driver'] == {
        'robot_profile': 'p', 'hardware': 'gazebo', 'use_sim_time': 'true'}


@pytest.mark.parametrize('use_sim_time, expected', [('true', True), ('false', False)])
def test_foxglove_gets_use_sim_time(use_sim_time, expected):
    context, actions = expand(hardware='gazebo', use_sim_time=use_sim_time)

    assert foxglove_parameters(context, actions)['use_sim_time'] is expected


def test_foxglove_off_starts_no_bridge():
    _, actions = expand(hardware='gazebo', use_sim_time='true', foxglove='false')

    assert not any(isinstance(a, Node) for a in actions)


# --- invalid arguments --------------------------------------------------------

@pytest.mark.parametrize('value', ['', 'fake', 'kinematic'])
def test_an_unknown_hardware_fails_the_launch(value):
    with pytest.raises(RuntimeError, match='"hardware"'):
        expand(hardware=value)


def test_an_unknown_use_sim_time_fails_the_launch():
    with pytest.raises(RuntimeError, match='"use_sim_time"'):
        expand(use_sim_time='yes')
