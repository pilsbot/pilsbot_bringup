"""Record a Kohlfahrt log: the topics of config/recording.yaml as MCAP."""
from datetime import datetime
import os
import shutil
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

DEFAULT_OUTPUT_DIR = '/workspaces/bags'
# Where the devcontainers bind-mount DEFAULT_OUTPUT_DIR from on the robot.
HOST_OUTPUT_DIR = '/media/externSSD/bags'
# Recording is refused unless output_dir contains this file.
DEFAULT_MARKER = '.pilsbot_bags'


def banner(*lines):
    """Return text that stands out in the launch output."""
    return '\n'.join(['', '!' * 72, *[f'!! {line}' for line in lines], '!' * 72])


def bag_name(now=None):
    """
    Return a new bag directory name.

    The Nano has no RTC, so after a reboot the date may repeat; the random
    suffix keeps names unique anyway.
    """
    now = now or datetime.now()
    return now.strftime('kohl_%Y_%m_%d-%H_%M_%S_') + uuid.uuid4().hex[:6]


def record_command(config, groups, output):
    """Build the ros2 bag record command for the selected topic groups."""
    available = config['groups']
    selected = list(available) if groups == 'all' else [g.strip() for g in groups.split(',')]
    unknown = [g for g in selected if g not in available]
    if unknown:
        raise RuntimeError(
            f'Unknown recording group(s) {unknown}; available: {", ".join(available)}')
    topics = []
    for group in selected:
        topics += [t for t in available[group] if t not in topics]
    return ['ros2', 'bag', 'record',
            '--storage', config['storage'],
            '--max-bag-duration', str(config['max_bag_duration']),
            '--output', output,
            *topics]


def _existing_ancestor(path):
    path = os.path.abspath(path)
    while not os.path.exists(path):
        path = os.path.dirname(path)
    return path


def output_dir_problems(output_dir, min_free_gb, marker=DEFAULT_MARKER):
    """
    Return why recording to output_dir is refused (empty if fine).

    Without the SSD, /media/externSSD/bags is a plain directory on the Nano's
    root filesystem, which recording would fill. Filesystem heuristics do not
    tell the two apart reliably in a container (overlayfs root, Docker's
    data-root possibly on the SSD itself), so the SSD carries a marker file
    instead: no marker, no recording. The free space check stays.
    """
    problems = []
    marker_path = os.path.join(output_dir, marker)
    if not os.path.exists(marker_path):
        problems.append(f'no marker file {marker_path}')
    existing = _existing_ancestor(output_dir)
    free_gb = shutil.disk_usage(existing).free / 1e9
    if free_gb < min_free_gb:
        problems.append(f'only {free_gb:.1f} GB free in {existing} (min_free_gb:={min_free_gb:g})')
    return problems


def refusal(output_dir, marker, problems):
    """Return the banner lines for not recording, saying what to do."""
    lines = ['NOT RECORDING to ' + output_dir + ':', *[f'  - {p}' for p in problems]]
    if any(p.startswith('no marker file') for p in problems):
        if os.path.normpath(output_dir) == DEFAULT_OUTPUT_DIR:
            where = f'on the robot host (not in the container): touch {HOST_OUTPUT_DIR}/{marker}'
        else:
            where = f'touch {os.path.join(output_dir, marker)}'
        lines += ['Is the external SSD mounted? Mount it first.',
                  'If it is mounted and was never marked, mark it ONCE, with the SSD mounted,',
                  f'  {where}',
                  'Then start log_kohl.launch.py again.']
    return lines + ['To record anyway (not recommended): check_output_dir:=false']


def recorder_exited(event, context):
    """Report loudly that recording stopped; the driving stack keeps running."""
    if context.is_shutdown:
        return [LogInfo(msg=f'Recorder stopped (return code {event.returncode}).')]
    return [LogInfo(msg=banner(
        f'THE RECORDER EXITED (return code {event.returncode}). NOTHING IS RECORDED ANY MORE.',
        'Driving continues. See the messages above: disk full, output exists,',
        'MCAP plugin missing? Start log_kohl.launch.py again to record.'))]


def launch_setup(context):
    with open(LaunchConfiguration('recording_config').perform(context)) as f:
        config = yaml.safe_load(f)
    output_dir = LaunchConfiguration('output_dir').perform(context)
    check = LaunchConfiguration('check_output_dir').perform(context).lower() == 'true'
    min_free_gb = float(LaunchConfiguration('min_free_gb').perform(context))
    period = float(LaunchConfiguration('static_republish_period').perform(context))
    actions = []
    if not output_dir:
        output_dir = DEFAULT_OUTPUT_DIR
        actions.append(LogInfo(msg=banner(
            f'Recording to the default {DEFAULT_OUTPUT_DIR}.',
            'Is that the external SSD? Pass output_dir:=<dir> to record elsewhere.')))
    output = os.path.join(output_dir, bag_name())
    # Before the output dir check, so a typo in groups is reported either way.
    cmd = record_command(config, LaunchConfiguration('groups').perform(context), output)

    # Refusing only skips the recorder; the driving stack must not depend on it.
    marker = LaunchConfiguration('output_dir_marker').perform(context)
    problems = output_dir_problems(output_dir, min_free_gb, marker)
    if problems and check:
        return actions + [LogInfo(msg=banner(*refusal(output_dir, marker, problems)))]
    if problems:
        actions.append(LogInfo(msg=banner(
            'check_output_dir:=false, so recording although:',
            *[f'  - {p}' for p in problems])))

    if period > 0:
        actions.append(Node(
            package='pilsbot_bringup',
            executable='static_data_republisher',
            parameters=[{'period': period}],
            output='screen',
        ))
    recorder = ExecuteProcess(cmd=cmd, output='screen')
    actions.append(recorder)
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=recorder, on_exit=recorder_exited)))
    actions.append(LogInfo(msg=f'Recording to {output}'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_dir', default_value='',
            description='Directory for the new bag. CHANGE THIS unless '
                        f'{DEFAULT_OUTPUT_DIR} (the SSD mount in the devcontainers) is right.'),
        DeclareLaunchArgument(
            'groups', default_value='all',
            description='Comma-separated topic groups from the recording config, or "all"'),
        DeclareLaunchArgument(
            'recording_config',
            default_value=os.path.join(get_package_share_directory('pilsbot_bringup'),
                                       'config', 'recording.yaml'),
            description='Topic groups and storage settings'),
        DeclareLaunchArgument(
            'check_output_dir', default_value='true',
            description='Do not record unless output_dir contains output_dir_marker and has '
                        'at least min_free_gb free; false only warns.'),
        DeclareLaunchArgument(
            'output_dir_marker', default_value=DEFAULT_MARKER,
            description='File that marks output_dir as the bag disk (the external SSD). '
                        f'Create it once on the host: touch {HOST_OUTPUT_DIR}/{DEFAULT_MARKER}'),
        DeclareLaunchArgument(
            'min_free_gb', default_value='10',
            description='Free space in GB that check_output_dir requires'),
        DeclareLaunchArgument(
            'static_republish_period', default_value='10.0',
            description='Seconds between re-publishing /tf_static and /robot_description, '
                        'so that every split file contains them; 0 disables.'),
        OpaqueFunction(function=launch_setup),
    ])
