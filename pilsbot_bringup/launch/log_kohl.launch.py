"""Record a Kohlfahrt log: the topics of config/recording.yaml as MCAP."""
from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
import yaml

DEFAULT_OUTPUT_DIR = '/workspaces/bags'


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


def launch_setup(context):
    with open(LaunchConfiguration('recording_config').perform(context)) as f:
        config = yaml.safe_load(f)
    output_dir = LaunchConfiguration('output_dir').perform(context)
    actions = []
    if not output_dir:
        output_dir = DEFAULT_OUTPUT_DIR
        actions.append(LogInfo(msg=(
            '\n' + '!' * 72 +
            f'\n!! Recording to the default {DEFAULT_OUTPUT_DIR}.'
            '\n!! Is that the external SSD? Pass output_dir:=<dir> to record elsewhere.'
            '\n' + '!' * 72)))
    output = os.path.join(output_dir, datetime.now().strftime('kohl_%Y_%m_%d-%H_%M_%S'))
    cmd = record_command(config, LaunchConfiguration('groups').perform(context), output)
    actions.append(ExecuteProcess(cmd=cmd, output='screen'))
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
        OpaqueFunction(function=launch_setup),
    ])
