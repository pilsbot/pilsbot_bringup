"""
Convert a Foxy-era Pilsbot bag (sqlite3) to the Humble frames, as MCAP.

- /odom from the front axle (odom_raw -> base_link) is re-rooted at the rear
  axle centre (odom -> base_footprint), ADR 0002.
- Recorded /tf, /tf_static and /robot_description are replaced: the current
  URDF from the robot profile gives /robot_description and /tf_static, and
  /tf gets odom -> base_footprint plus the moving joints from /joint_states.
- /hoverboard_api and /head_mcu are also written as /dynamic_joint_states,
  ADR 0006. Everything else is copied unchanged.

The output is written next to OUTPUT as OUTPUT.partial and renamed when
complete; a failed run removes it.

Reading a WAL-mode sqlite3 bag (as the Kohlfahrt bags are) creates -wal and -shm side
files next to the input .db3, and sqlite may checkpoint the .db3 itself.
Convert from a copy if the original must stay byte-identical.
"""
import argparse
from dataclasses import dataclass
from dataclasses import field
import math
from pathlib import Path
import shutil
import sys

from builtin_interfaces.msg import Time
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from pilsbot_description import profile as robot_profile
from pilsbot_log_tools.odometry import is_plausible_steering_angle
from pilsbot_log_tools.odometry import OdometryReRooter
from pilsbot_log_tools.odometry import stamp_ns
from pilsbot_log_tools.sensors import head_mcu_to_dynamic_joint_state
from pilsbot_log_tools.sensors import hoverboard_to_dynamic_joint_state
from pilsbot_log_tools import urdf_tf

LEGACY_ODOM_FRAME = 'odom_raw'
STEERING_JOINT = 'steering_axle_joint'
REPLACED_TOPICS = ('/tf', '/tf_static', '/robot_description')

# Humble's rosbag2 QoS format: durability 1 = transient local, reliability 1
# = reliable, history 3 = unknown (as written by ros2 bag record).
_LATCHED_QOS = (
    '- history: 3\n  depth: 0\n  reliability: 1\n  durability: 1\n'
    '  deadline:\n    sec: 9223372036\n    nsec: 854775807\n'
    '  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n'
    '  liveliness: 1\n'
    '  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n'
    '  avoid_ros_namespace_conventions: false')


@dataclass
class Report:
    wheelbase: float
    odometry_re_rooted: int = 0
    odometry_dropped: int = 0
    # Odometry not in odom_raw: copied unchanged, so nothing reaches /tf.
    odometry_not_re_rooted: dict = field(default_factory=dict)
    # Re-rooted with a steering angle older than STALE_STEERING_NS (rate 0).
    stale_steering_rate: int = 0
    replaced_messages: dict = field(default_factory=dict)


def _time(ns):
    return Time(sec=ns // 10**9, nanosec=ns % 10**9)


def _open_reader(uri, storage_id):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id=storage_id),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return r


def _stamp_or(stamp, recorded_ns):
    """Header stamp, or the recording time if the publisher left it empty."""
    return stamp if stamp_ns(stamp) else _time(recorded_ns)


def resolve_input(path):
    """
    Return (uri, storage id) for a bag directory or a single .mcap file.

    The 2023 bags were converted to MCAP afterwards but kept metadata.yaml
    naming the deleted .db3, so a directory with a .mcap and no .db3 opens
    the .mcap file directly.
    """
    path = Path(path)
    if path.is_file():
        return str(path), 'mcap' if path.suffix == '.mcap' else 'sqlite3'
    mcaps = sorted(path.glob('*.mcap'))
    if mcaps and not list(path.glob('*.db3')):
        if len(mcaps) > 1:
            raise ValueError(f'{path} holds several .mcap files; pass one of them')
        return str(mcaps[0]), 'mcap'
    return str(path), 'sqlite3'


def current_urdf(profile):
    import xacro
    from ament_index_python.packages import get_package_share_directory
    path = get_package_share_directory('pilsbot_description') + '/urdf/pilsbot.urdf.xacro'
    # The same arguments as pilsbot_driver's launch: the real hardware.
    return xacro.process_file(path, mappings={
        'robot_profile': str(profile.path),
        'use_fake_hardware': 'false',
        'controller_config': 'acker_diff_controller.yaml',
    }).toxml()


def convert(input_path, output_uri, profile_name='pilsbot'):
    """
    Convert input_path into a new MCAP bag directory output_uri.

    The bag is written to output_uri.partial/ and only moved to output_uri
    when complete, so an existing output_uri is always a finished one.
    """
    output = Path(output_uri)
    if output.exists():
        raise FileExistsError(f'{output} already exists')
    partial = output.with_name(output.name + '.partial')
    if partial.exists():
        print(f'Removing {partial} left by an earlier, interrupted run', file=sys.stderr)
        shutil.rmtree(partial)
    partial.mkdir(parents=True)
    try:
        report = _convert(input_path, partial / output.name, profile_name)
        (partial / output.name).rename(output)
    finally:
        shutil.rmtree(partial, ignore_errors=True)
    return report


def _convert(input_path, output_uri, profile_name):
    profile = robot_profile.load(profile_name)
    urdf = current_urdf(profile)
    joints = urdf_tf.joints(urdf)
    moving = {j.name: j for j in joints if j.type in urdf_tf.MOVING_TYPES}
    report = Report(wheelbase=profile.wheelbase)

    input_uri, input_storage = resolve_input(input_path)
    reader = _open_reader(input_uri, input_storage)
    topics = {t.name: t for t in reader.get_all_topics_and_types()}
    start_ns = reader.get_metadata().starting_time.nanoseconds

    # Pass 1: all steering angles, so each odometry message finds the latest
    # angle at its stamp even if that arrived later in the bag.
    re_rooter = OdometryReRooter(profile.wheelbase)
    if '/joint_states' in topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=['/joint_states']))
        joint_state_type = get_message(topics['/joint_states'].type)
        while reader.has_next():
            _, data, t = reader.read_next()
            js = deserialize_message(data, joint_state_type)
            if STEERING_JOINT in js.name:
                re_rooter.add_steering_angle(
                    stamp_ns(_stamp_or(js.header.stamp, t)),
                    js.position[js.name.index(STEERING_JOINT)])
    reader = _open_reader(input_uri, input_storage)

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output_uri), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    try:
        _copy(reader, writer, topics, start_ns, urdf, joints, moving, re_rooter, report)
    finally:
        # Close before the caller renames or removes the directory, not in GC.
        writer.close()
    report.stale_steering_rate = re_rooter.stale_steering
    return report


def _copy(reader, writer, topics, start_ns, urdf, joints, moving, re_rooter, report):
    created = set()

    def write(topic, type_name, data, t, qos=''):
        if topic not in created:
            writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic, type=type_name, serialization_format='cdr',
                offered_qos_profiles=qos))
            created.add(topic)
        writer.write(topic, data, t)

    # Copied topics keep their recorded QoS, even those without messages.
    for topic in topics.values():
        if topic.name not in REPLACED_TOPICS:
            writer.create_topic(topic)
            created.add(topic.name)

    start = _time(start_ns)
    write('/robot_description', 'std_msgs/msg/String',
          serialize_message(String(data=urdf)), start_ns, _LATCHED_QOS)
    write('/tf_static', 'tf2_msgs/msg/TFMessage',
          serialize_message(TFMessage(transforms=[
              j.transform(start) for j in joints if j.type == 'fixed'])),
          start_ns, _LATCHED_QOS)

    # Copied topics are never decoded, so their types need not be installed.
    message_types = {}

    def decode(topic, data):
        if topic not in message_types:
            message_types[topic] = get_message(topics[topic].type)
        return deserialize_message(data, message_types[topic])

    while reader.has_next():
        topic, data, t = reader.read_next()
        type_name = topics[topic].type

        if topic in REPLACED_TOPICS:
            report.replaced_messages[topic] = report.replaced_messages.get(topic, 0) + 1
            continue

        if type_name == 'nav_msgs/msg/Odometry':
            odom = decode(topic, data)
            if odom.header.frame_id == LEGACY_ODOM_FRAME:
                # The Foxy controller stamped odometry with the steady clock
                # (uptime), so the recording time is the only usable stamp.
                odom.header.stamp = _time(t)
                result = re_rooter.re_root(odom)
                if result is None:
                    report.odometry_dropped += 1
                    continue
                new_odom, tf = result
                report.odometry_re_rooted += 1
                write(topic, type_name, serialize_message(new_odom), t)
                write('/tf', 'tf2_msgs/msg/TFMessage',
                      serialize_message(TFMessage(transforms=[tf])), t)
                continue
            report.odometry_not_re_rooted[topic] = (
                report.odometry_not_re_rooted.get(topic, 0) + 1)

        write(topic, type_name, data, t)

        if topic == '/joint_states':
            js = decode(topic, data)
            stamp = _stamp_or(js.header.stamp, t)
            tfs = [moving[name].transform(stamp, position)
                   for name, position in zip(js.name, js.position)
                   if name in moving and (
                       name != STEERING_JOINT and math.isfinite(position)
                       or is_plausible_steering_angle(position))]
            if tfs:
                write('/tf', 'tf2_msgs/msg/TFMessage',
                      serialize_message(TFMessage(transforms=tfs)), t)
        elif type_name == 'pilsbot_driver_msgs/msg/HoverboardAPISensors':
            write('/dynamic_joint_states', 'control_msgs/msg/DynamicJointState',
                  serialize_message(hoverboard_to_dynamic_joint_state(
                      decode(topic, data), t)), t)
        elif type_name == 'pilsbot_driver_msgs/msg/SteeringAxleSensorsStamped':
            head = decode(topic, data)
            head.stamp = _stamp_or(head.stamp, t)
            write('/dynamic_joint_states', 'control_msgs/msg/DynamicJointState',
                  serialize_message(head_mcu_to_dynamic_joint_state(head)), t)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__.strip(), formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input', help='Foxy bag directory or .mcap file')
    parser.add_argument('output', help='MCAP bag directory to create')
    parser.add_argument('--robot-profile', default='pilsbot',
                        help='profile name or path (default: pilsbot)')
    args = parser.parse_args(argv)
    report = convert(args.input, args.output, args.robot_profile)
    print(f'wheelbase {report.wheelbase} m; odometry re-rooted: {report.odometry_re_rooted}, '
          f'dropped (no steering angle yet): {report.odometry_dropped}; '
          f'stale steering angle (rate 0): {report.stale_steering_rate}; '
          f'replaced: {report.replaced_messages}')
    if report.odometry_not_re_rooted:
        warning = (f'WARNING: odometry not in {LEGACY_ODOM_FRAME} was copied unchanged: '
                   f'{report.odometry_not_re_rooted}.')
        if not report.odometry_re_rooted:
            warning += (' Nothing was re-rooted and the recorded /tf was dropped, '
                        'so the output has no odom -> base_footprint.')
        print(warning, file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
