# Whole conversion: a Foxy-style sqlite3 bag in, an MCAP bag out.
# The input bag is written here with Foxy's topics and frames; that Humble
# reads real Foxy metadata (version 4) is checked separately on real bags.
import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from pilsbot_description import profile as robot_profile
from pilsbot_driver_msgs.msg import HoverboardAPISensors
from pilsbot_driver_msgs.msg import SteeringAxleSensorsStamped
import pytest
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import JointState
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import yaml

from pilsbot_log_tools import convert_legacy_bag
from pilsbot_log_tools.convert_legacy_bag import convert
from pilsbot_log_tools.convert_legacy_bag import main
from pilsbot_log_tools.convert_legacy_bag import resolve_input

approx = lambda v: pytest.approx(v, abs=1e-6)  # noqa: E731
T = 1_000_000_000  # 1.0 s in ns
STAMP = Time(sec=1)
UPTIME = Time(sec=4711)


def write_foxy_bag(uri, odom_frame='odom_raw'):
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))

    def put(topic, msg, t=T):
        package = type(msg).__module__.split('.')[0]
        w.create_topic(rosbag2_py.TopicMetadata(
            name=topic, type=f'{package}/msg/{type(msg).__name__}', serialization_format='cdr'))
        w.write(topic, serialize_message(msg), t)

    put('/robot_description', String(data='<robot name="old"><link name="odom_raw"/></robot>'))
    old_tf = TransformStamped()
    old_tf.header.stamp, old_tf.header.frame_id = STAMP, 'odom_raw'
    old_tf.child_frame_id = 'base_link'
    put('/tf', TFMessage(transforms=[old_tf]))
    put('/tf_static', TFMessage(transforms=[old_tf]))

    js = JointState(name=['front_wheel_left_joint', 'steering_axle_joint'], position=[1.0, 0.2])
    js.header.stamp = STAMP
    put('/joint_states', js)

    odom = Odometry()
    # The Foxy controller stamped odometry with the steady clock (uptime).
    odom.header.stamp, odom.header.frame_id, odom.child_frame_id = UPTIME, odom_frame, 'base_link'
    odom.pose.pose.orientation.w = 1.0
    put('/odom', odom)

    put('/hoverboard_api', HoverboardAPISensors(voltage=38.5))
    head = SteeringAxleSensorsStamped(stamp=STAMP)
    head.sensors.endstop_l = True
    put('/head_mcu', head)
    put('/fix', NavSatFix(latitude=53.1, longitude=8.8))
    w.close()


def read_bag(uri, storage_id):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id=storage_id),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    metadata = {t.name: t for t in r.get_all_topics_and_types()}
    messages = []
    while r.has_next():
        topic, data, t = r.read_next()
        messages.append((topic, data, t))
    return metadata, messages


@pytest.fixture(scope='module')
def converted(tmp_path_factory):
    d = tmp_path_factory.mktemp('bags')
    write_foxy_bag(d / 'foxy')
    report = convert(d / 'foxy', d / 'humble', 'pilsbot')
    metadata, messages = read_bag(d / 'humble', 'mcap')

    def decoded(topic):
        return [deserialize_message(data, get_message(metadata[topic].type))
                for name, data, _ in messages if name == topic]
    return d, report, metadata, messages, decoded


def is_transient_local(topic):
    # Humble writes QoS enums as numbers; durability 1 is transient local.
    [qos] = yaml.safe_load(topic.offered_qos_profiles)
    return qos['durability'] == 1


def transforms(tf_messages):
    return {(t.header.frame_id, t.child_frame_id): t
            for m in tf_messages for t in m.transforms}


def test_output_is_mcap(converted):
    d = converted[0]
    assert 'storage_identifier: mcap' in (d / 'humble' / 'metadata.yaml').read_text()


def test_odometry_is_re_rooted_at_the_rear_axle_centre(converted):
    _, _, _, _, decoded = converted
    [odom] = decoded('/odom')
    assert (odom.header.frame_id, odom.child_frame_id) == ('odom', 'base_footprint')
    assert odom.pose.pose.position.x == approx(-0.656645)


def test_legacy_odometry_gets_the_recording_time(converted):
    _, _, _, _, decoded = converted
    [odom] = decoded('/odom')
    assert odom.header.stamp == STAMP


def test_tf_holds_base_footprint_and_moving_joints_from_the_current_urdf(converted):
    _, _, _, _, decoded = converted
    tf = transforms(decoded('/tf'))
    assert set(tf) == {('odom', 'base_footprint'),
                       ('steering_axle', 'front_axle'),
                       ('front_axle', 'front_wheel_left')}
    assert tf['odom', 'base_footprint'].transform.translation.y == approx(-0.133108)
    # steering_axle_joint turns about -z: 0.2 rad -> quaternion z = -sin(0.1).
    steer = tf['steering_axle', 'front_axle'].transform
    assert (steer.translation.z, steer.rotation.z, steer.rotation.w) == (
        approx(-0.05), approx(-0.0998334), approx(0.9950042))
    # Wheel turns about +y: 1.0 rad -> quaternion y = sin(0.5).
    wheel = tf['front_axle', 'front_wheel_left'].transform
    assert (wheel.translation.y, wheel.rotation.y) == (approx(0.31), approx(0.4794255))
    assert all(t.header.stamp == STAMP for t in tf.values())


def test_tf_static_holds_the_fixed_joints_of_the_current_urdf(converted):
    _, _, metadata, _, decoded = converted
    [static] = decoded('/tf_static')
    tf = transforms([static])
    assert ('odom_raw', 'base_link') not in tf
    assert tf['base_footprint', 'base_link'].transform.translation.z == approx(0.125)
    oak = tf['oak_rgb_camera_frame', 'oak_rgb_camera_optical_frame'].transform.rotation
    # rpy (-pi/2, 0, -pi/2) -> quaternion (-0.5, 0.5, -0.5, 0.5)
    assert (oak.x, oak.y, oak.z, oak.w) == (approx(-0.5), approx(0.5), approx(-0.5), approx(0.5))
    assert is_transient_local(metadata['/tf_static'])


def test_robot_description_is_the_current_urdf(converted):
    _, _, metadata, _, decoded = converted
    [description] = decoded('/robot_description')
    assert '<link name="base_footprint"' in description.data
    assert 'odom_raw' not in description.data
    assert is_transient_local(metadata['/robot_description'])


def test_robot_description_is_the_real_robot_not_the_simulation(converted):
    # As pilsbot_driver runs xacro: hardware:=real.
    _, _, _, _, decoded = converted
    [description] = decoded('/robot_description')
    assert 'GazeboSystem' not in description.data


def test_current_urdf_asks_xacro_for_the_real_hardware(monkeypatch):
    # The same xacro arguments as pilsbot_driver's launch with hardware:=real.
    import xacro
    seen = {}

    class Document:
        def toxml(self):
            return '<robot/>'

    def process_file(path, mappings):
        seen.update(mappings)
        return Document()

    monkeypatch.setattr(xacro, 'process_file', process_file)
    convert_legacy_bag.current_urdf(robot_profile.load('pilsbot'))
    assert seen['hardware'] == 'real'
    # Transition guard until pilsbot_description#6: the installed description
    # still reads use_fake_hardware (default true: Gazebo).
    assert seen['use_fake_hardware'] == 'false'


def test_sensors_are_added_as_dynamic_joint_states_and_kept(converted):
    _, _, _, _, decoded = converted
    sensors = {m.joint_names[0]: m for m in decoded('/dynamic_joint_states')}
    assert set(sensors) == {'hoverboard_api', 'head_mcu'}
    assert sensors['hoverboard_api'].interface_values[0].values[0] == 38.5
    assert len(decoded('/hoverboard_api')) == 1
    assert len(decoded('/head_mcu')) == 1


def test_other_topics_are_copied_unchanged(converted):
    d, _, _, messages, _ = converted
    _, original = read_bag(d / 'foxy', 'sqlite3')
    for topic in ('/fix', '/joint_states'):
        assert [(t, bytes(b)) for n, b, t in original if n == topic] == \
               [(t, bytes(b)) for n, b, t in messages if n == topic]


def test_report_counts_what_happened(converted):
    report = converted[1]
    assert report.odometry_re_rooted == 1
    assert report.odometry_dropped == 0
    assert report.odometry_not_re_rooted == {}
    assert report.stale_steering_rate == 0
    assert report.replaced_messages == {'/tf': 1, '/tf_static': 1, '/robot_description': 1}
    assert math.isclose(report.wheelbase, 0.67)


def test_bag_directory_opens_as_recorded(tmp_path):
    (tmp_path / 'metadata.yaml').write_text('storage_identifier: sqlite3\n')
    (tmp_path / 'bag_0.db3').write_bytes(b'')
    assert resolve_input(tmp_path) == (str(tmp_path), 'sqlite3')


def test_directory_converted_to_mcap_with_stale_metadata_opens_the_mcap_file(tmp_path):
    # The 2023 bags: metadata says sqlite3, but only bag_0.mcap is left.
    (tmp_path / 'metadata.yaml').write_text('storage_identifier: sqlite3\n')
    (tmp_path / 'bag_0.db3-wal').write_bytes(b'')
    (tmp_path / 'bag_0.mcap').write_bytes(b'')
    assert resolve_input(tmp_path) == (str(tmp_path / 'bag_0.mcap'), 'mcap')


def test_mcap_file_opens_directly(tmp_path):
    (tmp_path / 'bag_0.mcap').write_bytes(b'')
    assert resolve_input(tmp_path / 'bag_0.mcap') == (str(tmp_path / 'bag_0.mcap'), 'mcap')


def test_failed_conversion_leaves_no_output_and_can_be_rerun(tmp_path, monkeypatch):
    write_foxy_bag(tmp_path / 'foxy')

    def broken(*_):
        raise RuntimeError('boom')
    monkeypatch.setattr(convert_legacy_bag, 'head_mcu_to_dynamic_joint_state', broken)
    with pytest.raises(RuntimeError, match='boom'):
        convert(tmp_path / 'foxy', tmp_path / 'humble', 'pilsbot')
    assert sorted(p.name for p in tmp_path.iterdir()) == ['foxy']

    monkeypatch.undo()
    convert(tmp_path / 'foxy', tmp_path / 'humble', 'pilsbot')
    assert sorted(p.name for p in tmp_path.iterdir()) == ['foxy', 'humble']
    assert sorted(p.name for p in (tmp_path / 'humble').iterdir()) == [
        'humble_0.mcap', 'metadata.yaml']


def test_missing_parent_directories_of_the_output_are_created(tmp_path):
    write_foxy_bag(tmp_path / 'foxy')
    output = tmp_path / 'not' / 'yet' / 'humble'
    convert(tmp_path / 'foxy', output, 'pilsbot')
    assert sorted(p.name for p in output.parent.iterdir()) == ['humble']
    assert sorted(p.name for p in output.iterdir()) == ['humble_0.mcap', 'metadata.yaml']


def test_existing_output_is_not_touched(tmp_path):
    write_foxy_bag(tmp_path / 'foxy')
    (tmp_path / 'humble').mkdir()
    (tmp_path / 'humble' / 'keep').write_text('x')
    with pytest.raises(FileExistsError):
        convert(tmp_path / 'foxy', tmp_path / 'humble', 'pilsbot')
    assert [p.name for p in (tmp_path / 'humble').iterdir()] == ['keep']


def test_odometry_in_another_frame_is_copied_and_reported(tmp_path, capsys):
    write_foxy_bag(tmp_path / 'foxy', odom_frame='odom')
    assert main([str(tmp_path / 'foxy'), str(tmp_path / 'humble')]) == 0
    assert 'WARNING' in capsys.readouterr().err
    report = convert(tmp_path / 'foxy', tmp_path / 'humble2', 'pilsbot')
    assert report.odometry_re_rooted == 0
    assert report.odometry_not_re_rooted == {'/odom': 1}


def test_help_warns_about_wal_side_files(capsys):
    with pytest.raises(SystemExit):
        main(['--help'])
    out = capsys.readouterr().out
    assert '-wal' in out and '-shm' in out
