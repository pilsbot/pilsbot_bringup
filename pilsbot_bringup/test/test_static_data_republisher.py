"""Run static_data_republisher against several /tf_static publishers."""
from geometry_msgs.msg import TransformStamped
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from tf2_msgs.msg import TFMessage

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'static_data_republisher.py'
CHILDREN = {f'c{i}' for i in range(1, 7)}
STATIC = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE)


def load_script():
    spec = importlib.util.spec_from_file_location('static_data_republisher', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def start(cmd):
    return subprocess.Popen(cmd, start_new_session=True,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop(processes):
    for p in processes:
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    for p in processes:
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait()


def spin_until(node, condition, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not condition():
        rclpy.spin_once(node, timeout_sec=0.1)
    return condition()


def test_latched_messages_that_arrive_together_are_all_merged():
    """
    Deliver all latched /tf_static messages before the first callback runs.

    That is what happens when the publishers are up before the republisher:
    their messages arrive at once, while the executor does not spin yet.
    """
    republisher_module = load_script()
    rclpy.init()
    try:
        node = rclpy.create_node('static_publishers')
        publishers = [node.create_publisher(TFMessage, '/tf_static', republisher_module.LATCHED)
                      for _ in CHILDREN]
        for publisher, child in zip(publishers, sorted(CHILDREN)):
            transform = TransformStamped()
            transform.header.frame_id = 'base'
            transform.child_frame_id = child
            publisher.publish(TFMessage(transforms=[transform]))
        republisher = republisher_module.StaticDataRepublisher()
        published = []
        republisher._tf_pub.publish = published.append
        time.sleep(2)  # matching and delivery, without taking the messages
        for _ in range(20):
            rclpy.spin_once(republisher, timeout_sec=0.05)
        republisher._republish()
        [msg] = published
        assert {t.child_frame_id for t in msg.transforms} == CHILDREN
        assert len(msg.transforms) == len(CHILDREN)
    finally:
        rclpy.shutdown()


def test_merged_message_keeps_the_frames_of_all_publishers():
    processes = [start(['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                        '--frame-id', 'base', '--child-frame-id', child])
                 for child in sorted(CHILDREN)]
    rclpy.init()
    node = rclpy.create_node('test_static_data_republisher')
    originals = set()
    merged = []

    def on_tf_static(msg):
        # Each publisher's latched sample arrives once; anything after the
        # republisher started comes from it.
        children = {t.child_frame_id for t in msg.transforms}
        if republisher_started:
            merged.append(children)
        else:
            originals.update(children)

    republisher_started = False
    node.create_subscription(TFMessage, '/tf_static', on_tf_static, STATIC)
    try:
        # The publishers must all be up before the republisher starts.
        assert spin_until(node, lambda: originals == CHILDREN, 30), originals
        processes.append(start([sys.executable, str(SCRIPT),
                                '--ros-args', '-p', 'period:=0.5']))
        republisher_started = True
        # The first messages may miss publishers it has not discovered yet.
        assert spin_until(node, lambda: merged and merged[-1] == CHILDREN, 30), merged
        complete = len(merged)
        assert spin_until(node, lambda: len(merged) >= complete + 3, 30)
        assert all(children == CHILDREN for children in merged[complete:]), merged
    finally:
        node.destroy_node()
        rclpy.shutdown()
        stop(processes)
