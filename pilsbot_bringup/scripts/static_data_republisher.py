#!/usr/bin/env python3
"""
Re-publish /tf_static and /robot_description periodically while recording.

Humble's rosbag2 writes latched messages only into the first file of a
split bag. Re-publishing them every few seconds puts them into each split
file, so every file opens in Foxglove with the URDF and static TF.

Static transforms of all publishers are merged into one message, by
child frame (the latest transform wins). Our own
messages come back through the subscriptions; they are recognised by being
equal to what was last published and ignored, so they never overwrite a
newer message from the real publisher.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
# Every /tf_static publisher hands its latched message to a new subscriber.
# With depth 1 they would overwrite each other before the callback runs, so
# subscribe with a deep history, like tf2's static listener does.
LATCHED_MANY = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE)


class StaticDataRepublisher(Node):

    def __init__(self):
        super().__init__('static_data_republisher')
        period = self.declare_parameter('period', 10.0).value
        self._transforms = {}
        self._description = None
        self._published_tf = None
        self._published_description = None
        self._tf_pub = self.create_publisher(TFMessage, '/tf_static', LATCHED)
        self._description_pub = self.create_publisher(String, '/robot_description', LATCHED)
        self.create_subscription(TFMessage, '/tf_static', self._on_tf_static, LATCHED_MANY)
        self.create_subscription(
            String, '/robot_description', self._on_description, LATCHED_MANY)
        self.create_timer(period, self._republish)

    def _on_tf_static(self, msg):
        if msg == self._published_tf:
            return
        for t in msg.transforms:
            self._transforms[t.child_frame_id] = t

    def _on_description(self, msg):
        if msg != self._published_description:
            self._description = msg

    def _republish(self):
        if self._transforms:
            self._published_tf = TFMessage(transforms=list(self._transforms.values()))
            self._tf_pub.publish(self._published_tf)
        if self._description is not None:
            self._published_description = self._description
            self._description_pub.publish(self._description)


def main():
    rclpy.init()
    node = StaticDataRepublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
