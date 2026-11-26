"""
Re-root recorded front-axle odometry at the rear axle centre (ADR 0002).

Foxy recorded odom_raw -> base_link with base_link at the front axle. The
steering angle needed for the transform comes from /joint_states. Each
odometry message uses the latest valid steering angle at its stamp; the
steering rate comes from that angle and the valid one before it, as in the
Humble controller.
"""
import bisect
import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from pilsbot_log_tools.rear_axle import front_axle_to_rear_axle_centre
from pilsbot_log_tools.rear_axle import front_axle_twist_to_rear_axle_centre
from pilsbot_log_tools.rear_axle import Pose2D

ODOM_FRAME = 'odom'
BASE_FRAME = 'base_footprint'

# Steering angles older than this (relative to the newest odometry) are
# dropped so a long bag doesn't keep its whole history in memory.
_HISTORY_NS = 10 * 10**9


def is_plausible_steering_angle(angle):
    """Reject NaN and garbage such as the 9.7e189 in 2023 bags."""
    return math.isfinite(angle) and abs(angle) <= math.pi


def stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdometryReRooter:

    def __init__(self, wheelbase):
        self._wheelbase = wheelbase
        self._stamps = []
        self._angles = []

    def add_steering_angle(self, stamp, angle):
        """Record a steering angle measured at stamp (ns); implausible ones are ignored."""
        if not is_plausible_steering_angle(angle):
            return
        i = bisect.bisect_right(self._stamps, stamp)
        self._stamps.insert(i, stamp)
        self._angles.insert(i, angle)

    def re_root(self, front):
        """
        Return (Odometry, TransformStamped) for the rear axle centre.

        Returns None while no steering angle at or before the stamp is known.
        """
        t = stamp_ns(front.header.stamp)
        k = bisect.bisect_right(self._stamps, t) - 1
        if k < 0:
            return None
        angle = self._angles[k]
        rate = 0.0
        if k > 0:
            rate = (angle - self._angles[k - 1]) / ((self._stamps[k] - self._stamps[k - 1]) * 1e-9)
        self._forget_before(t - _HISTORY_NS)

        p = front.pose.pose
        pose = front_axle_to_rear_axle_centre(
            Pose2D(p.position.x, p.position.y, _yaw(p.orientation)), angle, self._wheelbase)
        twist = front_axle_twist_to_rear_axle_centre(
            front.twist.twist.linear.x, front.twist.twist.angular.z, angle, rate,
            self._wheelbase)

        odom = Odometry()
        odom.header.stamp = front.header.stamp
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.position.z = p.position.z
        odom.pose.pose.orientation.z = math.sin(pose.heading / 2)
        odom.pose.pose.orientation.w = math.cos(pose.heading / 2)
        odom.pose.covariance = front.pose.covariance
        odom.twist.twist.linear.x = twist.linear_x
        odom.twist.twist.linear.y = twist.linear_y
        odom.twist.twist.angular.z = twist.angular
        odom.twist.covariance = front.twist.covariance

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = BASE_FRAME
        tf.transform.translation.x = pose.x
        tf.transform.translation.y = pose.y
        tf.transform.translation.z = p.position.z
        tf.transform.rotation = odom.pose.pose.orientation
        return odom, tf

    def _forget_before(self, stamp):
        # Keep one older angle so the rate at the boundary stays defined.
        cut = bisect.bisect_left(self._stamps, stamp) - 1
        if cut > 0:
            del self._stamps[:cut]
            del self._angles[:cut]
