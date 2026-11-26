# Re-rooting a recorded front-axle odometry stream at the rear axle centre.
# Foxy recorded odom_raw -> base_link with base_link at the front axle; the
# steering angle comes from /joint_states. Hand values as in test_rear_axle.py,
# plus cos(0.2) = 0.980067, sin(0.2) = 0.198669.
import math

from nav_msgs.msg import Odometry
import pytest

from pilsbot_log_tools.odometry import OdometryReRooter

L = 0.67
approx = lambda v: pytest.approx(v, abs=1e-6)  # noqa: E731


def ns(seconds):
    return int(round(seconds * 1e9))


def front_odometry(t, x=0.0, y=0.0, heading=0.0, linear=0.0, angular=0.0):
    m = Odometry()
    m.header.frame_id = 'odom_raw'
    m.child_frame_id = 'base_link'
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = ns(t) - int(t) * 10**9
    m.pose.pose.position.x = x
    m.pose.pose.position.y = y
    m.pose.pose.orientation.z = math.sin(heading / 2)
    m.pose.pose.orientation.w = math.cos(heading / 2)
    m.twist.twist.linear.x = linear
    m.twist.twist.angular.z = angular
    return m


def re_rooter_with(*angles):
    r = OdometryReRooter(L)
    for t, angle in angles:
        r.add_steering_angle(ns(t), angle)
    return r


def test_odometry_before_any_steering_angle_is_dropped():
    assert OdometryReRooter(L).re_root(front_odometry(1.0)) is None


def test_standing_robot_puts_base_footprint_behind_the_steered_front():
    odom, tf = re_rooter_with((1.0, 0.2)).re_root(front_odometry(1.0))

    assert (odom.header.frame_id, odom.child_frame_id) == ('odom', 'base_footprint')
    assert odom.header.stamp == front_odometry(1.0).header.stamp
    p = odom.pose.pose
    assert (p.position.x, p.position.y) == (approx(-0.656645), approx(-0.133108))
    assert (p.orientation.z, p.orientation.w) == (approx(0.0998334), approx(0.9950042))

    assert (tf.header.frame_id, tf.child_frame_id) == ('odom', 'base_footprint')
    assert tf.header.stamp == odom.header.stamp
    assert tf.transform.translation.x == approx(-0.656645)
    assert tf.transform.translation.y == approx(-0.133108)
    assert tf.transform.rotation.z == approx(0.0998334)


def test_uses_the_latest_steering_angle_at_the_odometry_stamp():
    # A later angle may already be in the bag (different receive order).
    r = re_rooter_with((1.0, 0.2), (1.5, 0.3))
    odom, _ = r.re_root(front_odometry(1.2))
    assert odom.pose.pose.position.x == approx(-0.656645)


def test_moving_robot_without_slip_has_no_lateral_velocity():
    chassis_yaw_rate = -0.295520 / 0.67
    odom, _ = re_rooter_with((1.0, 0.3)).re_root(
        front_odometry(1.0, linear=1.0, angular=chassis_yaw_rate))
    t = odom.twist.twist
    assert (t.linear.x, t.linear.y, t.angular.z) == (
        approx(0.955336), approx(0.0), approx(chassis_yaw_rate))


def test_first_steering_angle_has_no_rate():
    odom, _ = re_rooter_with((1.0, 0.2)).re_root(front_odometry(1.0))
    assert odom.twist.twist.angular.z == approx(0.0)


def test_steering_rate_comes_from_consecutive_angles():
    odom, _ = re_rooter_with((1.0, 0.2), (1.02, 0.21)).re_root(front_odometry(1.02))
    t = odom.twist.twist
    assert (t.angular.z, t.linear.y) == (approx(0.5), approx(-0.335))


def test_invalid_steering_angle_is_skipped():
    r = re_rooter_with((1.0, 0.2), (1.02, float('nan')), (1.04, 0.21))
    odom, _ = r.re_root(front_odometry(1.04))
    assert odom.twist.twist.angular.z == approx(0.25)


def test_front_axle_heading_is_carried_into_the_chassis_heading():
    # Front axle at (1, 2) heading 0.1, steering 0.2 -> chassis heading 0.3.
    odom, _ = re_rooter_with((1.0, 0.2)).re_root(front_odometry(1.0, 1.0, 2.0, 0.1))
    p = odom.pose.pose
    assert (p.position.x, p.position.y) == (approx(1.0 - 0.640075), approx(2.0 - 0.197998))
    assert 2 * math.atan2(p.orientation.z, p.orientation.w) == approx(0.3)
