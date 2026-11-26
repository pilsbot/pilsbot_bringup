# Same hand-derived values as test_rear_axle_centre.cpp in pils_control, so
# the Python and C++ versions of the transform cannot drift apart.
# Positive steering steers right; chassis heading = front-axle heading + delta.
# L = 0.67, cos(0.3) = 0.955336, sin(0.3) = 0.295520.
import pytest

from pilsbot_log_tools.rear_axle import front_axle_to_rear_axle_centre
from pilsbot_log_tools.rear_axle import front_axle_twist_to_rear_axle_centre
from pilsbot_log_tools.rear_axle import Pose2D

L = 0.67
approx = lambda v: pytest.approx(v, abs=1e-6)  # noqa: E731


def test_straight_ahead_sits_one_wheelbase_behind():
    rear = front_axle_to_rear_axle_centre(Pose2D(1.0, 2.0, 0.0), 0.0, L)
    assert (rear.x, rear.y, rear.heading) == (
        approx(1.0 - 0.67), approx(2.0), approx(0.0))


def test_steering_right_turns_chassis_left_of_front_axle():
    rear = front_axle_to_rear_axle_centre(Pose2D(0.0, 0.0, 0.0), 0.3, L)
    assert rear.heading == approx(0.3)
    assert rear.x == approx(-0.640075)
    assert rear.y == approx(-0.197998)


def test_rolling_without_slip_has_no_lateral_velocity():
    chassis_yaw_rate = -0.295520 / 0.67
    rear = front_axle_twist_to_rear_axle_centre(1.0, chassis_yaw_rate, 0.3, 0.0, L)
    assert rear.linear_x == approx(0.955336)
    assert rear.linear_y == approx(0.0)
    assert rear.angular == approx(chassis_yaw_rate)


def test_steering_in_place_swings_the_rear_sideways():
    rear = front_axle_twist_to_rear_axle_centre(0.0, 0.0, 0.0, 0.5, L)
    assert (rear.linear_x, rear.linear_y, rear.angular) == (
        approx(0.0), approx(-0.335), approx(0.5))
