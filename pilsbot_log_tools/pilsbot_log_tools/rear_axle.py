"""
Front axle -> rear axle centre, the Python twin of acker_diff_core.cpp.

Positive steering angle steers right: the front axle is yawed -delta
relative to the chassis, so chassis heading = front-axle heading + delta.
See docs/adr/0002 in the workspace repository.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    heading: float


@dataclass(frozen=True)
class PlanarTwist:
    linear_x: float
    linear_y: float
    angular: float


def front_axle_to_rear_axle_centre(front_axle, steering_angle, wheelbase):
    heading = front_axle.heading + steering_angle
    return Pose2D(front_axle.x - wheelbase * math.cos(heading),
                  front_axle.y - wheelbase * math.sin(heading),
                  heading)


def front_axle_twist_to_rear_axle_centre(front_linear, front_angular,
                                         steering_angle, steering_rate, wheelbase):
    """
    Rigid-body transfer of the front-axle twist to the rear axle centre.

    front_linear is along the front axle's heading; the result is expressed
    in the chassis frame.
    """
    angular = front_angular + steering_rate
    return PlanarTwist(front_linear * math.cos(steering_angle),
                       -front_linear * math.sin(steering_angle) - angular * wheelbase,
                       angular)
