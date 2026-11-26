"""
The few URDF kinematics the converter needs, like robot_state_publisher.

Fixed joints become /tf_static; revolute and continuous joints become /tf
from their positions in /joint_states. Other joint types don't occur in the
Pilsbot URDF and are ignored.
"""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

from geometry_msgs.msg import TransformStamped

MOVING_TYPES = ('revolute', 'continuous')


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quaternion_from_axis_angle(axis, angle):
    n = math.sqrt(sum(a * a for a in axis))
    s = math.sin(angle / 2) / n
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _floats(text, default):
    return tuple(float(v) for v in text.split()) if text else default


@dataclass(frozen=True)
class Joint:
    name: str
    type: str
    parent: str
    child: str
    xyz: tuple
    rotation: tuple  # quaternion x, y, z, w of the joint origin
    axis: tuple

    def transform(self, stamp, position=0.0):
        rotation = self.rotation
        if self.type in MOVING_TYPES:
            rotation = quaternion_multiply(
                rotation, quaternion_from_axis_angle(self.axis, position))
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.parent
        t.child_frame_id = self.child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = self.xyz
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = rotation
        return t


def joints(urdf_xml):
    result = []
    for j in ET.fromstring(urdf_xml).findall('joint'):
        origin = j.find('origin')
        axis = j.find('axis')
        result.append(Joint(
            name=j.get('name'),
            type=j.get('type'),
            parent=j.find('parent').get('link'),
            child=j.find('child').get('link'),
            xyz=_floats(origin.get('xyz') if origin is not None else None, (0.0, 0.0, 0.0)),
            rotation=quaternion_from_rpy(
                *_floats(origin.get('rpy') if origin is not None else None, (0.0, 0.0, 0.0))),
            axis=_floats(axis.get('xyz') if axis is not None else None, (1.0, 0.0, 0.0)),
        ))
    return result
