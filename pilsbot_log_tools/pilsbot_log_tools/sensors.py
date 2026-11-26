"""
Foxy sensor topics -> /dynamic_joint_states (ADR 0006).

Interface names match what the Humble PilsbotDriver exports, so converted
bags and new recordings look the same to joint_state_broadcaster consumers.
The original /hoverboard_api and /head_mcu topics are kept as well.
"""
from builtin_interfaces.msg import Time
from control_msgs.msg import DynamicJointState
from control_msgs.msg import InterfaceValue


def _single(name, stamp, values):
    out = DynamicJointState()
    out.header.stamp = stamp
    out.joint_names = [name]
    out.interface_values = [InterfaceValue(interface_names=list(values),
                                           values=[float(v) for v in values.values()])]
    return out


def hoverboard_to_dynamic_joint_state(msg, recorded_ns):
    """Map a hoverboard reading; it has no header, so it gets the recording time."""
    stamp = Time(sec=recorded_ns // 10**9, nanosec=recorded_ns % 10**9)
    return _single('hoverboard_api', stamp, {
        'voltage': msg.voltage,
        'avg_amperage_motor.0': msg.avg_amperate_motor_0,
        'avg_amperage_motor.1': msg.avg_amperate_motor_1,
        'tx_bufferlevel': msg.tx_bufferlevel,
    })


def head_mcu_to_dynamic_joint_state(msg):
    """Map a Head MCU reading; steering_angle_normalized is already in /joint_states."""
    s = msg.sensors
    return _single('head_mcu', msg.stamp, {
        'steering_angle_raw': s.steering_angle_raw,
        'endstop_l': s.endstop_l,
        'endstop_r': s.endstop_r,
    })
