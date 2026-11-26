# Foxy sensor topics -> /dynamic_joint_states with the names the Humble
# PilsbotDriver exports (pilsbot_driver.cpp, export_state_interfaces).
from builtin_interfaces.msg import Time
from pilsbot_driver_msgs.msg import HoverboardAPISensors
from pilsbot_driver_msgs.msg import SteeringAxleSensorsStamped

from pilsbot_log_tools.sensors import head_mcu_to_dynamic_joint_state
from pilsbot_log_tools.sensors import hoverboard_to_dynamic_joint_state


def as_dict(dynamic):
    return {name: dict(zip(v.interface_names, v.values))
            for name, v in zip(dynamic.joint_names, dynamic.interface_values)}


def test_hoverboard_reading_becomes_the_hoverboard_api_sensor():
    m = HoverboardAPISensors(voltage=38.5, avg_amperate_motor_0=1.25,
                             avg_amperate_motor_1=-0.5, tx_bufferlevel=3.0)
    out = hoverboard_to_dynamic_joint_state(m, 1_500_000_000)
    assert as_dict(out) == {'hoverboard_api': {
        'voltage': 38.5, 'avg_amperage_motor.0': 1.25,
        'avg_amperage_motor.1': -0.5, 'tx_bufferlevel': 3.0}}


def test_hoverboard_reading_is_stamped_with_the_recording_time():
    # HoverboardAPISensors has no header.
    out = hoverboard_to_dynamic_joint_state(HoverboardAPISensors(), 1_500_000_000)
    assert out.header.stamp == Time(sec=1, nanosec=500_000_000)


def test_head_mcu_reading_becomes_the_head_mcu_sensor_with_endstops_as_numbers():
    m = SteeringAxleSensorsStamped()
    m.stamp = Time(sec=7, nanosec=20)
    m.sensors.steering_angle_raw = 32358.0
    m.sensors.endstop_l = True
    m.sensors.endstop_r = False
    m.sensors.steering_angle_normalized = 0.0
    out = head_mcu_to_dynamic_joint_state(m)
    assert out.header.stamp == Time(sec=7, nanosec=20)
    assert as_dict(out) == {'head_mcu': {
        'steering_angle_raw': 32358.0, 'endstop_l': 1.0, 'endstop_r': 0.0}}
