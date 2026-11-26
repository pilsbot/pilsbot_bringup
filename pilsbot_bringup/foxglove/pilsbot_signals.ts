// Foxglove user script (embedded in pilsbot.json; edit here, then paste
// into the layout). Looks values up by name, because the order of joints
// and interfaces in /joint_states and /dynamic_joint_states is not fixed,
// and converted legacy bags put each sensor in its own message.
import { Input } from "./types";

export const inputs = ["/joint_states", "/dynamic_joint_states"];
export const output = "/studio_script/pilsbot_signals";

type Signals = {
  steering_angle: number;
  voltage: number;
  current_motor_0: number;
  current_motor_1: number;
  steering_angle_raw: number;
  endstop_left: number;
  endstop_right: number;
};

// Latest value of every signal, so each output message is complete.
const latest: Signals = {
  steering_angle: NaN,
  voltage: NaN,
  current_motor_0: NaN,
  current_motor_1: NaN,
  steering_angle_raw: NaN,
  endstop_left: NaN,
  endstop_right: NaN,
};

const interfaces: Record<string, Record<string, keyof Signals>> = {
  hoverboard_api: {
    "voltage": "voltage",
    "avg_amperage_motor.0": "current_motor_0",
    "avg_amperage_motor.1": "current_motor_1",
  },
  head_mcu: {
    "steering_angle_raw": "steering_angle_raw",
    "endstop_l": "endstop_left",
    "endstop_r": "endstop_right",
  },
};

export default function script(
  event: Input<"/joint_states"> | Input<"/dynamic_joint_states">,
): Signals {
  if (event.topic === "/joint_states") {
    const i = event.message.name.indexOf("steering_axle_joint");
    if (i >= 0) {
      latest.steering_angle = event.message.position[i];
    }
  } else {
    event.message.joint_names.forEach((joint, j) => {
      const wanted = interfaces[joint];
      const values = event.message.interface_values[j];
      if (!wanted || !values) {
        return;
      }
      values.interface_names.forEach((name, k) => {
        const signal = wanted[name];
        if (signal) {
          latest[signal] = values.values[k];
        }
      });
    });
  }
  return { ...latest };
}
