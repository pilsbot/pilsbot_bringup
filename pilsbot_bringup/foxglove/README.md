# Foxglove layouts

`pilsbot.json` is the layout for live driving (via `foxglove_bridge`, started by
`pilsbot.launch.py`, port 8765) and for MCAP bags, including converted legacy bags.
Import it in Foxglove with *Layouts → Import from file*.

The bridge is read-only by default (`foxglove_capabilities:=connectionGraph,assets`):
Foxglove can show everything but cannot publish, call services or change
parameters, because the bridge has no authentication. For development on a
closed course, opt in, e.g. `foxglove_capabilities:=all` or
`foxglove_capabilities:=connectionGraph,assets,parameters,parametersSubscribe`.
Never do that at public outings: any client could drive the robot past the
gamepad dead-man.

- 3D: URDF from `/robot_description`, TF, and `/odom`, following `base_footprint`.
- Image: OAK colour JPEG (`/camera/color/image_raw/compressed`).
- Map: GNSS `/fix`.
- Plots read `/studio_script/pilsbot_signals`, made by the user script
  `pilsbot_signals` (source: `pilsbot_signals.ts`). It looks values up by
  name because the joint and interface order in `/joint_states` and
  `/dynamic_joint_states` is not fixed.

After changing the layout in Foxglove, export it over `pilsbot.json`. If you
edit the user script there, copy it back to `pilsbot_signals.ts`.
