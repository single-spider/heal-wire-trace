# Technical notes

The core scripts are the team's original `teleop_v4.py` and final `heal_smooth_replay.py`. The included route was copied from `heal_waypoints_latest_clean.csv`; timestamps were removed, but waypoint IDs, angles, and row order were preserved. This is a curated route, not an automated reconstruction from the dense teaching log. The course starter bundle is not used by these scripts and is not redistributed.

Replay expects fresh feedback for `joint1` through `joint6` in `/joint_states`, the HEAL URDF on the `robot_description` parameter of `/robot_state_publisher`, and a root-namespace `velocity_controller` subscribing to `/velocity_controller/commands`. The command vector is ordered joint1 through joint6. Keep host clocks synchronized. Teleop uses the transient-local `/robot_description` topic.

## Legacy velocity fault

During earlier stop-and-go replay, the lab diagnosis traced a joint-5 velocity fault to scientific-notation handling in the controller: a command serialized as `-6e-05` was interpreted as `-6` rad/s. Smooth replay applies a `1e-4` rad/s deadband to avoid this case. This is an application-side workaround, not a controller patch. Do not assume it is present in teleop.

## Evidence limits

The archived forward and reverse logs ended in `completed` and the lab report describes both runs as fault-free. Logged elapsed time starts at the program's logging epoch, so it can include preflight and operator confirmation. The reported 3:43 turbo run is listed separately: no corresponding log or run settings were supplied. Neither software completion nor lack of a server fault measures wire contact.
