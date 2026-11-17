# HEAL Wire Trace

**Introduction to Robotics · Project 3 — The Wire-Trace Trial**

We taught an Addverb HEAL arm to guide a probe along a vertical buzz-wire course. An operator steered the arm with a keyboard and captured six-joint poses. The robot then replayed the taught path autonomously using smooth joint-space interpolation and closed-loop velocity control.

**Demo video:** Pending — add the unlisted video link here before submission.

## Method

1. **Teach.** `teleop_v4.py` uses the robot URDF and KDL Jacobian to convert Cartesian keyboard motions to joint velocities with damped least-squares inverse kinematics (DLS-IK). A separate joint-6 jog lets the operator reorient the probe around turns. The script saves selected waypoints, optional 10 Hz teaching samples, and a command log.
2. **Process.** The replay program splits the curated route at sharp joint-space turns and wrist-dominated moves. It fits shape-preserving cubic (PCHIP) curves across waypoint stretches and uses quintic timing to start and end each chunk at rest.
3. **Replay.** At a nominal 100 Hz, the controller combines joint-velocity feedforward with measured-position feedback. It checks joint limits, feedback freshness, tracking error, and lack of measured motion. A small-command deadband works around a legacy controller parsing issue; see the [technical notes](docs/technical-notes.md).

**How this differs from the [assignment brief](https://debojit.notion.site/Task-3-The-Wire-Trace-Trial-28ff4aa874c4808dae54eb198ae7f410):** Teaching used powered keyboard teleoperation rather than hand guidance in gravity-compensation mode. DLS-IK runs during teaching, not replay: the saved route is already in joint coordinates. This implementation does not include Stage 2 learning, Cartesian boundary enforcement, or automatic contact detection.

## Results

| Measurement | Elapsed time | Evidence |
| --- | ---: | --- |
| Manual teaching | ~14 min | Team-reported |
| Latest turbo replay | 3 min 43 s | Team-reported; corresponding log not supplied |
| Archived smooth forward run (1 → 182) | 210.28 s | Replay log reports completion |
| Archived smooth return run (182 → 1) | 209.19 s | Replay log reports completion |

The reported turbo replay took **about 73% less time** than teaching (a **3.8×** teaching-to-replay time ratio). This compares two phases, not two replay configurations. Software completion does not prove zero contact. The available records do not establish three independently verified contact-free trials.

## Setup

- **Offline:** Python 3.10+ and the packages in `requirements.txt`. Preview and tests do not use ROS or hardware.
- **On the lab arm:** Use the lab-approved Ubuntu/ROS 2 installation and source the matching Addverb `addverb_cobot_control` overlay. The HEAL server, ROS drivers, and URDF are external lab dependencies and are not bundled.
- **Live Python dependencies:** `rclpy`, `sensor_msgs`, `std_msgs`, `rcl_interfaces`, and `controller_manager_msgs`. Teaching also needs `pygame`, `PyKDL`, `kdl_parser_py`, and `urdf_parser_py`. Use the interpreter from the sourced ROS setup; install the Python-only dependencies without replacing the lab's ROS packages.

### Offline preview and tests

From the repository root:

```sh
python -m pip install -r requirements.txt
python heal_smooth_replay.py data/heal_waypoints_182.csv
python -m unittest discover -s tests -v
```

Preview builds the trajectory and prints an estimated duration. It does not verify workspace clearance or predict the exact hardware run time. The bundled CSV contains 182 measured configurations in radians, specific to the original fixture, probe, and arm setup.

### Teach on the lab arm

After lab staff bring up the HEAL server and you source the ROS overlay, start the velocity controller if it is not already running. Verify feedback, robot description, and controller ownership before starting teleop.

```sh
ros2 launch addverb_cobot_control velocity_control.launch.py
python3 teleop_v4.py
```

Run the two commands in separate ROS-sourced terminals. Do not run multiple motion publishers.

| Controls | Action |
| --- | --- |
| Q/A, W/S, E/D | Translate along base-frame X, Y, Z |
| R/F, T/G, Y/H | Rotate about base-frame axes |
| Left / Right | Jog joint 6 alone |
| Space | Save the measured pose after releasing movement keys |
| F5 / F6 | Start / stop and save a sampled teaching take |
| N | Start a new waypoint file and archive prior recordings |
| Esc / X | Stop commanded motion / exit; neither replaces the physical E-stop |

The script saves poses beside itself as `heal_waypoints.csv`. Review the waypoint order and clearance before replay and save the reviewed route as a separate CSV. Keep the original capture for reference.

### Replay on the lab arm

1. Use two trained operators: one operates the controls and the other stays at the E-stop. Keep the probe in view. Check Cartesian clearance independently and start at reduced speed (10–20%) under the lab's procedure.
2. Position the robot at the route start using the approved positioning procedure, and exit teleop and all other motion publishers. Preview the route without `--execute` first.
3. For an authorized, supervised forward test, use the reduced-speed example below. After reviewing preflight, enter `RUN` only when both operators are ready.

```sh
python3 heal_smooth_replay.py data/heal_waypoints_182.csv \
  --from-waypoint 1 --continue-forward \
  --speed 0.007 --wrist-speed 0.028 --accel 0.06 --execute
```

For a return test starting at the measured end pose, replace the forward options with `--from-waypoint 182 --backtrace-only`. During replay, type `P` then Enter to pause, `R` then Enter to resume, or `Q` then Enter to end the run. Use the **physical E-stop immediately** for contact or unexpected motion. Do not restart after a fault until the lab authorizes recovery.

### Evaluate repeatability

Live replay writes `heal_smooth_*.csv` beside the input route. Summarize an individual log with:

```sh
python tools/summarize_replay.py path/to/heal_smooth_run.csv
```

The tool reports completion time, peak commanded speeds, and maximum logged tracking error. It cannot infer contact. Record contact and outcome separately for each run using the [trial sheet](docs/trials.csv).

## Repository layout

| Path | Purpose |
| --- | --- |
| `teleop_v4.py` | Teach and capture waypoints |
| `heal_smooth_replay.py` | Preview and replay the trajectory |
| `data/heal_waypoints_182.csv` | Curated 182-point route |
| `tools/summarize_replay.py` | Offline replay log analysis |
| `tests/` | Offline trajectory and log checks |
| `docs/` | Trial records, technical notes, and demo edit plan |
