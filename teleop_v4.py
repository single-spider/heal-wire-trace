#!/usr/bin/env python3
"""HEAL pygame teleop, live joint display, waypoints and trajectory recording.

Run with the same ROS 2 environment as the original teleop.
CSV output is next to this script, independent of the shell's directory.
"""
from __future__ import annotations

import csv
import math
import os
import shutil
import tempfile
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pygame
import PyKDL as kdl
import rclpy
from kdl_parser_py.urdf import treeFromUrdfModel
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from urdf_parser_py.urdf import URDF


JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
CSV_HEADER = ("waypoint", "timestamp_utc", *JOINT_NAMES)
CSV_PATH = Path(__file__).resolve().with_name("heal_waypoints.csv")
LEGACY_DIR = CSV_PATH.with_name("waypoint_legacy")
WRIST_SPEED = 0.2  # rad/s, same as the tested single-joint jog
WRIST_LIMIT_MARGIN = 0.1  # rad
MAX_FEEDBACK_AGE = 0.25  # s
MAX_KEYBOARD_AGE = 0.15  # s
CARTESIAN_KEYS = ((0, "qa"), (1, "ws"), (2, "ed"), (3, "rf"), (4, "tg"), (5, "yh"))
RECORD_RATE_HZ = 10
MAX_RECORD_SAMPLES = 180000
TAKE_HEADER = ("sample", "elapsed_sec", "joint_stamp_ns", "waypoint_mark", *JOINT_NAMES)
COMMAND_HEADER = (
    "timestamp_utc",
    "elapsed_sec",
    "keys",
    "twist_x",
    "twist_y",
    "twist_z",
    "twist_rx",
    "twist_ry",
    "twist_rz",
    "wrist_dir",
    *(f"cmd_j{i}" for i in range(1, 7)),
    *JOINT_NAMES,
    "joint_stamp_ns",
)


def ordered_positions(msg, names):
    if len(msg.name) != len(msg.position):
        raise ValueError("joint names and positions have different lengths")
    by_name = dict(zip(msg.name, msg.position))
    if len(by_name) != len(msg.name):
        raise ValueError("duplicate joint names")
    try:
        positions = [float(by_name[name]) for name in names]
    except KeyError as exc:
        raise ValueError(f"missing joint {exc.args[0]}") from exc
    if not all(math.isfinite(q) for q in positions):
        raise ValueError("non-finite joint position")
    return positions


class WaypointCSV:
    def __init__(self, path):
        self.path = path
        self.next_number = 1
        with path.open("a+", encoding="utf-8", newline="") as stream:
            stream.seek(0)
            reader = csv.reader(stream)
            header = next(reader, None)
            if header is None:
                csv.writer(stream).writerow(CSV_HEADER)
            elif tuple(header) != CSV_HEADER:
                raise ValueError("existing CSV has a different header")
            else:
                for row in reader:
                    if not row:
                        continue
                    if len(row) != len(CSV_HEADER) or row[0] != str(self.next_number):
                        raise ValueError("existing CSV contains an invalid waypoint row")
                    self.next_number += 1

    def append(self, timestamp, positions):
        number = self.next_number
        with self.path.open("a", encoding="utf-8", newline="") as stream:
            csv.writer(stream).writerow((number, timestamp, *positions))
        self.next_number += 1
        return number


def utc_file_stamp():
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")


def archive_to_legacy(src_path, archived_name=None):
    if not src_path.exists():
        return None
    LEGACY_DIR.mkdir(parents=True, exist_ok=True)
    dest = LEGACY_DIR / (archived_name or src_path.name)
    if dest.exists():
        dest = LEGACY_DIR / f"{src_path.stem}_{utc_file_stamp()}{src_path.suffix}"
    shutil.move(str(src_path), str(dest))
    return dest


class CommandLogger:
    """Buffered CSV logger for teleop input commands and matching joint configurations."""

    def __init__(self, path):
        self.path = path
        self._lock = threading.Lock()
        self._rows = []
        self._header_written = path.exists() and path.stat().st_size > 0
        self._start_ns = time.monotonic_ns()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._flush_loop, daemon=True)
        self._thread.start()

    def record(self, keys_label, twist, wrist_dir, cmd_qdot, positions, stamp_ns):
        now_utc = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        elapsed = f"{(time.monotonic_ns() - self._start_ns) / 1e9:.6f}"
        pos_cols = positions if positions is not None else ("",) * 6
        row = (
            now_utc,
            elapsed,
            keys_label,
            *(float(v) for v in twist),
            int(wrist_dir),
            *(float(v) for v in cmd_qdot),
            *pos_cols,
            int(stamp_ns) if stamp_ns is not None else 0,
        )
        with self._lock:
            self._rows.append(row)

    def flush(self):
        with self._lock:
            if not self._rows:
                return
            batch = self._rows
            self._rows = []
        write_header = not self._header_written
        with self.path.open("a", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            if write_header:
                writer.writerow(COMMAND_HEADER)
                self._header_written = True
            writer.writerows(batch)

    def _flush_loop(self):
        while not self._stop.wait(0.5):
            try:
                self.flush()
            except OSError:
                pass

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        self.flush()


class RecordingTake:
    """One take; positions are measured radians, elapsed time uses ROS header stamps."""

    def __init__(self, path):
        self.path = path
        self.rows = []
        self.first_ns = None
        self.last_ns = None

    def capture(self, stamp_ns, positions, mark=False):
        if self.last_ns is not None:
            delta = stamp_ns - self.last_ns
            if delta <= 0 or (not mark and delta < 1_000_000_000 // RECORD_RATE_HZ):
                return
        if self.first_ns is None:
            self.first_ns = stamp_ns
        self.last_ns = stamp_ns
        self.rows.append((len(self.rows) + 1, f"{(stamp_ns - self.first_ns) / 1e9:.9f}",
                          stamp_ns, int(mark), *positions))

    def save(self):
        temp_path = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", newline="",
                                             dir=self.path.parent, prefix=".heal_take_",
                                             delete=False) as stream:
                temp_path = Path(stream.name)
                writer = csv.writer(stream)
                writer.writerow(TAKE_HEADER)
                writer.writerows(self.rows)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_path, self.path)
        finally:
            if temp_path is not None:
                temp_path.unlink(missing_ok=True)


def position_limits(joint):
    if joint is None or joint.type != "revolute" or joint.limit is None:
        return None
    lower, upper = joint.limit.lower, joint.limit.upper
    if (lower is not None and upper is not None and math.isfinite(lower)
            and math.isfinite(upper) and upper > lower):
        return lower, upper
    return None


class JacobianKeyboardTeleop(Node):
    def __init__(self):
        super().__init__("jacobian_keyboard_teleop")
        defaults = {
            "joint_states_topic": "/joint_states",
            "robot_description_topic": "/robot_description",
            "cmd_topic": "/velocity_controller/commands",
            "base_link": "base_link",
            "tip_link": "end-effector",
            "rate_hz": 200.0,
            "v": 0.075,
            "damping": 0.05,
            "keyboard_hz": 200.0,
            "pygame_window_w": 420,
            "pygame_window_h": 170,
            "pygame_grab_input": True,
            "exit_key": "x",
            "exit_on_window_close": True,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            setattr(self, name, self.get_parameter(name).value)
        self.exit_key = str(self.exit_key).strip().lower()[:1] or "x"
        if self.exit_key in "nrfqawsedtgyh":
            self.get_logger().warning("Exit key conflicts with a control. Using X to exit.")
            self.exit_key = "x"
        self._exit_pg_key = getattr(pygame, f"K_{self.exit_key}", pygame.K_x)

        self._keymap = {}
        for i, (positive, negative) in CARTESIAN_KEYS:
            for char, sign in ((positive, 1), (negative, -1)):
                vector = np.zeros(6)
                vector[i] = sign * self.v
                self._keymap[getattr(pygame, f"K_{char}")] = (char, vector)
        self._lock = threading.Lock()
        self._twist = np.zeros(6)
        self._wrist_direction = 0
        self._keys_label = "none"
        self._keyboard_at = None
        self._stop_ui = threading.Event()
        self._exit_requested = threading.Event()

        self._waypoint_lock = threading.Lock()
        self._deadline = None
        self._busy = False
        self._feedback_error = ""
        self._status = "Ready. Press SPACE to save a waypoint."
        self._worker = ThreadPoolExecutor(max_workers=1)
        self._take_lock = threading.Lock()
        self._active_take = None
        self._unsaved_take = None
        self._take_saving = False
        self._take_status = "Idle | F5 starts a new take"
        self._last_cmd_active = False
        self._cmd_logger = CommandLogger(CSV_PATH.with_name(f"heal_commands_{utc_file_stamp()}.csv"))
        self.get_logger().info(f"Command log CSV: {self._cmd_logger.path}")
        try:
            self._csv = WaypointCSV(CSV_PATH)
        except (OSError, ValueError, csv.Error) as exc:
            self._csv = None
            self._status = "CSV unavailable. See terminal."
            self.get_logger().error(f"Cannot open {CSV_PATH}: {exc}")
        else:
            self.get_logger().info(f"Waypoint CSV: {CSV_PATH}")

        self._arm_feedback = None
        self._wrist_limits = None
        self._joint_limits = (None,) * 6
        self._chain = None
        self._chain_names = ()
        self._jac_solver = None
        self._warned_at = float("-inf")
        self._pub = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)
        self.create_subscription(JointState, self.joint_states_topic, self._joint_callback, 50)
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._description_sub = self.create_subscription(
            String, self.robot_description_topic, self._description_callback, qos
        )
        self.create_timer(1.0 / self.rate_hz, self._control_tick)
        self._ui_thread = threading.Thread(target=self._pygame_loop, daemon=True)
        self._ui_thread.start()

    def _set_twist(self, twist=None, wrist_direction=0, keys_label="none"):
        with self._lock:
            self._twist = np.zeros(6) if twist is None else twist
            self._wrist_direction = wrist_direction
            self._keys_label = keys_label
            self._keyboard_at = time.monotonic()

    def _pygame_loop(self):
        try:
            pygame.init()
            screen = pygame.display.set_mode((
                max(800, self.pygame_window_w), max(480, self.pygame_window_h)
            ))
            pygame.display.set_caption("HEAL Teleop + Joint Monitor + Recorder")
            pygame.event.set_grab(self.pygame_grab_input)
            pygame.key.set_repeat()  # One KEYDOWN per physical press.
            clock = pygame.time.Clock()
            font = pygame.font.Font(None, 24)
            wrist_armed = False
            while rclpy.ok() and not self._stop_ui.is_set():
                shortcuts = set()
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self._set_twist()
                        self._stop_ui.set()
                        if self.exit_on_window_close:
                            self._exit_requested.set()
                        break
                    if event.type == pygame.KEYDOWN:
                        shortcuts.add(event.key)
                if self._stop_ui.is_set():
                    break

                held = []
                twist = np.zeros(6)
                wrist_direction = 0
                focused = pygame.key.get_focused()
                if focused:
                    keys = pygame.key.get_pressed()
                    if keys[self._exit_pg_key]:
                        self._set_twist()
                        self._exit_requested.set()
                        break
                    right, left = keys[pygame.K_RIGHT], keys[pygame.K_LEFT]
                    wrist_held = right or left
                    if keys[pygame.K_ESCAPE]:
                        wrist_armed = False
                    else:
                        for key, (char, vector) in self._keymap.items():
                            if keys[key]:
                                held.append(char)
                                twist += vector
                        if wrist_held:
                            if wrist_armed and not held and right != left:
                                wrist_direction = 1 if right else -1
                            elif held or (right and left):
                                wrist_armed = False
                            twist.fill(0.0)
                            held.extend(label for active, label in ((right, "Right"), (left, "Left")) if active)
                        else:
                            wrist_armed = True
                    if pygame.K_F6 in shortcuts:
                        self._end_take()
                    elif pygame.K_F5 in shortcuts:
                        self._start_take()
                    if pygame.K_n in shortcuts:
                        self._rotate_waypoint_file()
                    if pygame.K_SPACE in shortcuts:
                        if wrist_held or any(keys[key] for key in self._keymap):
                            with self._waypoint_lock:
                                if not self._busy and self._csv is not None:
                                    self._status = "Release the movement keys, then press SPACE."
                        else:
                            self._request_waypoint()
                else:
                    wrist_armed = False
                self._set_twist(twist, wrist_direction, "+".join(held) if held else "none")

                screen.fill((25, 25, 25))
                lines = [
                    "HEAL Cartesian teleop",
                    "Q/A: X     W/S: Y     E/D: Z",
                    "R/F: Cartesian roll     T/G: pitch     Y/H: yaw (base axes)",
                    "Right / Left: joint 6 only (+/-), use on its own",
                    f"ESC: stop     {self.exit_key.upper()}: exit",
                    "F5: start take   F6: stop/save take   SPACE: save waypoint   N: new waypoint file",
                    "Pressed: " + (" ".join(held) if held else "(none)")
                    + ("" if focused else "   Click this window to focus"),
                    self._waypoint_status(),
                    self._recording_status(),
                ]
                for i, line in enumerate(lines):
                    color = (255, 220, 140) if i in (3, 5, 7) else (230, 230, 230)
                    screen.blit(font.render(line, True, color), (12, 12 + i * 27))
                self._draw_joint_monitor(screen, font, 12 + len(lines) * 27 + 5)
                pygame.display.flip()
                clock.tick(max(60.0, self.keyboard_hz))
        except Exception as exc:
            self.get_logger().error(f"Pygame window failed: {exc}")
            self._exit_requested.set()
        finally:
            self._set_twist()
            pygame.quit()

    def _request_waypoint(self):
        with self._waypoint_lock:
            if self._csv is None or self._busy:
                return
            self._busy = True
            self._deadline = time.monotonic() + 2.0
            self._feedback_error = ""
            self._status = "Waiting for joint feedback..."

    def _archive_previous_takes_locked(self):
        active_path = self._active_take.path if self._active_take is not None else None
        unsaved_path = self._unsaved_take.path if self._unsaved_take is not None else None
        for take_path in sorted(CSV_PATH.parent.glob("heal_take_*.csv")):
            if take_path == active_path or take_path == unsaved_path:
                continue
            try:
                archived = archive_to_legacy(take_path)
                if archived is not None:
                    print(f"Archived previous take to {archived}", flush=True)
            except OSError as exc:
                self.get_logger().warning(f"Could not archive {take_path}: {exc}")

    def _rotate_waypoint_file(self):
        with self._waypoint_lock:
            if self._busy:
                self._status = "Save in progress; wait and press N again."
                return
            self._deadline = None
            self._feedback_error = ""
            archived_waypoints = None
            try:
                if CSV_PATH.exists() and (self._csv is None or self._csv.next_number > 1):
                    archived_waypoints = archive_to_legacy(
                        CSV_PATH, f"heal_waypoints_{utc_file_stamp()}.csv"
                    )
                self._csv = WaypointCSV(CSV_PATH)
            except (OSError, ValueError, csv.Error) as exc:
                self._csv = None
                self._status = "CSV rotate failed. See terminal."
                self.get_logger().error(f"Cannot rotate {CSV_PATH}: {exc}")
                return
            if archived_waypoints is not None:
                self._status = f"New waypoint file ready (archived to {archived_waypoints.parent.name}/)"
                print(f"Archived waypoints to {archived_waypoints}; started new {CSV_PATH}", flush=True)
            else:
                self._status = "Waypoint file already empty (next: 1)."
        with self._take_lock:
            if not self._take_saving:
                self._archive_previous_takes_locked()

    def _start_take(self):
        with self._take_lock:
            if self._active_take is not None:
                return
            if self._take_saving or self._unsaved_take is not None:
                return  # F6 can retry an unsaved take.
            self._archive_previous_takes_locked()
            path = CSV_PATH.with_name(f"heal_take_{utc_file_stamp()}.csv")
            self._active_take = RecordingTake(path)
            self._take_status = "REC | waiting for fresh arm feedback"

    def _end_take(self, expected=None):
        with self._take_lock:
            if self._take_saving:
                return
            take = self._active_take or self._unsaved_take
            if take is None or (expected is not None and take is not expected):
                return
            self._active_take = None
            if not take.rows:
                self._take_status = "Empty take | no valid joint samples; nothing saved"
                return
            self._unsaved_take = take
            self._take_saving = True
            self._take_status = "Saving take..."
        self._worker.submit(self._save_take, take)

    def _save_take(self, take):
        error = None
        try:
            take.save()
        except (OSError, csv.Error) as exc:
            error = exc
        with self._take_lock:
            self._take_saving = False
            if error is None:
                self._unsaved_take = None
                self._take_status = f"Saved {len(take.rows)} samples | {take.path.name}"
            else:
                self._take_status = "Take save failed | see terminal; F6 retries"
        if error is None:
            print(f"Saved {len(take.rows)} samples to {take.path}", flush=True)
        else:
            print(f"Could not save {take.path}: {error}", flush=True)

    def _recording_status(self):
        with self._take_lock:
            take = self._active_take
            if take is not None and take.rows:
                elapsed = (take.last_ns - take.first_ns) / 1e9
                waiting = " | waiting for fresh feedback" if self._arm_feedback is None or (
                    time.monotonic() - self._arm_feedback[1] > MAX_FEEDBACK_AGE
                ) else ""
                return f"REC | {len(take.rows)} samples | {elapsed:.1f} s of feedback{waiting}"
            return self._take_status

    def _draw_joint_monitor(self, screen, font, y):
        sample = self._arm_feedback
        limits = self._joint_limits
        fresh = sample is not None and time.monotonic() - sample[1] <= MAX_FEEDBACK_AGE
        title = "Joint feedback (degrees) | limits from loaded URDF"
        if not fresh:
            title += " | STALE" if sample is not None else " | waiting for feedback"
        screen.blit(font.render(title, True, (230, 230, 230)), (12, y))
        for i, name in enumerate(JOINT_NAMES):
            y_row = y + 27 + i * 25
            q = sample[0][i] if sample is not None else None
            bounds = limits[i]
            ink = (230, 230, 230) if fresh else (155, 155, 155)
            angle = f"{math.degrees(q):+8.2f}" if q is not None else "       --"
            label = f"{name}    {angle}"
            if bounds is None:
                label += "    limits unavailable"
            else:
                lower, upper = bounds
                label += f"    [{math.degrees(lower):+.1f}, {math.degrees(upper):+.1f}]"
                bar = pygame.Rect(495, y_row + 4, 255, 10)
                pygame.draw.rect(screen, (65, 65, 65), bar, border_radius=5)
                if q is not None:
                    fraction = min(1.0, max(0.0, (q - lower) / (upper - lower)))
                    if fresh and not lower <= q <= upper:
                        ink = (255, 110, 110)
                    elif fresh and min(q - lower, upper - q) <= WRIST_LIMIT_MARGIN:
                        ink = (255, 220, 140)
                    pygame.draw.circle(screen, ink, (bar.x + int(bar.w * fraction), bar.centery), 6)
            screen.blit(font.render(label, True, ink), (12, y_row))

    def _waypoint_status(self):
        with self._waypoint_lock:
            if self._deadline is not None and time.monotonic() >= self._deadline:
                self._deadline = None
                self._busy = False
                self._status = "No valid joint feedback. Press SPACE to retry."
                if self._feedback_error:
                    print(f"Waypoint skipped: {self._feedback_error}", flush=True)
            return self._status

    def _joint_callback(self, msg):
        try:
            positions = ordered_positions(msg, JOINT_NAMES)
        except ValueError as exc:
            with self._waypoint_lock:
                if self._deadline is not None:
                    self._feedback_error = str(exc)
            return  # Partial messages must not replace complete arm feedback.
        stamp = msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if (stamp_ns == 0 or not -0.05 <= age <= MAX_FEEDBACK_AGE
                or (self._arm_feedback is not None and stamp_ns <= self._arm_feedback[2])):
            return
        self._arm_feedback = (positions, time.monotonic(), stamp_ns)
        with self._waypoint_lock:
            mark = self._deadline is not None and time.monotonic() < self._deadline
            if mark:
                self._deadline = None
                self._status = "Saving waypoint..."
        with self._take_lock:
            take = self._active_take
            if take is not None and len(take.rows) < MAX_RECORD_SAMPLES:
                take.capture(stamp_ns, positions, mark=mark)
            full = take is not None and len(take.rows) >= MAX_RECORD_SAMPLES
        if mark:
            timestamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
            self._worker.submit(self._write_waypoint, timestamp, positions)
        if full:
            self._end_take(expected=take)

    def _write_waypoint(self, timestamp, positions):
        status = "Save failed. See terminal."
        try:
            number = self._csv.append(timestamp, positions)
            status = f"Saved waypoint {number}"
            print(f"{status} to {CSV_PATH}", flush=True)
        except (OSError, ValueError, csv.Error) as exc:
            print(f"Waypoint save failed: {exc}", flush=True)
        finally:
            with self._waypoint_lock:
                self._status = status
                self._busy = False

    def _description_callback(self, msg):
        if self._jac_solver is not None:
            return
        try:
            robot = URDF.from_xml_string(msg.data.strip())
            ok, tree = treeFromUrdfModel(robot)
            if not ok:
                raise ValueError("could not build KDL tree")
            chain = tree.getChain(self.base_link, self.tip_link)
            names = []
            for i in range(chain.getNrOfSegments()):
                joint = chain.getSegment(i).getJoint()
                if joint.getTypeName() not in ("None", "Fixed"):
                    names.append(joint.getName())
            if chain.getNrOfJoints() != 6 or tuple(names) != JOINT_NAMES:
                raise ValueError(f"expected HEAL joint order {JOINT_NAMES}, got {names}")
            self._chain = chain
            self._chain_names = tuple(names)
            self._joint_limits = tuple(position_limits(robot.joint_map.get(name)) for name in JOINT_NAMES)
            wrist_limits = self._joint_limits[5]
            if wrist_limits is not None and wrist_limits[1] - wrist_limits[0] > 2 * WRIST_LIMIT_MARGIN:
                self._wrist_limits = wrist_limits
            if self._wrist_limits is None:
                self.get_logger().warning("Arrow jog disabled: joint6 has no usable revolute limits in URDF")
            self._jac_solver = kdl.ChainJntToJacSolver(chain)
            self.get_logger().info(f"KDL ready: {self.base_link} to {self.tip_link}")
            self.destroy_subscription(self._description_sub)
        except Exception as exc:
            self.get_logger().error(f"URDF/KDL setup failed: {exc}")

    def _publish(self, velocities):
        msg = Float64MultiArray()
        msg.data = velocities.tolist()
        self._pub.publish(msg)

    def _wrist_command(self, position, direction):
        command = np.zeros(6)
        if self._wrist_limits is not None:
            lower, upper = self._wrist_limits
            if lower + WRIST_LIMIT_MARGIN < position < upper - WRIST_LIMIT_MARGIN:
                command[5] = direction * WRIST_SPEED
        return command

    def _control_tick(self):
        if self._exit_requested.is_set():
            if self._jac_solver is not None:
                self._publish(np.zeros(6))
            rclpy.try_shutdown()
            return
        if self._jac_solver is None:
            return
        with self._lock:
            twist = self._twist.copy()
            wrist_direction = self._wrist_direction
            keys_label = self._keys_label
            keyboard_at = self._keyboard_at
        now = time.monotonic()
        feedback = self._arm_feedback
        if (feedback is None or now - feedback[1] > MAX_FEEDBACK_AGE
                or keyboard_at is None or now - keyboard_at > MAX_KEYBOARD_AGE):
            zeros = np.zeros(6)
            self._publish(zeros)
            if self._last_cmd_active:
                pos = feedback[0] if feedback is not None else None
                stamp_ns = feedback[2] if feedback is not None else None
                self._cmd_logger.record(keys_label, np.zeros(6), 0, zeros, pos, stamp_ns)
                self._last_cmd_active = False
            return
        positions, _, stamp_ns = feedback
        if wrist_direction:
            qdot = self._wrist_command(positions[5], wrist_direction)
            self._publish(qdot)
            self._cmd_logger.record(keys_label, twist, wrist_direction, qdot, positions, stamp_ns)
            self._last_cmd_active = True
            return
        if np.allclose(twist, 0.0):
            zeros = np.zeros(6)
            self._publish(zeros)
            if self._last_cmd_active or keys_label != "none":
                self._cmd_logger.record(keys_label, twist, wrist_direction, zeros, positions, stamp_ns)
                self._last_cmd_active = keys_label != "none"
            return
        try:
            joints = kdl.JntArray(6)
            for i, position in enumerate(positions):
                joints[i] = position
            jac = kdl.Jacobian(6)
            if self._jac_solver.JntToJac(joints, jac) < 0:
                raise ValueError("KDL Jacobian calculation failed")
            j = np.array([[jac[i, k] for k in range(6)] for i in range(6)])
            qdot = j.T @ np.linalg.solve(j @ j.T + self.damping * np.eye(6), twist)
            if not np.isfinite(qdot).all():
                raise ValueError("non-finite joint velocity")
            qdot[np.abs(qdot) < 1e-3] = 0.0
        except (ValueError, np.linalg.LinAlgError) as exc:
            now = time.monotonic()
            if now - self._warned_at >= 2.0:
                self.get_logger().warning(f"Teleop command skipped: {exc}")
                self._warned_at = now
            qdot = np.zeros(6)
        self._publish(qdot)
        self._cmd_logger.record(keys_label, twist, wrist_direction, qdot, positions, stamp_ns)
        self._last_cmd_active = True

    def destroy_node(self):
        self._stop_ui.set()
        self._set_twist()
        if self._jac_solver is not None and rclpy.ok():
            self._publish(np.zeros(6))
        self._ui_thread.join(timeout=1.0)
        self._end_take()  # Save any non-empty take on a normal exit.
        self._worker.shutdown(wait=True)
        self._cmd_logger.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = JacobianKeyboardTeleop()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
