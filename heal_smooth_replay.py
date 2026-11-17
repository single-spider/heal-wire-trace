#!/usr/bin/env python3
"""Smooth polynomial/PCHIP buzz-wire replay with separate wrist-rotation stops.

- Translation stretches between stops are fitted with monotone PCHIP splines
  (C1-continuous, zero overshoot, passes through every taught waypoint).
- Pure/dominant joint6 rotations and sharp corners stop cleanly (v = 0) and
  execute with 5th-order rest-to-rest polynomials at --wrist-speed.
- Includes all server-safety guards: 100 Hz loop, CMD_DEADBAND=1e-4 (avoids
  parser.h scientific-notation truncation), SERVER_VEL_CAP=0.45, and freeze detection.
"""

import argparse
import csv
import math
import select
import signal
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from scipy.interpolate import PchipInterpolator

JOINTS = tuple(f"joint{i}" for i in range(1, 7))
INTERFACES = tuple(n + "/velocity" for n in JOINTS)
TOPIC = "/velocity_controller/commands"
ZERO = (0.0,) * 6
PERIOD = 0.01
MAX_AGE = 0.25
MARGIN = 0.02
START_TOL = 0.025
END_TOL = 0.008
TRACK_TOL = 0.05
SERVER_VEL_CAP = 0.45
SERVER_STEP_CAP = 0.15
CMD_DEADBAND = 1e-4
FREEZE_CMD_THRESH = 0.010
FREEZE_ERR_THRESH = 0.008
FREEZE_TIMEOUT = 0.6
STOP = False


def gap(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def load_points(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames or []
        if len(set(fields)) != len(fields) or not {"waypoint", *JOINTS} <= set(fields):
            raise ValueError("CSV needs unique waypoint and joint1 through joint6 columns")
        points, seen = [], set()
        for line, row in enumerate(reader, 2):
            waypoint = int(row["waypoint"])
            q = tuple(float(row[n]) for n in JOINTS)
            if waypoint in seen or not all(map(math.isfinite, q)):
                raise ValueError(f"duplicate ID or non-finite angle on CSV line {line}")
            seen.add(waypoint)
            points.append((waypoint, q))
    if len(points) < 2:
        raise ValueError("need at least two waypoints")
    return points


def read_limits(description):
    root = ET.fromstring(description)
    bounds = []
    for name in JOINTS:
        joint = root.find(f"./joint[@name='{name}']")
        lim = None if joint is None else joint.find("limit")
        if joint is None or joint.get("type") != "revolute" or lim is None:
            raise ValueError(f"no bounded revolute joint in URDF for {name}")
        lo, hi, vmax = (float(lim.attrib[k]) for k in ("lower", "upper", "velocity"))
        bounds.append((lo, hi, vmax))
    return bounds


def inside(q, bounds, label):
    for name, x, (lo, hi, _) in zip(JOINTS, q, bounds):
        if not lo + MARGIN <= x <= hi - MARGIN:
            raise RuntimeError(f"{label}: {name}={x:.5f} rad outside URDF range [{lo:.5f}, {hi:.5f}]")


def quintic_s(elapsed, duration):
    u = min(1.0, max(0.0, elapsed / duration))
    s = u**3 * (10.0 + u * (-15.0 + 6.0 * u))
    ds = 30.0 * u * u * (1.0 - u) ** 2 / duration
    return s, ds


def build_chunks(points, caps, accel, corner_deg=45.0):
    """Partition waypoints into rest-to-rest rotation chunks and smooth PCHIP translation chunks."""
    N = len(points)
    Q = np.array([q for _, q in points], float)
    dQ = np.diff(Q, axis=0)
    is_rot = np.array([
        abs(dQ[i, 5]) > 0.04 and abs(dQ[i, 5]) > 2.0 * np.linalg.norm(dQ[i, :5])
        for i in range(N - 1)
    ])
    stop = np.zeros(N, bool)
    stop[0] = stop[-1] = True
    for i in range(N - 1):
        if is_rot[i]:
            stop[i] = stop[i + 1] = True
    cos_lim = math.cos(math.radians(corner_deg))
    for i in range(1, N - 1):
        if stop[i]:
            continue
        v1, v2 = dQ[i - 1, :5], dQ[i, :5]
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 < 1e-5 or n2 < 1e-5 or float(np.dot(v1, v2) / (n1 * n2)) < cos_lim:
            stop[i] = True

    stops = np.where(stop)[0]
    chunks = []
    caps_arr = np.array(caps, float)
    for a, b in zip(stops[:-1], stops[1:]):
        seg = Q[a:b + 1]
        id_a, id_b = points[a][0], points[b][0]
        if len(seg) == 2:
            d = np.abs(seg[1] - seg[0])
            dur = max(0.25, float(np.max(1.875 * d / caps_arr)), math.sqrt(5.774 * float(np.max(d)) / accel))
            kind = "rotation" if is_rot[a] else "segment"
            chunks.append(dict(kind=kind, label=f"{id_a}->{id_b}", start=tuple(seg[0]), target=tuple(seg[1]),
                               duration=dur, pchip=None, s_max=1.0))
        else:
            ds = np.maximum(np.linalg.norm(np.diff(seg[:, :5], axis=0), axis=1), 1e-4)
            s_arr = np.concatenate([[0.0], np.cumsum(ds)])
            pchip = PchipInterpolator(s_arr, seg, axis=0)
            ss = np.linspace(0.0, s_arr[-1], len(seg) * 20)
            dq_ds = pchip(ss, 1)
            s_vel_max = float(np.min(caps_arr / np.maximum(np.abs(dq_ds), 1e-9)))
            dur = max(0.3, 1.875 * s_arr[-1] / max(s_vel_max, 1e-4), math.sqrt(5.774 * s_arr[-1] / accel))
            chunks.append(dict(kind="smooth", label=f"{id_a}..{id_b} ({len(seg)} pts)",
                               start=tuple(seg[0]), target=tuple(seg[-1]),
                               duration=dur, pchip=pchip, s_max=float(s_arr[-1])))
    return chunks


def eval_chunk(chunk, elapsed):
    s_u, ds_u = quintic_s(elapsed, chunk["duration"])
    if chunk["pchip"] is None:
        delta = tuple(b - a for a, b in zip(chunk["start"], chunk["target"]))
        q_ref = tuple(a + s_u * d for a, d in zip(chunk["start"], delta))
        v_ref = tuple(ds_u * d for d in delta)
        return q_ref, v_ref
    s = s_u * chunk["s_max"]
    ds = ds_u * chunk["s_max"]
    q_ref = tuple(float(v) for v in chunk["pchip"](s))
    v_ref = tuple(float(v) * ds for v in chunk["pchip"](s, 1))
    return q_ref, v_ref


def compute_step_command(chunk, elapsed, caps, accel, period, last_command, actual, progress_pose, progress_at, now):
    q_ref, v_ref = eval_chunk(chunk, elapsed)
    error = gap(q_ref, actual)
    if error > TRACK_TOL:
        raise RuntimeError(f"tracking error {error:.4f} rad exceeded {TRACK_TOL:g}")
    if elapsed > chunk["duration"] + 5.0:
        raise RuntimeError("target did not settle before chunk timeout")
    if gap(actual, progress_pose) >= 0.001:
        progress_at, progress_pose = now, actual
    if max(map(abs, last_command)) < FREEZE_CMD_THRESH and error < FREEZE_ERR_THRESH:
        progress_at, progress_pose = now, actual
    elif now - progress_at > FREEZE_TIMEOUT:
        raise RuntimeError("no measured motion despite velocity commands; inspect hardware stop")
    done = elapsed >= chunk["duration"] and gap(actual, chunk["target"]) <= END_TOL
    desired = ZERO if done else tuple(
        max(-cap, min(cap, v + 1.5 * (ref - q)))
        for cap, v, ref, q in zip(caps, v_ref, q_ref, actual))
    dv = min(accel * period, SERVER_STEP_CAP)
    command = tuple(
        0.0 if abs(c) < CMD_DEADBAND else c
        for c in (
            max(-SERVER_VEL_CAP, min(SERVER_VEL_CAP, max(old - dv, min(old + dv, new))))
            for old, new in zip(last_command, desired)
        )
    )
    return command, q_ref, done, progress_at, progress_pose


def stop_signal(*_):
    global STOP
    STOP = True


def run_live(args, points, writer, stream):
    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from rcl_interfaces.srv import GetParameters
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray

    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = pub = sample = None
    attempted_motion = False
    epoch = time.monotonic()

    def log(event, label="", command=(), note="", ref=()):
        stamp, q = (sample[1], sample[0]) if sample else ("", ("",) * 6)
        writer.writerow((event, f"{time.monotonic()-epoch:.6f}", label, stamp,
                         *q, *(ref or ("",) * 6), *(command or ("",) * 6), note))

    try:
        node = Node("heal_smooth_replay")

        def receive(msg):
            nonlocal sample
            if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
                return
            positions = dict(zip(msg.name, msg.position))
            if not all(n in positions and math.isfinite(positions[n]) for n in JOINTS):
                return
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            age = (node.get_clock().now().nanoseconds - stamp) / 1e9
            if stamp <= 0 or not -0.05 <= age <= MAX_AGE or (sample and stamp <= sample[1]):
                return
            sample = tuple(positions[n] for n in JOINTS), stamp, time.monotonic()

        node.create_subscription(JointState, "/joint_states", receive,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        def check_running():
            if STOP or not rclpy.ok():
                raise RuntimeError("interrupted")

        def wait_for(test, label, seconds=4):
            deadline = time.monotonic() + seconds
            while True:
                check_running()
                if test():
                    return
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"timed out waiting for {label}")
                rclpy.spin_once(node, timeout_sec=0.02)

        def service(kind, path, request):
            client = node.create_client(kind, path)
            try:
                wait_for(client.service_is_ready, path)
                future = client.call_async(request)
                wait_for(future.done, path + " response")
                return future.result()
            finally:
                node.destroy_client(client)

        def parameter(name, key):
            req = GetParameters.Request()
            req.names = [key]
            return service(GetParameters, name + "/get_parameters", req).values[0]

        def graph_check():
            subs = [(e.node_name, e.node_namespace, e.topic_type) for e in node.get_subscriptions_info_by_topic(TOPIC)]
            if subs != [("velocity_controller", "/", "std_msgs/msg/Float64MultiArray")]:
                raise RuntimeError(f"unexpected command listeners: {subs}")
            pubs = [(e.node_name, e.node_namespace) for e in node.get_publishers_info_by_topic(TOPIC)]
            expected = [(node.get_name(), node.get_namespace())] if pub else []
            if pubs != expected:
                raise RuntimeError(f"unexpected command publishers: {pubs}")

        def measured():
            if sample is None or time.monotonic() - sample[2] > MAX_AGE:
                raise RuntimeError("no fresh complete six-joint feedback")
            return sample[0]

        wait_for(lambda: sample is not None, "fresh joint feedback")
        graph_check()
        bounds = read_limits(parameter("/robot_state_publisher", "robot_description").string_value)
        caps = (*(min(args.speed, b[2] * 0.2) for b in bounds[:5]), min(args.wrist_speed, bounds[5][2] * 0.2))
        actual = measured()
        inside(actual, bounds, "measured start")
        if args.approach_points is not None:
            if gap(actual, args.approach_points[0][1]) > START_TOL:
                raise RuntimeError("measured pose does not match first approach row")
            route = args.approach_points if args.backtrace_only else args.approach_points
        else:
            matches = [i for i, (wid, q) in enumerate(points)
                       if gap(actual, q) <= START_TOL and (args.from_waypoint is None or wid == args.from_waypoint)]
            if not matches:
                nearest = min(points, key=lambda x: gap(actual, x[1]))
                raise RuntimeError(f"no matching start; nearest is {nearest[0]} ({gap(actual, nearest[1]):.4f} rad max error)")
            idx = min(matches, key=lambda i: (gap(actual, points[i][1]), i))
            if args.backtrace_only:
                route = points[idx::-1]
            elif idx == 0 or args.continue_forward:
                route = points[idx:]
            else:
                route = points[idx:0:-1] + points
        chunks = build_chunks(route, caps, args.accel, args.corner_deg)
        n_rot = sum(1 for c in chunks if c["kind"] == "rotation")
        n_smooth = sum(1 for c in chunks if c["kind"] == "smooth")
        total_s = sum(c["duration"] for c in chunks)
        print(f"Plan: {len(chunks)} chunks ({n_smooth} smooth splines, {n_rot} wrist rotations), est {total_s/60:.1f} min.", flush=True)
        if input("Clear work area. Type RUN to start: ").strip() != "RUN":
            raise RuntimeError("cancelled before motion")
        ready = time.monotonic()
        wait_for(lambda: sample is not None and sample[2] >= ready, "post-confirmation feedback")
        pub = node.create_publisher(Float64MultiArray, TOPIC, 1)
        wait_for(lambda: pub.get_subscription_count() == 1, "velocity subscriber")
        print("Controls: P pause | R resume | Q quit.", flush=True)

        last_command = ZERO
        paused = False
        c_idx = 0
        elapsed = 0.0
        last_tick = progress_at = time.monotonic()
        progress_pose = measured()
        print(f"Chunk 1/{len(chunks)}: {chunks[0]['kind']} {chunks[0]['label']} ({chunks[0]['duration']:.1f}s)", flush=True)

        while True:
            check_running()
            tick = time.monotonic()
            actual = measured()
            inside(actual, bounds, "measured feedback")
            graph_check()
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.readline().strip().lower()
                if key == "q":
                    raise RuntimeError("operator ended run")
                if key == "p" and not paused:
                    pub.publish(Float64MultiArray(data=list(ZERO)))
                    last_command = ZERO
                    paused = True
                    print("Paused. Type R to resume or Q to quit.", flush=True)
                elif key == "r" and paused:
                    paused = False
                    last_tick = progress_at = time.monotonic()
                    progress_pose = actual
                    print("Resumed.", flush=True)
            if not paused:
                chunk = chunks[c_idx]
                now = time.monotonic()
                elapsed += max(0.0, now - last_tick)
                last_tick = now
                command, q_ref, done, progress_at, progress_pose = compute_step_command(
                    chunk, elapsed, caps, args.accel, PERIOD, last_command, actual, progress_pose, progress_at, now)
                attempted_motion |= any(command)
                pub.publish(Float64MultiArray(data=list(command)))
                last_command = command
                log("command", chunk["label"], command, ref=q_ref)
                if done and command == ZERO:
                    c_idx += 1
                    if c_idx >= len(chunks):
                        log("completed")
                        print("Completed smooth replay!", flush=True)
                        break
                    elapsed = 0.0
                    last_tick = progress_at = time.monotonic()
                    progress_pose = actual
                    nc = chunks[c_idx]
                    print(f"Chunk {c_idx+1}/{len(chunks)}: {nc['kind']} {nc['label']} ({nc['duration']:.1f}s)", flush=True)
            until = tick + PERIOD
            while not STOP and rclpy.ok() and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=min(0.005, max(0.0, until - time.monotonic())))
    except (Exception, KeyboardInterrupt) as exc:
        log("abort", note=str(exc) or "interrupted")
        raise
    finally:
        if attempted_motion and pub is not None and rclpy.ok():
            for _ in range(5):
                pub.publish(Float64MultiArray(data=list(ZERO)))
                time.sleep(PERIOD)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv", type=Path)
    p.add_argument("--from-waypoint", type=int)
    p.add_argument("--approach", type=Path, help="explicit route CSV starting from current measured pose")
    p.add_argument("--backtrace-only", action="store_true", help="backtrace to waypoint 1 and stop")
    p.add_argument("--continue-forward", action="store_true")
    p.add_argument("--speed", type=float, default=0.035)
    p.add_argument("--wrist-speed", type=float, default=0.14)
    p.add_argument("--accel", type=float, default=0.3)
    p.add_argument("--corner-deg", type=float, default=45.0)
    p.add_argument("--execute", action="store_true")
    args = p.parse_args()
    points = load_points(args.csv.expanduser())
    args.approach_points = load_points(args.approach.expanduser()) if args.approach else None
    preview_pts = args.approach_points or points
    caps = (*(args.speed,) * 5, args.wrist_speed)
    chunks = build_chunks(preview_pts, caps, args.accel, args.corner_deg)
    n_rot = sum(1 for c in chunks if c["kind"] == "rotation")
    n_smooth = sum(1 for c in chunks if c["kind"] == "smooth")
    total_s = sum(c["duration"] for c in chunks)
    print(f"{len(points)} waypoints -> {len(chunks)} chunks ({n_smooth} smooth PCHIP splines, {n_rot} wrist rotations), est {total_s/60:.2f} min.")
    if not args.execute:
        print("Offline preview only. No ROS connection or motion.")
        return 0
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    log_path = args.csv.expanduser().resolve().with_name(f"heal_smooth_{stamp}.csv")
    with log_path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("event", "elapsed_s", "chunk", "state_stamp_ns", *JOINTS,
                         *(n + "_ref" for n in JOINTS), *(n + "_cmd" for n in JOINTS), "note"))
        run_live(args, points, writer, stream)
    return 0


if __name__ == "__main__":
    sys.exit(main())
