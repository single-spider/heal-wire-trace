"""Summarize a smooth replay CSV without ROS or robot access."""

import argparse
import csv
import json
import math
from pathlib import Path

JOINTS = tuple(f"joint{i}" for i in range(1, 7))


def summarize(path):
    peaks = dict.fromkeys(JOINTS, 0.0)
    max_error = 0.0
    elapsed = None
    samples = 0
    result = "incomplete"
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"event", "elapsed_s", *JOINTS}
        required.update(f"{joint}_{kind}" for joint in JOINTS for kind in ("ref", "cmd"))
        if not required <= set(reader.fieldnames or ()):
            raise ValueError("input does not have the smooth replay log columns")
        for row in reader:
            elapsed = float(row["elapsed_s"])
            if not math.isfinite(elapsed) or elapsed < 0:
                raise ValueError(f"invalid elapsed time on CSV line {reader.line_num}")
            if row["event"] in {"completed", "abort"}:
                result = row["event"]
            if row["event"] != "command":
                continue
            samples += 1
            for joint in JOINTS:
                cmd, ref, measured = (float(row[key]) for key in
                                      (f"{joint}_cmd", f"{joint}_ref", joint))
                if not all(map(math.isfinite, (cmd, ref, measured))):
                    raise ValueError(f"non-finite command or feedback on CSV line {reader.line_num}")
                peaks[joint] = max(peaks[joint], abs(cmd))
                max_error = max(max_error, abs(ref - measured))
    return {
        "result": result,
        "logged_elapsed_s": elapsed,
        "command_samples": samples,
        "peak_command_rad_s": peaks if samples else None,
        "max_tracking_error_rad": max_error if samples else None,
        "wire_contact": "not measured by replay",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_csv", type=Path)
    args = parser.parse_args()
    try:
        print(json.dumps(summarize(args.log_csv), indent=2))
    except (OSError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
