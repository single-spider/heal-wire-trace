"""Offline tests; these never initialize ROS."""

import csv
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from heal_smooth_replay import build_chunks, eval_chunk, load_points
from tools.summarize_replay import JOINTS, summarize


class ReplayTests(unittest.TestCase):
    def test_route_in_both_directions(self):
        path = Path(__file__).resolve().parents[1] / "data" / "heal_waypoints_182.csv"
        points = load_points(path)
        self.assertEqual([waypoint for waypoint, _ in points], list(range(1, 183)))
        for route in (points, points[::-1]):
            chunks = build_chunks(route, (0.035,) * 5 + (0.14,), 0.3)
            self.assertEqual(chunks[0]["start"], route[0][1])
            self.assertEqual(chunks[-1]["target"], route[-1][1])
            self.assertIn("smooth", {chunk["kind"] for chunk in chunks})
            self.assertIn("rotation", {chunk["kind"] for chunk in chunks})
            for chunk in chunks:
                for t, endpoint in ((0.0, "start"), (chunk["duration"], "target")):
                    q, v = eval_chunk(chunk, t)
                    for actual, expected in zip(q, chunk[endpoint]):
                        self.assertAlmostEqual(actual, expected)
                    self.assertEqual(v, (0.0,) * 6)

    def test_log_summary_does_not_claim_no_contact(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "run.csv"
            with path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(("event", "elapsed_s", *JOINTS,
                                 *(f"{j}_ref" for j in JOINTS),
                                 *(f"{j}_cmd" for j in JOINTS)))
                writer.writerow(("command", 1, *([0.1] * 6), *([0.2] * 6), *([-0.03] * 6)))
                writer.writerow(("completed", 2))
            result = summarize(path)
        self.assertEqual(result["result"], "completed")
        self.assertEqual(result["logged_elapsed_s"], 2)
        self.assertEqual(result["peak_command_rad_s"]["joint5"], 0.03)
        self.assertAlmostEqual(result["max_tracking_error_rad"], 0.1)
        self.assertEqual(result["wire_contact"], "not measured by replay")
