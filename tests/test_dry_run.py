"""End-to-end dry run: fake phone + fake robot + synthetic mocap, no hardware.
    python -m pytest tests/ -q      (or)   python tests/test_dry_run.py
"""
import csv
import json
import os
import subprocess
import sys
import tempfile
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "coordinator"))

from capture_coordinator.coordinator import Coordinator          # noqa: E402
from capture_coordinator.fake_phone import FakePhone            # noqa: E402
from capture_coordinator.mocap import SyntheticMocapSource      # noqa: E402
from capture_coordinator.phone import PhoneClient               # noqa: E402
from capture_coordinator.plan import Plan                       # noqa: E402
from capture_coordinator.pull import LocalDirPuller             # noqa: E402
from capture_coordinator.robot import RobotClient               # noqa: E402
from capture_coordinator.session import SessionWriter           # noqa: E402


class DryRun(unittest.TestCase):
    def run_session(self, steps, poses, fail_every=0, phone_port=5701, robot_port=5700, frames=1):
        tmp = tempfile.mkdtemp(prefix="capture_dry_")
        docs = os.path.join(tmp, "phone_docs")
        phone_srv = FakePhone(docs, port=phone_port, fail_every=fail_every, capture_s=0.05)
        robot_proc = subprocess.Popen([sys.executable, os.path.join(ROOT, "robot", "fake_robot.py"),
                                       "--port", str(robot_port), "--poses", str(poses),
                                       "--cycles", str(steps), "--move-time", "0.05"],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        mocap = SyntheticMocapSource(rate=200.0, turn_delay_s=0.3)
        plan = Plan(name="test", turntable_steps=steps, poses=poses, lens_position=[0.4] * poses,
                    frames_per_stop=frames, focus_stack_start=0.0, focus_stack_end=0.5,
                    settle_ms=20, still_window_s=0.3, still_timeout_s=5.0,
                    phone_port=phone_port, robot_port=robot_port, output_root=tmp)
        writer = SessionWriter(tmp)
        answers = []
        def operator(q, choices):
            answers.append(q); return "skip" if "skip" in choices else "accept"
        coord = Coordinator(plan, PhoneClient("127.0.0.1", phone_port),
                            lambda: RobotClient("127.0.0.1", robot_port), mocap,
                            LocalDirPuller(docs), writer, operator=operator,
                            on_turntable_prompt=lambda s, target: mocap.set_target_yaw(target),
                            log=lambda *a: None)
        try:
            stats = coord.run()
        finally:
            robot_proc.wait(timeout=30)
            mocap.close(); phone_srv.close()
        return writer, stats, answers

    def test_two_steps_three_poses(self):
        writer, stats, answers = self.run_session(steps=2, poses=3)
        rows = list(csv.DictReader(open(os.path.join(writer.dir, "frames.csv"))))
        self.assertEqual(len(rows), 6)
        self.assertEqual(stats["stops_ok"], 6)
        self.assertEqual(answers, [])
        self.assertEqual(sorted(int(r["step"]) for r in rows), [0, 0, 0, 1, 1, 1])
        self.assertEqual([int(r["pose"]) for r in rows], [1, 2, 3, 1, 2, 3])
        for r in rows:
            self.assertTrue(os.path.exists(os.path.join(writer.images_dir, r["file"])))
            self.assertTrue(os.path.exists(os.path.join(writer.sidecar_dir, r["file"][:-4] + ".json")))
            self.assertLess(float(r["phone_jitter_mm"]), 0.1)
            self.assertAlmostEqual(float(r["fx"]), 1976.0 - 300 * 0.4, places=3)
        events = [json.loads(l)["event"] for l in open(os.path.join(writer.dir, "session.jsonl"))]
        self.assertEqual(events[0], "session_start")
        self.assertEqual(events[-1], "session_end")
        self.assertEqual(events.count("turntable"), 2)
        self.assertEqual(events.count("robot_done"), 2)
        # turntable step 1 was verified at yaw0 + 10 deg
        turns = [json.loads(l) for l in open(os.path.join(writer.dir, "session.jsonl")) if '"turntable"' in l]
        self.assertAlmostEqual(turns[1]["target_yaw"] - turns[0]["target_yaw"], 10.0, places=3)
        self.assertLess(abs(turns[1]["yaw"] - turns[1]["target_yaw"]), 0.5)

    def test_focus_stack_40_frames(self):
        writer, stats, answers = self.run_session(steps=1, poses=1, phone_port=5721, robot_port=5720, frames=40)
        rows = list(csv.DictReader(open(os.path.join(writer.dir, "frames.csv"))))
        self.assertEqual(len(rows), 40)
        lens = [float(r["lens_position"]) for r in rows]
        self.assertAlmostEqual(lens[0], 0.0, places=3); self.assertAlmostEqual(lens[-1], 0.5, places=3)
        self.assertEqual([int(r["focus_index"]) for r in rows], list(range(40)))
        self.assertEqual(len({r["file"] for r in rows}), 40)

    def test_retry_on_phone_failure(self):
        writer, stats, answers = self.run_session(steps=1, poses=4, fail_every=3, phone_port=5711, robot_port=5710)
        rows = list(csv.DictReader(open(os.path.join(writer.dir, "frames.csv"))))
        self.assertEqual(len(rows), 4)           # every failure was retried successfully
        self.assertGreaterEqual(stats["retries"], 1)
        self.assertEqual(stats["skipped"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
