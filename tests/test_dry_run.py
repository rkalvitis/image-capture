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
from capture_coordinator.mocap import NoMocapSource, SyntheticMocapSource      # noqa: E402
from capture_coordinator.phone import PhoneClient               # noqa: E402
from capture_coordinator.plan import Plan                       # noqa: E402
from capture_coordinator.pull import LocalDirPuller             # noqa: E402
from capture_coordinator.robot import RobotClient               # noqa: E402
from capture_coordinator.session import SessionWriter           # noqa: E402
from capture_coordinator.turntable import make_turntable        # noqa: E402


class DryRun(unittest.TestCase):
    def run_session(self, steps, poses, fail_every=0, phone_port=5701, robot_port=5700, frames=1,
                    real_frames=False, tmp=None, session_dir=None, resume=False, start_step=0, **plan_extra):
        tmp = tmp or tempfile.mkdtemp(prefix="capture_dry_")
        docs = os.path.join(tmp, "phone_docs")
        phone_srv = FakePhone(docs, port=phone_port, fail_every=fail_every, capture_s=0.05, real_frames=real_frames)
        robot_proc = subprocess.Popen([sys.executable, os.path.join(ROOT, "robot", "fake_robot.py"),
                                       "--port", str(robot_port), "--poses", str(poses),
                                       "--cycles", str(1 if plan_extra.get("loop_order") == "rotations-per-pose" else steps - start_step),
                                       "--move-time", "0.05"],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        mocap = NoMocapSource() if plan_extra.pop("no_mocap", False) else SyntheticMocapSource(rate=200.0, turn_delay_s=0.3)
        operator_answers = plan_extra.pop("operator_answers", None)
        turntable_transport = plan_extra.pop("turntable_transport", None)
        plan_kw = dict(name="test", turntable_steps=steps, poses=poses, lens_position=[0.4] * poses,
                       frames_per_stop=frames, focus_stack_start=0.0, focus_stack_end=0.5,
                       settle_ms=20, settle_after_move_s=0.0, still_window_s=0.3, still_timeout_s=5.0,
                       phone_port=phone_port, robot_port=robot_port, output_root=tmp)
        plan_kw.update(plan_extra)
        plan = Plan(**plan_kw)
        writer = SessionWriter(tmp, session_dir=session_dir, resume=resume)
        answers = []
        def operator(q, choices):
            answers.append(q)
            if operator_answers is not None:
                return operator_answers.pop(0) if operator_answers else "enter"
            return "skip" if "skip" in choices else "accept"
        turntable = make_turntable(plan, log=lambda *a: None, transport=turntable_transport)
        coord = Coordinator(plan, PhoneClient("127.0.0.1", phone_port),
                            lambda: RobotClient("127.0.0.1", robot_port), mocap,
                            LocalDirPuller(docs), writer, operator=operator,
                            on_turntable_prompt=lambda s, target: mocap.set_target_yaw(target),
                            log=lambda *a: None, start_step=start_step, turntable=turntable)
        self.turntable = turntable
        try:
            stats = coord.run()
        finally:
            robot_proc.wait(timeout=30)
            mocap.close(); phone_srv.close()
        return writer, stats, answers

    def test_two_steps_three_poses(self):
        writer, stats, answers = self.run_session(steps=2, poses=3)
        self.assertEqual(len(answers), 1); self.assertIn("Turn the insect BACK", answers[0])   # manual: one Enter at the end
        answers = []
        rows = list(csv.DictReader(open(os.path.join(writer.dir, "frames.csv"))))
        self.assertEqual(len(rows), 6)
        self.assertEqual(stats["stops_ok"], 6)
        self.assertEqual(answers, [])
        self.assertEqual(sorted(int(r["step"]) for r in rows), [0, 0, 0, 1, 1, 1])
        self.assertEqual([int(r["pose"]) for r in rows], [1, 2, 3, 1, 2, 3])
        for r in rows:
            self.assertEqual(r["image"], f"s{int(r['step']):02d}_p{r['pose']}")
            stack_dir = os.path.join(writer.images_dir, r["image"], "stack")
            self.assertTrue(os.path.exists(os.path.join(stack_dir, r["file"])))
            self.assertTrue(os.path.exists(os.path.join(stack_dir, r["file"][:-4] + ".json")))
            self.assertTrue(os.path.exists(os.path.join(stack_dir, "mocap.csv")))
            self.assertLess(float(r["phone_jitter_mm"]), 0.1)
            self.assertAlmostEqual(float(r["fx"]), 1976.0 - 300 * 0.4, places=3)
            self.assertAlmostEqual(float(r["phone_z"]), 1.62, delta=0.01)         # body poses (world)
            self.assertEqual(len(r["pop_quat_xyzw"].split()), 4)
            self.assertEqual(r["cam_x"], "")                                      # no rig file -> no camera pose
        events = [json.loads(l)["event"] for l in open(os.path.join(writer.dir, "session.jsonl"))]
        self.assertEqual(events[0], "session_start")
        self.assertEqual(events[-1], "session_end")
        self.assertEqual(events.count("turntable"), 2)
        self.assertEqual(events.count("robot_done"), 2)
        # turntable step 1 was verified at yaw0 + 10 deg
        turns = [json.loads(l) for l in open(os.path.join(writer.dir, "session.jsonl")) if '"turntable"' in l]
        self.assertAlmostEqual(turns[1]["target_yaw"] - turns[0]["target_yaw"], 10.0, places=3)
        self.assertLess(abs(turns[1]["yaw"] - turns[1]["target_yaw"]), 0.5)
        robot = list(csv.DictReader(open(os.path.join(writer.dir, "robot", "robot_poses.csv"))))
        self.assertEqual([(r["step"], r["pose"]) for r in robot], [("0", "1"), ("0", "2"), ("0", "3"), ("1", "1"), ("1", "2"), ("1", "3")])
        self.assertEqual(len(robot[0]["j1"]) > 0, True)

    def test_motor_turntable_with_yaw_gate(self):
        # fake motor + synthetic mocap: one Enter (insect placed), then GOTO 10, 20 with the yaw gate
        writer, stats, answers = self.run_session(steps=3, poses=1, phone_port=5741, robot_port=5740,
                                                  turntable_drive="fake", turntable_confirm="mocap")
        self.assertEqual(stats["stops_ok"], 3)
        self.assertEqual(answers, [])                                        # motor: no prompt at all
        self.assertEqual(self.turntable.moves, [10.0, 20.0, 0.0])                 # ... and back to 0 at the end (reverse)
        ev = [json.loads(l) for l in open(os.path.join(writer.dir, "session.jsonl"))]
        # the motor move of step 1 was started right after the last stack of step 0, before the robot run ended
        kinds = [(e["event"], e.get("step")) for e in ev]
        self.assertLess(kinds.index(("turntable_early", 1)), kinds.index(("robot_done", 0)))
        self.assertEqual(kinds.count(("turntable_early", 1)) + kinds.count(("turntable_early", 2)), 2)   # not for a step beyond the last
        self.assertEqual([e["pose"] for e in ev if e["event"] == "turntable_return"], ["session"])
        self.assertEqual([e["event"] for e in ev if e["event"] == "turntable_ready"], ["turntable_ready"])
        turns = [e for e in ev if e["event"] == "turntable"]
        self.assertEqual([t["confirmed"] for t in turns], ["auto", "motor+mocap", "motor+mocap"])
        self.assertEqual([t["commanded_deg"] for t in turns], [0.0, 10.0, 20.0])
        self.assertEqual([t["pos_deg"] for t in turns], [0.0, 10.0, 20.0])
        for t in turns[1:]:
            self.assertLess(abs(t["yaw"] - t["target_yaw"]), 0.5)
        self.assertAlmostEqual(turns[2]["target_yaw"] - turns[0]["target_yaw"], 20.0, places=3)

    def test_portenta_turntable_full_loop(self):
        # the user's Portenta stepper controller (protocol fake): ZERO at the insect placement, MOVETO per rotation
        from capture_coordinator.turntable import FakePortentaTransport
        board = FakePortentaTransport()
        writer, stats, answers = self.run_session(steps=4, poses=1, phone_port=5771, robot_port=5770,
                                                  turntable_drive="portenta", turntable_confirm="mocap",
                                                  turntable_transport=board)
        self.assertEqual(stats["stops_ok"], 4)
        self.assertEqual(answers, [])                                            # motor: no prompt
        self.assertEqual([l for l in board.log if l.startswith(("ZERO", "MOVETO"))],
                         ["ZERO 0", "MOVETO 89", "MOVETO 178", "MOVETO 267", "MOVETO 0"])   # absolute microsteps, 10 deg each, then back
        turns = [json.loads(l) for l in open(os.path.join(writer.dir, "session.jsonl")) if '"turntable"' in l]
        self.assertEqual([t["confirmed"] for t in turns], ["auto", "motor+mocap", "motor+mocap", "motor+mocap"])
        self.assertEqual([round(t["pos_deg"], 4) for t in turns], [0.0, 10.0125, 20.025, 30.0375])
        self.assertEqual(turns[0]["drive"], "portenta")

    def test_motor_turntable_no_mocap(self):
        # fake motor, no mocap, turntable_confirm enter = no gate: the operator is asked once only
        writer, stats, answers = self.run_session(steps=2, poses=2, phone_port=5751, robot_port=5750,
                                                  no_mocap=True, turntable_drive="fake", turntable_confirm="enter")
        self.assertEqual(stats["stops_ok"], 4)
        self.assertEqual(answers, [])
        self.assertEqual(self.turntable.moves, [10.0, 0.0])
        turns = [json.loads(l) for l in open(os.path.join(writer.dir, "session.jsonl")) if '"turntable"' in l]
        self.assertEqual([t["confirmed"] for t in turns], ["auto", "motor"])
        self.assertIsNone(turns[1]["yaw"])

    def test_motor_turntable_resume_sets_position(self):
        # --resume --start-step 1: the platter is declared to be at stop 1, then GOTO 20
        tmp = tempfile.mkdtemp(prefix="capture_dry_")
        sdir = os.path.join(tmp, "sess")
        self.run_session(steps=1, poses=1, phone_port=5761, robot_port=5760, tmp=tmp, session_dir=sdir,
                         no_mocap=True, turntable_drive="fake", turntable_confirm="enter")
        writer, stats, answers = self.run_session(steps=3, poses=1, phone_port=5763, robot_port=5762, tmp=tmp,
                                                  session_dir=sdir, resume=True, start_step=1,
                                                  no_mocap=True, turntable_drive="fake", turntable_confirm="enter")
        self.assertEqual(stats["stops_ok"], 2)
        self.assertEqual(answers, [])
        self.assertEqual(self.turntable.pos_deg, 0.0)                              # back at 0 after the session
        self.assertEqual(self.turntable.moves, [20.0, 0.0])

    def test_rotations_per_pose_motor(self):
        # robot pose outer loop, fake motor: at every pose 3 rotations, then the platter reverses to 0
        writer, stats, answers = self.run_session(steps=3, poses=2, phone_port=5771, robot_port=5770,
                                                  no_mocap=True, turntable_drive="fake", turntable_confirm="enter",
                                                  loop_order="rotations-per-pose", subject="popillia test")
        self.assertEqual(stats["stops_ok"], 6)
        self.assertEqual(answers, [])                                            # motor: no prompt at all
        self.assertEqual(self.turntable.moves, [10.0, 20.0, 0.0, 10.0, 20.0, 0.0])   # return after each pose
        rows = list(csv.DictReader(open(os.path.join(writer.dir, "frames.csv"))))
        self.assertEqual([(int(r["step"]), int(r["pose"])) for r in rows], [(0, 1), (1, 1), (2, 1), (0, 2), (1, 2), (2, 2)])
        self.assertEqual([r["image"] for r in rows][:3], ["s00_p1", "s01_p1", "s02_p1"])
        ev = [json.loads(l) for l in open(os.path.join(writer.dir, "session.jsonl"))]
        kinds = [e["event"] for e in ev]
        self.assertEqual(kinds.count("robot_done"), 1)                              # one robot run
        self.assertEqual(kinds.count("robot_reached"), 2)
        self.assertEqual([e["pose"] for e in ev if e["event"] == "turntable_return"], [1, 2])
        self.assertEqual(ev[0]["loop_order"], "rotations-per-pose")
        self.assertEqual(ev[0]["turntable_drive"], "fake")
        robot = list(csv.DictReader(open(os.path.join(writer.dir, "robot", "robot_poses.csv"))))
        self.assertEqual([(r["step"], r["pose"]) for r in robot], [("0", "1"), ("1", "1"), ("2", "1"), ("0", "2"), ("1", "2"), ("2", "2")])
        # the summary files
        summ = json.load(open(os.path.join(writer.dir, "summary.json")))
        self.assertEqual(summ["session"]["status"], "finished")
        self.assertEqual(summ["session"]["subject"], "popillia test")
        self.assertEqual(summ["collection"]["loop_order"], "rotations-per-pose")
        self.assertEqual(summ["collection"]["turntable_drive"], "fake")
        self.assertEqual(summ["collection"]["turntable_returns"], 2)
        self.assertEqual(summ["results"]["stops_ok"], 6)
        self.assertEqual(summ["results"]["stops_planned"], 6)
        self.assertEqual(summ["results"]["frames_captured"], 6)
        self.assertEqual(summ["results"]["rotations_complete"], 3)
        self.assertEqual(summ["results"]["stops_per_robot_pose"], {"pose 1": 3, "pose 2": 3})
        self.assertIsNotNone(summ["session"]["started"]); self.assertIsNotNone(summ["session"]["ended"])
        self.assertEqual(summ["timing"]["stop_capture_s"]["n"], 6)
        self.assertEqual(summ["quality"]["thermal_frames"], {"nominal": 6})
        md = open(os.path.join(writer.dir, "SUMMARY.md")).read()
        self.assertIn("robot pose outer loop", md)
        self.assertIn("popillia test", md)
        self.assertIn("6 / 6 stops", md)

    def test_rotations_per_pose_manual_and_resume(self):
        # manual platter: Enter per rotation + "turn BACK" after each pose; resume skips the done stops
        tmp = tempfile.mkdtemp(prefix="capture_dry_")
        sdir = os.path.join(tmp, "sess")
        w1, stats1, answers1 = self.run_session(steps=2, poses=1, phone_port=5781, robot_port=5780, tmp=tmp,
                                                session_dir=sdir, no_mocap=True, turntable_confirm="enter",
                                                loop_order="rotations-per-pose")
        self.assertEqual(stats1["stops_ok"], 2)
        self.assertEqual(len(answers1), 3)                                       # place, turn to 1, turn BACK
        self.assertIn("TURN the insect to step 1", answers1[1])
        self.assertIn("BACK to step 0", answers1[2])
        w2, stats2, answers2 = self.run_session(steps=2, poses=2, phone_port=5783, robot_port=5782, tmp=tmp,
                                                session_dir=sdir, resume=True, no_mocap=True, turntable_confirm="enter",
                                                loop_order="rotations-per-pose", turntable_drive="fake")
        self.assertEqual(stats2["already_done"], 2)
        self.assertEqual(stats2["stops_ok"], 2)
        self.assertEqual(self.turntable.moves, [10.0, 0.0])                      # pose 1 skipped without moving
        summ = json.load(open(os.path.join(sdir, "summary.json")))
        self.assertEqual(summ["session"]["runs"], 2)
        self.assertEqual(summ["session"]["resumes"], 1)
        self.assertEqual(summ["results"]["stops_ok"], 4)
        self.assertEqual(summ["results"]["stops_already_done_on_resume"], 2)

    def test_focus_stack_40_frames(self):
        writer, stats, answers = self.run_session(steps=1, poses=1, phone_port=5721, robot_port=5720, frames=40)
        rows = list(csv.DictReader(open(os.path.join(writer.dir, "frames.csv"))))
        self.assertEqual(len(rows), 40)
        lens = [float(r["lens_position"]) for r in rows]
        self.assertAlmostEqual(lens[0], 0.0, places=3); self.assertAlmostEqual(lens[-1], 0.5, places=3)
        self.assertEqual([int(r["focus_index"]) for r in rows], list(range(40)))
        self.assertEqual(len({r["file"] for r in rows}), 40)

    def test_inline_merge_with_rig(self):
        try:
            import cv2  # noqa: F401
        except ImportError:
            self.skipTest("opencv not installed")
        import yaml
        rig = os.path.join(tempfile.mkdtemp(prefix="rig_"), "rig.yaml")
        yaml.safe_dump({"T_body_from_cam": {"translation_m": [0.065, -0.019, -0.048],
                                            "quaternion_xyzw": [0.35535, 0.849822, 0.357605, 0.153776]}}, open(rig, "w"))
        writer, stats, answers = self.run_session(steps=1, poses=2, phone_port=5731, robot_port=5730, frames=5,
                                                  real_frames=True, rig_transforms=rig, merge_inline=True,
                                                  merge_delete_raw=True, merge_python=sys.executable)
        self.assertEqual(stats["stops_ok"], 2)
        self.assertEqual(stats.get("merged_ok"), 2, stats)
        rows = list(csv.DictReader(open(os.path.join(writer.dir, "frames.csv"))))
        self.assertEqual(len(rows), 10)
        self.assertNotEqual(rows[0]["cam_x"], "")                                # camera pose from the rig
        self.assertEqual(len(rows[0]["quat_xyzw"].split()), 4)
        merged = list(csv.DictReader(open(os.path.join(writer.dir, "stacks.csv"))))
        self.assertEqual([m["stop"] for m in merged], ["0", "1"])
        self.assertEqual([m["image"] for m in merged], ["s00_p1", "s00_p2"])
        self.assertEqual(merged[0]["ref_focus_index"], merged[0]["best_focus_index"])   # reference = sharpest on the beetle
        self.assertEqual(merged[0]["raw_deleted_on_mac"], "1")
        self.assertEqual(merged[0]["fusion"], "pyramid"); self.assertEqual(merged[0]["format"], "tif")
        for m in merged:
            d = os.path.join(writer.images_dir, m["image"])
            self.assertTrue(os.path.exists(os.path.join(d, m["file"])))                     # s00_p1.tif
            self.assertTrue(os.path.exists(os.path.join(d, m["image"] + ".jpg")))            # preview
            stack = sorted(os.listdir(os.path.join(d, "stack")))
            self.assertEqual([f for f in stack if f.endswith(".png") and not f.endswith("_beetle_mask.png")], [])   # raw frames gone from the Mac
            self.assertEqual(len([f for f in stack if f.endswith(".json") and f != "stack.json"]), 5)   # sidecars kept
            summary = json.load(open(os.path.join(d, "stack", "stack.json")))
            self.assertEqual(summary["n_frames"], 5); self.assertTrue(summary["raw_deleted_on_mac"])
            self.assertEqual(len(summary["alignment"]), 5); self.assertEqual(len(summary["frames"]), 5)
        self.assertTrue(os.path.exists(os.path.join(writer.dir, "alignment.csv")))
        self.assertEqual(len(os.listdir(os.path.join(writer.dir, "..", "phone_docs", writer.session_id))), 20)  # phone untouched
        events = [json.loads(l)["event"] for l in open(os.path.join(writer.dir, "session.jsonl"))]
        self.assertIn("merge_done", events)

    def test_inline_merge_then_upload_deletes_local_and_phone(self):
        # merge -> upload (fake rclone) -> rclone check -> delete dng + tif locally and the frames on the phone
        try:
            import cv2  # noqa: F401
        except ImportError:
            self.skipTest("opencv not installed")
        tmp = tempfile.mkdtemp(prefix="capture_dry_")
        remote = os.path.join(tmp, "onedrive")
        fake = os.path.join(HERE, "fake_rclone.py")
        writer, stats, answers = self.run_session(steps=1, poses=2, phone_port=5851, robot_port=5850, frames=5, tmp=tmp,
                                                  real_frames=True, merge_inline=True, merge_delete_raw=False,
                                                  merge_python=sys.executable, upload_remote="fakeremote:" + remote,
                                                  upload_what="all", upload_delete_local=True, upload_delete_phone=True,
                                                  upload_rclone=fake, no_mocap=True, turntable_confirm="enter")
        self.assertEqual(stats["stops_ok"], 2)
        self.assertEqual(stats.get("merged_ok"), 2, stats)
        self.assertEqual(stats.get("uploaded_ok"), 2, stats)
        self.assertEqual(stats.get("uploaded_failed"), 0)
        self.assertEqual(len([f for f in os.listdir(os.path.join(remote, "raw")) if f.endswith(".json")]), 10)  # sidecars, flat
        for image in ("s00_p1", "s00_p2"):
            self.assertTrue(os.path.exists(os.path.join(remote, "stacked", image + ".tif")))
            self.assertTrue(os.path.exists(os.path.join(remote, "previews", image + ".jpg")))
            self.assertTrue(os.path.exists(os.path.join(remote, "stack_meta", image + ".json")))
            self.assertFalse(os.path.exists(os.path.join(writer.images_dir, image)))                       # whole stop folder gone
        self.assertEqual(os.listdir(os.path.join(tmp, "phone_docs", writer.session_id)), [])                # phone emptied
        for f in ("frames.csv", "session.jsonl", "summary.json", "SUMMARY.md"):      # plan.json comes from main()
            self.assertTrue(os.path.exists(os.path.join(remote, f)), f)
        self.assertTrue(os.path.exists(os.path.join(remote, "poses", "stops.csv")))
        self.assertTrue(os.path.exists(os.path.join(remote, "poses", "robot_poses.csv")))
        self.assertTrue(os.path.exists(os.path.join(remote, "robot", "robot_poses.csv")))
        stops = list(csv.DictReader(open(os.path.join(writer.dir, "poses", "stops.csv"))))
        self.assertEqual([s["image"] for s in stops], ["s00_p1", "s00_p2"])
        self.assertEqual(stops[0]["n_frames"], "5"); self.assertNotEqual(stops[0]["j1"], "")
        events = [json.loads(l) for l in open(os.path.join(writer.dir, "session.jsonl"))]
        kinds = [e["event"] for e in events]
        self.assertEqual(kinds.count("stop_uploaded"), 2); self.assertEqual(kinds.count("phone_deleted"), 2)
        self.assertEqual(kinds.count("stop_raw_uploaded"), 2)                      # raw phase right after the pull
        self.assertIn("upload_done", kinds); self.assertIn("session_files_uploaded", kinds)
        self.assertLess(kinds.index("merge_done"), kinds.index("upload_done"))
        raw = [e for e in events if e["event"] == "stop_raw_uploaded"]
        self.assertEqual(raw[0]["deleted_phone"], 10)                               # phone emptied after the raw phase
        self.assertLess(kinds.index("stop_raw_uploaded"), kinds.index("stop_uploaded"))
        up = [e for e in events if e["event"] == "stop_uploaded"]
        self.assertEqual(up[0]["deleted_phone"], 0); self.assertEqual(up[0]["deleted_local"], 14)  # 5 png + 5 json + tif + jpg + stack.json + beetle mask
        summary = json.load(open(os.path.join(writer.dir, "summary.json")))
        self.assertEqual(summary["results"]["stops_uploaded"], 2)
        self.assertEqual(summary["results"]["stops_still_local"], 0)
        self.assertEqual(summary["results"]["files_deleted_on_phone"], 20)
        self.assertEqual(summary["collection"]["upload"]["remote"], "fakeremote:" + remote)

    def test_raw_upload_without_merge(self):
        # no inline merge: the raw stack goes to the cloud right after the pull; local + phone copies deleted
        tmp = tempfile.mkdtemp(prefix="capture_dry_")
        remote = os.path.join(tmp, "onedrive")
        writer, stats, answers = self.run_session(steps=2, poses=1, phone_port=5861, robot_port=5860, frames=2, tmp=tmp,
                                                  upload_remote="fakeremote:" + remote, upload_what="raw",
                                                  upload_delete_local=True, upload_delete_phone=True,
                                                  upload_rclone=os.path.join(HERE, "fake_rclone.py"))
        self.assertEqual(stats["stops_ok"], 2); self.assertEqual(stats.get("uploaded_ok"), 2, stats)
        self.assertEqual(len([f for f in os.listdir(os.path.join(remote, "raw")) if f.endswith(".dng")]), 4)   # 2 stops x 2, flat
        self.assertFalse(os.path.exists(os.path.join(remote, "stacked")))
        for image in ("s00_p1", "s01_p1"):
            self.assertEqual([f for f in os.listdir(os.path.join(writer.images_dir, image, "stack")) if f.endswith(".dng")], [])
        self.assertEqual(os.listdir(os.path.join(tmp, "phone_docs", writer.session_id)), [])
        self.assertTrue(os.path.exists(os.path.join(remote, "poses", "mocap", "s01_p1.csv")))
        self.assertEqual(len(list(csv.DictReader(open(os.path.join(remote, "poses", "turntable.csv"))))), 3)   # 2 turns + the return to 0

    def test_autofocus_stack(self):
        writer, stats, answers = self.run_session(steps=1, poses=2, phone_port=5741, robot_port=5740, frames=21,
                                                  focus_mode="auto", focus_stack_step=0.01)
        rows = list(csv.DictReader(open(os.path.join(writer.dir, "frames.csv"))))
        self.assertEqual(len(rows), 42)
        lens = [float(r["lens_position"]) for r in rows[:21]]
        self.assertAlmostEqual(lens[10], 0.3, places=4)                    # the AF value is the middle frame
        self.assertAlmostEqual(lens[0], 0.2, places=4); self.assertAlmostEqual(lens[-1], 0.4, places=4)
        self.assertTrue(all(r["focus_centre"] == "0.3" and r["focus_source"] == "auto" for r in rows))
        events = [json.loads(l) for l in open(os.path.join(writer.dir, "session.jsonl"))]
        self.assertEqual(sum(1 for e in events if e["event"] == "autofocus"), 2)

    def test_pose_centred_stack(self):
        writer, stats, answers = self.run_session(steps=1, poses=2, phone_port=5751, robot_port=5750, frames=5,
                                                  focus_mode="pose", focus_stack_step=0.02,
                                                  lens_position=[0.10, 0.30])
        rows = list(csv.DictReader(open(os.path.join(writer.dir, "frames.csv"))))
        lens = [round(float(r["lens_position"]), 4) for r in rows]
        self.assertEqual(lens[:5], [0.06, 0.08, 0.10, 0.12, 0.14])
        self.assertEqual(lens[5:], [0.26, 0.28, 0.30, 0.32, 0.34])
        self.assertEqual({r["focus_source"] for r in rows}, {"pose"})
        self.assertEqual([r["focus_centre"] for r in rows[::5]], ["0.1", "0.3"])

    def test_focus_table_by_yaw(self):
        import yaml
        # synthetic mocap: popillia yaw = 0 at step 0, +10 deg per step; table entries per pose at those yaws
        tbl = os.path.join(tempfile.mkdtemp(prefix="ftab_"), "focus.yaml")
        entries = []
        for k, base in ((1, 0.10), (2, 0.30)):
            for yaw, dl in ((0.0, 0.0), (10.0, 0.05), (20.0, 0.10)):
                entries.append({"robot_pose": k, "dataset_pose": k - 1, "image": f"IMG_{k}_{int(yaw)}",
                                "yaw_deg": yaw, "lens": base + dl, "t": yaw})
        yaml.safe_dump({"table": entries}, open(tbl, "w"))
        writer, stats, answers = self.run_session(steps=2, poses=2, phone_port=5761, robot_port=5760, frames=3,
                                                  focus_mode="table", focus_table=tbl, focus_stack_step=0.01)
        rows = list(csv.DictReader(open(os.path.join(writer.dir, "frames.csv"))))
        self.assertEqual(len(rows), 12)
        centres = [(r["step"], r["pose"], r["focus_centre"], r["focus_source"]) for r in rows[::3]]
        self.assertEqual(centres, [("0", "1", "0.1", "table:IMG_1_0"), ("0", "2", "0.3", "table:IMG_2_0"),
                                   ("1", "1", "0.15", "table:IMG_1_10"), ("1", "2", "0.35", "table:IMG_2_10")])
        self.assertEqual([round(float(r["lens_position"]), 3) for r in rows[:3]], [0.09, 0.10, 0.11])
        events = [json.loads(l) for l in open(os.path.join(writer.dir, "session.jsonl"))]
        ft = [e for e in events if e["event"] == "focus_table"]
        self.assertEqual(len(ft), 4)
        self.assertLess(abs(ft[2]["yaw_diff_deg"]), 0.5)

    def test_enter_confirmations_without_mocap(self):
        # main lab test: no OptiTrack, Enter to start / after each turn, Enter (or a lens value) at every pose
        answers = ["enter",                       # step 0: insect placed
                   "enter", 0.25,  "enter",       # pose 1: shoot; pose 2: override the centre to 0.25 then shoot
                   "enter",                       # turn to step 1
                   "enter", "skip",               # pose 1 shoot, pose 2 skipped
                   "enter"]                       # end: insect turned back to step 0
        writer, stats, prompts = self.run_session(steps=2, poses=2, phone_port=5771, robot_port=5770, frames=3,
                                                  no_mocap=True, turntable_confirm="enter", pose_confirm=True,
                                                  focus_mode="pose", focus_stack_step=0.01, lens_position=[0.1, 0.3],
                                                  operator_answers=answers)
        self.assertEqual(stats["stops_ok"], 3)
        self.assertEqual(stats["skipped"], 1)
        self.assertEqual(len(prompts), 8)
        self.assertIn("TURN the insect to step 1", prompts[4]); self.assertIn("BACK to step 0", prompts[7])
        rows = list(csv.DictReader(open(os.path.join(writer.dir, "frames.csv"))))
        self.assertEqual(len(rows), 9)
        self.assertEqual([r["focus_centre"] for r in rows[::3]], ["0.1", "0.25", "0.1"])
        self.assertEqual(rows[3]["focus_source"], "operator")
        self.assertEqual(rows[0]["phone_jitter_mm"], "")                  # no mocap -> no jitter, no poses
        self.assertEqual(rows[0]["phone_x"], "")
        events = [json.loads(l) for l in open(os.path.join(writer.dir, "session.jsonl"))]
        self.assertEqual(sum(1 for e in events if e["event"] == "pose_confirmed"), 3)
        self.assertEqual([e["confirmed"] for e in events if e["event"] == "turntable"], ["enter", "enter"])

    def test_anchored_grid_contains_init_focus(self):
        writer, stats, answers = self.run_session(steps=1, poses=2, phone_port=5781, robot_port=5780, frames=50,
                                                  focus_mode="pose", focus_stack_step=0.0, focus_stack_start=0.0,
                                                  focus_stack_end=0.5, lens_position=[0.322, 0.012])
        rows = list(csv.DictReader(open(os.path.join(writer.dir, "frames.csv"))))
        self.assertEqual(len(rows), 100)
        for pose, centre in ((1, 0.322), (2, 0.012)):
            lens = [float(r["lens_position"]) for r in rows if r["pose"] == str(pose)]
            self.assertEqual(len(lens), 50)
            self.assertIn(centre, lens)                                          # exact
            steps = {round(lens[i + 1] - lens[i], 4) for i in range(49)}
            self.assertEqual(steps, {0.0102})                                     # uniform
            self.assertGreaterEqual(min(lens), 0.0); self.assertLess(max(lens), 0.5 + 0.0103)
        merged_ref = list(csv.DictReader(open(os.path.join(writer.dir, "frames.csv"))))
        self.assertEqual(rows[0]["focus_centre"], "0.322")

    def test_focus_table_fixed_yaw(self):
        import yaml
        tbl = os.path.join(tempfile.mkdtemp(prefix="ftab_"), "focus.yaml")
        entries = [{"robot_pose": 1, "dataset_pose": 0, "image": f"IMG_{int(y)}", "yaw_deg": y, "lens": l, "t": i}
                   for i, (y, l) in enumerate(((176.48, 0.33), (166.0, 0.30), (-174.0, 0.35)))]
        yaml.safe_dump({"table": entries}, open(tbl, "w"))
        writer, stats, answers = self.run_session(steps=2, poses=1, phone_port=5791, robot_port=5790, frames=3,
                                                  focus_mode="table", focus_table=tbl, focus_table_yaw=176.48,
                                                  focus_stack_step=0.01, no_mocap=True, turntable_confirm="enter")
        rows = list(csv.DictReader(open(os.path.join(writer.dir, "frames.csv"))))
        self.assertEqual([(r["focus_centre"], r["focus_source"]) for r in rows[::3]],
                         [("0.33", "table:IMG_176"), ("0.33", "table:IMG_176")])   # both steps: the 176.48 entry

    def test_shoot_initial_pose(self):
        writer, stats, answers = self.run_session(steps=1, poses=2, phone_port=5801, robot_port=5800, frames=3,
                                                  shoot_initial_pose=True, lens_position_initial=0.33,
                                                  focus_mode="pose", focus_stack_step=0.01, lens_position=[0.1, 0.2])
        self.assertEqual(stats["stops_ok"], 3)                                   # initial pose + 2 poses
        rows = list(csv.DictReader(open(os.path.join(writer.dir, "frames.csv"))))
        self.assertEqual([(r["pose"], r["focus_centre"]) for r in rows[::3]], [("0", "0.33"), ("1", "0.1"), ("2", "0.2")])
        events = [json.loads(l)["event"] for l in open(os.path.join(writer.dir, "session.jsonl"))]
        self.assertLess(events.index("turntable"), events.index("stop_ok"))     # Enter first, then the stack at pose 0

    def test_resume_skips_captured_stops(self):
        tmp = tempfile.mkdtemp(prefix="capture_resume_")
        sdir = os.path.join(tmp, "dataset")
        w1, stats1, _ = self.run_session(steps=1, poses=2, phone_port=5811, robot_port=5810, frames=2,
                                         tmp=tmp, session_dir=sdir)
        self.assertEqual(stats1["stops_ok"], 2)
        with self.assertRaises(FileExistsError):
            SessionWriter(tmp, session_dir=sdir)                                  # no accidental overwrite
        w2, stats2, _ = self.run_session(steps=2, poses=2, phone_port=5812, robot_port=5813, frames=2,
                                         tmp=tmp, session_dir=sdir, resume=True)
        self.assertEqual(w2.dir, w1.dir)
        self.assertEqual(w2.session_id, w1.session_id)                           # same phone folder
        self.assertEqual(stats2["already_done"], 2)                              # step 0 skipped
        self.assertEqual(stats2["stops_ok"], 2)                                  # step 1 captured
        rows = list(csv.DictReader(open(os.path.join(sdir, "frames.csv"))))
        self.assertEqual([(r["step"], r["pose"], r["stop"]) for r in rows[::2]],
                         [("0", "1", "0"), ("0", "2", "1"), ("1", "1", "2"), ("1", "2", "3")])   # stop ids continue
        self.assertEqual(sorted(os.listdir(os.path.join(sdir, "images"))), ["s00_p1", "s00_p2", "s01_p1", "s01_p2"])
        starts = [json.loads(l) for l in open(os.path.join(sdir, "session.jsonl")) if '"session_start"' in l]
        self.assertEqual([e["resumed"] for e in starts], [False, True])
        turns = [json.loads(l) for l in open(os.path.join(sdir, "session.jsonl")) if '"turntable"' in l]
        self.assertAlmostEqual(turns[-1]["target_yaw"] - turns[0]["target_yaw"], 10.0, places=3)   # yaw0 restored

    def test_exposure_check(self):
        try:
            import cv2  # noqa: F401
        except ImportError:
            self.skipTest("opencv not installed")
        answers = ["enter",            # step 0: insect placed
                   1.0,                # exposure check 1: apply +1 EV
                   "keep"]             # exposure check 2: accept
        writer, stats, prompts = self.run_session(steps=1, poses=1, phone_port=5821, robot_port=5820, frames=3,
                                                  real_frames=True, no_mocap=True, turntable_confirm="enter",
                                                  exposure_check=True, operator_answers=answers)
        self.assertEqual(stats["stops_ok"], 1)
        self.assertEqual(len(prompts), 3)
        self.assertIn("EV to apply", prompts[1])
        events = [json.loads(l) for l in open(os.path.join(writer.dir, "session.jsonl"))]
        checks = [e for e in events if e["event"] == "exposure_check"]
        self.assertEqual(len(checks), 2)
        self.assertEqual(len(checks[0]["rows"]), 7)                          # EV 0 .. +3 in 0.5 steps
        applied = [e for e in events if e["event"] == "exposure_applied"]
        self.assertEqual(len(applied), 1)
        self.assertAlmostEqual(applied[0]["exposure"]["duration_s"], 0.02, places=6)   # 0.01 s * 2^1
        self.assertEqual(len([f for f in os.listdir(os.path.join(writer.dir, "exposure_check")) if f.endswith(".png")]), 2)
        rows = list(csv.DictReader(open(os.path.join(writer.dir, "frames.csv"))))
        self.assertEqual(len(rows), 3)                                            # test frames are not dataset frames

    def test_thermal_wait_polls_status(self):
        # after a "serious" stack the coordinator idles until the phone's status says fair (or thermal_wait_s)
        import time as _t
        c = Coordinator.__new__(Coordinator)
        c.plan = Plan(thermal_wait_s=20.0); c.stop_id = 7; c.log = lambda *a: None
        events = []; c.writer = type("W", (), {"event": lambda self, k, **d: events.append(dict(d, event=k))})()
        answers = ["serious", "serious", "fair"]                                            # status polled before the stack
        c.phone = type("P", (), {"status": lambda self: {"thermal": answers.pop(0)}})()
        c._thermal_last = "nominal"
        t0 = _t.time(); c._thermal_wait(); dt = _t.time() - t0
        waits = [e for e in events if e["event"] == "thermal_wait"]
        self.assertEqual((waits[0]["before"], waits[0]["after"]), ("serious", "fair")); self.assertEqual(c._thermal_last, "fair")
        self.assertGreaterEqual(dt, 9.0); self.assertLess(dt, 12.0)                         # 2 x 5 s, then fair
        events.clear(); c.phone = type("P", (), {"status": lambda self: {"thermal": "fair"}})()
        c._thermal_wait(); self.assertEqual([e["event"] for e in events], ["phone_status"])  # cool phone: no wait
        c.plan.thermal_wait_s = 6.0; c.phone = type("P", (), {"status": lambda self: {"thermal": "critical"}})()
        t0 = _t.time(); c._thermal_wait(); self.assertLess(_t.time() - t0, 12.0)             # capped by thermal_wait_s
        self.assertEqual(events[-1]["after"], "critical")

    def test_settle_after_move(self):
        writer, stats, _ = self.run_session(steps=1, poses=2, phone_port=5831, robot_port=5830, frames=1,
                                            settle_after_move_s=0.8)
        self.assertEqual(stats["stops_ok"], 2)
        events = [json.loads(l) for l in open(os.path.join(writer.dir, "session.jsonl"))]
        reached = {e["pose"]: e["t"] for e in events if e["event"] == "robot_reached"}
        stills = {e["stop"]: e["t"] for e in events if e["event"] == "still"}     # the gate runs after the settle
        for pose, stop in ((1, 0), (2, 1)):
            self.assertGreaterEqual(stills[stop] - reached[pose], 0.8 - 0.05)
        self.assertEqual(sum(1 for e in events if e["event"] == "settled"), 2)

    def test_pause_file_and_resume(self):
        # a PAUSE file in the session folder pauses at the first checkpoint; Enter resumes; the file is removed
        tmp = tempfile.mkdtemp(prefix="capture_pause_")
        sdir = os.path.join(tmp, "dataset")
        os.makedirs(sdir)
        open(os.path.join(sdir, "PAUSE"), "w").close()
        writer, stats, prompts = self.run_session(steps=1, poses=2, phone_port=5841, robot_port=5840, frames=1,
                                                  tmp=tmp, session_dir=sdir, operator_answers=["enter"])
        self.assertEqual(stats["stops_ok"], 2)
        self.assertEqual(len(prompts), 1)
        self.assertIn("Paused", prompts[0])
        self.assertFalse(os.path.exists(os.path.join(sdir, "PAUSE")))
        events = [json.loads(l)["event"] for l in open(os.path.join(sdir, "session.jsonl"))]
        self.assertEqual(events.count("paused"), 1); self.assertEqual(events.count("resumed"), 1)
        self.assertLess(events.index("paused"), events.index("stop_ok"))

    def test_retry_on_phone_failure(self):
        writer, stats, answers = self.run_session(steps=1, poses=4, fail_every=3, phone_port=5711, robot_port=5710)
        rows = list(csv.DictReader(open(os.path.join(writer.dir, "frames.csv"))))
        self.assertEqual(len(rows), 4)           # every failure was retried successfully
        self.assertGreaterEqual(stats["retries"], 1)
        self.assertEqual(stats["skipped"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
