"""Upload worker (rclone -> OneDrive) against tests/fake_rclone.py: copy + check, deletion of
the local and the phone copies only after the verified upload, retries, session-file sync,
poses/ collection, plan validation.
    python tests/test_upload.py
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

from capture_coordinator.plan import Plan                       # noqa: E402
from capture_coordinator.poses_collect import collect_poses     # noqa: E402
from capture_coordinator.pull import LocalDirPuller             # noqa: E402
from capture_coordinator.upload_worker import UploadWorker, apply_rules, stop_specs   # noqa: E402

FAKE = os.path.join(HERE, "fake_rclone.py")


def make_session(tmp, images=("s00_p1", "s00_p2"), n_frames=3, master=True):
    """A session folder + a fake phone Documents dir with the same DNGs."""
    sdir = os.path.join(tmp, "session")
    phone = os.path.join(tmp, "phone_docs", "20260929_120000")
    os.makedirs(phone)
    names = {}
    for stop, image in enumerate(images):
        st = os.path.join(sdir, "images", image, "stack")
        os.makedirs(st)
        names[stop] = []
        for k in range(n_frames):
            base = f"stop{stop:04d}_k{k:02d}"
            data = os.urandom(3000 + 100 * k)
            for d in (st, phone):
                open(os.path.join(d, base + ".dng"), "wb").write(data)
                json.dump({"file": base + ".dng", "k": k}, open(os.path.join(d, base + ".json"), "w"))
            names[stop] += [base + ".dng", base + ".json"]
        open(os.path.join(st, "mocap.csv"), "w").write("body,t\nphone,1\n")
        json.dump({"image": image}, open(os.path.join(st, "stack.json"), "w"))
        if master:
            open(os.path.join(sdir, "images", image, image + ".tif"), "wb").write(os.urandom(20000))
            open(os.path.join(sdir, "images", image, image + ".jpg"), "wb").write(os.urandom(2000))
    for sub in ("calibrations", "robot", "mocap", "exposure_check"):
        os.makedirs(os.path.join(sdir, sub), exist_ok=True)
    open(os.path.join(sdir, "frames.csv"), "w").write("image,file\n")
    open(os.path.join(sdir, "session.jsonl"), "w").write("")
    open(os.path.join(sdir, "PAUSE"), "w").write("")
    return sdir, os.path.join(tmp, "phone_docs"), names


def listing(root):
    out = []
    for dp, _, fs in os.walk(root):
        for f in fs:
            out.append(os.path.relpath(os.path.join(dp, f), root))
    return sorted(out)


class UploadWorkerTest(unittest.TestCase):
    def worker(self, sdir, remote, phone_root, **kw):
        events = []
        kw.setdefault("what", "all")
        w = UploadWorker(sdir, "fakeremote:" + remote, puller=LocalDirPuller(phone_root), phone_session="20260929_120000",
                         rclone=FAKE, log=lambda *a: None, event=lambda kind, **d: events.append(dict(d, event=kind)),
                         retry_wait_s=0.05, **kw)
        return w, events

    def test_filters(self):
        self.assertEqual([r for _, r, _ in stop_specs("s00_p1", "masters")], ["stacked", "previews"])
        self.assertEqual([r for _, r, _ in stop_specs("s00_p1", "raw")], ["raw"])
        self.assertEqual([r for _, r, _ in stop_specs("s00_p1", "all")], ["raw", "stacked", "previews"])
        raw_rules = stop_specs("s00_p1", "all")[0][2]
        self.assertTrue(apply_rules("stop0000_k00.dng", raw_rules)); self.assertTrue(apply_rules("stop0000_k00.json", raw_rules))
        self.assertFalse(apply_rules("stack.json", raw_rules)); self.assertFalse(apply_rules("mocap.csv", raw_rules))
        tif_rules = stop_specs("s00_p1", "all")[1][2]
        self.assertTrue(apply_rules("s00_p1.tif", tif_rules)); self.assertFalse(apply_rules(os.path.join("stack", "x.dng"), tif_rules))

    def test_upload_all_deletes_local_and_phone(self):
        tmp = tempfile.mkdtemp(prefix="upload_")
        sdir, phone_root, names = make_session(tmp)
        remote = os.path.join(tmp, "remote")
        w, events = self.worker(sdir, remote, phone_root, delete_local=True, delete_phone=True)
        self.assertTrue(w.check_remote())
        w.submit(0, "s00_p1", names[0], merged_ok=True)
        w.submit(1, "s00_p2", names[1], merged_ok=False)          # merge failed: upload, delete nothing
        self.assertTrue(w.drain(timeout=30))
        w.close()
        rem = listing(remote)
        self.assertIn(os.path.join("stacked", "s00_p1.tif"), rem)
        self.assertIn(os.path.join("previews", "s00_p1.jpg"), rem)
        self.assertIn(os.path.join("stack_meta", "s00_p1.json"), rem)
        self.assertIn(os.path.join("raw", "stop0000_k00.dng"), rem); self.assertIn(os.path.join("raw", "stop0000_k00.json"), rem)
        self.assertNotIn(os.path.join("raw", "mocap.csv"), rem); self.assertNotIn(os.path.join("raw", "stack.json"), rem)
        self.assertEqual(len([r for r in rem if r.startswith("raw")]), 2 * 3 * 2)            # both stops, dng + json
        self.assertEqual([r for r in rem if r.startswith("images")], [])
        # local: image files of stop 0 gone, sidecars + mocap.csv + stack.json kept; stop 1 untouched
        self.assertFalse(os.path.exists(os.path.join(sdir, "images", "s00_p1")))          # whole stop folder gone
        loc1 = listing(os.path.join(sdir, "images", "s00_p2"))
        self.assertEqual(len([f for f in loc1 if f.endswith(".dng")]), 3)
        self.assertTrue(os.path.exists(os.path.join(sdir, "images", "s00_p2", "s00_p2.tif")))
        # phone: stop 0 files removed, stop 1 kept
        ph = sorted(os.listdir(os.path.join(phone_root, "20260929_120000")))
        self.assertEqual([f for f in ph if f.startswith("stop0000")], [])
        self.assertEqual(len([f for f in ph if f.startswith("stop0001")]), 6)
        up = [e for e in events if e["event"] == "stop_uploaded"]
        self.assertEqual([e["stop"] for e in up], [0, 1])
        self.assertEqual(up[0]["deleted_local"], 10); self.assertEqual(up[0]["deleted_phone"], 6)   # 3 dng + 3 json + tif + jpg + mocap.csv + stack.json
        self.assertEqual(up[1]["deleted_local"], 0); self.assertEqual(up[1]["deleted_phone"], 0)
        self.assertIn("merge failed", up[1]["kept_reason"])
        self.assertEqual([e["event"] for e in events if e["event"] == "phone_deleted"], ["phone_deleted"])
        s = w.summary()
        self.assertEqual((s["uploaded_ok"], s["uploaded_failed"], s["deleted_local"], s["deleted_phone"]), (2, 0, 10, 6))

    def test_masters_only_keeps_raw_and_phone(self):
        tmp = tempfile.mkdtemp(prefix="upload_")
        sdir, phone_root, names = make_session(tmp, images=("s00_p1",))
        remote = os.path.join(tmp, "remote")
        w, events = self.worker(sdir, remote, phone_root, what="masters", delete_local=True, delete_phone=False)
        w.submit(0, "s00_p1", names[0], merged_ok=True)
        self.assertTrue(w.drain(timeout=30)); w.close()
        rem = listing(remote)
        self.assertEqual([r for r in rem if r.endswith(".dng")], [])
        self.assertIn(os.path.join("stacked", "s00_p1.tif"), rem); self.assertIn(os.path.join("stack_meta", "s00_p1.json"), rem)
        self.assertEqual([r for r in rem if r.startswith("raw")], [])                        # masters only: no sidecars either
        loc = listing(os.path.join(sdir, "images", "s00_p1"))
        self.assertEqual(len([f for f in loc if f.endswith(".dng")]), 3)          # raw stays (the merge deletes it)
        self.assertNotIn("s00_p1.tif", loc)
        self.assertEqual(len(os.listdir(os.path.join(phone_root, "20260929_120000"))), 6)

    def test_delete_local_raw_keeps_the_masters(self):
        tmp = tempfile.mkdtemp(prefix="upload_")
        sdir, phone_root, names = make_session(tmp, images=("s00_p1",))
        remote = os.path.join(tmp, "remote")
        w, events = self.worker(sdir, remote, phone_root, what="all", delete_local="raw", delete_phone=True)
        w.submit(0, "s00_p1", names[0], merged_ok=True)
        self.assertTrue(w.drain(timeout=30)); w.close()
        loc = listing(os.path.join(sdir, "images", "s00_p1"))
        self.assertEqual([f for f in loc if f.endswith(".dng")], [])                                # raw gone
        self.assertIn("s00_p1.tif", loc); self.assertIn("s00_p1.jpg", loc)                           # masters kept
        self.assertEqual(len([f for f in loc if f.endswith(".json")]), 4)                            # sidecars + stack.json kept
        self.assertIn(os.path.join("stack", "mocap.csv"), loc)
        self.assertEqual(os.listdir(os.path.join(phone_root, "20260929_120000")), [])               # phone emptied
        self.assertIn(os.path.join("stacked", "s00_p1.tif"), listing(remote))
        with self.assertRaises(ValueError):
            Plan(upload_remote="onedrive:x", upload_what="masters", upload_delete_local="raw").validate_upload()
        with self.assertRaises(ValueError):
            Plan(upload_remote="onedrive:x", upload_what="all", upload_delete_local="raw", upload_delete_session=True).validate_upload()
        Plan(upload_remote="onedrive:x", upload_what="all", upload_delete_local="raw", merge_inline=True,
             merge_delete_raw=False, upload_delete_phone=True).validate_upload()

    def test_raw_only_upload_without_merge(self):
        tmp = tempfile.mkdtemp(prefix="upload_")
        sdir, phone_root, names = make_session(tmp, images=("s00_p1",), master=False)
        remote = os.path.join(tmp, "remote")
        w, events = self.worker(sdir, remote, phone_root, what="raw", delete_local=True, delete_phone=True)
        w.submit(0, "s00_p1", names[0], merged_ok=None)
        self.assertTrue(w.drain(timeout=30)); w.close()
        self.assertEqual(len([r for r in listing(os.path.join(remote, "raw")) if r.endswith(".dng")]), 3)
        self.assertEqual([f for f in listing(os.path.join(sdir, "images")) if f.endswith((".dng", ".json")) and not f.endswith("stack.json")], [])   # raw: sidecars too
        self.assertEqual(os.listdir(os.path.join(phone_root, "20260929_120000")), [])

    def test_check_failure_keeps_everything(self):
        tmp = tempfile.mkdtemp(prefix="upload_")
        sdir, phone_root, names = make_session(tmp, images=("s00_p1",))
        remote = os.path.join(tmp, "remote")
        os.environ["FAKE_RCLONE_DROP"] = "stop0000_k01.dng"
        try:
            w, events = self.worker(sdir, remote, phone_root, delete_local=True, delete_phone=True, retries=2)
            w.submit(0, "s00_p1", names[0], merged_ok=True)
            self.assertTrue(w.drain(timeout=30)); w.close()
        finally:
            del os.environ["FAKE_RCLONE_DROP"]
        kinds = [e["event"] for e in events]
        self.assertEqual(kinds.count("stop_upload_retry"), 2)
        self.assertIn("stop_upload_failed", kinds); self.assertNotIn("stop_uploaded", kinds)
        self.assertIn("check raw/ failed", [e for e in events if e["event"] == "stop_upload_failed"][0]["reason"])
        self.assertEqual(len([f for f in listing(os.path.join(sdir, "images")) if f.endswith(".dng")]), 3)
        self.assertEqual(len(os.listdir(os.path.join(phone_root, "20260929_120000"))), 6)
        self.assertEqual(w.summary()["uploaded_failed"], 1)

    def test_copy_retry_after_transient_error(self):
        tmp = tempfile.mkdtemp(prefix="upload_")
        sdir, phone_root, names = make_session(tmp, images=("s00_p1",))
        remote = os.path.join(tmp, "remote")
        os.environ["FAKE_RCLONE_FAIL_COPY_ONCE"] = os.path.join(remote, "raw")
        try:
            w, events = self.worker(sdir, remote, phone_root, delete_local=True)
            w.submit(0, "s00_p1", names[0], merged_ok=True)
            self.assertTrue(w.drain(timeout=30)); w.close()
        finally:
            del os.environ["FAKE_RCLONE_FAIL_COPY_ONCE"]
        kinds = [e["event"] for e in events]
        self.assertEqual(kinds.count("stop_upload_retry"), 1)
        self.assertIn("stop_uploaded", kinds)

    def test_session_files_and_poses_sync(self):
        tmp = tempfile.mkdtemp(prefix="upload_")
        sdir, phone_root, names = make_session(tmp, images=("s00_p1",))
        remote = os.path.join(tmp, "remote")
        # robot run files "written by the robot script" on this PC
        runs = os.path.join(tmp, "robot_scripts")
        os.makedirs(runs)
        open(os.path.join(runs, "manual_poses_20260929_120100.csv"), "w").write("a,b\n1,2\n")
        open(os.path.join(runs, "joint_states_20260929_120000.db"), "wb").write(b"sqlite")
        old = os.path.join(runs, "manual_poses_20250101_000000.csv")
        open(old, "w").write("old\n"); os.utime(old, (time.time() - 86400 * 30,) * 2)
        open(os.path.join(sdir, "robot", "robot_poses.csv"), "w").write(
            "t,step,pose,stop,image,j1,j2,j3,j4,j5,j6,j7\n1,0,1,0,s00_p1,0.1,0.2,0.3,0.4,0.5,0.6,0.7\n")
        open(os.path.join(sdir, "frames.csv"), "w").write(
            "image,file,stop,step,pose,focus_index,lens_position,focus_centre,focus_source,fx,fy,cx,cy,capture_time,mocap_t0,mocap_t1,"
            "phone_jitter_mm,pop_jitter_mm,phone_x,phone_y,phone_z,phone_quat_xyzw,pop_x,pop_y,pop_z,pop_quat_xyzw,popillia_yaw_deg,"
            "cam_x,cam_y,cam_z,quat_xyzw,phone_world\n"
            "s00_p1,stop0000_k00.dng,0,0,1,0,0.4,0.4,range,1,1,1,1,10.0,9.9,10.1,0,0,1,2,3,0 0 0 1,4,5,6,0 0 0 1,12.5,,,,,\n"
            "s00_p1,stop0000_k01.dng,0,0,1,1,0.5,0.4,range,1,1,1,1,11.0,9.9,10.1,0,0,1,2,3,0 0 0 1,4,5,6,0 0 0 1,12.5,,,,,\n")
        open(os.path.join(sdir, "session.jsonl"), "w").write(
            json.dumps({"t": 5.0, "event": "turntable", "step": 0, "yaw": 12.5, "target_yaw": 12.5, "confirmed": "enter",
                        "drive": "fake", "commanded_deg": 0.0, "pos_deg": 0.0}) + "\n")
        open(os.path.join(sdir, "mocap", "bag.db3"), "wb").write(b"bag")
        res = collect_poses(sdir, [os.path.join(runs, "manual_poses_*.csv"), os.path.join(runs, "joint_states_*.db")],
                            since=time.time() - 600)
        self.assertEqual((res["stops"], res["mocap_files"], res["robot_runs"]), (1, 1, 2))
        stops = list(csv.DictReader(open(os.path.join(sdir, "poses", "stops.csv"))))
        self.assertEqual(len(stops), 1)
        self.assertEqual((stops[0]["image"], stops[0]["n_frames"], stops[0]["j7"], stops[0]["pop_x"]), ("s00_p1", "2", "0.7", "4"))
        self.assertEqual(stops[0]["capture_time_ref"], "10.0")                     # the focus-centre frame
        self.assertEqual(stops[0]["platter_commanded_deg"], "0.0")
        self.assertEqual(sorted(os.listdir(os.path.join(sdir, "poses", "robot_runs"))),
                         ["joint_states_20260929_120000.db", "manual_poses_20260929_120100.csv"])   # the old file skipped
        self.assertTrue(os.path.exists(os.path.join(sdir, "poses", "mocap", "s00_p1.csv")))
        turns = list(csv.DictReader(open(os.path.join(sdir, "poses", "turntable.csv"))))
        self.assertEqual(turns[0]["confirmed"], "enter")
        w, events = self.worker(sdir, remote, phone_root)
        self.assertTrue(w.upload_session_files(final=True, check=True)); w.close()
        rem = listing(remote)
        self.assertIn("frames.csv", rem); self.assertIn("session.jsonl", rem); self.assertNotIn("PAUSE", rem)
        self.assertIn(os.path.join("poses", "stops.csv"), rem)
        self.assertIn(os.path.join("poses", "robot_runs", "manual_poses_20260929_120100.csv"), rem)
        self.assertIn(os.path.join("poses", "mocap", "s00_p1.csv"), rem)
        self.assertIn(os.path.join("poses", "mocap_bag", "bag.db3"), rem)          # mocap/ -> poses/mocap_bag/
        self.assertIn(os.path.join("robot", "robot_poses.csv"), rem)
        self.assertEqual([r for r in rem if r.startswith(("images", "raw", "stacked"))], [])   # stops are not part of this sync

    def test_check_remote_failure(self):
        tmp = tempfile.mkdtemp(prefix="upload_")
        w = UploadWorker(tmp, "fakeremote:" + os.path.join(tmp, "r"), rclone="/nonexistent/rclone", log=lambda *a: None)
        with self.assertRaises(RuntimeError):
            w.check_remote()
        w.close()

    def test_plan_validation(self):
        p = Plan(upload_remote="onedrive:x", upload_what="masters", upload_delete_phone=True)
        with self.assertRaises(ValueError):
            p.validate_upload()
        p = Plan(upload_remote="onedrive:x", upload_what="all", merge_inline=True, merge_delete_raw=True)
        with self.assertRaises(ValueError):
            p.validate_upload()
        p = Plan(upload_remote="no-colon")
        with self.assertRaises(ValueError):
            p.validate_upload()
        Plan(upload_remote="onedrive:x", upload_what="all", merge_inline=True, merge_delete_raw=False,
             upload_delete_local=True, upload_delete_phone=True).validate_upload()
        Plan(upload_remote="", upload_delete_phone=True).validate_upload()        # no upload: keys ignored

    def test_rotations_split_evenly(self):
        p = Plan(turntable_steps_per_rev=200, turntable_microsteps=16)
        pr = p.set_turntable_steps(36)
        self.assertAlmostEqual(p.turntable_step_deg, 10.0)
        self.assertFalse(pr["exact"]); self.assertLess(pr["max_error_deg"], 0.1125 / 2 + 1e-9)
        pr = p.set_turntable_steps(40)
        self.assertAlmostEqual(p.turntable_step_deg, 9.0); self.assertTrue(pr["exact"])
        p.set_turntable_steps(1)
        self.assertEqual((p.turntable_steps, p.turntable_step_deg), (1, 360.0))
        self.assertEqual(p.stops_planned(), 9)
        with self.assertRaises(ValueError):
            p.set_turntable_steps(0)

    def test_auto_ev_from_the_mac_test_run(self):
        from capture_coordinator.exposure import auto_ev
        st = {"frame_p99": 0.1342822164297104}                    # 2026-09-30 14:37, ISO 80 1/100 s: foam p99 0.40 sRGB
        ev = auto_ev(st)
        self.assertAlmostEqual(ev, 2.4, places=1)                  # -> ~1/19 s at ISO 80
        self.assertEqual(auto_ev({"frame_p99": 0.69}), 0.0)        # foam already at 0.85 sRGB
        self.assertEqual(auto_ev({"frame_p99": 0.0}), 0.0)
        self.assertEqual(auto_ev({"frame_p99": 0.001}), 4.0)       # clamped
        p = Plan(exposure_check_ev="auto"); self.assertEqual(p.exposure_check_ev, "auto")

    def test_fake_rclone_cli(self):
        r = subprocess.run([sys.executable, FAKE, "version"], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
