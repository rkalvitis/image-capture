"""Session (= dataset) folder writer.

Layout (2026-09-15, "stacks"):
    <dir>/
      plan.json, plan.yaml            the plan used (+ a copy of the YAML)
      session.jsonl                   every event (robot, turntable, focus, stops, merges, exposure check)
      frames.csv                      one row per RAW frame (all of them, also after the DNGs are deleted)
      stacks.csv, alignment.csv       one row per merged image / per aligned frame (written by stack_merge)
      calibrations/, mocap/, robot/   copied calibrations, the mocap rosbag, the robot pose files + reached joints
      exposure_check/                 the test frames of the session-start exposure check
      images/<image>/stack/<frame>.dng + .json      raw frames + sidecars (DNGs deleted after the merge)
      images/<image>/stack/mocap.csv                mocap samples of both bodies over the capture window
      images/<image>/stack/stack.json               merge summary
      images/<image>/<image>.tif (+ .jpg)           the merged image;  <image> = s<step:02d>_p<pose>

Body poses in frames.csv are the mocap rigid bodies in the world frame (mean
over the capture interval). cam_* / quat_xyzw are T_insect<-cam (camera in
the popillia frame, after the hand-eye transform) — the same meaning as in
camera_poses.csv of the optitrack repo — and are filled only when the plan
names a rig_transforms.yaml; colmap_export.py can also compute them later.

Resume: `SessionWriter(session_dir=..., resume=True)` re-opens an existing
folder: the phone session id, the finished (step, pose) pairs and the last
stop id are read back from session.jsonl / frames.csv.
"""
import csv
import datetime
import json
import os
import shutil
import threading
import time

FRAME_COLUMNS = ["image", "file", "stop", "step", "pose", "focus_index", "lens_position", "focus_centre", "focus_source",
                 "fx", "fy", "cx", "cy", "capture_time", "mocap_t0", "mocap_t1",
                 "phone_jitter_mm", "pop_jitter_mm",
                 "phone_x", "phone_y", "phone_z", "phone_quat_xyzw",
                 "pop_x", "pop_y", "pop_z", "pop_quat_xyzw", "popillia_yaw_deg",
                 "cam_x", "cam_y", "cam_z", "quat_xyzw", "phone_world"]
ROBOT_COLUMNS = ["t", "step", "pose", "stop", "image", "j1", "j2", "j3", "j4", "j5", "j6", "j7"]
MOCAP_COLUMNS = ["body", "t", "x", "y", "z", "qx", "qy", "qz", "qw"]
SUBDIRS = ("images", "calibrations", "mocap", "robot", "exposure_check")


def stack_name(step, pose):
    return f"s{int(step):02d}_p{int(pose)}"


class SessionWriter:
    def __init__(self, output_root=None, name=None, when=None, session_dir=None, resume=False):
        when = when or datetime.datetime.now()
        self.session_id = when.strftime("%Y%m%d_%H%M%S")
        self.dir = os.path.abspath(session_dir) if session_dir else os.path.join(output_root, f"phone_{self.session_id}")
        self.name = name
        self.images_dir = os.path.join(self.dir, "images")
        self.resumed = False
        self.done = {}              # (step, pose) -> image name (stops already captured)
        self.max_stop = -1
        self.yaw0 = None
        self.exposure = None        # custom exposure applied by the exposure check of a resumed session
        self._lock = threading.Lock()           # events/rows come from the capture loop AND the finalize thread
        self._frames_path = os.path.join(self.dir, "frames.csv")
        existing = os.path.exists(self._frames_path) or os.path.exists(os.path.join(self.dir, "session.jsonl"))
        if existing and not resume:
            raise FileExistsError(f"{self.dir} already holds a session — start with --resume to continue it")
        for d in SUBDIRS:
            os.makedirs(os.path.join(self.dir, d), exist_ok=True)
        if existing:
            self._load_existing()
        self._events = open(os.path.join(self.dir, "session.jsonl"), "a")
        new = not os.path.exists(self._frames_path)
        self._frames = open(self._frames_path, "a", newline="")
        self._csv = csv.DictWriter(self._frames, fieldnames=FRAME_COLUMNS)
        if new:
            self._csv.writeheader()
        rp = os.path.join(self.dir, "robot", "robot_poses.csv")
        new_r = not os.path.exists(rp)
        self._robot = open(rp, "a", newline="")
        self._robot_csv = csv.DictWriter(self._robot, fieldnames=ROBOT_COLUMNS)
        if new_r:
            self._robot_csv.writeheader()

    def _load_existing(self):
        self.resumed = True
        ev_path = os.path.join(self.dir, "session.jsonl")
        if os.path.exists(ev_path):
            with open(ev_path) as f:
                for line in f:
                    try:
                        e = json.loads(line)
                    except ValueError:
                        continue
                    if e.get("event") == "session_start" and e.get("session"):
                        self.session_id = e["session"]                  # keep the phone's folder name
                    if e.get("event") == "exposure_applied" and e.get("exposure"):
                        self.exposure = e["exposure"]
                    if e.get("event") == "turntable" and e.get("step") == 0 and self.yaw0 is None:
                        y = e.get("target_yaw", e.get("yaw"))
                        if y is not None:
                            self.yaw0 = float(y)
        if os.path.exists(self._frames_path):
            with open(self._frames_path) as f:
                for r in csv.DictReader(f):
                    try:
                        key = (int(r["step"]), int(r["pose"]))
                        self.done[key] = r.get("image") or stack_name(*key)
                        self.max_stop = max(self.max_stop, int(r["stop"]))
                    except (KeyError, ValueError):
                        continue

    # -- per stop -------------------------------------------------------
    @staticmethod
    def stack_name(step, pose):
        return stack_name(step, pose)

    def stack_dir(self, image):
        d = os.path.join(self.images_dir, image, "stack")
        os.makedirs(d, exist_ok=True)
        return d

    def event(self, kind, **data):
        rec = {"t": time.time(), "event": kind}
        rec.update(data)
        with self._lock:
            self._events.write(json.dumps(rec, default=str) + "\n")
            self._events.flush()

    def frame(self, row):
        with self._lock:
            self._csv.writerow({k: row.get(k, "") for k in FRAME_COLUMNS})
            self._frames.flush()
        try:
            self.done[(int(row["step"]), int(row["pose"]))] = row.get("image", "")
            self.max_stop = max(self.max_stop, int(row["stop"]))
        except (KeyError, ValueError, TypeError):
            pass

    def robot_pose(self, step, pose, stop, image, joints):
        row = {"t": time.time(), "step": step, "pose": pose, "stop": stop, "image": image}
        for i, j in enumerate(list(joints or [])[:7], 1):
            row[f"j{i}"] = j
        with self._lock:
            self._robot_csv.writerow({c: row.get(c, "") for c in ROBOT_COLUMNS})
            self._robot.flush()

    def mocap_samples(self, image, samples):
        """samples: {body: [(t, pos[3], quat_xyzw[4]), ...]} -> images/<image>/stack/mocap.csv"""
        path = os.path.join(self.stack_dir(image), "mocap.csv")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(MOCAP_COLUMNS)
            for body, lst in samples.items():
                for t, p, q in lst:
                    w.writerow([body, f"{t:.6f}", f"{p[0]:.6f}", f"{p[1]:.6f}", f"{p[2]:.6f}",
                                f"{q[0]:.6f}", f"{q[1]:.6f}", f"{q[2]:.6f}", f"{q[3]:.6f}"])
        return path

    # -- session-level files -------------------------------------------
    def save_plan(self, plan_dict, plan_path=None):
        with open(os.path.join(self.dir, "plan.json"), "w") as f:
            json.dump(plan_dict, f, indent=2)
        if plan_path and os.path.exists(plan_path):
            shutil.copy2(plan_path, os.path.join(self.dir, "plan.yaml"))

    def copy_in(self, src, subdir, name=None):
        """Copy a file (or a folder) into <dir>/<subdir>/; returns the destination or None."""
        if not src or not os.path.exists(src):
            return None
        dst = os.path.join(self.dir, subdir, name or os.path.basename(src))
        if os.path.isdir(src):
            shutil.copytree(src, dst, dirs_exist_ok=True)
        else:
            shutil.copy2(src, dst)
        return dst

    def close(self):
        self._events.close()
        self._frames.close()
        self._robot.close()
