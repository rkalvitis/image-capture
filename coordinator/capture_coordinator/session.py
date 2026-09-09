"""Session folder writer: session.jsonl (events) + frames.csv."""
import csv
import datetime
import json
import os
import time

FRAME_COLUMNS = ["file", "stop", "step", "pose", "focus_index", "lens_position",
                 "fx", "fy", "cx", "cy", "capture_time", "mocap_t0", "mocap_t1",
                 "phone_jitter_mm", "pop_jitter_mm",
                 "cam_x", "cam_y", "cam_z", "quat_xyzw", "popillia_yaw_deg",
                 "pop_x", "pop_y", "pop_z", "pop_quat_xyzw", "phone_world"]


class SessionWriter:
    def __init__(self, output_root, name=None, when=None):
        when = when or datetime.datetime.now()
        self.session_id = when.strftime("%Y%m%d_%H%M%S")
        self.dir = os.path.join(output_root, f"phone_{self.session_id}")
        self.images_dir = os.path.join(self.dir, "images")
        self.sidecar_dir = os.path.join(self.dir, "sidecars")
        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.sidecar_dir, exist_ok=True)
        self._events = open(os.path.join(self.dir, "session.jsonl"), "a")
        self._frames_path = os.path.join(self.dir, "frames.csv")
        new = not os.path.exists(self._frames_path)
        self._frames = open(self._frames_path, "a", newline="")
        self._csv = csv.DictWriter(self._frames, fieldnames=FRAME_COLUMNS)
        if new:
            self._csv.writeheader()
        self.name = name

    def event(self, kind, **data):
        rec = {"t": time.time(), "event": kind}
        rec.update(data)
        self._events.write(json.dumps(rec, default=str) + "\n")
        self._events.flush()

    def frame(self, row):
        self._csv.writerow({k: row.get(k, "") for k in FRAME_COLUMNS})
        self._frames.flush()

    def save_plan(self, plan_dict):
        with open(os.path.join(self.dir, "plan.json"), "w") as f:
            json.dump(plan_dict, f, indent=2)

    def close(self):
        self._events.close()
        self._frames.close()
