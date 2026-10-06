"""<session>/poses/ — every pose-related output of a session in ONE subfolder (user, 2026-09-29),
synced to <upload_remote>/poses/ by the upload worker (docs/ONEDRIVE.md):

    poses/stops.csv              one row per captured stop: step, robot pose, image, capture window,
                                 platter angle commanded / popillia yaw, phone + popillia body poses
                                 (world frame, mean over the capture window), cam_* if a rig file was
                                 given, the 7 robot joints reached — built from frames.csv,
                                 robot/robot_poses.csv and session.jsonl
    poses/robot_poses.csv        copy of robot/robot_poses.csv (joints reported by the robot per stop)
    poses/turntable.csv          every turntable event (step, commanded deg, yaw, confirmed how)
    poses/mocap/<image>.csv      the per-stop mocap samples (images/<image>/stack/mocap.csv) of both bodies
    poses/robot_runs/            files the ROBOT script writes on this PC, matched by the plan's
                                 `poses_collect` globs and not older than 1 h before the session start (the robot run and the joint recorder start first):
                                 manual_poses_<ts>.csv (panda_manual_poses.py run log: joints, end-effector
                                 pose, lens position per waypoint) and joint_states_<ts>.db
                                 (record_joint_states.py, /joint_states + pose events), from
                                 ~/Documents/repos/robo_arm on the Mac, the ROS workspace on the PC
    (poses/mocap_bag/ exists only in OneDrive: the upload worker sends <session>/mocap/ there at the end)

`collect_poses(session_dir, plan, since)` is cheap and idempotent (copies only changed files);
the coordinator runs it after every stop and at the end. Regenerate by hand:

    python -m capture_coordinator.poses_collect <session_dir> [--glob PATTERN ...]
"""
import csv
import glob
import json
import os
import shutil
import sys

STOP_COLUMNS = ["stop", "step", "pose", "image", "n_frames", "t0", "t1", "capture_time_ref",
                "platter_commanded_deg", "platter_reported_deg", "popillia_yaw_deg", "target_yaw_deg",
                "phone_x", "phone_y", "phone_z", "phone_quat_xyzw",
                "pop_x", "pop_y", "pop_z", "pop_quat_xyzw",
                "cam_x", "cam_y", "cam_z", "quat_xyzw",
                "j1", "j2", "j3", "j4", "j5", "j6", "j7", "focus_centre", "focus_source"]
TURN_COLUMNS = ["t", "step", "commanded_deg", "pos_deg", "yaw", "target_yaw", "confirmed", "drive", "accepted", "event"]


def _read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _events(path):
    out = []
    if not os.path.exists(path):
        return out
    with open(path) as f:
        for line in f:
            try:
                out.append(json.loads(line))
            except ValueError:
                continue
    return out


def _write_csv(path, columns, rows):
    tmp = path + ".part"
    with open(tmp, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=columns)
        w.writeheader()
        for r in rows:
            w.writerow({c: ("" if r.get(c) is None else r.get(c)) for c in columns})
    os.replace(tmp, path)


def _copy_if_newer(src, dst):
    try:
        if os.path.exists(dst) and os.path.getmtime(dst) >= os.path.getmtime(src) and os.path.getsize(dst) == os.path.getsize(src):
            return False
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(src, dst + ".part")
        os.replace(dst + ".part", dst)
        return True
    except OSError:
        return False


def build_stops(session_dir):
    from .session import read_frames_csv
    frames = read_frames_csv(os.path.join(session_dir, "frames.csv"))
    robot = {}
    for r in _read_csv(os.path.join(session_dir, "robot", "robot_poses.csv")):
        try:
            robot[int(r["stop"])] = r
        except (KeyError, ValueError):
            continue
    ev = _events(os.path.join(session_dir, "session.jsonl"))
    turns = {}
    for e in ev:
        if e.get("event") == "turntable" and e.get("step") is not None:
            turns[int(e["step"])] = e                       # the last event of a step wins (retry / accept)
    stops = {}
    for r in frames:
        try:
            stop = int(r["stop"])
        except (KeyError, ValueError):
            continue
        s = stops.setdefault(stop, {"stop": stop, "step": r.get("step"), "pose": r.get("pose"), "image": r.get("image"),
                                    "n_frames": 0, "t0": r.get("mocap_t0"), "t1": r.get("mocap_t1"),
                                    "focus_centre": r.get("focus_centre"), "focus_source": r.get("focus_source")})
        s["n_frames"] += 1
        for c in ("phone_x", "phone_y", "phone_z", "phone_quat_xyzw", "pop_x", "pop_y", "pop_z", "pop_quat_xyzw",
                  "cam_x", "cam_y", "cam_z", "quat_xyzw", "popillia_yaw_deg"):
            if r.get(c) not in (None, ""):
                s[c] = r[c]
        # the reference frame = the one at the focus centre (or the first)
        try:
            if abs(float(r.get("lens_position") or 0) - float(r.get("focus_centre") or -1)) < 1e-6 or "capture_time_ref" not in s:
                s["capture_time_ref"] = r.get("capture_time")
        except ValueError:
            pass
    rows = []
    for stop in sorted(stops):
        s = stops[stop]
        j = robot.get(stop, {})
        for i in range(1, 8):
            s[f"j{i}"] = j.get(f"j{i}", "")
        try:
            t = turns.get(int(s["step"]))
        except (TypeError, ValueError):
            t = None
        if t:
            s["platter_commanded_deg"] = t.get("commanded_deg")
            s["platter_reported_deg"] = t.get("pos_deg")
            s["target_yaw_deg"] = t.get("target_yaw")
            if s.get("popillia_yaw_deg") in (None, "") and t.get("yaw") is not None:
                s["popillia_yaw_deg"] = t.get("yaw")
        rows.append(s)
    turn_rows = []
    for e in ev:
        if e.get("event") in ("turntable", "turntable_return"):
            turn_rows.append({"t": e.get("t"), "step": e.get("step", 0 if e.get("event") == "turntable_return" else None),
                              "commanded_deg": e.get("commanded_deg", 0.0 if e.get("event") == "turntable_return" else None),
                              "pos_deg": e.get("pos_deg"), "yaw": e.get("yaw"), "target_yaw": e.get("target_yaw"),
                              "confirmed": e.get("confirmed"), "drive": e.get("drive"), "accepted": e.get("accepted"),
                              "event": e.get("event")})
    return rows, turn_rows


def collect_poses(session_dir, globs=(), since=0.0, log=None):
    """Build/refresh <session>/poses/. Returns {'stops': n, 'mocap_files': n, 'robot_runs': n, 'copied': n}."""
    out = os.path.join(session_dir, "poses")
    os.makedirs(os.path.join(out, "mocap"), exist_ok=True)
    os.makedirs(os.path.join(out, "robot_runs"), exist_ok=True)
    rows, turn_rows = build_stops(session_dir)
    _write_csv(os.path.join(out, "stops.csv"), STOP_COLUMNS, rows)
    _write_csv(os.path.join(out, "turntable.csv"), TURN_COLUMNS, turn_rows)
    copied = 0
    rp = os.path.join(session_dir, "robot", "robot_poses.csv")
    if os.path.exists(rp):
        copied += _copy_if_newer(rp, os.path.join(out, "robot_poses.csv"))
    n_mocap = 0
    images = os.path.join(session_dir, "images")
    if os.path.isdir(images):
        for image in sorted(os.listdir(images)):
            src = os.path.join(images, image, "stack", "mocap.csv")
            if os.path.exists(src):
                n_mocap += 1
                copied += _copy_if_newer(src, os.path.join(out, "mocap", image + ".csv"))
    n_runs = 0
    for pattern in globs or ():
        pattern = os.path.expandvars(os.path.expanduser(pattern))
        for src in sorted(glob.glob(pattern)):
            if not os.path.isfile(src):
                continue
            try:
                if since and os.path.getmtime(src) < since - 3600:     # written > 1 h before the session start: not ours
                    continue
            except OSError:
                continue
            n_runs += 1
            copied += _copy_if_newer(src, os.path.join(out, "robot_runs", os.path.basename(src)))
    res = {"stops": len(rows), "mocap_files": n_mocap, "robot_runs": n_runs, "copied": copied}
    if log and copied:
        log(f"[poses] {res['stops']} stops, {n_mocap} mocap files, {n_runs} robot run files -> {out} ({copied} copied)")
    return res


def main(argv=None):
    args = list(argv if argv is not None else sys.argv[1:])
    if not args:
        sys.exit("usage: python -m capture_coordinator.poses_collect <session_dir> [--glob PATTERN ...]")
    session_dir = args[0]
    globs = [args[i + 1] for i, a in enumerate(args) if a == "--glob" and i + 1 < len(args)]
    if not globs and os.path.exists(os.path.join(session_dir, "plan.json")):
        with open(os.path.join(session_dir, "plan.json")) as f:
            globs = json.load(f).get("poses_collect", [])
    print(collect_poses(session_dir, globs, log=print))


if __name__ == "__main__":
    main()
