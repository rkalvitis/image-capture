#!/usr/bin/env python3
"""Fill missing mocap poses of a stacks-layout dataset WITHOUT touching the
recorded files: writes <dataset>/derived/frames_filled.csv and
stacks_filled.csv (same columns + `pose_source`), and a per-stop
derived/mocap/<image>.csv for stops filled from the bag.

    python tools/fill_poses.py <dataset> [--bag <db3>] [--rig rig_transforms.yaml] [--axis <yaml>] [--out-dir derived_<tag>]

Sources, in this order per stop:
  mocap        the row already has poses (recorded live)              -> kept as is
  bag          both bodies have samples in the bag inside [mocap_t0, mocap_t1]
               (the coordinator's subscription missed them)          -> mean + jitter like the coordinator
  repeat       the ROBOT pose repeats every rotation: the phone (camera body) pose = mean
               over the rotations where the same robot pose k was measured by mocap;
               the popillia pose stays EMPTY (the insect rotation of that rotation is
               unknown), so cam_* (T_insect<-cam) stays empty too
  repeat+indexed  (--indexed) additionally the popillia pose of a rotation without mocap =
               the popillia pose of the last mocap rotation before it, rotated about the
               insect axis (calibrations/insect_axis.yaml, in the popillia body frame) by
               n x step. The step (magnitude and SENSE) is deduced from the mocap rotations
               on both sides of the gap (--step-deg overrides the magnitude, the sense
               always comes from the data). The same rule applied from one mocap rotation to
               the next reports the indexing error (validation).
The camera-in-insect pose (cam_*, quat_xyzw) is computed only where both bodies are known.
"""
import argparse
import csv
import json
import os
import sqlite3
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "coordinator"))
from capture_coordinator.mocap import yaw_deg                       # noqa: E402
from capture_coordinator.poses import fmt_vec, insect_from_cam, load_rig, mat_quat, parse_vec, quat_mat   # noqa: E402
from capture_coordinator.session import FRAME_COLUMNS               # noqa: E402
from capture_coordinator.stack_merge import MERGED_COLUMNS          # noqa: E402


def read_csv(p):
    with open(p) as f:
        return list(csv.DictReader(f))


def write_csv(p, cols, rows):
    with open(p, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c, "") for c in cols})


def mean_quat(Q):
    Q = np.array(Q, float)
    Q[np.dot(Q, Q[0]) < 0] *= -1
    q = Q.mean(axis=0)
    return q / np.linalg.norm(q)


class Bag:
    def __init__(self, path):
        self.con = sqlite3.connect(path)
        self.tid = {n: i for n, i in self.con.execute("select name,id from topics")}
        from rclpy.serialization import deserialize_message
        from geometry_msgs.msg import PoseStamped
        self._des, self._msg = deserialize_message, PoseStamped

    def samples(self, body, t0, t1):
        topic = "/optitrack/" + body
        if topic not in self.tid:
            return []
        out = []
        for ts, data in self.con.execute("select timestamp,data from messages where topic_id=? and timestamp between ? and ?",
                                         (self.tid[topic], int(t0 * 1e9), int(t1 * 1e9))):
            m = self._des(data, self._msg)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            p, q = m.pose.position, m.pose.orientation
            out.append((t, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))
        return out


def axis_rotation(axis_yaml, theta_deg):
    """4x4 transform in the popillia BODY frame: rotation by theta about the insect axis."""
    import yaml
    with open(axis_yaml) as f:
        ax = yaml.safe_load(f)
    c = np.array(ax["axis_point_insect_frame_m"], float)
    d = np.array(ax["axis_dir_insect_frame"], float); d /= np.linalg.norm(d)
    th = np.radians(theta_deg)
    K = np.array([[0, -d[2], d[1]], [d[2], 0, -d[0]], [-d[1], d[0], 0]])
    R = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = c - R @ c
    return T


def pose_T(p, q):
    T = np.eye(4); T[:3, :3] = quat_mat(q); T[:3, 3] = p
    return T


def unwrap_to(delta, expected):
    """delta in degrees, shifted by multiples of 360 to be closest to `expected`."""
    while delta - expected > 180: delta -= 360
    while delta - expected < -180: delta += 360
    return delta


def body_cols(prefix, P, Q):
    P = np.array(P, float); Q = mean_quat(Q)
    p = P.mean(axis=0)
    d = {f"{prefix}_x": round(float(p[0]), 6), f"{prefix}_y": round(float(p[1]), 6), f"{prefix}_z": round(float(p[2]), 6),
         f"{prefix}_quat_xyzw": fmt_vec(Q)}
    return d, p, Q


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("--bag", help="rosbag .db3 with /optitrack/phone and /optitrack/popillia (default: the newest in mocap/)")
    ap.add_argument("--rig", help="rig_transforms.yaml (default: <dataset>/calibrations/rig_transforms.yaml)")
    ap.add_argument("--indexed", action="store_true", help="also fill the popillia pose of rotations without mocap by the indexed rotation")
    ap.add_argument("--step-deg", type=float, default=None, help="indexing step magnitude (default: fitted between the mocap rotations around the gap)")
    ap.add_argument("--axis", help="insect_axis.yaml (default: <dataset>/calibrations/insect_axis.yaml)")
    ap.add_argument("--out-dir", default="derived", help="output folder relative to the dataset (default: derived); "
                    "use a new tagged folder to keep an earlier fill reproducible")
    a = ap.parse_args()
    D = a.dataset
    out_dir = os.path.join(D, a.out_dir)
    os.makedirs(os.path.join(out_dir, "mocap"), exist_ok=True)
    rig = a.rig or os.path.join(D, "calibrations", "rig_transforms.yaml")
    T_body_cam = load_rig(rig) if os.path.exists(rig) else None
    bag_path = a.bag
    if not bag_path:
        import glob
        cands = sorted(glob.glob(os.path.join(D, "mocap", "bag_*", "*.db3")), key=os.path.getsize)
        bag_path = cands[-1] if cands else None
    bag = Bag(bag_path) if bag_path else None

    frames = read_csv(os.path.join(D, "frames.csv"))
    stacks = read_csv(os.path.join(D, "stacks.csv"))
    by_image = {}
    for r in frames:
        by_image.setdefault(r["image"], []).append(r)

    # 1) per-image pose from live mocap or from the bag
    pose = {}                     # image -> (phone_p, phone_q, pop_p, pop_q, source, jitters)
    for image, rows in by_image.items():
        r0 = rows[0]
        if r0.get("phone_x") and r0.get("pop_x"):
            pose[image] = (np.array([float(r0["phone_x"]), float(r0["phone_y"]), float(r0["phone_z"])]), parse_vec(r0["phone_quat_xyzw"]),
                           np.array([float(r0["pop_x"]), float(r0["pop_y"]), float(r0["pop_z"])]), parse_vec(r0["pop_quat_xyzw"]),
                           "mocap", (r0.get("phone_jitter_mm"), r0.get("pop_jitter_mm")))
            continue
        if bag is None:
            continue
        t0, t1 = float(r0["mocap_t0"]), float(r0["mocap_t1"])
        sp, sq = bag.samples("phone", t0, t1), bag.samples("popillia", t0, t1)
        if len(sp) >= 10 and len(sq) >= 10:
            _, pp, pq = body_cols("phone", [s[1] for s in sp], [s[2] for s in sp])
            _, qp, qq = body_cols("pop", [s[1] for s in sq], [s[2] for s in sq])
            jp = float(np.linalg.norm(np.array([s[1] for s in sp]).std(axis=0)) * 1000)
            jq = float(np.linalg.norm(np.array([s[1] for s in sq]).std(axis=0)) * 1000)
            pose[image] = (pp, pq, qp, qq, "bag", (round(jp, 4), round(jq, 4)))
            with open(os.path.join(out_dir, "mocap", image + ".csv"), "w", newline="") as f:
                w = csv.writer(f); w.writerow(["body", "t", "x", "y", "z", "qx", "qy", "qz", "qw"])
                for body, ss in (("phone", sp), ("popillia", sq)):
                    for t, p, q in ss:
                        w.writerow([body, f"{t:.6f}"] + [f"{v:.6f}" for v in p] + [f"{v:.6f}" for v in q])

    # 2) the phone pose per ROBOT pose k from the rotations where it was measured
    per_k = {}
    for image, (pp, pq, qp, qq, src, _) in pose.items():
        k = int(by_image[image][0]["pose"])
        per_k.setdefault(k, []).append((pp, pq))
    k_mean, k_stats = {}, {}
    for k, lst in per_k.items():
        P = np.array([p for p, _ in lst]); Q = [q for _, q in lst]
        k_mean[k] = (P.mean(axis=0), mean_quat(Q))
        k_stats[k] = {"n": len(lst), "std_mm": round(float(np.linalg.norm(P.std(axis=0)) * 1000), 3),
                      "max_dev_mm": round(float(np.max(np.linalg.norm(P - P.mean(axis=0), axis=1)) * 1000), 3)}

    # 2b) indexed popillia pose for rotations without mocap
    pop_by_step = {}                 # step -> (pop_p, pop_q) of the LAST mocap stop of that rotation
    for image, (pp, pq, qp, qq, src, _) in pose.items():
        st_ = int(by_image[image][0]["step"]); stop = int(by_image[image][0]["stop"])
        if st_ not in pop_by_step or stop > pop_by_step[st_][0]:
            pop_by_step[st_] = (stop, qp, qq)
    indexed = {}
    idx_info = {}
    if a.indexed:
        axis_yaml = a.axis or os.path.join(D, "calibrations", "insect_axis.yaml")
        steps_m = sorted(pop_by_step)
        all_steps = sorted({int(r["step"]) for r in frames})
        missing = [st_ for st_ in all_steps if st_ not in pop_by_step]
        # measured yaw per rotation (last stop) -> per-step increment between consecutive mocap rotations
        yaws = {st_: yaw_deg(pop_by_step[st_][2]) for st_ in steps_m}
        incs = []
        for a_, b_ in zip(steps_m, steps_m[1:]):
            n = b_ - a_
            d = unwrap_to(yaws[b_] - yaws[a_], -10.0 * n if incs and incs[-1] < 0 else (10.0 * n if incs and incs[-1] > 0 else None) if incs else None) if False else None
            incs.append((a_, b_, n, yaws[a_], yaws[b_]))
        # sense + magnitude: from the consecutive mocap rotations (n == 1) first
        single = [(unwrap_to(yb - ya, 0.0)) for a_, b_, n, ya, yb in incs if n == 1]
        sense = -1.0 if (single and np.median(single) < 0) else 1.0
        # magnitude across the gap: total change between the anchors, unwrapped towards sense*10*n
        gap_fit = None
        for a_, b_, n, ya, yb in incs:
            if n > 1:
                total = unwrap_to(yb - ya, sense * 10.0 * n)
                gap_fit = (a_, b_, n, total / n)
        step_mag = abs(a.step_deg) if a.step_deg else (abs(gap_fit[3]) if gap_fit else (abs(np.median(single)) if single else 10.0))
        step_signed = sense * step_mag
        idx_info = {"axis": axis_yaml, "mocap_rotations": steps_m, "missing_rotations": missing,
                    "single_step_increments_deg": [round(v, 3) for v in single], "sense": sense,
                    "gap_fit": None if gap_fit is None else {"from": gap_fit[0], "to": gap_fit[1], "steps": gap_fit[2], "deg_per_step": round(gap_fit[3], 4)},
                    "step_used_deg": step_signed}
        # validation: predict each mocap rotation from the previous mocap rotation with the same rule
        val = []
        for a_, b_, n, ya, yb in incs:
            _, qp_a, qq_a = pop_by_step[a_]; _, qp_b, qq_b = pop_by_step[b_]
            T_pred = pose_T(qp_a, qq_a) @ axis_rotation(axis_yaml, step_signed * n)
            T_meas = pose_T(qp_b, qq_b)
            dpos = float(np.linalg.norm(T_pred[:3, 3] - T_meas[:3, 3]) * 1000)
            dR = T_pred[:3, :3].T @ T_meas[:3, :3]
            dang = float(np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))))
            val.append({"from": a_, "to": b_, "steps": n, "pos_err_mm": round(dpos, 2), "rot_err_deg": round(dang, 2)})
        idx_info["validation"] = val
        # fill: from the last mocap rotation before each missing rotation
        for st_ in missing:
            prev = max([x for x in steps_m if x < st_], default=None)
            if prev is None:
                continue
            _, qp0, qq0 = pop_by_step[prev]
            T = pose_T(qp0, qq0) @ axis_rotation(axis_yaml, step_signed * (st_ - prev))
            indexed[st_] = (T[:3, 3], mat_quat(T[:3, :3]), prev)

    # 3) fill
    counts = {"mocap": 0, "bag": 0, "repeat": 0, "repeat+indexed": 0, "repeat+same-rotation": 0, "none": 0}
    filled_rows = []
    per_image_out = {}
    for image, rows in by_image.items():
        k = int(rows[0]["pose"])
        if image in pose:
            pp, pq, qp, qq, src, jit = pose[image]
        elif k in k_mean:
            pp, pq = k_mean[k]; qp = qq = None; src = "repeat"; jit = ("", "")
            st_ = int(rows[0]["step"])
            if st_ in indexed:
                qp, qq, _ = indexed[st_]; src = "repeat+indexed"
            elif st_ in pop_by_step:
                # the rotation itself has mocap: the insect did not move between its stops (unless the arm
                # turned it, which the mocap of the later stops would show) -> its last measured popillia pose
                _, qp, qq = pop_by_step[st_]; src = "repeat+same-rotation"
        else:
            pp = pq = qp = qq = None; src = "none"; jit = ("", "")
        counts[src] += 1
        upd = {"pose_source": src}
        if pp is not None:
            upd.update({"phone_x": round(float(pp[0]), 6), "phone_y": round(float(pp[1]), 6), "phone_z": round(float(pp[2]), 6),
                        "phone_quat_xyzw": fmt_vec(pq), "phone_world": fmt_vec(pp, 4), "phone_jitter_mm": jit[0]})
        if qp is not None:
            upd.update({"pop_x": round(float(qp[0]), 6), "pop_y": round(float(qp[1]), 6), "pop_z": round(float(qp[2]), 6),
                        "pop_quat_xyzw": fmt_vec(qq), "popillia_yaw_deg": round(yaw_deg(qq), 3), "pop_jitter_mm": jit[1]})
        if pp is not None and qp is not None and T_body_cam is not None:
            T_ic = insect_from_cam(pp, pq, qp, qq, T_body_cam)
            upd.update({"cam_x": round(float(T_ic[0, 3]), 6), "cam_y": round(float(T_ic[1, 3]), 6), "cam_z": round(float(T_ic[2, 3]), 6),
                        "quat_xyzw": fmt_vec(mat_quat(T_ic[:3, :3]))})
        per_image_out[image] = upd
        for r in rows:
            rr = dict(r); rr.update(upd); filled_rows.append(rr)
    stacks_out = []
    for s in stacks:
        ss = dict(s); ss.update(per_image_out.get(s["image"], {"pose_source": "none"})); stacks_out.append(ss)

    cols_f = FRAME_COLUMNS + ["pose_source"]
    cols_s = MERGED_COLUMNS + ["pose_source"]
    write_csv(os.path.join(out_dir, "frames_filled.csv"), cols_f, filled_rows)
    write_csv(os.path.join(out_dir, "stacks_filled.csv"), cols_s, stacks_out)
    info = {"bag": bag_path, "rig": rig, "stops": counts, "phone_pose_per_robot_pose": k_stats, "indexed": idx_info,
            "note": "derived files; frames.csv / stacks.csv are the recorded originals and were not modified"}
    with open(os.path.join(out_dir, "fill_poses_info.json"), "w") as f:
        json.dump(info, f, indent=1)
    print(f"stops: {counts}")
    print("phone pose repeatability per robot pose:", k_stats)
    print(f"wrote {out_dir}/frames_filled.csv, stacks_filled.csv, fill_poses_info.json")


if __name__ == "__main__":
    main()
