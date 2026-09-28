#!/usr/bin/env python3
"""Fit the insect rotation axis (in the popillia BODY frame) from a session's OWN
measured popillia poses — nothing from any other dataset.

    python tools/fit_insect_axis.py <dataset> --tag sep15 [--compare calibrations/insect_axis.yaml]

Reads <dataset>/stacks.csv (recorded, never modified), keeps the stops whose
popillia pose was measured live, groups them into rotation STATES (consecutive
stops of a rotation whose yaw agrees within --same-deg; a jump inside a rotation
= the arm knocked the platter = a new state), averages the pose per state, and
takes the screw axis of every pair of states (relative angle between --min-deg
and 170 deg) exactly like optitrack/scripts/build_ring_poses.py did for aug31.
Writes <dataset>/calibrations/insect_axis_<tag>.yaml (refuses to overwrite).
"""
import argparse
import csv
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "coordinator"))
from capture_coordinator.mocap import yaw_deg                       # noqa: E402
from capture_coordinator.poses import parse_vec, quat_mat           # noqa: E402


def mean_quat(Q):
    Q = np.array(Q, float)
    Q[np.dot(Q, Q[0]) < 0] *= -1
    q = Q.mean(axis=0)
    return q / np.linalg.norm(q)


def pose_T(p, q):
    T = np.eye(4); T[:3, :3] = quat_mat(q); T[:3, 3] = p
    return T


def screw_axis(A):
    """axis direction + a point on it (body frame) of the rigid motion A; None if ~pure translation."""
    ang = np.degrees(np.arccos(np.clip((np.trace(A[:3, :3]) - 1) / 2, -1, 1)))
    w, V = np.linalg.eig(A[:3, :3])
    a = np.real(V[:, np.argmin(np.abs(w - 1))]); a /= np.linalg.norm(a)
    M = np.eye(3) - A[:3, :3]
    p, *_ = np.linalg.lstsq(M + np.outer(a, a), A[:3, 3], rcond=None)
    p = p - (p @ a) * a
    return a, p, ang


def line_dist(p0, d0, p1, d1):
    """distance between the two lines near p0 (mm-relevant region = the body origin)."""
    n = np.cross(d0, d1)
    if np.linalg.norm(n) < 1e-9:
        return float(np.linalg.norm(np.cross(p1 - p0, d0)))
    return float(abs((p1 - p0) @ n) / np.linalg.norm(n))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("--tag", required=True, help="output = calibrations/insect_axis_<tag>.yaml")
    ap.add_argument("--same-deg", type=float, default=1.0, help="yaw agreement that keeps stops in one state")
    ap.add_argument("--min-deg", type=float, default=3.0, help="smallest relative angle of a usable pair")
    ap.add_argument("--compare", help="another insect_axis.yaml to compare against (report only)")
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args()
    D = a.dataset
    out = os.path.join(D, "calibrations", f"insect_axis_{a.tag}.yaml")
    if os.path.exists(out) and not a.force:
        sys.exit(f"refusing to overwrite {out} (use --force or another --tag)")

    with open(os.path.join(D, "stacks.csv")) as f:
        rows = [r for r in csv.DictReader(f) if r.get("pop_quat_xyzw") and r.get("pop_x")]
    rows.sort(key=lambda r: int(r["stop"]))
    print(f"{len(rows)} stops with a live popillia pose")

    # states: consecutive stops of one rotation with the same yaw
    states = []
    for r in rows:
        p = np.array([float(r["pop_x"]), float(r["pop_y"]), float(r["pop_z"])]); q = parse_vec(r["pop_quat_xyzw"])
        y = yaw_deg(q)
        if states and states[-1]["step"] == int(r["step"]) and abs(((y - states[-1]["yaw"][-1]) + 180) % 360 - 180) < a.same_deg:
            s = states[-1]
        else:
            s = {"step": int(r["step"]), "images": [], "P": [], "Q": [], "yaw": []}; states.append(s)
        s["images"].append(r["image"]); s["P"].append(p); s["Q"].append(q); s["yaw"].append(y)
    for s in states:
        s["p"] = np.mean(s["P"], axis=0); s["q"] = mean_quat(s["Q"]); s["T"] = pose_T(s["p"], s["q"])
        s["spread_mm"] = float(np.max(np.linalg.norm(np.array(s["P"]) - s["p"], axis=1)) * 1000)
        print(f"  state rot {s['step']:2d} {s['images'][0]}..{s['images'][-1]} n={len(s['images'])} "
              f"yaw {np.mean(s['yaw']):8.3f} spread {s['spread_mm']:.2f} mm")

    axes, pts, angs, pairs = [], [], [], []
    for i in range(len(states)):
        for j in range(i + 1, len(states)):
            A = np.linalg.inv(states[i]["T"]) @ states[j]["T"]
            d, p, ang = screw_axis(A)
            if not (a.min_deg < ang < 170):
                continue
            if axes and d @ axes[0] < 0:
                d = -d
            axes.append(d); pts.append(p); angs.append(ang); pairs.append((states[i]["step"], states[j]["step"]))
    axes = np.array(axes); pts = np.array(pts)
    # joint least squares over ALL pairs (large rotations weigh more, as they should):
    #   direction: (R_ij - I) d = 0  -> smallest eigenvector of sum (R_ij - I)^T (R_ij - I)
    #   point:     (I - R_ij) p = t_ij, p in the plane normal to d
    Rs = [np.linalg.inv(states[i]["T"]) @ states[j]["T"] for i in range(len(states)) for j in range(i + 1, len(states))]
    Rs = [A for A in Rs if a.min_deg < np.degrees(np.arccos(np.clip((np.trace(A[:3, :3]) - 1) / 2, -1, 1))) < 170]
    M = sum((A[:3, :3] - np.eye(3)).T @ (A[:3, :3] - np.eye(3)) for A in Rs)
    w, V = np.linalg.eigh(M)
    a_dir = V[:, 0] / np.linalg.norm(V[:, 0])
    if a_dir @ axes.mean(0) < 0:
        a_dir = -a_dir
    L = np.vstack([np.eye(3) - A[:3, :3] for A in Rs] + [a_dir[None, :] * 10.0])
    rhs = np.concatenate([A[:3, 3] for A in Rs] + [[0.0]])
    a_pt, *_ = np.linalg.lstsq(L, rhs, rcond=None)
    a_pt = a_pt - (a_pt @ a_dir) * a_dir
    pts_pl = pts - np.outer(pts @ a_dir, a_dir)
    print(f"[axis] per-pair mean dir {np.round(axes.mean(0) / np.linalg.norm(axes.mean(0)), 5)} / median point {np.round(np.median(pts_pl, axis=0), 5)}"
          f" vs joint LSQ (used) below")
    dev_dir = np.degrees(np.arccos(np.clip(axes @ a_dir, -1, 1)))
    dev_pt = np.linalg.norm(pts_pl - a_pt, axis=1) * 1000
    print(f"[axis] {len(axes)} state pairs ({len(states)} states): dir {np.round(a_dir, 5)} "
          f"(dev p50 {np.median(dev_dir):.2f} / max {dev_dir.max():.2f} deg), point {np.round(a_pt, 5)} m "
          f"(lateral dev p50 {np.median(dev_pt):.2f} / max {dev_pt.max():.2f} mm), angles {angs and round(min(angs),1)}..{round(max(angs),1)} deg")
    print(f"[axis] marker-centroid orbit radius {np.linalg.norm(a_pt)*1000:.1f} mm")
    # leave-one-pair-out style check: axis from each single pair vs the mean line at the body origin
    info = {
        "axis_point_insect_frame_m": a_pt.tolist(),
        "axis_dir_insect_frame": a_dir.tolist(),
        "centroid_orbit_radius_m": float(np.linalg.norm(a_pt)),
        "n_step_motions": int(len(axes)),
        "n_states": int(len(states)),
        "states": [{"rotation": s["step"], "images": [s["images"][0], s["images"][-1]], "n": len(s["images"]),
                    "yaw_deg": round(float(np.mean(s["yaw"])), 3), "spread_mm": round(s["spread_mm"], 3)} for s in states],
        "pair_dev": {"dir_deg_p50": round(float(np.median(dev_dir)), 3), "dir_deg_max": round(float(dev_dir.max()), 3),
                     "point_mm_p50": round(float(np.median(dev_pt)), 3), "point_mm_max": round(float(dev_pt.max()), 3)},
        "source": f"{D}/stacks.csv (live mocap popillia poses of this session only)",
        "note": "rod/rotation axis in the popillia BODY frame: joint least squares over all pairs of measured "
                "rotation states of THIS session (tools/fit_insect_axis.py; pair_dev = spread of the single-pair "
                "screw axes); the body origin is the marker centroid and orbits this axis",
    }
    if a.compare:
        with open(a.compare) as f:
            o = yaml.safe_load(f)
        od = np.array(o["axis_dir_insect_frame"], float); od /= np.linalg.norm(od)
        op = np.array(o["axis_point_insect_frame_m"], float)
        ang = float(np.degrees(np.arccos(np.clip(abs(od @ a_dir), -1, 1))))
        # lateral offset of the other line from ours, at our axis point and at the body origin
        off_pt = float(np.linalg.norm(np.cross(op - a_pt, a_dir)) * 1000)
        off_origin = float(np.linalg.norm(np.cross(-op, od) - np.cross(-a_pt, a_dir)) * 1000)
        info["compare"] = {"file": a.compare, "angle_deg": round(ang, 3), "lateral_offset_mm_at_axis_point": round(off_pt, 3),
                           "offset_of_foot_points_from_body_origin_mm": round(off_origin, 3),
                           "orbit_radius_mm_other": round(float(np.linalg.norm(op)) * 1000, 2)}
        print(f"[compare] vs {a.compare}: angle {ang:.3f} deg, lateral offset {off_pt:.2f} mm, "
              f"orbit radius {np.linalg.norm(op)*1000:.1f} vs {np.linalg.norm(a_pt)*1000:.1f} mm")
    with open(out, "w") as f:
        yaml.safe_dump(info, f, sort_keys=False)
    print("wrote", out)


if __name__ == "__main__":
    main()
