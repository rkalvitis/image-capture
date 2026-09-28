#!/usr/bin/env python3
"""Focus table from a previous dataset's DNGs (Apple MakerNote), per robot
pose AND turntable angle.

Apple writes the lens position into every DNG/HEIC as the MakerNote tag
`FocusPosition` (0..255); measured 2026-09-09 against the app's sidecars:
lens_position = 1 - FocusPosition / 255 (exact). A dataset shot with manual
autofocus at the same robot poses therefore says where the focus was for
every (pose, insect yaw) — the focus depends on the pose AND on the
insect's rotation. The coordinator (`focus_mode: table`) looks up, at each
stop, the entry of the same robot pose with the nearest popillia yaw
(mocap, same rigid body on the platter) and shoots the stack around it.

    python3 tools/focus_from_dataset.py ~/Documents/repos/optitrack/datasets/aug31_20deg_9vertical \\
        --pose-order 0,1,2,3,4,5,6,7,9 --yaml coordinator/focus_aug31.yaml

--pose-order maps ROBOT poses 1..N (in the order the robot visits them) to
the dataset's pose ids; poses left out (8 above) are excluded. Needs
`exiftool` (brew install exiftool).
"""
import argparse
import csv
import glob
import os
import statistics as st
import subprocess
import sys


def focus_positions(dataset):
    files = sorted(glob.glob(os.path.join(dataset, "images", "**", "*.DNG"), recursive=True)
                   + glob.glob(os.path.join(dataset, "images", "**", "*.dng"), recursive=True))
    if not files:
        sys.exit(f"no DNGs under {dataset}/images")
    res = subprocess.run(["exiftool", "-csv", "-FocusPosition", "-AFStable", *files], capture_output=True, text=True)
    if res.returncode != 0:
        sys.exit("exiftool failed (brew install exiftool): " + res.stderr.strip()[-300:])
    out = {}
    for r in csv.DictReader(res.stdout.splitlines()):
        if r.get("FocusPosition"):
            stem = os.path.splitext(os.path.basename(r["SourceFile"]))[0]
            out[stem] = (1.0 - int(r["FocusPosition"]) / 255.0, r.get("AFStable"))
    return out


def build_table(dataset, poses_csv="camera_poses.csv", pose_col="pose", pose_order=None, yaw_col="popillia_yaw_deg",
                first_robot_pose=1):
    fp = focus_positions(dataset)
    rows = list(csv.DictReader(open(os.path.join(dataset, poses_csv))))
    ds_poses = sorted({int(r[pose_col]) for r in rows})
    order = pose_order if pose_order is not None else ds_poses
    table, per_pose = [], {}
    for r in rows:
        v = fp.get(r["image"])
        dp = int(r[pose_col])
        if not v or dp not in order:
            continue
        k = order.index(dp) + first_robot_pose        # robot pose numbering (0 = the initial pose when it is shot too)
        yaw = float(r[yaw_col]) if r.get(yaw_col) not in (None, "") else None
        entry = {"robot_pose": k, "dataset_pose": dp, "image": r["image"], "yaw_deg": yaw,
                 "lens": round(v[0], 4), "t": float(r.get("t_bag_s") or 0)}
        table.append(entry)
        per_pose.setdefault(k, []).append(v[0])
    table.sort(key=lambda e: (e["robot_pose"], e["t"]))
    poses = [{"robot_pose": k, "dataset_pose": order[k - first_robot_pose], "n": len(L), "median": round(st.median(L), 4),
              "min": round(min(L), 4), "max": round(max(L), 4)} for k, L in sorted(per_pose.items())]
    unstable = sum(1 for v in fp.values() if v[1] not in (None, "", "Yes"))
    return {"source": os.path.abspath(dataset), "mapping": "lens_position = 1 - FocusPosition/255",
            "pose_order": list(order), "poses": poses, "lens_position": [p["median"] for p in poses],
            "table": table, "n_images": len(fp), "af_unstable": unstable}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("--poses-csv", default="camera_poses.csv", help="relative to the dataset; columns image, pose, popillia_yaw_deg")
    ap.add_argument("--pose-col", default="pose")
    ap.add_argument("--pose-order", help="dataset pose ids in robot order, e.g. 0,1,2,3,4,5,6,7,9 (others excluded)")
    ap.add_argument("--yaml", help="write the table here (plan: focus_table)")
    ap.add_argument("--first-robot-pose", type=int, default=1,
                    help="robot pose number of the first entry of --pose-order (0 when the initial pose is a photo pose)")
    a = ap.parse_args()
    order = [int(x) for x in a.pose_order.split(",")] if a.pose_order else None
    t = build_table(a.dataset, a.poses_csv, a.pose_col, order, first_robot_pose=a.first_robot_pose)
    print(f"{t['n_images']} DNGs with FocusPosition, {len(t['table'])} table entries, {t['af_unstable']} with AFStable != Yes")
    print(f"{'robot':>5} {'dataset':>7} {'n':>3}  {'median':>6} {'min':>6} {'max':>6}  {'spread':>6}")
    for p in t["poses"]:
        print(f"{p['robot_pose']:5d} {p['dataset_pose']:7d} {p['n']:3d}  {p['median']:6.3f} {p['min']:6.3f} {p['max']:6.3f}  {p['max'] - p['min']:6.3f}")
    print("\nplan.yaml:")
    lp = t["lens_position"]
    if t["poses"] and t["poses"][0]["robot_pose"] == 0:
        print(f"poses: {len(t['poses']) - 1}\nshoot_initial_pose: true\nlens_position_initial: {lp[0]:.4f}")
        lp = lp[1:]
    else:
        print(f"poses: {len(t['poses'])}")
    print("lens_position: [" + ", ".join(f"{v:.4f}" for v in lp) + "]   # focus_mode: pose")
    if a.yaml:
        import yaml
        with open(a.yaml, "w") as f:
            yaml.safe_dump(t, f, sort_keys=False)
        print(f"focus_table: {a.yaml}   # focus_mode: table (per pose AND yaw)")
        print(f"wrote {a.yaml}")


if __name__ == "__main__":
    main()
