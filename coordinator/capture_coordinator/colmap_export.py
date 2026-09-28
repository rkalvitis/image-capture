"""frames.csv / merged_frames.csv -> COLMAP text model + camera_poses.csv
(docs/DEVELOPMENT_PLAN.md Stage 5, replaces the EXIF matching + single
shared camera of the old pipeline).

    python -m capture_coordinator.colmap_export <session_dir> [--source merged|frames]
        [--rig calibrations/rig_transforms.yaml] [--out gs_upload] [--scale 1.0]
        [--link] [--beetle-radius-mm 6]

Writes <session>/<out>/
    images/<name>.png|jpg          8-bit sRGB PNG (lossless) rendered from the 16-bit linear merged
                                   TIFFs / the DNGs of single-frame sessions; jpg/png sources are copied
                                   (--masters: the 16-bit TIFF masters as they are, --link: hard-linked)
    sparse/0/cameras.txt           ONE PINHOLE CAMERA PER IMAGE with the frame's own fx fy cx cy
                                   (Apple per-frame intrinsics; the trainer honours cx,cy since 2026-09-02)
    sparse/0/images.txt            world(insect)->cam poses from T_insect<-cam
    sparse/0/points3D.txt          random cloud in a ball around the point the optical axes converge on
    camera_poses.csv               image, ring(=pose), cam_x.., quat_xyzw, popillia_yaw_deg, jitters,
                                   phone_world, pose — the layout plot_ring_dome.py reads

Camera poses: cam_x/cam_y/cam_z/quat_xyzw (T_insect<-cam) are taken from the
CSV when present, otherwise computed from the body poses with --rig.
"""
import argparse
import csv
import os
import shutil
import sys

import numpy as np

from .poses import (colmap_pose, fmt_vec, insect_from_cam, load_rig, mat_quat, parse_vec, quat_mat,
                    rays_closest_point)

POSE_COLUMNS = ["image", "ring", "t_bag_s", "cam_x", "cam_y", "cam_z", "quat_xyzw", "popillia_yaw_deg",
                "phone_jitter_mm", "pop_jitter_mm", "sharpness", "phone_world", "pose", "stop", "step",
                "fx", "fy", "cx", "cy", "lens_position", "file", "pose_source"]


def read_rows(session_dir, source, stacks_csv=None):
    if source == "merged":
        path = stacks_csv or os.path.join(session_dir, "stacks.csv")        # stacks layout (2026-09-15)
        if not os.path.isabs(path):
            path = os.path.join(session_dir, path)
        if not os.path.exists(path) and not stacks_csv:
            path = os.path.join(session_dir, "merged", "merged_frames.csv")   # flat layout
    else:
        path = os.path.join(session_dir, "frames.csv")
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if source == "frames":
        # one image per stop: the reference frame is whatever the single-frame session shot
        by_stop = {}
        for r in rows:
            by_stop.setdefault(int(r["stop"]), []).append(r)
        rows = [sorted(v, key=lambda r: int(r["focus_index"]))[len(v) // 2] for v in by_stop.values()]
        for r in rows:
            r.setdefault("ref_lens_position", r.get("lens_position", ""))
    return sorted(rows, key=lambda r: int(r["stop"]))


def camera_pose(row, T_body_cam):
    """T_insect<-cam for a row (from cam_* columns, else from the body poses + rig)."""
    if row.get("quat_xyzw") and row.get("cam_x") not in ("", None):
        T = np.eye(4)
        T[:3, :3] = quat_mat(parse_vec(row["quat_xyzw"]))
        T[:3, 3] = [float(row["cam_x"]), float(row["cam_y"]), float(row["cam_z"])]
        return T
    if T_body_cam is None:
        raise ValueError("no cam_* columns in the CSV and no --rig given")
    if not row.get("phone_quat_xyzw") or not row.get("pop_quat_xyzw"):
        raise ValueError(f"row {row.get('file')} has no body poses")
    phone_p = [float(row["phone_x"]), float(row["phone_y"]), float(row["phone_z"])]
    pop_p = [float(row["pop_x"]), float(row["pop_y"]), float(row["pop_z"])]
    return insect_from_cam(phone_p, parse_vec(row["phone_quat_xyzw"]), pop_p, parse_vec(row["pop_quat_xyzw"]), T_body_cam)


def render_png(src, dst, linear=True):
    """8-bit sRGB PNG from a DNG (rawpy, sensor-native) or a 16-bit TIFF
    (linear=True: linear master -> sRGB encode; False: already sRGB-encoded -> scale only)."""
    from .stack_merge import decode_dng, write_image
    if src.lower().endswith((".tif", ".tiff")):
        import cv2
        import numpy as np
        img = cv2.imread(src, cv2.IMREAD_UNCHANGED | cv2.IMREAD_COLOR)
        if not linear:
            img8 = (img.astype(np.float32) / (257.0 if img.dtype == np.uint16 else 1.0) + 0.5).astype(np.uint8)
            cv2.imwrite(dst, img8)
            return
        lin = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype("float32") / (65535.0 if img.dtype == "uint16" else 255.0)
    else:
        lin, _, _ = decode_dng(src)
    write_image(dst, lin, "png")


def export(session_dir, source="merged", rig=None, out="gs_upload", scale=1.0, link=False,
           beetle_radius_mm=6.0, n_points=20000, log=print, stacks_csv=None, masters=False):
    rows = read_rows(session_dir, source, stacks_csv)
    T_body_cam = load_rig(rig) if rig else None
    out_dir = os.path.join(session_dir, out)
    img_dir = os.path.join(out_dir, "images")
    sdir = os.path.join(out_dir, "sparse", "0")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(sdir, exist_ok=True)
    from .stack_merge import frame_path, merged_path

    entries = []
    for r in rows:
        stem, ext = os.path.splitext(r["file"])
        render = ext.lower() in (".dng", ".tif", ".tiff") and not masters
        name = stem + (".png" if render else ext.lower())
        src = merged_path(session_dir, "merged", r) if source == "merged" else frame_path(session_dir, r)
        dst = os.path.join(img_dir, name)
        if not os.path.exists(src):
            log(f"missing image {src}, skipping stop {r['stop']}")
            continue
        if not os.path.exists(dst):
            if render:
                fmt = r.get("format") or ("tif-linear" if ext.lower() in (".tif", ".tiff") else "")
                render_png(src, dst, linear=(fmt != "tif"))
            elif link:
                os.link(src, dst)
            else:
                shutil.copy2(src, dst)
        try:
            T_ic = camera_pose(r, T_body_cam)
        except ValueError as err:
            log(f"stop {r['stop']}: {err}; skipping")
            continue
        w = int(float(r.get("w") or 4032)); h = int(float(r.get("h") or 3024))
        intr = [float(r["fx"]), float(r["fy"]), float(r["cx"]), float(r["cy"])]
        entries.append((name, r, T_ic, intr, w, h))
    if not entries:
        raise RuntimeError("nothing to export (no images with poses)")

    with open(os.path.join(sdir, "cameras.txt"), "w") as f:
        f.write("# Camera list: CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]\n# one PINHOLE camera per image (Apple per-frame intrinsics)\n")
        for i, (name, r, T_ic, intr, w, h) in enumerate(entries, 1):
            f.write(f"{i} PINHOLE {w} {h} {intr[0]:.4f} {intr[1]:.4f} {intr[2]:.4f} {intr[3]:.4f}\n")
    centres, dirs = [], []
    with open(os.path.join(sdir, "images.txt"), "w") as f:
        f.write("# Image list: IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME\n#   (empty points2D line follows each)\n")
        for i, (name, r, T_ic, intr, w, h) in enumerate(entries, 1):
            q, t = colmap_pose(T_ic, scale)
            f.write(f"{i} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f} {t[0]:.9f} {t[1]:.9f} {t[2]:.9f} {i} {name}\n\n")
            centres.append(T_ic[:3, 3]); dirs.append(T_ic[:3, 2])
    beetle = rays_closest_point(centres, dirs)
    rng = np.random.default_rng(0)
    pts = rng.normal(size=(n_points, 3))
    pts /= np.linalg.norm(pts, axis=1, keepdims=True)
    pts = beetle + pts * (beetle_radius_mm / 1000.0 * rng.random((n_points, 1)) ** (1 / 3))
    with open(os.path.join(sdir, "points3D.txt"), "w") as f:
        f.write("# 3D point list: POINT3D_ID X Y Z R G B ERROR TRACK[]\n")
        for i, p in enumerate(pts * scale, 1):
            g = int(rng.integers(90, 160))
            f.write(f"{i} {p[0]:.7f} {p[1]:.7f} {p[2]:.7f} {g} {g} {g} 1.0\n")
    ply = os.path.join(sdir, "points3D.ply")
    if os.path.exists(ply):
        os.remove(ply)                                   # a stale ply silently overrides the txt

    with open(os.path.join(out_dir, "camera_poses.csv"), "w", newline="") as f:
        wri = csv.DictWriter(f, fieldnames=POSE_COLUMNS)
        wri.writeheader()
        for name, r, T_ic, intr, w, h in entries:
            wri.writerow({
                "image": os.path.splitext(name)[0], "ring": r.get("pose", ""), "t_bag_s": r.get("capture_time", ""),
                "cam_x": f"{T_ic[0, 3]:.6f}", "cam_y": f"{T_ic[1, 3]:.6f}", "cam_z": f"{T_ic[2, 3]:.6f}",
                "quat_xyzw": fmt_vec(mat_quat(T_ic[:3, :3])), "popillia_yaw_deg": r.get("popillia_yaw_deg", ""),
                "phone_jitter_mm": r.get("phone_jitter_mm", ""), "pop_jitter_mm": r.get("pop_jitter_mm", ""),
                "sharpness": "", "phone_world": r.get("phone_world", ""), "pose": r.get("pose", ""),
                "stop": r.get("stop", ""), "step": r.get("step", ""), "fx": intr[0], "fy": intr[1],
                "cx": intr[2], "cy": intr[3], "lens_position": r.get("ref_lens_position", r.get("lens_position", "")),
                "file": r["file"], "pose_source": r.get("pose_source", "")})
    log(f"exported {len(entries)} images to {out_dir} (beetle centre estimate {fmt_vec(beetle, 4)} m, "
        f"cameras {np.mean([np.linalg.norm(c - beetle) for c in centres]) * 1000:.0f} mm away on average)")
    return out_dir, len(entries)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_dir")
    ap.add_argument("--source", choices=["merged", "frames"], default="merged",
                    help="merged = stacks.csv / merged/merged_frames.csv (focus stacks); frames = frames.csv (single-frame sessions)")
    ap.add_argument("--rig", help="rig_transforms.yaml with T_body_from_cam (needed if the CSV has no cam_* columns)")
    ap.add_argument("--stacks-csv", help="alternative stacks CSV (e.g. derived/stacks_filled.csv with filled poses; relative to the session)")
    ap.add_argument("--out", default="gs_upload")
    ap.add_argument("--scale", type=float, default=1.0, help="unit scale for COLMAP (1.0 = metres)")
    ap.add_argument("--link", action="store_true", help="hard-link images instead of copying")
    ap.add_argument("--masters", action="store_true",
                    help="publish the merged 16-bit TIFF masters themselves (bit-exact, no 8-bit PNG rendering); "
                         "combine with --link to avoid a second copy")
    ap.add_argument("--beetle-radius-mm", type=float, default=6.0)
    a = ap.parse_args(argv)
    try:
        export(a.session_dir, a.source, a.rig, a.out, a.scale, a.link, a.beetle_radius_mm, stacks_csv=a.stacks_csv,
               masters=a.masters)
    except (FileNotFoundError, RuntimeError, ValueError) as err:
        sys.exit(f"error: {err}")


if __name__ == "__main__":
    main()
