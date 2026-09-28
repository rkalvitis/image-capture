#!/usr/bin/env python3
"""Stage 3: Apple's per-frame intrinsics vs a ChArUco board, per lens position.

Shoot the board with the app at the lens positions of the stack (the board
at the matching working distance, tilted, pushed into the corners), e.g.

    python3 tools/mac_client.py hello --session board --lens 0.3
    python3 tools/mac_client.py stack --stop 1 --lens 0.2 0.3 0.4 0.5   # repeat with the board moved, ~20-40 stops
    python3 tools/pull_frames.py board /tmp/board
    python3 tools/check_intrinsics.py /tmp/board --square-mm 7.98 [--out intrinsics_by_lens_position.yaml]

For every lens position found in the sidecars it calibrates a PINHOLE +
distortion model (Zhang, cv2.aruco.CharucoBoard, same board as
scripts/make_charuco_board.py in the optitrack repo: 8x6 squares, DICT_5X5_100)
and prints Apple's fx fy cx cy (mean over those frames) next to the board's,
plus the scale factor Apple->board. The YAML it writes is the per-position
table docs/DEVELOPMENT_PLAN.md Stage 3 asks for.

Reads the DNGs with rawpy when available (sensor-native), otherwise the
embedded JPEG preview via sips.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "coordinator"))

try:
    import cv2
except ImportError:
    sys.exit("opencv is required: pip install -r coordinator/requirements-merge.txt")

DEFAULT_BOARD = dict(squares_x=8, squares_y=6, square_mm=8.0, marker_mm=6.0, dictionary="DICT_5X5_100")


def make_board(spec):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec["dictionary"]))
    return cv2.aruco.CharucoBoard((spec["squares_x"], spec["squares_y"]),
                                  spec["square_mm"] / 1000.0, spec["marker_mm"] / 1000.0, dictionary)


def load_gray(path):
    from capture_coordinator.stack_merge import decode_dng, to_srgb8
    lin, _, _ = decode_dng(path)
    return cv2.cvtColor(to_srgb8(lin), cv2.COLOR_RGB2GRAY)


def detect(gray, board, min_corners=12):
    detector = cv2.aruco.CharucoDetector(board)
    ch_corners, ch_ids, _, _ = detector.detectBoard(gray)
    if ch_ids is None or len(ch_ids) < min_corners:
        return None, None
    return ch_corners, ch_ids


def calibrate(views, board, size):
    obj_all, img_all = [], []
    for corners, ids in views:
        obj, img = board.matchImagePoints(corners, ids)
        if obj is not None and len(obj) >= 6:
            obj_all.append(obj); img_all.append(img)
    if len(obj_all) < 4:
        return None
    flags = cv2.CALIB_FIX_K3 | cv2.CALIB_ZERO_TANGENT_DIST
    rms, K, dist, _, _ = cv2.calibrateCamera(obj_all, img_all, size, None, None, flags=flags)
    return {"rms_px": float(rms), "fx": float(K[0, 0]), "fy": float(K[1, 1]), "cx": float(K[0, 2]),
            "cy": float(K[1, 2]), "dist": [float(v) for v in dist.ravel()], "views": len(obj_all)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder", help="pulled session folder (images/ + sidecars/) or a flat folder of DNG+JSON")
    ap.add_argument("--square-mm", type=float, required=True, help="caliper-measured printed square size")
    ap.add_argument("--marker-mm", type=float, default=None, help="default: 6/8 of the square")
    ap.add_argument("--squares-x", type=int, default=DEFAULT_BOARD["squares_x"])
    ap.add_argument("--squares-y", type=int, default=DEFAULT_BOARD["squares_y"])
    ap.add_argument("--dict", default=DEFAULT_BOARD["dictionary"])
    ap.add_argument("--lens-bin", type=float, default=0.01, help="group frames whose lens position differs less than this")
    ap.add_argument("--out", default=None, help="YAML table (default <folder>/intrinsics_by_lens_position.yaml)")
    a = ap.parse_args()

    spec = dict(squares_x=a.squares_x, squares_y=a.squares_y, square_mm=a.square_mm,
                marker_mm=a.marker_mm or a.square_mm * 6.0 / 8.0, dictionary=a.dict)
    board = make_board(spec)
    sidecars = sorted(glob.glob(os.path.join(a.folder, "sidecars", "*.json")) or glob.glob(os.path.join(a.folder, "*.json")))
    if not sidecars:
        sys.exit("no sidecars found")
    groups = {}
    for sc in sidecars:
        d = json.load(open(sc))
        dng = os.path.join(os.path.dirname(sc).replace("sidecars", "images"), d["file"])
        if not os.path.exists(dng):
            dng = os.path.join(os.path.dirname(sc), d["file"])
        if not os.path.exists(dng) or not d.get("intrinsics"):
            continue
        key = round(float(d["lens_position"]) / a.lens_bin) * a.lens_bin
        groups.setdefault(round(key, 4), []).append((dng, d))
    print(f"{sum(len(v) for v in groups.values())} frames in {len(groups)} lens-position group(s)")
    table = {"board": spec, "positions": []}
    print(f"{'lens':>6} {'n':>3} {'det':>3} | {'Apple fx':>9} {'fy':>8} {'cx':>8} {'cy':>8} | {'board fx':>9} {'fy':>8} {'cx':>8} {'cy':>8} {'rms':>5} | scale")
    for lens in sorted(groups):
        items = groups[lens]
        views, size = [], None
        for dng, d in items:
            g = load_gray(dng)
            size = (g.shape[1], g.shape[0])
            c, ids = detect(g, board)
            if c is not None:
                views.append((c, ids))
        apple = {k: float(np.mean([d["intrinsics"][k] for _, d in items])) for k in ("fx", "fy", "cx", "cy")}
        res = calibrate(views, board, size) if views else None
        row = {"lens_position": float(lens), "frames": len(items), "detected": len(views), "apple": apple, "board": res}
        table["positions"].append(row)
        if res:
            print(f"{lens:6.3f} {len(items):3d} {len(views):3d} | {apple['fx']:9.1f} {apple['fy']:8.1f} {apple['cx']:8.1f} {apple['cy']:8.1f} | "
                  f"{res['fx']:9.1f} {res['fy']:8.1f} {res['cx']:8.1f} {res['cy']:8.1f} {res['rms_px']:5.2f} | {res['fx'] / apple['fx']:.4f}")
        else:
            print(f"{lens:6.3f} {len(items):3d} {len(views):3d} | {apple['fx']:9.1f} {apple['fy']:8.1f} {apple['cx']:8.1f} {apple['cy']:8.1f} | (board not detected in enough frames)")
    out = a.out or os.path.join(a.folder, "intrinsics_by_lens_position.yaml")
    with open(out, "w") as f:
        yaml.safe_dump(table, f, sort_keys=False)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
