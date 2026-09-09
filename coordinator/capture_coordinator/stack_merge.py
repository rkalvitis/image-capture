"""Mac-side focus-stack merge (docs/DEVELOPMENT_PLAN.md Stage 5).

The phone writes N ProRAW frames per stop, one per lens position. Because
the ultra-wide breathes with focus (fx 1827 px at lens 0.0 -> 1691 px at
0.5), the frames of one stop have different magnification. This module:

1. decodes every DNG linearly (rawpy, sensor-native orientation, camera WB),
2. rescales each frame about its own principal point by fx_ref / fx_k so it
   matches the geometry of the REFERENCE frame (a fixed focus index, the
   same at every stop), optionally refined by ECC on a downsampled copy,
3. fuses the aligned frames with a Laplacian-pyramid maximum-energy rule,
   frame by frame (memory ~2 pyramids, not N),
4. writes one all-in-focus image per stop plus a CSV row that inherits the
   reference frame's intrinsics and mocap pose from frames.csv.

The merged image therefore has exactly the reference frame's K, which is
what the COLMAP export (colmap_export.py) uses. Raw stacks are kept.

    python -m capture_coordinator.stack_merge <session_dir> [--ref-index 20]
        [--out merged] [--format jpg|png|tif] [--refine] [--stops 0 1 2]

Dependencies: numpy, opencv (cv2), rawpy. `sips` (macOS) is the decoder
fallback when rawpy is missing (Apple-rendered, not linear — flagged in
the CSV).
"""
import argparse
import csv
import json
import os
import subprocess
import sys
import tempfile
import time

import numpy as np

try:
    import cv2
except ImportError:                                            # pragma: no cover
    cv2 = None

MERGED_COLUMNS = ["file", "stop", "step", "pose", "ref_file", "ref_focus_index", "ref_lens_position",
                  "n_frames", "fx", "fy", "cx", "cy", "w", "h", "capture_time", "mocap_t0", "mocap_t1",
                  "phone_jitter_mm", "pop_jitter_mm", "cam_x", "cam_y", "cam_z", "quat_xyzw",
                  "popillia_yaw_deg", "pop_x", "pop_y", "pop_z", "pop_quat_xyzw", "phone_world",
                  "decoder", "linear", "aligned", "refine_rms_px"]


# ---------------------------------------------------------------------------
# decoding
def decode_dng(path, half=False):
    """Linear RGB float32 in [0, 1], sensor-native orientation (no flip),
    camera white balance, sRGB primaries. Returns (img, decoder_name, linear)."""
    try:
        import rawpy
    except ImportError:
        rawpy = None
    if rawpy is not None:
        with rawpy.imread(path) as r:
            rgb = r.postprocess(gamma=(1, 1), no_auto_bright=True, output_bps=16, use_camera_wb=True,
                                user_flip=0, half_size=half, output_color=rawpy.ColorSpace.sRGB)
        return rgb.astype(np.float32) / 65535.0, "rawpy", True
    # fallback: Apple's renderer through sips (tone-mapped, 16-bit TIFF)
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, "f.tiff")
        subprocess.run(["sips", "-s", "format", "tiff", path, "--out", out],
                       check=True, capture_output=True)
        img = cv2.imread(out, cv2.IMREAD_UNCHANGED | cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError(f"sips could not render {path}")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32)
        img /= 65535.0 if img.max() > 255 else 255.0
        if half:
            img = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2), interpolation=cv2.INTER_AREA)
        return img, "sips", False


def to_srgb8(lin):
    """Linear [0,1] -> 8-bit sRGB."""
    x = np.clip(lin, 0.0, 1.0)
    s = np.where(x <= 0.0031308, 12.92 * x, 1.055 * np.power(x, 1 / 2.4) - 0.055)
    return (s * 255.0 + 0.5).astype(np.uint8)


# ---------------------------------------------------------------------------
# alignment
def breathing_affine(intr_k, intr_ref):
    """2x3 affine mapping frame k pixels -> reference pixels, from the
    intrinsics: scale fx_ref/fx_k (fy likewise) about k's principal point,
    then shift so the principal points coincide."""
    sx = intr_ref["fx"] / intr_k["fx"]
    sy = intr_ref["fy"] / intr_k["fy"]
    tx = intr_ref["cx"] - sx * intr_k["cx"]
    ty = intr_ref["cy"] - sy * intr_k["cy"]
    return np.array([[sx, 0.0, tx], [0.0, sy, ty]], dtype=np.float64)


def refine_affine(gray_k, gray_ref, A, scale=0.25, iters=60, eps=1e-5):
    """ECC refinement of A (frame k -> ref) on downsampled grey images.
    Returns (A_refined, rms shift in full-res px relative to A) or (A, None)."""
    small = lambda g: cv2.resize(g, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    gk, gr = small(gray_k), small(gray_ref)
    S = np.diag([scale, scale, 1.0])
    A3 = np.vstack([A, [0, 0, 1]])
    warp = (S @ A3 @ np.linalg.inv(S))[:2].astype(np.float32)
    crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, iters, eps)
    try:
        _, warp2 = cv2.findTransformECC(gr, gk, warp, cv2.MOTION_AFFINE, crit, None, 5)
    except cv2.error:
        return A, None
    A2 = (np.linalg.inv(S) @ np.vstack([warp2, [0, 0, 1]]) @ S)[:2]
    h, w = gray_k.shape
    pts = np.array([[0, 0, 1], [w, 0, 1], [0, h, 1], [w, h, 1], [w / 2, h / 2, 1]], float).T
    d = (A2 @ pts) - (A @ pts)
    return A2, float(np.sqrt((d ** 2).sum(axis=0).mean()))


def warp_to_ref(img, A, w, h):
    """Warp a frame into the reference geometry; returns (image, valid mask)."""
    out = cv2.warpAffine(img, A.astype(np.float32), (w, h), flags=cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    ones = np.ones(img.shape[:2], np.float32)
    valid = cv2.warpAffine(ones, A.astype(np.float32), (w, h), flags=cv2.INTER_NEAREST,
                           borderMode=cv2.BORDER_CONSTANT, borderValue=0) > 0.5
    return out, valid


# ---------------------------------------------------------------------------
# fusion
def _gauss_down(x):
    return cv2.pyrDown(x)


def laplacian_pyramid(img, levels):
    pyr, cur = [], img
    for _ in range(levels):
        down = _gauss_down(cur)
        up = cv2.pyrUp(down, dstsize=(cur.shape[1], cur.shape[0]))
        pyr.append(cur - up)
        cur = down
    pyr.append(cur)
    return pyr


def collapse(pyr):
    cur = pyr[-1]
    for lap in reversed(pyr[:-1]):
        cur = cv2.pyrUp(cur, dstsize=(lap.shape[1], lap.shape[0])) + lap
    return cur


class PyramidFuser:
    """Incremental Laplacian-pyramid focus fusion: keeps, per level and per
    pixel, the coefficient of the frame with the highest local energy."""

    def __init__(self, levels=6, energy_ksize=7):
        self.levels, self.ks = levels, energy_ksize
        self.best, self.energy, self.count = None, None, None
        self.index_map = None    # which frame index won at full resolution (for inspection)

    def add(self, img, valid, index):
        pyr = laplacian_pyramid(img, self.levels)
        vmask = valid.astype(np.float32)
        if self.best is None:
            self.best = [p.copy() for p in pyr]
            self.energy = []
            self.count = np.zeros(img.shape[:2], np.uint16)
            self.index_map = np.full(img.shape[:2], -1, np.int16)
            first = True
        else:
            first = False
        for lvl, lap in enumerate(pyr):
            v = vmask if lvl == 0 else cv2.resize(vmask, (lap.shape[1], lap.shape[0]), interpolation=cv2.INTER_AREA)
            if lvl < self.levels:
                e = cv2.blur(np.abs(lap).sum(axis=2), (self.ks, self.ks))
            else:
                e = np.zeros(lap.shape[:2], np.float32)     # base: average over valid frames
            e = np.where(v > 0.999, e, -1.0).astype(np.float32)
            if first:
                self.energy.append(e)
                if lvl == self.levels:
                    self.best[lvl] = lap * (v > 0.999)[..., None]
                    self._base_n = (v > 0.999).astype(np.float32)
                continue
            if lvl < self.levels:
                better = e > self.energy[lvl]
                self.best[lvl][better] = lap[better]
                self.energy[lvl][better] = e[better]
                if lvl == 0:
                    self.index_map[better] = index
            else:
                ok = (v > 0.999)
                self.best[lvl][ok] += lap[ok]
                self._base_n[ok] += 1
        if first:
            self.index_map[valid] = index
        self.count[valid] += 1

    def result(self):
        pyr = list(self.best)
        n = np.maximum(self._base_n, 1.0)[..., None]
        pyr[-1] = pyr[-1] / n
        return collapse(pyr)


# ---------------------------------------------------------------------------
# session handling
def load_frames(session_dir):
    with open(os.path.join(session_dir, "frames.csv")) as f:
        rows = list(csv.DictReader(f))
    stops = {}
    for r in rows:
        stops.setdefault(int(r["stop"]), []).append(r)
    for s in stops.values():
        s.sort(key=lambda r: int(r["focus_index"]))
    return stops


def intrinsics_of(row, session_dir):
    """fx, fy, cx, cy for a frame: frames.csv first, sidecar as fallback."""
    try:
        return {k: float(row[k]) for k in ("fx", "fy", "cx", "cy")}
    except (KeyError, ValueError):
        side = os.path.join(session_dir, "sidecars", os.path.splitext(row["file"])[0] + ".json")
        with open(side) as f:
            intr = json.load(f)["intrinsics"]
        return {k: float(intr[k]) for k in ("fx", "fy", "cx", "cy")}


def merge_stop(session_dir, rows, ref_index, refine=False, levels=6, log=print, half=False):
    """Merge the frames of one stop. Returns (linear float image, info dict)."""
    frames = [r for r in rows if os.path.exists(os.path.join(session_dir, "images", r["file"]))]
    if not frames:
        raise FileNotFoundError("no images for this stop")
    by_idx = {int(r["focus_index"]): r for r in frames}
    ref_idx = ref_index if ref_index in by_idx else min(by_idx, key=lambda i: abs(i - ref_index))
    ref_row = by_idx[ref_idx]
    ref_intr = intrinsics_of(ref_row, session_dir)
    ref_img, decoder, linear = decode_dng(os.path.join(session_dir, "images", ref_row["file"]), half=half)
    if half:
        for k in ("fx", "fy", "cx", "cy"):
            ref_intr[k] /= 2.0
    h, w = ref_img.shape[:2]
    ref_gray = cv2.cvtColor(ref_img, cv2.COLOR_RGB2GRAY) if refine else None
    ref_med = float(np.median(ref_img))
    fuser = PyramidFuser(levels=levels)
    rms_all = []
    t0 = time.time()
    for r in frames:
        k = int(r["focus_index"])
        if k == ref_idx:
            img, valid = ref_img, np.ones((h, w), bool)
        else:
            img, _, _ = decode_dng(os.path.join(session_dir, "images", r["file"]), half=half)
            intr = intrinsics_of(r, session_dir)
            if half:
                intr = {kk: v / 2.0 for kk, v in intr.items()}
            A = breathing_affine(intr, ref_intr)
            if refine:
                A, rms = refine_affine(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY), ref_gray, A)
                if rms is not None:
                    rms_all.append(rms)
            # exposure/gain normalisation to the reference (locked exposure -> ~1.0)
            med = float(np.median(img))
            if med > 0:
                img = img * (ref_med / med)
            img, valid = warp_to_ref(img, A, w, h)
        fuser.add(img, valid, k)
    merged = fuser.result()
    info = {"ref_row": ref_row, "ref_intr": ref_intr, "n_frames": len(frames), "decoder": decoder,
            "linear": linear, "refine_rms_px": (float(np.mean(rms_all)) if rms_all else None),
            "index_map": fuser.index_map, "seconds": time.time() - t0, "w": w, "h": h}
    return merged, info


def write_image(path, lin, fmt):
    if fmt == "tif":
        out = np.clip(lin, 0, 1) * 65535.0 + 0.5
        cv2.imwrite(path, cv2.cvtColor(out.astype(np.uint16), cv2.COLOR_RGB2BGR))
    else:
        img8 = cv2.cvtColor(to_srgb8(lin), cv2.COLOR_RGB2BGR)
        params = [cv2.IMWRITE_JPEG_QUALITY, 97] if fmt == "jpg" else []
        cv2.imwrite(path, img8, params)


def merge_session(session_dir, ref_index, out_name="merged", fmt="jpg", refine=False, stops=None,
                  levels=6, log=print, half=False, index_maps=False):
    if cv2 is None:
        raise RuntimeError("opencv (cv2) is required: pip install opencv-python-headless rawpy")
    all_stops = load_frames(session_dir)
    todo = sorted(all_stops) if not stops else [s for s in sorted(all_stops) if s in set(stops)]
    out_dir = os.path.join(session_dir, out_name)
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, "merged_frames.csv")
    done = {}
    if os.path.exists(csv_path):
        with open(csv_path) as f:
            done = {int(r["stop"]): r for r in csv.DictReader(f)}
    rows_out = dict(done)
    for stop in todo:
        rows = all_stops[stop]
        name = f"MERGED_{stop:04d}.{fmt}"
        path = os.path.join(out_dir, name)
        if stop in done and os.path.exists(path):
            log(f"[stop {stop}] already merged, skipping")
            continue
        try:
            merged, info = merge_stop(session_dir, rows, ref_index, refine=refine, levels=levels, log=log, half=half)
        except FileNotFoundError as err:
            log(f"[stop {stop}] {err}")
            continue
        write_image(path + ".part", merged, fmt) if False else write_image(path, merged, fmt)
        if index_maps:
            im = info["index_map"].astype(np.float32)
            im = np.clip(im / max(1, info["n_frames"] - 1) * 255.0, 0, 255).astype(np.uint8)
            cv2.imwrite(os.path.join(out_dir, f"MERGED_{stop:04d}_index.png"), im)
        rr, ri = info["ref_row"], info["ref_intr"]
        row = {c: rr.get(c, "") for c in MERGED_COLUMNS}
        row.update({"file": name, "stop": stop, "ref_file": rr["file"], "ref_focus_index": rr["focus_index"],
                    "ref_lens_position": rr["lens_position"], "n_frames": info["n_frames"],
                    "fx": ri["fx"], "fy": ri["fy"], "cx": ri["cx"], "cy": ri["cy"], "w": info["w"], "h": info["h"],
                    "decoder": info["decoder"], "linear": int(info["linear"]), "aligned": "intrinsics+ecc" if refine else "intrinsics",
                    "refine_rms_px": "" if info["refine_rms_px"] is None else f"{info['refine_rms_px']:.2f}"})
        rows_out[stop] = row
        with open(csv_path, "w", newline="") as f:
            wri = csv.DictWriter(f, fieldnames=MERGED_COLUMNS)
            wri.writeheader()
            for s in sorted(rows_out):
                wri.writerow({c: rows_out[s].get(c, "") for c in MERGED_COLUMNS})
        log(f"[stop {stop}] {info['n_frames']} frames -> {name} in {info['seconds']:.1f}s "
            f"(ref k={rr['focus_index']} lens {rr['lens_position']}, fx {ri['fx']:.1f}"
            + (f", ecc rms {info['refine_rms_px']:.2f} px" if info["refine_rms_px"] is not None else "") + ")")
    return csv_path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_dir", help="datasets/phone_<ts> written by the coordinator")
    ap.add_argument("--ref-index", type=int, default=None,
                    help="focus index used as the geometric reference at EVERY stop (default: middle of the stack)")
    ap.add_argument("--out", default="merged", help="output sub-folder name")
    ap.add_argument("--format", choices=["jpg", "png", "tif"], default="jpg")
    ap.add_argument("--refine", action="store_true", help="ECC refinement of the intrinsics-based alignment")
    ap.add_argument("--stops", type=int, nargs="*", help="only these stop ids")
    ap.add_argument("--levels", type=int, default=6)
    ap.add_argument("--half", action="store_true", help="half-resolution decode (quick preview)")
    ap.add_argument("--index-maps", action="store_true", help="also write which frame won per pixel")
    a = ap.parse_args(argv)
    stops = load_frames(a.session_dir)
    if not stops:
        sys.exit("frames.csv has no rows")
    n = max(len(v) for v in stops.values())
    ref = a.ref_index if a.ref_index is not None else n // 2
    print(f"{len(stops)} stops, up to {n} frames each; reference focus index {ref}")
    path = merge_session(a.session_dir, ref, a.out, a.format, a.refine, a.stops, a.levels, half=a.half,
                         index_maps=a.index_maps)
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
