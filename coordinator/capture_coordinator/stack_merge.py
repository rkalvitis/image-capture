"""Mac-side focus-stack merge (docs/DEVELOPMENT_PLAN.md Stage 5).

The phone writes N ProRAW frames per stop, one per lens position. The
ultra-wide breathes with focus (measured on the rig 2026-09-15: scale
0.972 at lens 0.0 -> 1.032 at lens 0.5 relative to lens 0.255; Apple's
per-frame fx ratio overpredicts this by ~25 %), so the frames of one stop
differ in magnification. This module:

1. decodes every DNG linearly (rawpy, sensor-native orientation, camera WB),
2. boxes the beetle in EVERY frame (unwarped, half size; exposure.find_subject),
   measures the focus inside the union of those boxes (variance of the
   Laplacian, gain-normalised) and takes the frame where the beetle is
   sharpest as the REFERENCE (anchor) frame (2026-10-06; before: the frame
   at the init focus, which for `focus_mode: range` was just the middle of
   the stack and often out of focus on the beetle),
   then aligns every frame to the reference:
   a similarity per frame, estimated by ECC between NEIGHBOURING frames
   (similar blur) and chained to the reference, seeded by the intrinsics
   ratio fx_ref/fx_k about the principal point (`--align intrinsics` uses
   the seed only),
3. fuses the aligned frames with one of two rules (`--fusion`):
     pyramid  Laplacian-pyramid maximum-energy selection (Burt & Adelson 1983;
              multi-focus rule after Wang & Chang 2011), one pass, ~2 pyramids
              of memory;
     wavelet  complex-wavelet maximum-modulus selection after Forster, Van De
              Ville, Berent, Sage & Unser 2004 ("Complex Wavelets for Extended
              Depth-of-Field", Microsc. Res. Tech. 65:33-42), using the dual-tree
              complex wavelet transform (Kingsbury 2001; python package `dtcwt`)
              with a majority (median) consistency filter on the selection map
              per level, as in P. Aimonen's focus-stack. Two passes over the
              frames (grey energies -> selection maps; then gather the colour
              coefficients), ~4x the pyramid's time.
4. writes one all-in-focus image per stop plus a CSV row that inherits the
   reference frame's intrinsics and mocap pose from frames.csv, and an
   alignment.csv with the measured vs predicted breathing per frame.

Two session layouts are supported:
  stacks (2026-09-15, frames.csv has an `image` column):
      images/<image>/stack/<frame>.dng + <frame>.json      raw frames + sidecars
      images/<image>/<image>.tif (+ .jpg preview)          the merged image
      images/<image>/stack/stack.json                      lens grid, reference, alignment, frame rows
      stacks.csv, alignment.csv                            at the session root
  flat (older sessions): images/<frame>.dng, sidecars/<frame>.json ->
      <out>/MERGED_<stop>.<fmt>, <out>/merged_frames.csv, <out>/alignment.csv

The merged image has exactly the reference frame's geometry, hence its K;
colmap_export.py uses that. `--delete-raw` removes the stop's DNGs from the
Mac after the merged file was written AND read back (the phone keeps all).

    python -m capture_coordinator.stack_merge <session_dir> [--fusion pyramid|wavelet]
        [--format tif|tif-linear|png|jpg] [--preview] [--jobs 3] [--stops 0 1 2]

Formats: tif = 16-bit sRGB-encoded TIFF (the master; opens correctly in any
viewer), tif-linear = 16-bit linear TIFF, png/jpg = 8-bit sRGB.
Dependencies: numpy, opencv (cv2), rawpy, dtcwt (wavelet fusion).
"""
import argparse
import csv
import json
import multiprocessing as mp
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

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MERGED_COLUMNS = ["image", "file", "stop", "step", "pose", "ref_file", "ref_focus_index", "ref_lens_position",
                  "n_frames", "fx", "fy", "cx", "cy", "w", "h", "capture_time", "mocap_t0", "mocap_t1",
                  "phone_jitter_mm", "pop_jitter_mm", "phone_x", "phone_y", "phone_z", "phone_quat_xyzw",
                  "pop_x", "pop_y", "pop_z", "pop_quat_xyzw", "popillia_yaw_deg",
                  "cam_x", "cam_y", "cam_z", "quat_xyzw", "phone_world",
                  "decoder", "linear", "format", "fusion", "align", "align_max_shift_px", "align_min_cc", "seconds",
                  "subject_box", "best_focus_index", "best_focus_lens", "sharpness_ref", "sharpness_best", "mask_method",
                  "raw_deleted_on_mac"]
ALIGN_COLUMNS = ["stop", "image", "focus_index", "lens_position", "scale_pred", "scale_meas", "shift_px_pred_x",
                 "shift_px_pred_y", "shift_px_meas_x", "shift_px_meas_y", "cc", "method", "sharpness"]
FUSIONS = ("pyramid", "wavelet")
FORMATS = ("tif", "tif-linear", "png", "jpg")


# ---------------------------------------------------------------------------
# decoding
def decode_dng(path, half=False):
    """Linear RGB float32 in [0, 1], sensor-native orientation (no flip),
    camera white balance, sRGB primaries. Returns (img, decoder_name, linear)."""
    if not path.lower().endswith(".dng"):                     # png/jpg/tif (tests, previews): sRGB -> linear
        img = cv2.imread(path, cv2.IMREAD_UNCHANGED | cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError(f"cannot read {path}")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32)
        img /= 65535.0 if img.max() > 255 else 255.0
        img = np.where(img <= 0.04045, img / 12.92, np.power((img + 0.055) / 1.055, 2.4)).astype(np.float32)
        if half:
            img = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2), interpolation=cv2.INTER_AREA)
        return img, "cv2", False
    try:
        import rawpy
    except ImportError:
        rawpy = None
    if rawpy is not None:
        with rawpy.imread(path) as r:
            rgb = r.postprocess(gamma=(1, 1), no_auto_bright=True, output_bps=16, use_camera_wb=True,
                                user_flip=0, half_size=half, output_color=rawpy.ColorSpace.sRGB)
            full_hw = (r.sizes.height, r.sizes.width)
        img = rgb.astype(np.float32) / 65535.0
        if half and img.shape[0] > full_hw[0] // 2 + 1:
            # ProRAW is an already demosaiced linear RGB "Stack" raw: rawpy ignores half_size for it and
            # returns the full 4032x3024. Shrink here so that callers get the size they asked for
            # (found 2026-09-15: the alignment assumed half-size greys -> every shift doubled -> smeared merges).
            img = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2), interpolation=cv2.INTER_AREA)
        return img, "rawpy", True
    with tempfile.TemporaryDirectory() as td:               # fallback: Apple's renderer
        out = os.path.join(td, "f.tiff")
        subprocess.run(["sips", "-s", "format", "tiff", path, "--out", out], check=True, capture_output=True)
        img = cv2.imread(out, cv2.IMREAD_UNCHANGED | cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError(f"sips could not render {path}")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32)
        img /= 65535.0 if img.max() > 255 else 255.0
        if half:
            img = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2), interpolation=cv2.INTER_AREA)
        return img, "sips", False


def to_srgb(lin):
    x = np.clip(lin, 0.0, 1.0)
    return np.where(x <= 0.0031308, 12.92 * x, 1.055 * np.power(x, 1 / 2.4) - 0.055)


def from_srgb(x):
    x = np.clip(x, 0.0, 1.0)
    return np.where(x <= 0.04045, x / 12.92, np.power((x + 0.055) / 1.055, 2.4))


def to_srgb8(lin):
    return (to_srgb(lin) * 255.0 + 0.5).astype(np.uint8)


# ---------------------------------------------------------------------------
# alignment
def breathing_affine(intr_k, intr_ref):
    """3x3 map frame k pixels -> reference pixels from the intrinsics:
    scale fx_ref/fx_k (fy likewise) about k's principal point, then shift so
    the principal points coincide."""
    sx = intr_ref["fx"] / intr_k["fx"]
    sy = intr_ref["fy"] / intr_k["fy"]
    return np.array([[sx, 0.0, intr_ref["cx"] - sx * intr_k["cx"]],
                     [0.0, sy, intr_ref["cy"] - sy * intr_k["cy"]],
                     [0.0, 0.0, 1.0]])


def _ecc_pair(g_template, g_input, init_t2i, iters=200, eps=1e-7, mask=None):
    """ECC: returns (W 3x3 mapping template coords -> input coords, cc) or (init, None)."""
    crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, iters, eps)
    try:
        cc, W = cv2.findTransformECC(g_template, g_input, init_t2i[:2].astype(np.float32),
                                     cv2.MOTION_AFFINE, crit, mask, 5)
    except cv2.error:
        return init_t2i, None
    return np.vstack([W, [0, 0, 1]]).astype(np.float64), float(cc)


def _similarity_from_affine(W, centre):
    """Project a 3x3 affine onto scale+translation (the physical model of focus
    breathing on a rigid mount) while keeping the mapping of `centre`
    unchanged. Dropping a small rotation without re-anchoring the translation
    shifts the whole image (t = c - R c), which accumulated along the chain."""
    s = float(np.sqrt(abs(np.linalg.det(W[:2, :2]))))
    c = np.array([centre[0], centre[1], 1.0])
    p = W @ c
    return np.array([[s, 0.0, p[0] - s * c[0]], [0.0, s, p[1] - s * c[1]], [0.0, 0.0, 1.0]])


def _roi_mask(shape, roi):
    """Central region (fraction of width/height) used for the alignment —
    the subject sits there; background at other depths moves differently."""
    if roi is None or roi >= 0.999:
        return None
    h, w = shape
    m = np.zeros((h, w), np.uint8)
    y0, y1 = int(h * (1 - roi) / 2), int(h * (1 + roi) / 2)
    x0, x1 = int(w * (1 - roi) / 2), int(w * (1 + roi) / 2)
    m[y0:y1, x0:x1] = 255
    return m


def align_stack(grays, intr, ref, small_scale, method="ecc", min_cc=0.3, roi=1.0, subject_mask=None):
    """grays: {k: grey float32 image at `small_scale` of full res}; intr: {k: fx..cy} full res.
    Returns ({k: A_full 3x3 frame->ref}, [alignment rows]).

    Pairwise ECC between NEIGHBOURING frames (similar blur), seeded by the
    intrinsics, projected to scale+shift (the physical model of breathing on
    a rigid mount) and chained to the reference. On the rig the phone does
    not move between frames, so the chain only carries the lens behaviour;
    alignment.csv shows how far it is from the intrinsics' prediction."""
    S = np.diag([small_scale, small_scale, 1.0])
    Sinv = np.linalg.inv(S)
    ks = sorted(grays)
    shape = next(iter(grays.values())).shape
    mask = _roi_mask(shape, roi)
    centre = (shape[1] / 2.0, shape[0] / 2.0)
    C = Cinv = None
    if subject_mask is not None and subject_mask.any():
        # fit on the beetle only (2026-10-06): fitted on the whole frame (mostly background at another depth)
        # the breathing scale was slightly wrong AT THE BEETLE -> it drifted ~0.4 px per frame (s19_p0)
        ys, xs = np.nonzero(subject_mask)
        centre = (float(xs.mean()), float(ys.mean()))     # the similarity keeps the beetle's centre exact
        # ECC on the crop around the beetle, not masked on the whole frame (masked full-frame ECC: 3 min per stop)
        m = 24
        cy0, cy1 = max(0, int(ys.min()) - m), min(shape[0], int(ys.max()) + 1 + m)
        cx0, cx1 = max(0, int(xs.min()) - m), min(shape[1], int(xs.max()) + 1 + m)
        mask = subject_mask[cy0:cy1, cx0:cx1].astype(np.uint8) * 255
        grays = {k: g[cy0:cy1, cx0:cx1] for k, g in grays.items()}
        C = np.array([[1.0, 0.0, cx0], [0.0, 1.0, cy0], [0.0, 0.0, 1.0]])      # crop px -> frame px
        Cinv = np.linalg.inv(C)
    M = {ref: np.eye(3)}          # ref -> k, small coords
    pair = {}
    for direction in (1, -1):
        b = ref
        while True:
            a = b + direction
            if a not in grays:
                break
            P_ba = S @ np.linalg.inv(breathing_affine(intr[a], intr[b])) @ Sinv     # predicted b -> a
            if method == "ecc":
                if C is None:
                    W, cc = _ecc_pair(grays[b], grays[a], P_ba, mask=mask)
                else:
                    W, cc = _ecc_pair(grays[b], grays[a], Cinv @ P_ba @ C, mask=mask)
                    W = C @ W @ Cinv
                used = "ecc"
                if cc is None or cc < min_cc:
                    W, used = P_ba, "intrinsics(fallback)"
                else:
                    W = _similarity_from_affine(W, centre)
            else:
                W, cc, used = P_ba, None, "intrinsics"
            M[a] = W @ M[b]
            pair[a] = (cc, used)
            b = a
    A_full, out = {}, []
    cfull = np.array([centre[0] / small_scale, centre[1] / small_scale, 1.0])
    for k in ks:
        A_full[k] = Sinv @ np.linalg.inv(M[k]) @ S            # frame k -> ref, full res
        pred = breathing_affine(intr[k], intr[ref])
        # where the frame centre lands in the reference (full-res px): predicted by the intrinsics vs measured
        dp = (pred @ cfull)[:2] - cfull[:2]
        dm = (A_full[k] @ cfull)[:2] - cfull[:2]
        cc, used = pair.get(k, (None, "reference"))
        out.append({"focus_index": k, "scale_pred": float(np.sqrt(abs(np.linalg.det(pred[:2, :2])))),
                    "scale_meas": float(np.sqrt(abs(np.linalg.det(A_full[k][:2, :2])))),
                    "shift_px_pred_x": float(dp[0]), "shift_px_pred_y": float(dp[1]),
                    "shift_px_meas_x": float(dm[0]), "shift_px_meas_y": float(dm[1]),
                    "cc": "" if cc is None else round(cc, 4), "method": used})
    return A_full, out


def focus_depth_map(grays, A, ref, small_scale, sigma=2.0, fill_sigma=24.0, reach=7):
    """Per pixel of the reference (alignment-scale px): the focus index of the photo that is sharpest there.
    Energy = blurred |Laplacian| of the UNWARPED grey, then warped (warping first would favour the reference).
    Next to a contrast edge the DEFOCUSED photos have the most energy (their blur reaches past the edge), so
    every pixel takes the index of the strongest pixel within `reach` px: the edge itself, in its sharp photo.
    Pixels without texture (foam, background) carry no information: they take the confidence-weighted
    average of their surroundings, so the foam next to the outline gets the outline's depth.
    Returns (depth float32, known bool); depth is NaN-free, `known` False where nothing textured is near."""
    S = np.diag([small_scale, small_scale, 1.0])
    Sinv = np.linalg.inv(S)
    h, w = next(iter(grays.values())).shape
    emax = np.full((h, w), -1.0, np.float32)
    idx = np.zeros((h, w), np.float32)
    for k in sorted(grays):
        e = cv2.GaussianBlur(np.abs(cv2.Laplacian(grays[k], cv2.CV_32F, ksize=3)), (0, 0), sigma)
        if k != ref:
            e = cv2.warpAffine(e, (S @ A[k] @ Sinv)[:2].astype(np.float32), (w, h), flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=-1.0)
        better = e > emax
        emax[better] = e[better]
        idx[better] = k
    # dilate (energy, index) together: the energy is quantised into the high part, the index rides in the low part
    q = np.floor(np.clip(emax, 0.0, None) / max(float(emax.max()), 1e-12) * 32767.0)
    code = cv2.dilate((q * 128.0 + idx).astype(np.float32), cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2 * reach + 1, 2 * reach + 1)))
    idx = np.mod(code, 128.0).astype(np.float32)
    idx = cv2.medianBlur(idx, 5)                             # single-pixel wins are noise
    floor = float(np.percentile(emax, 50))                   # most of the frame is foam/background = the noise floor
    conf = np.clip(emax - 2.0 * floor, 0.0, None)
    num, den = cv2.GaussianBlur(idx * conf, (0, 0), sigma), cv2.GaussianBlur(conf, (0, 0), sigma)
    numf, denf = cv2.GaussianBlur(idx * conf, (0, 0), fill_sigma), cv2.GaussianBlur(conf, (0, 0), fill_sigma)
    eps = 1e-3 * float(conf.max() or 1.0)
    depth = np.where(den > eps, num / np.maximum(den, 1e-12), numf / np.maximum(denf, 1e-12)).astype(np.float32)
    return depth, denf > 0.02 * eps


def warp_to_ref(img, A, w, h):
    # Lanczos, not bilinear: a bilinear warp keeps only 0.47-0.64 of the fine detail (measured 2026-10-06 on
    # s18_p12, Lanczos 0.89-0.95), so the never-warped reference won the max-energy fusion wherever it was
    # not far out of focus and every other frame arrived softened.
    out = cv2.warpAffine(img, A[:2].astype(np.float32), (w, h), flags=cv2.INTER_LANCZOS4,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    np.maximum(out, 0.0, out=out)                     # Lanczos overshoot at edges
    ones = np.ones(img.shape[:2], np.float32)
    valid = cv2.warpAffine(ones, A[:2].astype(np.float32), (w, h), flags=cv2.INTER_NEAREST,
                           borderMode=cv2.BORDER_CONSTANT, borderValue=0) > 0.5
    return out, valid


# ---------------------------------------------------------------------------
# fusion A: Laplacian pyramid, maximum local energy
def laplacian_pyramid(img, levels):
    pyr, cur = [], img
    for _ in range(levels):
        down = cv2.pyrDown(cur)
        pyr.append(cur - cv2.pyrUp(down, dstsize=(cur.shape[1], cur.shape[0])))
        cur = down
    pyr.append(cur)
    return pyr


def collapse(pyr):
    cur = pyr[-1]
    for lap in reversed(pyr[:-1]):
        cur = cv2.pyrUp(cur, dstsize=(lap.shape[1], lap.shape[0])) + lap
    return cur


class PyramidFuser:
    """Incremental Laplacian-pyramid focus fusion: per level and pixel keep
    the coefficient of the frame with the highest local energy; the base
    level is the mean over the frames that cover the pixel."""
    passes = 1

    def __init__(self, levels=6, energy_ksize=9, **_):
        self.levels, self.ks = levels, energy_ksize
        self.best = self.energy = self.base_n = self.index_map = None
        self.guard = None

    def set_depth_guard(self, depth, known, radius, spread=9):
        """Only photos within `radius` focus steps of the pixel's own sharpest photo (focus_depth_map) may win a
        coefficient. Without it every level picks independently, and at the coarse levels the DEFOCUSED photos
        win around every contrast edge: a grey band along the outline, dust grains drawn as stars (2026-10-06).
        The allowed range is the min..max depth of the neighbourhood (wider at coarse levels), so both sides
        of a depth step (a leg in front of the body) stay available."""
        self.guard = (depth, known, float(radius), spread)
        self._guard_lvl = {}

    def _allowed(self, lvl, shape, index):
        if self.guard is None:
            return None
        if lvl not in self._guard_lvl:
            depth, known, r, spread = self.guard
            k = spread * (2 ** max(0, lvl - 1)) | 1
            ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            lo = np.where(known, cv2.erode(depth, ker) - r, -1e9).astype(np.float32)
            hi = np.where(known, cv2.dilate(depth, ker) + r, 1e9).astype(np.float32)
            size = (shape[1], shape[0])
            self._guard_lvl[lvl] = (cv2.resize(lo, size, interpolation=cv2.INTER_NEAREST),
                                    cv2.resize(hi, size, interpolation=cv2.INTER_NEAREST))
        lo, hi = self._guard_lvl[lvl]
        return (index >= lo) & (index <= hi)

    def add(self, img, valid, index):
        pyr = laplacian_pyramid(img, self.levels)
        vmask = valid.astype(np.float32)
        first = self.best is None
        if first:
            self.best = [None] * (self.levels + 1)
            self.energy = [None] * self.levels
            self.index_map = np.full(img.shape[:2], -1, np.int16)
        for lvl, lap in enumerate(pyr):
            v = vmask if lvl == 0 else cv2.resize(vmask, (lap.shape[1], lap.shape[0]), interpolation=cv2.INTER_AREA)
            ok = v > 0.999
            if lvl == self.levels:                          # base
                if first:
                    self.best[lvl] = lap * ok[..., None]
                    self.base_n = ok.astype(np.float32)
                else:
                    self.best[lvl][ok] += lap[ok]
                    self.base_n[ok] += 1
                continue
            e = cv2.GaussianBlur(np.abs(lap).sum(axis=2), (self.ks, self.ks), 0)
            allowed = self._allowed(lvl, lap.shape, index)
            if allowed is not None:
                e = np.where(allowed, e, e * 1e-6)           # still beats "no photo covers this pixel" (-1)
            e = np.where(ok, e, -1.0).astype(np.float32)
            if first:
                self.best[lvl], self.energy[lvl] = lap.copy(), e
                if lvl == 0:
                    self.index_map[ok] = index
            else:
                better = e > self.energy[lvl]
                self.best[lvl][better] = lap[better]
                self.energy[lvl][better] = e[better]
                if lvl == 0:
                    self.index_map[better] = index

    def result(self):
        pyr = list(self.best)
        pyr[-1] = pyr[-1] / np.maximum(self.base_n, 1.0)[..., None]
        return collapse(pyr)


# ---------------------------------------------------------------------------
# fusion B: complex wavelets (Forster et al. 2004), dual-tree CWT
class WaveletFuser:
    """Complex-wavelet focus fusion after Forster, Van De Ville, Berent, Sage
    & Unser (2004) with the dual-tree complex wavelet transform (Kingsbury;
    package dtcwt). Pass 1 (`add_energy`): per level and position the energy
    sum_o |c_o|^2 over the 6 oriented subbands of the grey image of every
    frame -> the frame with the maximum energy wins (Forster's maximum-
    modulus rule), followed by a majority (median) consistency filter on the
    selection map of each level. Pass 2 (`add_colour`): gather the winning
    complex coefficients of every colour channel; the lowpass residual comes
    from the winner of the coarsest level. Inverse transform per channel."""
    passes = 2

    def __init__(self, levels=6, consistency=1, border=2, **_):
        import dtcwt
        self.T = dtcwt.Transform2d()
        self.levels, self.consistency, self.border = levels, consistency, border
        self.energy = None          # [level] -> list of (k, energy array)
        self.win = None             # [level] -> int16 winner index per position
        self.win_low = None
        self.hp = self.low = None   # gathered coefficients per channel
        self.shape = None
        self.index_map = None

    @staticmethod
    def _grey(img):
        return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY) if img.ndim == 3 else img

    def _valid_at(self, valid, shape):
        v = cv2.resize(valid.astype(np.float32), (shape[1], shape[0]), interpolation=cv2.INTER_AREA) > 0.999
        if self.border > 0:
            v = cv2.erode(v.astype(np.uint8), np.ones((3, 3), np.uint8), iterations=self.border) > 0
        return v

    def add_energy(self, img, valid, index):
        self.shape = img.shape[:2]
        p = self.T.forward(self._grey(img), nlevels=self.levels)
        if self.energy is None:
            self.energy = [[] for _ in p.highpasses]
            self.low_shape = p.lowpass.shape
        for lvl, hp in enumerate(p.highpasses):
            e = (np.abs(hp) ** 2).sum(axis=2).astype(np.float32)
            e[~self._valid_at(valid, e.shape)] = -1.0
            self.energy[lvl].append((index, e))

    def decide(self):
        """Winner maps per level from the collected energies (+ consistency filter)."""
        self.win = []
        for lvl, lst in enumerate(self.energy):
            ks = np.array([k for k, _ in lst], np.int16)
            E = np.stack([e for _, e in lst], axis=0)
            win = ks[np.argmax(E, axis=0)]
            if self.consistency > 0 and win.size > 9:
                ksize = 3 if self.consistency == 1 else 5
                win = cv2.medianBlur(win.astype(np.uint8), ksize).astype(np.int16)   # frame ids < 256
            self.win.append(win)
        self.energy = None
        last = self.win[-1]
        self.win_low = cv2.resize(last.astype(np.float32), (self.low_shape[1], self.low_shape[0]),
                                  interpolation=cv2.INTER_NEAREST).astype(np.int16)
        self.index_map = cv2.resize(self.win[0].astype(np.float32), (self.shape[1], self.shape[0]),
                                    interpolation=cv2.INTER_NEAREST).astype(np.int16)
        self.hp = [[None] * self.levels for _ in range(3)]
        self.low = [None] * 3

    def add_colour(self, img, valid, index):
        if self.win is None:
            self.decide()
        for c in range(3):
            p = self.T.forward(img[..., c], nlevels=self.levels)
            for lvl, hp in enumerate(p.highpasses):
                m = self.win[lvl] == index
                if self.hp[c][lvl] is None:
                    self.hp[c][lvl] = np.zeros_like(hp)
                self.hp[c][lvl][m] = hp[m]
            m = self.win_low == index
            if self.low[c] is None:
                self.low[c] = np.zeros_like(p.lowpass)
            self.low[c][m] = p.lowpass[m]

    def result(self):
        import dtcwt
        h, w = self.shape
        out = np.zeros((h, w, 3), np.float32)
        for c in range(3):
            rec = self.T.inverse(dtcwt.Pyramid(self.low[c], tuple(self.hp[c])))
            out[..., c] = rec[:h, :w]
        return out


def make_fuser(fusion, levels, consistency=1):
    if fusion == "wavelet":
        return WaveletFuser(levels=levels, consistency=consistency)
    if fusion == "pyramid":
        return PyramidFuser(levels=levels)
    raise ValueError(f"unknown fusion {fusion!r} (choices: {FUSIONS})")


# ---------------------------------------------------------------------------
# session handling (both layouts)
def load_frames(session_dir):
    from .session import read_frames_csv                 # tolerates a header-less frames.csv (2026-09-30)
    rows = read_frames_csv(os.path.join(session_dir, "frames.csv"))
    stops = {}
    for r in rows:
        stops.setdefault(int(r["stop"]), []).append(r)
    for s in stops.values():
        s.sort(key=lambda r: int(r["focus_index"]))
    return stops


def layout_of(rows):
    """'stacks' when frames.csv names the per-stop image folder, else 'flat'."""
    return "stacks" if any(r.get("image") for r in rows) else "flat"


def frame_path(session_dir, row):
    if row.get("raw_dir"):                           # the cloud layout: every DNG + sidecar flat in one folder (raw/)
        return os.path.join(row["raw_dir"], row["file"])
    if row.get("image"):
        return os.path.join(session_dir, "images", row["image"], "stack", row["file"])
    return os.path.join(session_dir, "images", row["file"])


def sidecar_path(session_dir, row):
    base = os.path.splitext(row["file"])[0] + ".json"
    if row.get("raw_dir"):
        return os.path.join(row["raw_dir"], base)
    if row.get("image"):
        return os.path.join(session_dir, "images", row["image"], "stack", base)
    return os.path.join(session_dir, "sidecars", base)


def sharpest_index(sharpness):
    """Focus index with the highest beetle sharpness, after a 3-tap mean over
    neighbouring focus indices (one noisy frame must not win)."""
    ks = sorted(sharpness)
    sm = {k: np.mean([sharpness[j] for j in (ks[max(0, i - 1)], k, ks[min(len(ks) - 1, i + 1)])])
          for i, k in enumerate(ks)}
    return max(sm, key=sm.get)


def choose_reference(frames, ref_index=None, sharpness=None):
    """Reference focus index for a stop: `ref_index` if given (>= 0), else the
    frame where the beetle is sharpest (`sharpness` from beetle_sharpness),
    else the frame whose lens position is the init focus (frames.csv
    focus_centre), else the middle of the stack."""
    by_idx = {int(r["focus_index"]): r for r in frames}
    if ref_index is not None and ref_index >= 0:
        return ref_index if ref_index in by_idx else min(by_idx, key=lambda i: abs(i - ref_index))
    if sharpness:
        return sharpest_index({k: v for k, v in sharpness.items() if k in by_idx})
    centres = [r.get("focus_centre") for r in frames if r.get("focus_centre") not in (None, "")]
    if centres:
        c = float(centres[0])
        return min(by_idx, key=lambda i: abs(float(by_idx[i]["lens_position"]) - c))
    ks = sorted(by_idx)
    return ks[len(ks) // 2]


def intrinsics_of(row, session_dir):
    try:
        return {k: float(row[k]) for k in ("fx", "fy", "cx", "cy")}
    except (KeyError, ValueError):
        with open(sidecar_path(session_dir, row)) as f:
            intr = json.load(f)["intrinsics"]
        return {k: float(intr[k]) for k in ("fx", "fy", "cx", "cy")}


def subject_sharpness(img, box):
    """Variance of the Laplacian of the grey image inside `box` (y0, y1, x0, x1)."""
    g = cv2.cvtColor(img[box[0]:box[1], box[2]:box[3]], cv2.COLOR_RGB2GRAY)
    return float(cv2.Laplacian(g, cv2.CV_32F, ksize=3).var())


def beetle_sharpness(greys, dark_frac=0.6, edge_px=6):
    """greys: {k: unwarped grey image, each divided by its own median}.
    1. boxes the beetle in every frame (exposure.find_subject), drops boxes whose
       centre is far from the median centre, union box (padded);
    2. MASK of the beetle itself: per frame the pixels inside the box darker than
       `dark_frac` x the surroundings (median of the box's border pixels: foam or
       grey background, never the beetle; not a high percentile, the bright foam
       rim made the grey background count as beetle), opened to cut the pin,
       largest blob; OR over all frames (defocused frames are wider), dilated by
       `edge_px` so the outline against the foam counts;
    3. sharpness per frame = variance of the Laplacian over the mask pixels only.
    Returns (box (y0, y1, x0, x1) in the greys' pixels or None, mask (uint8, box-sized) or None, {k: sharpness})."""
    from .exposure import find_subject
    boxes = [b for b in (find_subject(g) for g in greys.values()) if b is not None]
    if not boxes:
        return None, None, {}
    B = np.array(boxes, float)
    cy, cx = (B[:, 0] + B[:, 1]) / 2, (B[:, 2] + B[:, 3]) / 2
    size = max(1.0, float(np.median(np.maximum(B[:, 1] - B[:, 0], B[:, 3] - B[:, 2]))))
    keep = np.hypot(cy - np.median(cy), cx - np.median(cx)) <= 0.5 * size
    if keep.any():
        B = B[keep]
    h, w = next(iter(greys.values())).shape
    pad = max(10, int(0.1 * size))
    box = (max(0, int(B[:, 0].min()) - pad), min(h, int(B[:, 1].max()) + pad),
           max(0, int(B[:, 2].min()) - pad), min(w, int(B[:, 3].max()) + pad))
    ko = max(3, int(0.08 * size)) | 1                 # opening: removes the pin (thin), keeps the body
    open_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ko, ko))
    mask = None
    for g in greys.values():
        sub = cv2.GaussianBlur(g[box[0]:box[1], box[2]:box[3]], (0, 0), 1.0)
        white = float(np.median(np.concatenate([sub[0], sub[-1], sub[:, 0], sub[:, -1]])))
        m = cv2.morphologyEx((sub < dark_frac * white).astype(np.uint8), cv2.MORPH_OPEN, open_k)
        n, lab, stats, _ = cv2.connectedComponentsWithStats(m)
        if n < 2:
            continue
        m = (lab == 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))).astype(np.uint8)
        mask = m if mask is None else (mask | m)
    if mask is None or mask.sum() < 100:
        mask = np.ones((box[1] - box[0], box[3] - box[2]), np.uint8)     # no blob: whole box
    else:
        mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * edge_px + 1, 2 * edge_px + 1)))
    return box, mask, score_in_mask(greys, box, mask)


def score_in_mask(greys, box, mask):
    """{k: variance of the Laplacian (OpenCV, ksize 3) of greys[k] over the mask pixels}; mask is box-sized."""
    sel = mask > 0
    return {k: float(cv2.Laplacian(g[box[0]:box[1], box[2]:box[3]], cv2.CV_32F, ksize=3)[sel].var())
            for k, g in greys.items()}


def sam3_beetle_mask(rgb, session_dir, python=None, prompt="insect", pad=10):
    """SAM 3 text-prompt mask of the beetle on one linear RGB frame (sam3_mask.py in
    the optitrack `masking` env, one SAM 3 process at a time: session_dir/sam3.lock).
    Returns (box (y0, y1, x0, x1), box-sized uint8 mask, full-frame uint8 mask, info) or (None, None, None, info)."""
    from .sam3_mask import masking_python
    py = python or masking_python(os.path.join(REPO_ROOT, "optitrack"))
    info = {"method": "sam3", "prompt": prompt}
    if not py or not os.path.exists(py):
        info["error"] = f"no SAM 3 interpreter {py or ''} (pixi install -e masking in optitrack/)"
        return None, None, None, info
    with tempfile.TemporaryDirectory() as td:
        src, dst = os.path.join(td, "in.jpg"), os.path.join(td, "out.png")
        cv2.imwrite(src, cv2.cvtColor(to_srgb8(rgb), cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 95])
        env = dict(os.environ, PYTHONPATH=os.path.join(REPO_ROOT, "coordinator"))
        t = time.time()
        with _CsvLock(os.path.join(session_dir, "sam3")):
            r = subprocess.run([py, "-m", "capture_coordinator.sam3_mask", "--prompt", prompt, src, dst],
                               capture_output=True, text=True, env=env, timeout=900)
        info["seconds"] = round(time.time() - t, 1)
        res = next((json.loads(l) for l in reversed(r.stdout.splitlines()) if l.startswith("{")), None)
        if r.returncode != 0 or res is None or res.get("score") is None:
            info["error"] = (r.stderr.strip().splitlines() or ["no instance found"])[-1][:300] if res is None \
                else f"no '{prompt}' found"
            return None, None, None, info
        full = (cv2.imread(dst, cv2.IMREAD_GRAYSCALE) > 127).astype(np.uint8)
    info["score"] = res["score"]
    ys, xs = np.nonzero(full)
    if len(ys) < 100:
        info["error"] = "mask too small"
        return None, None, None, info
    h, w = full.shape
    box = (max(0, int(ys.min()) - pad), min(h, int(ys.max()) + 1 + pad),
           max(0, int(xs.min()) - pad), min(w, int(xs.max()) + 1 + pad))
    return box, full[box[0]:box[1], box[2]:box[3]].copy(), full, info


def _aligned_frames(paths, A, ref_idx, half, box):
    """Generator: (k, aligned linear float32 image, valid mask). Every frame is
    gained to the median of the first frame decoded (locked exposure -> ~1)."""
    h = w = None
    for k in sorted(paths):
        img, decoder, linear = decode_dng(paths[k], half=half)
        box.setdefault("decoder", decoder)
        box.setdefault("linear", linear)
        if h is None:
            h, w = img.shape[:2]
            box.setdefault("hw", (h, w))
        med = float(np.median(img))
        ref_med = box.setdefault("median", med)
        if med > 0 and abs(med / ref_med - 1.0) > 1e-3:
            img = img * (ref_med / med)
        if k == ref_idx:
            valid = np.ones((h, w), bool)
        else:
            img, valid = warp_to_ref(img, A[k], w, h)
        yield k, img, valid


def _encoded(frames, space):
    """Aligned frames in the fusion space: linear as decoded, or sRGB-encoded (the beetle sits at ~1 % linear
    next to foam at 80 %: in linear the foam edge outweighs every detail of the body)."""
    for k, img, valid in frames:
        yield k, (to_srgb(img).astype(np.float32) if space == "srgb" else img), valid


def merge_stop(session_dir, rows, ref_index, align="ecc", levels=6, half=False, small_scale=0.5, roi=1.0,
               fusion="pyramid", consistency=1, mask="threshold", sam3_python=None, sam3_prompt="insect",
               align_on="beetle", min_rel_sharpness=0.0, depth_guard=0.0, fusion_space="linear"):
    """Merge one stop. Returns (linear float image, info dict).
    fusion_space: linear | srgb = the encoding the fusion compares and blends in (the result is linear either way).
    depth_guard: pyramid fusion only; > 0 = a pixel takes its detail only from photos within this many focus
    steps of its own sharpest photo (PyramidFuser.set_depth_guard); 0 = every photo competes everywhere.
    align_on: beetle = fit the alignment on the beetle mask (dilated), frame = whole frame / `roi`.
    min_rel_sharpness: drop frames whose beetle score is below this fraction of the peak (0 = keep all)."""
    frames = [r for r in rows if os.path.exists(frame_path(session_dir, r))]
    if not frames:
        raise FileNotFoundError("no images for this stop")
    t0 = time.time()
    by_idx = {int(r["focus_index"]): r for r in frames}
    div = 2.0 if half else 1.0
    intr = {k: {kk: v / div for kk, v in intrinsics_of(r, session_dir).items()} for k, r in by_idx.items()}
    paths = {k: frame_path(session_dir, r) for k, r in by_idx.items()}

    # pass 0: half-size greys -> beetle box + sharpness per UNWARPED frame (picks the reference),
    # then shrunk for the alignment
    grays, halfs = {}, {}
    for k in sorted(by_idx):
        img, _, _ = decode_dng(paths[k], half=True)
        g = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        med = float(np.median(g))
        halfs[k] = g / med if med > 0 else g
        f = 2.0 * small_scale / div        # half-size decode = 0.5 of full = div/2 of the output; want small_scale of the output
        if abs(f - 1.0) > 1e-6:
            g = cv2.resize(g, None, fx=f, fy=f, interpolation=cv2.INTER_AREA)
        grays[k] = g
    hbox, hmask, sharp = beetle_sharpness(halfs)
    mask_info, sam_full = {"method": "threshold"}, None
    if mask == "sam3":
        # SAM 3 on the provisional sharpest frame (threshold mask): its outline is the crispest
        k0 = choose_reference(frames, -1, sharp)
        rgb, _, _ = decode_dng(paths[k0], half=True)
        b, m, sam_full, mask_info = sam3_beetle_mask(rgb, session_dir, sam3_python, sam3_prompt)
        mask_info["frame"] = k0
        if b is not None:
            hbox, hmask, sharp = b, m, score_in_mask(halfs, b, m)
        else:
            mask_info["fallback"] = "threshold"
    del halfs
    ref_idx = choose_reference(frames, ref_index, sharp)
    s = 2.0 / div                                          # half-size px -> output px
    subject_box = None if hbox is None else [int(round(v * s)) for v in hbox]
    dropped = []
    if min_rel_sharpness > 0 and sharp:
        peak = max(sharp.values())
        dropped = sorted(k for k, v in sharp.items() if v < min_rel_sharpness * peak and k != ref_idx)
        keep = [k for k in sorted(by_idx) if k not in dropped]   # contiguous in practice (one focus peak)
        for k in dropped:
            by_idx.pop(k); paths.pop(k); grays.pop(k); intr.pop(k)
        frames = [r for r in frames if int(r["focus_index"]) in by_idx]
    subject_mask = None
    if align_on == "beetle" and hbox is not None and hmask is not None and not hmask.all():
        full = np.zeros(next(iter(grays.values())).shape[:2], np.uint8)
        f2 = 2.0 * small_scale / div                       # half-size px -> alignment (small) px (as the greys)
        mb = cv2.resize(hmask, None, fx=f2, fy=f2, interpolation=cv2.INTER_NEAREST)
        y0, x0 = int(round(hbox[0] * f2)), int(round(hbox[2] * f2))
        mb = mb[:full.shape[0] - y0, :full.shape[1] - x0]
        full[y0:y0 + mb.shape[0], x0:x0 + mb.shape[1]] = mb
        subject_mask = cv2.dilate(full, np.ones((15, 15), np.uint8)) > 0
    A, align_rows = align_stack(grays, intr, ref_idx, small_scale, method=align, roi=roi, subject_mask=subject_mask)
    depth = None
    if depth_guard > 0 and fusion == "pyramid":
        depth = focus_depth_map(grays, A, ref_idx, small_scale)
    del grays
    for a in align_rows:
        a["stop"] = int(frames[0]["stop"]); a["lens_position"] = by_idx[a["focus_index"]]["lens_position"]
        a["image"] = frames[0].get("image", "")
    t_align = time.time() - t0

    # fusion passes
    fuser = make_fuser(fusion, levels, consistency)
    if depth is not None:
        fuser.set_depth_guard(depth[0], depth[1], depth_guard)
    box = {}
    if fuser.passes == 2:
        for k, img, valid in _encoded(_aligned_frames(paths, A, ref_idx, half, box), fusion_space):
            fuser.add_energy(img, valid, k)
        fuser.decide()
        for k, img, valid in _encoded(_aligned_frames(paths, A, ref_idx, half, box), fusion_space):
            fuser.add_colour(img, valid, k)
    else:
        for k, img, valid in _encoded(_aligned_frames(paths, A, ref_idx, half, box), fusion_space):
            fuser.add(img, valid, k)
    merged = fuser.result()
    if fusion_space == "srgb":
        merged = from_srgb(merged).astype(np.float32)
    h, w = box["hw"]
    for a in align_rows:
        a["sharpness"] = round(sharp[a["focus_index"]], 6) if a["focus_index"] in sharp else ""
    best_k = sharpest_index(sharp) if sharp else None
    ccs = [a["cc"] for a in align_rows if a["cc"] != ""]
    shifts = [np.hypot(a["shift_px_meas_x"] - a["shift_px_pred_x"], a["shift_px_meas_y"] - a["shift_px_pred_y"])
              for a in align_rows]
    info = {"ref_row": by_idx[ref_idx], "ref_intr": intr[ref_idx], "n_frames": len(frames), "decoder": box["decoder"],
            "linear": box["linear"], "align": f"{align}(beetle)" if subject_mask is not None else align if roi >= 0.999 else f"{align}(roi {roi:g})",
            "align_rows": align_rows, "index_map": fuser.index_map, "fusion": fusion,
            "align_max_shift_px": float(max(shifts)) if shifts else 0.0,
            "align_min_cc": float(min(ccs)) if ccs else None, "seconds": time.time() - t0,
            "seconds_align": t_align, "w": w, "h": h, "lens_positions": [by_idx[k]["lens_position"] for k in sorted(by_idx)],
            "files": [by_idx[k]["file"] for k in sorted(by_idx)],
            "align_on": "beetle" if subject_mask is not None else "frame", "dropped_frames": dropped,
            "fusion_space": fusion_space, "depth_guard": depth_guard if depth is not None else 0.0, "depth_map": None if depth is None else depth[0],
            "subject_box": subject_box, "beetle_mask": hmask, "mask_info": mask_info, "sam3_mask": sam_full, "beetle_box_half": hbox, "sharpness": {int(k): v for k, v in sharp.items()},
            "best_focus_index": best_k, "best_focus_lens": float(by_idx[best_k]["lens_position"]) if best_k is not None else None,
            "sharpness_ref": sharp.get(ref_idx), "sharpness_best": sharp.get(best_k) if best_k is not None else None}
    return merged, info


def write_image(path, lin, fmt):
    """tif = 16-bit sRGB-encoded, tif-linear = 16-bit linear, png/jpg = 8-bit sRGB."""
    if fmt in ("tif", "tif-linear"):
        data = np.clip(lin, 0, 1) if fmt == "tif-linear" else to_srgb(lin)
        out = data * 65535.0 + 0.5
        cv2.imwrite(path, cv2.cvtColor(out.astype(np.uint16), cv2.COLOR_RGB2BGR))
    else:
        img8 = cv2.cvtColor(to_srgb8(lin), cv2.COLOR_RGB2BGR)
        cv2.imwrite(path, img8, [cv2.IMWRITE_JPEG_QUALITY, 97] if fmt == "jpg" else [])


def write_preview(path, lin, max_side=2016, quality=92):
    img8 = cv2.cvtColor(to_srgb8(lin), cv2.COLOR_RGB2BGR)
    h, w = img8.shape[:2]
    s = min(1.0, max_side / max(h, w))
    if s < 1.0:
        img8 = cv2.resize(img8, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)
    cv2.imwrite(path, img8, [cv2.IMWRITE_JPEG_QUALITY, quality])


def _verify_image(path, h, w):
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    return img is not None and img.shape[0] == h and img.shape[1] == w


def _merge_one(job):
    session_dir, stop, rows, fmt = job["session_dir"], job["stop"], job["rows"], job["fmt"]
    ext = "tif" if fmt.startswith("tif") else fmt
    image = rows[0].get("image", "")
    if image:                                        # stacks layout
        out_dir = os.path.join(session_dir, "images", image)
        name = f"{image}.{ext}"
        stack_dir = os.path.join(out_dir, "stack")
    else:
        out_dir = job["out_dir"]
        name = f"MERGED_{stop:04d}.{ext}"
        stack_dir = out_dir
    path = os.path.join(out_dir, name)
    try:
        merged, info = merge_stop(session_dir, rows, job["ref_index"], align=job["align"], levels=job["levels"],
                                  half=job["half"], roi=job["roi"], fusion=job["fusion"], consistency=job["consistency"],
                                  mask=job.get("mask", "threshold"), sam3_python=job.get("sam3_python"),
                                  sam3_prompt=job.get("sam3_prompt", "insect"), align_on=job.get("align_on", "beetle"),
                                  min_rel_sharpness=job.get("min_rel_sharpness", 0.0),
                                  depth_guard=job.get("depth_guard", 0.0),
                                  fusion_space=job.get("fusion_space", "linear"))
    except FileNotFoundError as err:
        return stop, None, str(err)
    os.makedirs(out_dir, exist_ok=True)
    tmp = os.path.join(out_dir, f".{name}.part.{ext}")
    write_image(tmp, merged, fmt)
    os.replace(tmp, path)
    if job["preview"] and ext != "jpg":
        write_preview(os.path.join(out_dir, (image or f"MERGED_{stop:04d}") + ".jpg"), merged)
    if job["index_maps"] and info["index_map"] is not None:
        im = np.clip(info["index_map"].astype(np.float32) / max(1, info["n_frames"] - 1) * 255.0, 0, 255)
        cv2.imwrite(os.path.join(stack_dir, (image or f"MERGED_{stop:04d}") + "_index.png"), im.astype(np.uint8))
    if info["beetle_mask"] is not None:                # half-size mask the reference was chosen on (check it)
        y0, y1, x0, x1 = info["beetle_box_half"]
        g = cv2.cvtColor(to_srgb8(cv2.resize(merged, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
                                  if not job["half"] else merged), cv2.COLOR_RGB2BGR)[y0:y1, x0:x1].copy()
        edge = info["beetle_mask"] - cv2.erode(info["beetle_mask"], np.ones((3, 3), np.uint8))
        g[edge > 0] = (0, 0, 255)
        cv2.imwrite(os.path.join(stack_dir, (image or f"MERGED_{stop:04d}") + "_beetle_mask.png"), g)
    if info["depth_map"] is not None:                  # which photo is sharpest where (0 = first, 255 = last)
        dm = np.clip(info["depth_map"] / max(1, max(info["sharpness"] or [1])) * 255.0, 0, 255)
        cv2.imwrite(os.path.join(stack_dir, (image or f"MERGED_{stop:04d}") + "_depth.png"), dm.astype(np.uint8))
    if info["sam3_mask"] is not None:                  # the SAM 3 mask itself (half size, 255 = insect)
        cv2.imwrite(os.path.join(stack_dir, (image or f"MERGED_{stop:04d}") + "_sam3_mask.png"), info["sam3_mask"] * 255)
    rr, ri = info["ref_row"], info["ref_intr"]
    row = {c: rr.get(c, "") for c in MERGED_COLUMNS}
    row.update({"image": image, "file": name, "stop": stop, "ref_file": rr["file"], "ref_focus_index": rr["focus_index"],
                "ref_lens_position": rr["lens_position"], "n_frames": info["n_frames"],
                "fx": ri["fx"], "fy": ri["fy"], "cx": ri["cx"], "cy": ri["cy"], "w": info["w"], "h": info["h"],
                "decoder": info["decoder"], "linear": int(info["linear"] and fmt == "tif-linear"), "format": fmt,
                "fusion": info["fusion"], "align": info["align"],
                "align_max_shift_px": f"{info['align_max_shift_px']:.1f}",
                "align_min_cc": "" if info["align_min_cc"] is None else f"{info['align_min_cc']:.3f}",
                "seconds": f"{info['seconds']:.1f}", "raw_deleted_on_mac": 0,
                "subject_box": " ".join(str(v) for v in info["subject_box"]) if info["subject_box"] else "",
                "best_focus_index": "" if info["best_focus_index"] is None else info["best_focus_index"],
                "best_focus_lens": "" if info["best_focus_lens"] is None else f"{info['best_focus_lens']:.4f}",
                "sharpness_ref": "" if info["sharpness_ref"] is None else f"{info['sharpness_ref']:.6f}",
                "sharpness_best": "" if info["sharpness_best"] is None else f"{info['sharpness_best']:.6f}",
                "mask_method": info["mask_info"]["method"] + ("(failed->threshold)" if info["mask_info"].get("fallback") else "")})
    verified = os.path.getsize(path) > 10000 and _verify_image(path, info["h"], info["w"])
    if job["delete_raw"] and verified:
        # the MAC copies only (the phone keeps every frame); sidecars stay
        for r in rows:
            p = frame_path(session_dir, r)
            if os.path.exists(p):
                os.remove(p)
        row["raw_deleted_on_mac"] = 1
    if image:
        summary = {"image": image, "stop": stop, "step": rr.get("step"), "pose": rr.get("pose"),
                   "merged_file": name, "format": fmt, "fusion": info["fusion"], "align": info["align"],
                   "reference": {"file": rr["file"], "focus_index": int(rr["focus_index"]),
                                 "lens_position": float(rr["lens_position"]), "intrinsics": ri},
                   "n_frames": info["n_frames"], "lens_positions": [float(v) for v in info["lens_positions"]],
                   "files": info["files"], "decoder": info["decoder"], "linear_decode": bool(info["linear"]),
                   "seconds": round(info["seconds"], 1), "seconds_align": round(info["seconds_align"], 1),
                   "align_max_shift_px": info["align_max_shift_px"], "align_min_cc": info["align_min_cc"],
                   "subject_box": info["subject_box"], "beetle_mask": info["mask_info"],
                   "align_on": info["align_on"], "dropped_frames": info["dropped_frames"],
                   "depth_guard": info["depth_guard"], "fusion_space": info["fusion_space"], "sharpness_by_focus_index": info["sharpness"],
                   "best_focus_index": info["best_focus_index"], "best_focus_lens": info["best_focus_lens"],
                   "verified": bool(verified), "raw_deleted_on_mac": bool(row["raw_deleted_on_mac"]),
                   "alignment": info["align_rows"], "frames": rows}
        with open(os.path.join(stack_dir, "stack.json"), "w") as f:
            json.dump(summary, f, indent=1, default=str)
    return stop, row, info["align_rows"]


def _write_csv(path, columns, rows):
    tmp = path + f".part{os.getpid()}"
    with open(tmp, "w", newline="") as f:
        wri = csv.DictWriter(f, fieldnames=columns)
        wri.writeheader()
        for r in rows:
            wri.writerow({c: r.get(c, "") for c in columns})
    os.replace(tmp, path)


class _CsvLock:
    """Inter-process lock around the read-modify-write of the session CSVs
    (several single-stop merge subprocesses run in parallel; without the lock
    each rewrote the file from its own stale snapshot and rows were lost)."""

    def __init__(self, path):
        self.path = path + ".lock"
        self.f = None

    def __enter__(self):
        self.f = open(self.path, "w")
        if os.name == "nt":                          # Windows has no fcntl
            import msvcrt
            while True:
                try:
                    msvcrt.locking(self.f.fileno(), msvcrt.LK_LOCK, 1)     # retries for 10 s, then OSError
                    break
                except OSError:
                    continue
        else:
            import fcntl
            fcntl.flock(self.f, fcntl.LOCK_EX)
        return self

    def __exit__(self, *a):
        if os.name == "nt":
            import msvcrt
            self.f.seek(0)
            msvcrt.locking(self.f.fileno(), msvcrt.LK_UNLCK, 1)
        else:
            import fcntl
            fcntl.flock(self.f, fcntl.LOCK_UN)
        self.f.close()


def _read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def _update_csvs(csv_path, align_path, stop, row, extra):
    """Merge one stop's rows into the CSVs under the lock (re-reading them first)."""
    with _CsvLock(csv_path):
        done = {int(r["stop"]): r for r in _read_csv(csv_path)}
        done[stop] = row
        align_rows = [a for a in _read_csv(align_path) if int(a["stop"]) != stop] + extra
        _write_csv(csv_path, MERGED_COLUMNS, [done[s] for s in sorted(done)])
        _write_csv(align_path, ALIGN_COLUMNS, sorted(align_rows, key=lambda a: (int(a["stop"]), int(a["focus_index"]))))
        return done


def rebuild_csvs(session_dir, log=print):
    """Regenerate stacks.csv + alignment.csv of a stacks-layout session from
    every images/<image>/stack/stack.json (the per-stop record is complete)."""
    import glob
    rows, align = [], []
    for p in sorted(glob.glob(os.path.join(session_dir, "images", "*", "stack", "stack.json"))):
        with open(p) as f:
            d = json.load(f)
        image = d["image"]
        if not os.path.exists(os.path.join(session_dir, "images", image, d["merged_file"])):
            continue
        rr = d["frames"][int(d["reference"]["focus_index"])] if len(d["frames"]) > int(d["reference"]["focus_index"]) else d["frames"][0]
        ref = next((fr for fr in d["frames"] if int(fr["focus_index"]) == int(d["reference"]["focus_index"])), rr)
        ri = d["reference"]["intrinsics"]
        sharp = d.get("sharpness_by_focus_index") or {}
        row = {c: ref.get(c, "") for c in MERGED_COLUMNS}
        row.update({"image": image, "file": d["merged_file"], "stop": d["stop"], "ref_file": ref["file"],
                    "ref_focus_index": ref["focus_index"], "ref_lens_position": ref["lens_position"], "n_frames": d["n_frames"],
                    "fx": ri["fx"], "fy": ri["fy"], "cx": ri["cx"], "cy": ri["cy"], "decoder": d["decoder"],
                    "linear": int(bool(d.get("linear_decode")) and d["format"] == "tif-linear"), "format": d["format"],
                    "fusion": d["fusion"], "align": d["align"], "align_max_shift_px": f"{d['align_max_shift_px']:.1f}",
                    "align_min_cc": "" if d["align_min_cc"] is None else f"{d['align_min_cc']:.3f}", "seconds": d["seconds"],
                    "subject_box": " ".join(str(v) for v in d["subject_box"]) if d.get("subject_box") else "",
                    "best_focus_index": "" if d.get("best_focus_index") is None else d["best_focus_index"],
                    "best_focus_lens": "" if d.get("best_focus_lens") is None else f"{d['best_focus_lens']:.4f}",
                    "sharpness_ref": "" if not sharp else sharp.get(str(d["reference"]["focus_index"]), ""),
                    "sharpness_best": "" if d.get("best_focus_index") is None else sharp.get(str(d["best_focus_index"]), ""),
                    "raw_deleted_on_mac": int(bool(d.get("raw_deleted_on_mac")))})
        w = h = None
        try:
            import cv2 as _cv
            im = _cv.imread(os.path.join(session_dir, "images", image, d["merged_file"]), _cv.IMREAD_UNCHANGED)
            if im is not None:
                h, w = im.shape[:2]
        except Exception:
            pass
        row["w"], row["h"] = w or "", h or ""
        rows.append(row)
        for a in d["alignment"]:
            a = dict(a); a["stop"] = d["stop"]; a["image"] = image
            align.append(a)
    csv_path = os.path.join(session_dir, "stacks.csv"); align_path = os.path.join(session_dir, "alignment.csv")
    with _CsvLock(csv_path):
        _write_csv(csv_path, MERGED_COLUMNS, sorted(rows, key=lambda r: int(r["stop"])))
        _write_csv(align_path, ALIGN_COLUMNS, sorted(align, key=lambda a: (int(a["stop"]), int(a["focus_index"]))))
    log(f"rebuilt {csv_path}: {len(rows)} stops, {len(align)} alignment rows")
    return len(rows)


def merged_path(session_dir, out_name, row):
    if row.get("image"):
        return os.path.join(session_dir, "images", row["image"], row["file"])
    return os.path.join(session_dir, out_name, row["file"])


def merge_session(session_dir, ref_index, out_name="merged", fmt="tif", align="ecc", stops=None,
                  levels=6, half=False, index_maps=False, jobs=1, log=print, delete_raw=False, roi=1.0,
                  fusion="pyramid", preview=None, consistency=1, mask="threshold", sam3_python=None, sam3_prompt="insect",
                  align_on="beetle", min_rel_sharpness=0.0, depth_guard=0.0, fusion_space="linear"):
    if cv2 is None:
        raise RuntimeError("opencv (cv2) is required: pip install -r coordinator/requirements-merge.txt")
    all_stops = load_frames(session_dir)
    layout = layout_of([r for v in all_stops.values() for r in v])
    todo = sorted(all_stops) if not stops else [s for s in sorted(all_stops) if s in set(stops)]
    if layout == "stacks":
        out_dir = session_dir
        csv_path = os.path.join(session_dir, "stacks.csv")
        align_path = os.path.join(session_dir, "alignment.csv")
        if preview is None:
            preview = True
    else:
        out_dir = os.path.join(session_dir, out_name)
        os.makedirs(out_dir, exist_ok=True)
        csv_path = os.path.join(out_dir, "merged_frames.csv")
        align_path = os.path.join(out_dir, "alignment.csv")
        preview = bool(preview)
    done, align_rows = {}, []
    if os.path.exists(csv_path):
        with open(csv_path) as f:
            done = {int(r["stop"]): r for r in csv.DictReader(f)}
    if os.path.exists(align_path):
        with open(align_path) as f:
            align_rows = [r for r in csv.DictReader(f)]
    todo = [s for s in todo if not (s in done and os.path.exists(merged_path(session_dir, out_name, done[s])))]
    jobs_args = [{"session_dir": session_dir, "stop": s, "rows": all_stops[s], "ref_index": ref_index, "out_dir": out_dir,
                  "fmt": fmt, "align": align, "levels": levels, "half": half, "index_maps": index_maps,
                  "delete_raw": delete_raw, "roi": roi, "fusion": fusion, "preview": preview, "consistency": consistency,
                  "mask": mask, "sam3_python": sam3_python, "sam3_prompt": sam3_prompt,
                  "align_on": align_on, "min_rel_sharpness": min_rel_sharpness, "depth_guard": depth_guard,
                  "fusion_space": fusion_space}
                 for s in todo]
    log(f"{len(todo)} stop(s) to merge ({layout} layout, {fusion} fusion, {fmt}; {len(done)} already done), {jobs} job(s)")

    def handle(res):
        stop, row, extra = res
        if row is None:
            log(f"[stop {stop}] skipped: {extra}")
            return
        nonlocal done
        done = _update_csvs(csv_path, align_path, stop, row, extra)     # locked read-modify-write
        log(f"[stop {stop}] {row['n_frames']} frames -> {row['image'] or row['file']} in {row['seconds']}s "
            f"({fusion}; ref k={row['ref_focus_index']} lens {float(row['ref_lens_position']):.4f}, fx {float(row['fx']):.1f}, "
            f"align {row['align']}: max dev from intrinsics {row['align_max_shift_px']} px, min cc {row['align_min_cc']}"
            + (f"; sharpest on the beetle k={row['best_focus_index']} lens {row['best_focus_lens']}" if row['best_focus_index'] != "" else "; beetle not found")
            + f"{'; raw deleted' if str(row['raw_deleted_on_mac']) == '1' else ''})")

    if jobs > 1 and len(jobs_args) > 1:
        with mp.get_context("spawn").Pool(jobs) as pool:
            for res in pool.imap_unordered(_merge_one, jobs_args):
                handle(res)
    else:
        for a in jobs_args:
            handle(_merge_one(a))
    return csv_path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_dir", help="session/dataset folder written by the coordinator")
    ap.add_argument("--ref-index", type=int, default=None,
                    help="fixed reference focus index for every stop (default: per stop the frame where the beetle "
                         "is sharpest; if no beetle is found, the init-focus frame, else the middle of the stack)")
    ap.add_argument("--mask", choices=["threshold", "sam3"], default="threshold",
                    help="beetle mask for the focus measure that picks the reference: threshold = dark silhouette "
                         "(fast); sam3 = SAM 3 text prompt on the provisional sharpest frame (optitrack `masking` env, ~15 s)")
    ap.add_argument("--sam3-python", default=None, help="interpreter with SAM 3 (default: optitrack/.pixi/envs/masking)")
    ap.add_argument("--sam3-prompt", default="insect")
    ap.add_argument("--align-on", choices=["beetle", "frame"], default="beetle",
                    help="fit the alignment on the beetle mask (default) or on the whole frame (before 2026-10-06)")
    ap.add_argument("--depth-guard", type=float, default=0.0,
                    help="pyramid fusion: a pixel takes detail only from photos within this many focus steps of its "
                         "own sharpest photo (0 = off; removes the band along the outline and the star-shaped dust)")
    ap.add_argument("--fusion-space", choices=["linear", "srgb"], default="linear",
                    help="encoding the fusion compares and blends in; the written image is the same format either way")
    ap.add_argument("--min-rel-sharpness", type=float, default=0.0,
                    help="drop frames whose beetle score is below this fraction of the peak (0 = keep all 50)")
    ap.add_argument("--fusion", choices=FUSIONS, default="pyramid",
                    help="pyramid = Laplacian pyramid max energy (A); wavelet = complex-wavelet max modulus, Forster 2004 (B)")
    ap.add_argument("--consistency", type=int, default=1, choices=[0, 1, 2],
                    help="wavelet: majority filter on the selection maps (0 off, 1 = 3x3, 2 = 5x5)")
    ap.add_argument("--out", default="merged", help="output sub-folder name (flat layout only)")
    ap.add_argument("--format", choices=FORMATS, default="tif",
                    help="tif = 16-bit sRGB TIFF master (default); tif-linear = 16-bit linear; png/jpg = 8-bit sRGB")
    ap.add_argument("--preview", action="store_true", help="also write an sRGB JPEG preview (default on in the stacks layout)")
    ap.add_argument("--align", choices=["ecc", "intrinsics"], default="ecc",
                    help="ecc = neighbour-chained image alignment seeded by the intrinsics (default)")
    ap.add_argument("--roi", type=float, default=1.0,
                    help="central fraction of the frame the alignment is fitted on (1 = whole frame, default)")
    ap.add_argument("--stops", type=int, nargs="*", help="only these stop ids")
    ap.add_argument("--levels", type=int, default=6)
    ap.add_argument("--jobs", type=int, default=1, help="stops merged in parallel (each ~2-3 GB RAM at full res)")
    ap.add_argument("--half", action="store_true", help="half-resolution output (quick preview)")
    ap.add_argument("--index-maps", action="store_true", help="also write which frame won per pixel")
    ap.add_argument("--delete-raw", action="store_true",
                    help="after a verified merge delete that stop's raw frames from THIS folder (Mac space; the phone keeps them)")
    ap.add_argument("--rebuild-csv", action="store_true",
                    help="only regenerate stacks.csv + alignment.csv from the per-stop stack.json files, then exit")
    a = ap.parse_args(argv)
    if a.rebuild_csv:
        rebuild_csvs(a.session_dir)
        return
    stops = load_frames(a.session_dir)
    if not stops:
        sys.exit("frames.csv has no rows")
    n = max(len(v) for v in stops.values())
    ref = a.ref_index
    print(f"{len(stops)} stops, up to {n} frames each; reference = "
          + (f"focus index {ref}" if ref is not None and ref >= 0 else "the frame where the beetle is sharpest, per stop"))
    path = merge_session(a.session_dir, ref, a.out, a.format, a.align, a.stops, a.levels, half=a.half,
                         index_maps=a.index_maps, jobs=a.jobs, delete_raw=a.delete_raw, roi=a.roi,
                         fusion=a.fusion, preview=a.preview or None, consistency=a.consistency,
                         mask=a.mask, sam3_python=a.sam3_python, sam3_prompt=a.sam3_prompt,
                         align_on=a.align_on, min_rel_sharpness=a.min_rel_sharpness, depth_guard=a.depth_guard,
                         fusion_space=a.fusion_space)
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
