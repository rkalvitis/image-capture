"""Session-start exposure check + the beetle finder (2026-09-15).

`find_subject` is shared with stack_merge (per-frame sharpness inside the
beetle box -> the measured best-focus frame of every stack).

`locked` exposure meters the whole frame once at hello: with the white
foam filling most of the ultra-wide frame the beetle came out at ~2 % linear
(sRGB 0.16) on 2026-09-09, foam at 0.14 linear — nothing near clipping. The
coordinator therefore shoots one test frame at the first stop, prints what
+0 / +0.5 / ... EV would do to the foam, the beetle and the clipped fraction
(all from the linear ProRAW), and applies the EV the operator picks as a
CUSTOM exposure (longer shutter first — the rig is static — then ISO).
"""
import numpy as np


def _srgb(x):
    x = np.clip(np.asarray(x, np.float64), 0.0, 1.0)
    return np.where(x <= 0.0031308, 12.92 * x, 1.055 * np.power(x, 1 / 2.4) - 0.055)


def find_subject(gray, p50=None, central=0.9, min_area_frac=1.5e-4, ring_px=3, ring_bright=0.6, dark_frac=0.6,
                 max_aspect=3.0, min_fill=0.3, open_frac=0.015, work_w=1008):
    """Bounding box (y0, y1, x0, x1) of the pinned beetle in `gray` (float, linear), or None.

    The beetle always sits on the white foam and is SURROUNDED by it — the
    frame edges may show the dark lab, that does not matter. Rule: pixels
    darker than `dark_frac` x the foam brightness (75th percentile of the
    central region), opening with `open_frac` of the width (cuts the pin),
    connected components inside the central `central` fraction, not touching
    the border, compact (not the pin), with a mostly bright ring outside
    (ring_px..3*ring_px); the largest wins. Runs at `work_w` px wide (~30 ms)
    and scales the box back, so the same function serves the 1572 px preview
    and the 4032 px raw."""
    import cv2
    H, W = gray.shape
    scale = min(1.0, work_w / W)
    g = cv2.resize(np.asarray(gray, np.float32), (int(W * scale), int(H * scale)), interpolation=cv2.INTER_AREA) \
        if scale < 1.0 else np.asarray(gray, np.float32)
    h, w = g.shape
    blur = cv2.GaussianBlur(g, (0, 0), 1.0)
    cy0, cy1, cx0, cx1 = int(h * 0.2), int(h * 0.8), int(w * 0.2), int(w * 0.8)
    white = float(np.percentile(blur[cy0:cy1, cx0:cx1], 75))
    if white <= 0:
        return None
    m = (blur < dark_frac * white).astype(np.uint8)
    ko = max(5, int(w * open_frac)) | 1
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ko, ko)))
    bright = blur > ring_bright * white
    n, lab, stats, cent = cv2.connectedComponentsWithStats(m)
    best, best_area = None, 0
    kernel = np.ones((3, 3), np.uint8)
    min_area = min_area_frac * w * h
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        if area < min_area or area <= best_area:
            continue
        cx, cy = cent[i]
        if abs(cx - w / 2) > central * w / 2 or abs(cy - h / 2) > central * h / 2:
            continue
        if x == 0 or y == 0 or x + bw >= w or y + bh >= h:
            continue                                   # touches the border: not enclosed
        if max(bw, bh) > max_aspect * max(1, min(bw, bh)) or area < min_fill * bw * bh:
            continue                                   # elongated / sparse: the pin, a shadow line
        r3 = 3 * ring_px
        y0, y1 = max(0, y - r3 - 2), min(h, y + bh + r3 + 2)
        x0, x1 = max(0, x - r3 - 2), min(w, x + bw + r3 + 2)
        comp = (lab[y0:y1, x0:x1] == i).astype(np.uint8)
        inner = cv2.dilate(comp, kernel, iterations=ring_px) > 0
        ring = (cv2.dilate(comp, kernel, iterations=r3) > 0) & ~inner
        if ring.sum() == 0 or bright[y0:y1, x0:x1][ring].mean() < 0.3:
            continue                                   # not surrounded by white
        best, best_area = (y, y + bh, x, x + bw), area
    if best is None:
        return None
    return tuple(int(round(v / scale)) for v in best)


def exposure_stats(lin_rgb, evs=(0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0)):
    """lin_rgb: linear float32 RGB in [0, 1]. Returns a dict with the frame
    percentiles, the subject box and one row per EV correction."""
    import cv2
    g = cv2.cvtColor(np.asarray(lin_rgb, np.float32), cv2.COLOR_RGB2GRAY)
    p50, p99, p999 = (float(v) for v in np.percentile(g, [50, 99, 99.9]))
    box = find_subject(g, p50)
    if box is not None:
        sub = g[box[0]:box[1], box[2]:box[3]]
        sub = sub[sub < 0.5 * p50] if (sub < 0.5 * p50).sum() > 100 else sub    # the beetle pixels only, not the foam in the box
    else:
        h, w = g.shape
        sub = g[int(h * 0.375):int(h * 0.625), int(w * 0.375):int(w * 0.625)]
    s50 = float(np.median(sub))
    mx = np.asarray(lin_rgb, np.float32).max(axis=2)
    rows = []
    for ev in evs:
        gain = 2.0 ** ev
        rows.append({"ev": ev, "frame_p50_srgb": float(_srgb(p50 * gain)), "frame_p99_srgb": float(_srgb(p99 * gain)),
                     "subject_p50_srgb": float(_srgb(s50 * gain)), "subject_p50_linear": s50 * gain,
                     "clipped_pct": float(100.0 * (mx * gain >= 1.0).mean())})
    return {"frame_p50": p50, "frame_p99": p99, "frame_p999": p999, "subject_box": box, "subject_found": box is not None,
            "subject_p50": s50, "rows": rows}


def format_stats(st):
    lines = [f"  frame: p50 {st['frame_p50']:.3f}  p99 {st['frame_p99']:.3f}  p99.9 {st['frame_p999']:.3f} (linear)   "
             f"subject {'box ' + str(st['subject_box']) if st['subject_found'] else 'NOT found -> centre box'}: p50 {st['subject_p50']:.4f}",
             "   EV   foam p50   subject p50   clipped     (sRGB 0..1)"]
    for r in st["rows"]:
        lines.append(f"  {r['ev']:+.1f}   {r['frame_p50_srgb']:.3f}      {r['subject_p50_srgb']:.3f}       {r['clipped_pct']:6.2f} %")
    return "\n".join(lines)


def auto_ev(st, target_p99_srgb=0.85, lo=-3.0, hi=4.0):
    """EV that puts the frame's 99th percentile (the white foam) at `target_p99_srgb`, from the
    linear statistics of the test frame: the brightest common tone lands near white without
    clipping, the (dark) beetle follows. Rounded to 0.1 EV, clamped to [lo, hi]."""
    import math
    p99 = float(st.get("frame_p99") or 0.0)
    if p99 <= 0:
        return 0.0
    target_lin = _srgb_inverse(target_p99_srgb)
    ev = math.log2(target_lin / p99)
    return round(min(hi, max(lo, ev)), 1)


def _srgb_inverse(v):
    return v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4


def apply_ev(duration_s, iso, ev, max_duration_s=0.1, min_iso=32.0, max_iso=3200.0):
    """New (duration, iso) for an EV correction: shutter first (up to max_duration_s), then ISO."""
    gain = 2.0 ** float(ev)
    d = float(duration_s) * gain
    i = float(iso)
    if d > max_duration_s:
        i = i * d / max_duration_s
        d = max_duration_s
    i = min(max(i, min_iso), max_iso)
    return d, i
