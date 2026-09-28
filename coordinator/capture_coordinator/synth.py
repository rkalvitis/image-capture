"""Synthetic focus stacks for tests and the fake phone (needs cv2 + numpy).

A scene of textured tiles at different depths; frame k is sharp where the
tile depth matches lens position k and blurred elsewhere, and is magnified
by fx_k/fx_ref about the principal point (focus breathing), plus a small
lateral shift — the two effects the real stack showed. Output: 8-bit sRGB
PNGs whose names/sidecar fields mirror the app's.
"""
import numpy as np

try:
    import cv2
except ImportError:                                            # pragma: no cover
    cv2 = None


def scene(w=640, h=480, tiles=4, seed=0):
    """Textured base image (float32 RGB [0,1]) and a depth map in [0,1]."""
    if cv2 is None:
        raise ImportError("synthetic frames need opencv: pip install -r coordinator/requirements-merge.txt (or use the pixi env)")
    rng = np.random.default_rng(seed)
    img = np.zeros((h, w, 3), np.float32)
    depth = np.zeros((h, w), np.float32)
    tw = w // tiles
    for i in range(tiles):
        noise = rng.random((h // 8, tw // 8, 3)).astype(np.float32)
        tex = cv2.resize(noise, (tw, h), interpolation=cv2.INTER_NEAREST)
        tex = cv2.GaussianBlur(tex, (0, 0), 1.0)
        img[:, i * tw:(i + 1) * tw] = 0.15 + 0.7 * tex
        depth[:, i * tw:(i + 1) * tw] = i / max(1, tiles - 1)
    return img, depth


def render_frame(base, depth, lens, fx_ratio, shift_px=(0.0, 0.0), max_blur=6.0):
    """Frame at lens position `lens` (0..1): blur grows with |depth - lens|,
    then scale about the centre by fx_ratio and shift."""
    h, w = depth.shape
    out = np.zeros_like(base)
    levels = np.unique(depth)
    for d in levels:
        sigma = max_blur * abs(float(d) - lens)
        blurred = cv2.GaussianBlur(base, (0, 0), sigma) if sigma > 0.05 else base
        m = depth == d
        out[m] = blurred[m]
    cx, cy = w / 2.0, h / 2.0
    A = np.array([[fx_ratio, 0, cx - fx_ratio * cx + shift_px[0]],
                  [0, fx_ratio, cy - fx_ratio * cy + shift_px[1]]], np.float32)
    return cv2.warpAffine(out, A, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)


def srgb8(lin):
    x = np.clip(lin, 0, 1)
    s = np.where(x <= 0.0031308, 12.92 * x, 1.055 * np.power(x, 1 / 2.4) - 0.055)
    return (s * 255 + 0.5).astype(np.uint8)


class SyntheticStack:
    """Deterministic frame source: fx(lens) = fx0 - slope*lens like the fake phone."""

    def __init__(self, w=640, h=480, fx0=1976.0, slope=300.0, shift_per_lens_px=8.0, seed=0):
        self.w, self.h, self.fx0, self.slope, self.shift = w, h, fx0, slope, shift_per_lens_px
        self.base, self.depth = scene(w, h, seed=seed)

    def intrinsics(self, lens):
        return {"fx": self.fx0 - self.slope * lens, "fy": self.fx0 - 5.0 - self.slope * lens,
                "cx": self.w / 2.0, "cy": self.h / 2.0, "w": self.w, "h": self.h}

    def frame_png(self, lens, ref_lens=0.25):
        ratio = self.intrinsics(lens)["fx"] / self.intrinsics(ref_lens)["fx"]
        shift = ((lens - ref_lens) * self.shift, (lens - ref_lens) * self.shift * 0.5)
        img = render_frame(self.base, self.depth, lens, ratio, shift)
        ok, buf = cv2.imencode(".png", cv2.cvtColor(srgb8(img), cv2.COLOR_RGB2BGR))
        return buf.tobytes()

    def sharpness(self, img8):
        g = cv2.cvtColor(img8, cv2.COLOR_BGR2GRAY)
        return float(cv2.Laplacian(g, cv2.CV_32F).var())
