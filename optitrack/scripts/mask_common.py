"""Shared helpers for the background-masking experiments.

Used by scripts/mask_birefnet.py (experiment 1), scripts/mask_sam_geom.py
(experiment 2) and scripts/compare_masks.py. Runs in the `masking` pixi
environment: `pixi run -e masking ...` (no ROS).
"""

import csv
import os

import numpy as np
from PIL import Image, ImageOps


def find_stacks(session):
    """[(stack_n, composite_path)] for a stack session, ordered by stack.

    Prefers stacks.csv (authoritative order + paths); falls back to globbing
    images/<n>/*.JPG.
    """
    session = session.rstrip('/')
    csv_path = os.path.join(session, 'stacks.csv')
    if os.path.exists(csv_path):
        with open(csv_path) as f:
            rows = list(csv.DictReader(f))
        return [(int(r['stack']), os.path.join(session, r['stacked_image']))
                for r in sorted(rows, key=lambda r: int(r['stack']))]
    out = []
    images = os.path.join(session, 'images')
    for d in sorted(os.listdir(images), key=lambda s: int(s)):
        jpgs = [f for f in os.listdir(os.path.join(images, d))
                if f.lower().endswith(('.jpg', '.jpeg'))]
        if jpgs:
            out.append((int(d), os.path.join(images, d, jpgs[0])))
    return out


def load_image(path):
    """Composite JPG with EXIF orientation applied (upright pixels)."""
    return ImageOps.exif_transpose(Image.open(path).convert('RGB'))


def save_mask(mask01, path):
    """Save a float mask (H, W) in [0, 1] as an 8-bit grayscale PNG."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    Image.fromarray((np.clip(mask01, 0, 1) * 255).astype(np.uint8)).save(path)


def load_mask(path):
    """8-bit mask PNG → float (H, W) in [0, 1]."""
    return np.asarray(Image.open(path).convert('L'), dtype=np.float32) / 255.0


def overlay_preview(img, mask01, out_path, max_side=800, threshold=0.5):
    """Preview JPEG: background dimmed, mask boundary drawn.

    `img` is a PIL image, `mask01` a float mask at the same size.
    """
    import cv2
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    scale = max_side / max(img.size)
    small = img.resize((round(img.size[0] * scale), round(img.size[1] * scale)),
                       Image.LANCZOS)
    rgb = np.asarray(small).astype(np.float32)
    m = cv2.resize(mask01, small.size, interpolation=cv2.INTER_LINEAR)
    binary = (m >= threshold).astype(np.uint8)
    dim = 0.25 + 0.75 * m[..., None]
    rgb = rgb * dim
    contours, _ = cv2.findContours(binary, cv2.RETR_LIST,
                                   cv2.CHAIN_APPROX_SIMPLE)
    rgb = rgb.astype(np.uint8).copy()
    cv2.drawContours(rgb, contours, -1, (60, 255, 60), 2)
    Image.fromarray(rgb).save(out_path, quality=85)


def pick_device():
    import torch
    if torch.backends.mps.is_available():
        return 'mps'
    if torch.cuda.is_available():
        return 'cuda'
    return 'cpu'
