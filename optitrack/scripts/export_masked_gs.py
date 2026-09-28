#!/usr/bin/env python3
"""Export a stack session for Masked-3DGS (mask-supervised training).

Companion to export_gs.py. Where that script bakes the masks into white
images (for plain 3DGS with -w), this one emits what Jensen-JZ/Masked-3DGS
consumes: the ORIGINAL composites + the masks as separate files. The mask
loss then supervises alpha directly (0 outside / 1 inside) and the
photometric loss runs only inside the mask — no white baking, so the
white-floater failure mode of the -w pipeline cannot occur.

    images/stack_<n>.jpg   original (unmasked) composites, padded so the
                           principal point is centred (reader ignores cx/cy);
                           padding is black — those pixels are outside every
                           mask and never enter the photometric loss
    masks/stack_<n>.png    the chosen mask set, padded with black (=outside)
    sparse/0/              COLMAP model copied from an export_gs.py output —
                           the RIG-ANCHORED model (validated: mean camera-to-
                           origin must be < 0.05 m, else the copy aborts)

    pixi run -e masking python scripts/export_masked_gs.py \
        datasets/phone_<ts> /path/to/3DGS/popillia_masked \
        [--sparse-src /path/to/3DGS/popillia/sparse_anchored/0]
"""

import argparse
import os
import shutil
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mask_common import find_stacks, load_image


def validate_sparse(src, max_mean_dist=0.5):
    """Assert the model is the rig-anchored one (cameras ~0.24-0.33 units
    from the origin at scale 10), not the dead 07-13 marker model (~1.2)."""
    dists = []
    with open(os.path.join(src, 'images.txt')) as f:
        for line in f:
            t = line.strip().split()
            if len(t) >= 10 and t[0].isdigit():
                q = np.array([float(v) for v in t[1:5]])
                w, x, y, z = q
                R = np.array([
                    [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                    [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                    [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])
                tv = np.array([float(v) for v in t[5:8]])
                dists.append(np.linalg.norm(-R.T @ tv))
    mean = float(np.mean(dists))
    if mean > max_mean_dist:
        raise SystemExit(
            f'{src}: mean camera-to-origin {mean:.2f} units — this looks '
            f'like the 07-13 marker model (cameras ~1.2), not the '
            f'rig-anchored one (~0.28). Point --sparse-src at the anchored '
            f'model.')
    print(f'sparse model OK: mean camera-to-origin {mean * 100:.1f} mm')


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session')
    ap.add_argument('out')
    ap.add_argument('--masks', default='masks/hybrid_text_sam')
    ap.add_argument('--sparse-src',
                    default='/Users/robertskalvitis/Documents/repos/3DGS/'
                            'popillia/sparse/0')
    ap.add_argument('--intrinsics', default='calibration/intrinsics.yaml')
    args = ap.parse_args()

    from PIL import Image
    session = args.session.rstrip('/')

    with open(args.intrinsics) as f:
        calib = yaml.safe_load(f)
    K = np.array(calib['camera_matrix'])
    Wl, Hl = calib['image_size']                      # 4032, 3024 landscape
    cx_p, cy_p = (Hl - 1) - K[1, 2], K[0, 2]
    Wp = int(round(2 * cx_p / 2) * 2)
    Hp = int(round(2 * cy_p / 2) * 2)
    print(f'padding portrait {Hl}×{Wl} → {Wp}×{Hp} (pp centred)')

    validate_sparse(args.sparse_src)
    dst_sparse = os.path.join(args.out, 'sparse', '0')
    os.makedirs(dst_sparse, exist_ok=True)
    for fn in ('cameras.txt', 'images.txt', 'points3D.txt'):
        shutil.copy2(os.path.join(args.sparse_src, fn),
                     os.path.join(dst_sparse, fn))

    os.makedirs(os.path.join(args.out, 'images'), exist_ok=True)
    os.makedirs(os.path.join(args.out, 'masks'), exist_ok=True)
    n_done = 0
    for n, path in find_stacks(session):
        mp = os.path.join(session, args.masks, f'stack_{n:02d}.png')
        if not os.path.exists(mp):
            print(f'stack {n:2d}: no mask, skipped')
            continue
        img = load_image(path)                        # EXIF-upright original
        canvas = Image.new('RGB', (Wp, Hp), (0, 0, 0))
        canvas.paste(img, (0, 0))
        canvas.save(os.path.join(args.out, 'images', f'stack_{n:02d}.jpg'),
                    quality=95)
        mask = Image.open(mp).convert('L')
        mcanvas = Image.new('L', (Wp, Hp), 0)         # black = outside
        mcanvas.paste(mask, (0, 0))
        mcanvas.save(os.path.join(args.out, 'masks', f'stack_{n:02d}.png'))
        n_done += 1

    with open(os.path.join(args.out, 'README.md'), 'w') as f:
        f.write(f"""# popillia_masked — Masked-3DGS dataset

Source: optitrack `{session}` — ORIGINAL composites + masks
(`{args.masks}`), padded to {Wp}×{Hp} (principal point centred); COLMAP
model copied from the rig-anchored export (specimen at origin,
1 unit = 100 mm). Exported by `scripts/export_masked_gs.py`.

Train (Jensen-JZ/Masked-3DGS; masks path is relative to -s):

    python train.py -s <this dir> -m <out> \\
        --masks masks --lambda_mask 0.1 --eval --disable_viewer

Do NOT pass -w: the images keep their real background; the masked
photometric loss never looks at it, and the mask loss forces alpha to
zero there.
""")
    print(f'{n_done} image+mask pairs → {args.out}')


if __name__ == '__main__':
    main()
