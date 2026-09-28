#!/usr/bin/env python3
"""COLMAP feature masks for the pose refinement: beetle + mount assembly.

The GS masks deliberately exclude the rod, clip and hub — but those parts
are RIGID with the insect (the whole mount assembly rotates together), and
they carry the sharp metallic texture SIFT needs to match views across the
60° insect rotations. This script writes masks_sfm/: the training mask
UNION a geometric rod capsule and hub disk, dilated, padded like the
dataset images. Used only for COLMAP feature extraction, never training.

    pixi run -e masking python scripts/make_sfm_masks.py \
        datasets/phone_<ts> /path/to/popillia_masked
"""

import argparse
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mask_common import find_stacks, load_mask
from mask_sam_geom import camera_basis, project
from plot_stacks import fit_geometry, load, rotm


def main():
    ap_ = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap_.add_argument('session')
    ap_.add_argument('dataset', help='popillia_masked dir (for padded size)')
    ap_.add_argument('--masks', default='masks/hybrid_text_sam')
    ap_.add_argument('--intrinsics', default='calibration/intrinsics.yaml')
    ap_.add_argument('--working-distance', type=float, default=25.0)
    ap_.add_argument('--roll', type=float, default=270.0)
    ap_.add_argument('--orient', default='cw')
    ap_.add_argument('--rod-radius-mm', type=float, default=5.0)
    ap_.add_argument('--rod-span-mm', default='0,130')
    ap_.add_argument('--hub-mm', default='50,100')
    ap_.add_argument('--dilate-px', type=int, default=25)
    ap_.add_argument('--stacks-per-block', type=int, default=8)
    args = ap_.parse_args()

    import cv2
    session = args.session.rstrip('/')
    spb = args.stacks_per_block
    stacks = load(session)
    ap, ad, spec, o_b, _ = fit_geometry(stacks, spb)
    wd = args.working_distance * 1e-3
    pose1 = [s for s in stacks if (s['n'] - 1) % spb == 0]
    t_b = np.mean([rotm(s['rel_q']).T @ (spec + wd * ad - s['rel'])
                   for s in pose1], axis=0)
    with open(args.intrinsics) as f:
        calib = yaml.safe_load(f)
    K = np.array(calib['camera_matrix'])
    size_land = calib['image_size']
    R_bc = camera_basis(o_b, args.roll)
    t0, t1 = (float(v) * 1e-3 for v in args.rod_span_mm.split(','))
    hub_r, hub_d = (float(v) * 1e-3 for v in args.hub_mm.split(','))

    # padded canvas, same formula as the exporters
    Wl, Hl = size_land
    Wp = int(round(2 * ((Hl - 1) - K[1, 2]) / 2) * 2)
    Hp = int(round(2 * K[0, 2] / 2) * 2)

    out_dir = os.path.join(args.dataset, 'masks_sfm')
    os.makedirs(out_dir, exist_ok=True)
    by_n = {s['n']: s for s in stacks}
    kernel = np.ones((args.dilate_px, args.dilate_px), np.uint8)
    for n, _ in find_stacks(session):
        mp = os.path.join(session, args.masks, f'stack_{n:02d}.png')
        m = (load_mask(mp) >= 0.5).astype(np.uint8) * 255
        H, W = m.shape
        s = by_n[n]
        for t in np.linspace(t0, t1, 60):
            p, zz = project(spec - t * ad, s, R_bc, t_b, K, size_land,
                            args.orient)
            if p is None or not (0 <= p[0] < W and 0 <= p[1] < H):
                continue
            r_px = max(6, int(K[0, 0] * args.rod_radius_mm * 1e-3 / zz))
            cv2.circle(m, (int(p[0]), int(p[1])), r_px, 255, -1)
        p, zz = project(spec - hub_d * ad, s, R_bc, t_b, K, size_land,
                        args.orient)
        if p is not None:
            cv2.circle(m, (int(p[0]), int(p[1])),
                       int(K[0, 0] * hub_r / zz), 255, -1)
        m = cv2.dilate(m, kernel)
        canvas = np.zeros((Hp, Wp), np.uint8)
        canvas[:H, :W] = m
        cv2.imwrite(os.path.join(out_dir, f'stack_{n:02d}.png'), canvas)
        print(f'stack {n:2d}: sfm-mask {100 * (canvas > 0).mean():.1f}% of frame')
    print(f'→ {out_dir}')


if __name__ == '__main__':
    main()
