#!/usr/bin/env python3
"""Solve the true camera optical axis in the flange frame from the photos.

The mount model's nominal lens axis (robo_arm LENS_AXIS_LINK8) turned out to
be ~20 deg off for the 20260826 session: the beetle is nearly centered in the
photos while the nominal axis predicts it far off-frame. Since the beetle's
3D position is known (registration.yaml) and its pixel position is known in
every photo (the masking run's masks), the camera orientation in the flange
frame is solvable: Wahba's problem over the beetle directions, then a joint
Gauss-Newton refine of R_cam<-flange, focal length, and the beetle's visual
center (mask centroid != rotation-axis point for a real specimen).

Reads <dataset>/{phone_poses_robot/*.csv, masking/masks, masking/rgb,
registration.yaml}; appends the solution to registration.yaml.

Usage:  pixi run python scripts/solve_camera_axis.py datasets/phone_20260826_113325
"""

import argparse
import json
import os

import numpy as np
import yaml
from PIL import Image

from link_robot_mocap import load_run_csv

WS, HS = 4032, 3024   # sensor-native size
NOMINAL_AXIS = np.array([-0.70711, 0.0, 0.70711])
# July-calibrated intrinsics (calibration/intrinsics.yaml) - pinned by default:
# fixing f+pp removes the focal<->depth correlation from the solve, and the
# free solve independently lands within 1.5% of this f anyway.
JULY_F = 1975.9466
JULY_CX, JULY_CY = 2036.6826, 1420.9925


def rodrigues(v):
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def sensor_px(mask_path, rgb_path):
    """Mask centroid in sensor-native pixel coords (undo the EXIF rotation)."""
    m = np.array(Image.open(mask_path).convert('L')) > 127
    ys, xs = np.nonzero(m)
    xd, yd = xs.mean(), ys.mean()
    flag = Image.open(rgb_path).getexif().get(274)
    if flag == 6:        # displayed = sensor rotated 90 deg CW
        return yd, (HS - 1) - xd
    if flag == 3:        # displayed = sensor rotated 180 deg
        return (WS - 1) - xd, (HS - 1) - yd
    if flag in (1, None):
        return xd, yd
    raise SystemExit(f'unhandled EXIF orientation {flag} in {rgb_path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dataset')
    ap.add_argument('--free-intrinsics', action='store_true',
                    help='solve focal too (default: pin f and pp to the July '
                         'calibration - preferred)')
    args = ap.parse_args()
    ds = args.dataset.rstrip('/')
    reg = yaml.safe_load(open(os.path.join(ds, 'registration.yaml')))
    lens_fl = np.array(reg['lens_offset_in_flange_corrected_m'])
    bt0 = np.array(reg['object_center_robot_on_axis'])

    csvs = sorted(f for f in os.listdir(os.path.join(ds, 'phone_poses_robot'))
                  if f.startswith('manual_poses_2026'))
    images = sorted(i for i in os.listdir(os.path.join(ds, 'images'))
                    if i.startswith('IMG'))
    obs = []   # (dx, dy, R_flange, lens_pos)
    for b, fn in enumerate(csvs):
        T_wp, _ = load_run_csv(os.path.join(ds, 'phone_poses_robot', fn))
        for w in range(1, len(T_wp)):
            img = images[b * (len(T_wp) - 1) + (w - 1)]
            x_s, y_s = sensor_px(os.path.join(ds, 'masking/masks', img + '.png'),
                                 os.path.join(ds, 'masking/rgb', img + '.jpg'))
            R_fl = T_wp[w][:3, :3]
            lens = T_wp[w][:3, 3] + R_fl @ lens_fl
            obs.append((x_s - JULY_CX, y_s - JULY_CY, R_fl, lens))

    # Wahba init: align beetle directions (flange frame) with pixel directions
    B = np.zeros((3, 3))
    for dx, dy, R_fl, lens in obs:
        vc = np.array([dx / JULY_F, dy / JULY_F, 1.0])
        v = R_fl.T @ (bt0 - lens)
        B += np.outer(vc / np.linalg.norm(vc), v / np.linalg.norm(v))
    U, _, Vt = np.linalg.svd(B)
    Rcf0 = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt

    # joint Gauss-Newton: rotation (3), focal (1), beetle visual center (3)
    def resid(x):
        Rcf = rodrigues(x[:3]) @ Rcf0
        f, bt = x[3], x[4:7]
        out = []
        for dx, dy, R_fl, lens in obs:
            v = R_fl.T @ (bt - lens)
            vc = Rcf @ (v / np.linalg.norm(v))
            out += [dx - f * vc[0] / vc[2], dy - f * vc[1] / vc[2]]
        return np.array(out)

    x = np.array([0.0, 0.0, 0.0, JULY_F, *bt0])
    step = np.array([1e-3] * 3 + [1.0] + [1e-4] * 3)
    free_f = args.free_intrinsics
    for _ in range(50):
        r0 = resid(x)
        J = np.empty((len(r0), 7))
        cols = list(range(7)) if free_f else [0, 1, 2, 4, 5, 6]
        for j in cols:
            d = np.zeros(7)
            d[j] = step[j]
            J[:, j] = (resid(x + d) - r0) / step[j]
        dx_, *_ = np.linalg.lstsq(J[:, cols], -r0, rcond=None)
        full = np.zeros(7)
        full[cols] = dx_
        x = x + full
        if np.linalg.norm(full * np.array([1, 1, 1, 1e-3, 1, 1, 1])) < 1e-8:
            break

    Rcf = rodrigues(x[:3]) @ Rcf0
    f, bt = x[3], x[4:7]
    ang = np.degrees(np.arctan(np.linalg.norm(resid(x).reshape(-1, 2), axis=1) / f))
    axis_true = Rcf.T @ np.array([0, 0, 1.0])
    off_nom = np.degrees(np.arccos(np.clip(axis_true @ NOMINAL_AXIS, -1, 1)))
    print(f'n={len(obs)} photos: residual rms {np.sqrt((ang**2).mean()):.2f} deg, '
          f'max {ang.max():.2f} deg; f = {f:.0f} px')
    print(f'true optical axis in flange: {np.round(axis_true, 4)} '
          f'({off_nom:.1f} deg from the nominal mount-model axis)')
    print(f'beetle visual center: {np.round(bt, 4)} '
          f'({np.linalg.norm(bt - bt0) * 1000:.1f} mm from the axis point)')

    # Per-block beetle triangulation -> the beetle's orbit ring. The beetle
    # rotates WITH the stage, so the 6 per-block positions circle the TRUE
    # rotation axis at beetle height - a direct measurement that beats
    # extrapolating the marker-circle axis 530 mm up (which is ~2 cm off).
    # The fold axis = mocap axis DIRECTION through the ring CENTER.
    Rcf_ = Rcf
    ring = []
    for b0 in range(len(csvs)):
        A_ = np.zeros((3, 3))
        b_ = np.zeros(3)
        for dx, dy, R_fl, lens in obs[b0 * 8:(b0 + 1) * 8]:
            vc = np.array([dx / f, dy / f, 1.0])
            vc /= np.linalg.norm(vc)
            d = R_fl @ (Rcf_.T @ vc)
            P = np.eye(3) - np.outer(d, d)
            A_ += P
            b_ += P @ lens
        ring.append(np.linalg.solve(A_, b_))
    ring = np.array(ring)
    ring_c = ring.mean(0)
    orbit = np.linalg.norm(ring - ring_c, axis=1)
    axis_dir = np.array(reg['turntable_axis_robot']['dir'])
    axis_dir = axis_dir / np.linalg.norm(axis_dir)
    axis_pt = np.array(reg['turntable_axis_robot']['point'])
    off = ring_c - (axis_pt + ((ring_c - axis_pt) @ axis_dir) * axis_dir)
    print(f'beetle ring: center {np.round(ring_c, 4)}, per-block spread '
          f'{orbit.max() * 1000:.1f} mm (beetle ~on-axis); fold line moved '
          f'{np.linalg.norm(off) * 1000:.1f} mm vs the extrapolated mocap axis')
    reg['fold_axis_robot'] = {
        'point': ring_c.tolist(),
        'dir': axis_dir.tolist(),
        'note': 'USE THIS LINE for the +60 deg/block fold: direction from the '
                'mocap marker circle, position from the per-block beetle '
                'triangulation (the beetle orbits it at '
                f'{orbit.max() * 1000:.1f} mm). The turntable_axis_robot point '
                'is the marker-plane point and is ~2 cm off laterally at '
                'beetle height - do not fold about it.',
    }

    reg['camera_axis_solution'] = {
        'method': 'beetle-as-target: Wahba + joint GN over all photos '
                  '(mask centroids, EXIF-undone to sensor frame)',
        'optical_axis_in_flange': axis_true.tolist(),
        'R_cam_from_flange': Rcf.tolist(),
        'focal_px': float(f),
        'cx_px': JULY_CX,
        'cy_px': JULY_CY,
        'beetle_visual_center_robot_m': bt.tolist(),
        'residual_rms_deg': float(np.sqrt((ang ** 2).mean())),
        'nominal_axis_error_deg': float(off_nom),
    }
    with open(os.path.join(ds, 'registration.yaml'), 'w') as fh:
        yaml.safe_dump(reg, fh, sort_keys=False)
    print(f'appended camera_axis_solution to {ds}/registration.yaml')


if __name__ == '__main__':
    main()
