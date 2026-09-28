#!/usr/bin/env python3
"""ROBOT hand-eye calibration: panda_link8 (flange) -> camera, via ChArUco.

    pixi run calibrate-handeye-robot -- <photo_dir> --csv <manual_poses.csv> \
        [--intrinsics calibration/intrinsics.yaml]
    python scripts/calibrate_handeye_robot.py --self-test

The robot replaces the mocap: no clocks, no bags, no marker bodies. The
correspondence is BY ORDER - photo k (sorted by filename) was taken at
waypoint k of the CSV (waypoint 0 = the init hold, no photo, exactly like a
dataset block). Session recipe:

  - print/reuse a ChArUco board (calibration/*.png); the SMALL boards
    (1.5-2 mm squares) are for the ultra-wide macro working band;
  - board STATIC in the scene, well lit, filling a good part of the frame;
  - freedrive-record 15-25 poses that view the board from varied
    orientations (rotation about >= 2 different axes across the set - pure
    translation is degenerate for AX=XB) with panda_manual_poses.py
    _mode:=record, then replay with _execute:=true and take ONE photo per
    stop (same lens + focus contract as the dataset);
  - AirDrop the photos into one folder, sorted order = waypoint order.

Because the flange poses come from joint replay (sub-mm) and pairing is by
order, this is immune to every weakness of the mocap variant: no phone
markers needed, and the phone CASE is automatically included - the solve
measures the camera wherever it actually sits.

Output: calibration/handeye_robot.yaml
  camera_in_flange: T_flange<-camera (translation + xyzw quaternion)
  -> translation   = camera/lens position in link8 (replaces LENS_XYZ_LINK8)
  -> R @ (0,0,1)   = optical axis in link8      (replaces LENS_AXIS_LINK8)
"""

import argparse
import os
import sys
from datetime import datetime
from glob import glob

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibrate_handeye import (METHODS, R_to_quat, quat_to_R,
                               solve_all_methods)
from calibrate_intrinsics import detect_board, load_image, make_board
from link_robot_mocap import load_run_csv


def pnp_target_to_cam(img, board, K, dist, min_corners):
    det = detect_board(img, board, min_corners)
    if det is None:
        return None
    obj, imgp = det
    ok, rvec, tvec = cv2.solvePnP(obj, imgp, K, dist,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.reshape(3), obj, imgp


def self_test():
    rng = np.random.default_rng(7)
    X = np.eye(4)
    X[:3, :3] = cv2.Rodrigues(np.array([0.2, -0.5, 0.1]))[0]
    X[:3, 3] = [-0.02, -0.05, 0.12]
    T_base_target = np.eye(4)
    T_base_target[:3, :3] = cv2.Rodrigues(np.array([0.1, 0.1, 1.2]))[0]
    T_base_target[:3, 3] = [0.4, 0.1, 0.2]
    R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
    for _ in range(15):
        T_fl = np.eye(4)
        T_fl[:3, :3] = cv2.Rodrigues(rng.uniform(-1.2, 1.2, 3))[0]
        T_fl[:3, 3] = rng.uniform(-0.3, 0.6, 3)
        T_cam = T_fl @ X
        T_t2c = np.linalg.inv(T_cam) @ T_base_target
        R_g2b.append(T_fl[:3, :3])
        t_g2b.append(T_fl[:3, 3])
        R_t2c.append(T_t2c[:3, :3])
        t_t2c.append(T_t2c[:3, 3])
    results = solve_all_methods(R_g2b, t_g2b, R_t2c, t_t2c)
    ok = True
    for name, (R, t) in results.items():
        dr = np.degrees(np.arccos(np.clip((np.trace(R.T @ X[:3, :3]) - 1) / 2,
                                          -1, 1)))
        dt = np.linalg.norm(t - X[:3, 3]) * 1000
        print(f'  {name:8s} rot err {dr:8.4f} deg, trans err {dt:8.4f} mm')
        ok &= dr < 1e-3 and dt < 1e-3
    print('SELF-TEST', 'PASSED' if ok else 'FAILED')
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('photos', nargs='?', help='folder of board photos, '
                    'sorted order = waypoint 1..N of the CSV')
    ap.add_argument('--csv', help='manual_poses_*.csv of the hand-eye run '
                    '(waypoint 0 = init, photos start at waypoint 1)')
    ap.add_argument('--intrinsics', default='calibration/intrinsics.yaml')
    ap.add_argument('--min-corners', type=int, default=12)
    ap.add_argument('--method', default='PARK', choices=sorted(METHODS))
    ap.add_argument('--out', default='calibration/handeye_robot.yaml')
    ap.add_argument('--self-test', action='store_true')
    args = ap.parse_args()
    if args.self_test:
        sys.exit(self_test())
    if not args.photos or not args.csv:
        ap.error('photos folder and --csv are required (or --self-test)')

    intr = yaml.safe_load(open(args.intrinsics))
    K = np.array(intr['camera_matrix'])
    dist = np.array(intr['distortion'])
    board = make_board(intr['board'])

    T_wp, _ = load_run_csv(args.csv)
    files = sorted(f for f in glob(os.path.join(args.photos, '*'))
                   if f.lower().endswith(('.dng', '.jpg', '.jpeg', '.png',
                                          '.heic')))
    if len(files) != len(T_wp) - 1:
        print(f'WARNING: {len(files)} photos vs {len(T_wp) - 1} photo '
              f'waypoints - pairing by order, verify the session!')

    R_g2b, t_g2b, R_t2c, t_t2c, kept = [], [], [], [], []
    for k, f in enumerate(files):
        w = k + 1
        if w >= len(T_wp):
            break
        img = load_image(f)
        res = pnp_target_to_cam(img, board, K, dist, args.min_corners)
        if res is None:
            print(f'  SKIP {os.path.basename(f)}: board not detected')
            continue
        R, t, obj, imgp = res
        proj, _ = cv2.projectPoints(obj, cv2.Rodrigues(R)[0], t, K, dist)
        rms = float(np.sqrt(np.mean(np.sum(
            (proj.reshape(-1, 2) - imgp.reshape(-1, 2)) ** 2, axis=1))))
        print(f'  {os.path.basename(f)} -> wp {w}: {len(imgp)} corners, '
              f'PnP rms {rms:.2f} px')
        R_g2b.append(T_wp[w][:3, :3])
        t_g2b.append(T_wp[w][:3, 3])
        R_t2c.append(R)
        t_t2c.append(t)
        kept.append((os.path.basename(f), w))
    if len(kept) < 5:
        sys.exit(f'only {len(kept)} usable pairs - need >= 5 (10+ better)')

    results = solve_all_methods(R_g2b, t_g2b, R_t2c, t_t2c)
    R_pick, t_pick = results[args.method]
    rots = []
    trans = []
    for name, (R, t) in results.items():
        dr = np.degrees(np.arccos(np.clip(
            (np.trace(R.T @ R_pick) - 1) / 2, -1, 1)))
        dt = np.linalg.norm(t - t_pick) * 1000
        rots.append(dr)
        trans.append(dt)
        print(f'{name:8s} vs {args.method}: rot {dr:6.3f} deg, '
              f'trans {dt:6.2f} mm')
    axis = R_pick @ np.array([0, 0, 1.0])
    print(f'\ncamera position in link8: {np.round(t_pick, 5)}')
    print(f'optical axis in link8:    {np.round(axis, 4)}')

    out = {
        'date': datetime.now().isoformat(timespec='seconds'),
        'method': args.method,
        'n_pairs': len(kept),
        'pairs': [{'photo': p, 'waypoint': w} for p, w in kept],
        'intrinsics_file': args.intrinsics,
        'csv': args.csv,
        'camera_in_flange': {
            'translation_m': t_pick.tolist(),
            'quaternion_xyzw': R_to_quat(R_pick).tolist(),
        },
        'optical_axis_in_flange': axis.tolist(),
        'method_agreement': {'rot_deg_max': float(max(rots)),
                             'trans_mm_max': float(max(trans))},
        'note': 'T_flange<-camera. translation replaces LENS_XYZ_LINK8 '
                '(case-inclusive - measured with the phone as mounted); '
                'quaternion gives R_cam<-flange as its inverse.',
    }
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, 'w') as f:
        yaml.safe_dump(out, f, sort_keys=False)
    print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
