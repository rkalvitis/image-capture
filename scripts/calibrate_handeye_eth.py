#!/usr/bin/env python3
"""Hand-eye calibration via the ETH-ASL dual-quaternion solver (2nd opinion).

Standalone alternative to scripts/calibrate_handeye.py (OpenCV) — same
session input, same synchronized pose pairs, INDEPENDENT solver
implementation (ethz-asl/hand_eye_calibration: Daniilidis dual-quaternion
formulation with screw-axis prefiltering and RANSAC outlier rejection).

One-time setup (the repo is vendored untracked):

    git clone --depth 1 https://github.com/ethz-asl/hand_eye_calibration \
        vendor/hand_eye_calibration

Usage (same arguments as the OpenCV version):

    pixi run calibrate-handeye-eth -- calibration/phone_<ts> \
        [--clock-shows HH:MM:SS.mmm ...] [--intrinsics ...]

Output: calibration/<session>/hand_eye_eth.yaml + a side-by-side
comparison against all five OpenCV solvers on identical input data.
"""

import argparse
import os
import sys
import types

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibrate_handeye import (collect_pairs, resolve_ambiguity,
                               solve_all_methods, quat_to_R, R_to_quat,
                               rot_angle_deg)

ETH_PY = 'vendor/hand_eye_calibration/hand_eye_calibration/python'


def _patch_py3(path, old, new):
    """Idempotent source fix for the python-2-era vendored code."""
    with open(path) as f:
        src = f.read()
    if new in src:
        return
    if old not in src:
        raise SystemExit(f'{path}: expected snippet not found — upstream '
                         'changed, review the patch')
    with open(path, 'w') as f:
        f.write(src.replace(old, new))
    print(f'patched {os.path.basename(path)} for python 3')


def import_eth():
    if not os.path.isdir(ETH_PY):
        raise SystemExit(
            'ETH repo not found — clone it first:\n'
            '  git clone --depth 1 '
            'https://github.com/ethz-asl/hand_eye_calibration '
            'vendor/hand_eye_calibration')
    # py3: random.sample needs a sequence, not a set
    _patch_py3(os.path.join(ETH_PY, 'hand_eye_calibration',
                            'dual_quaternion_hand_eye_calibration.py'),
               'random.sample(indices_set,',
               'random.sample(sorted(indices_set),')
    # numpy >= 1.24 removed the np.str alias
    _patch_py3(os.path.join(ETH_PY, 'hand_eye_calibration',
                            'dual_quaternion.py'),
               'np.str(self.q_rot), np.str(self.q_dual)',
               'str(self.q_rot), str(self.q_dual)')
    _patch_py3(os.path.join(ETH_PY, 'hand_eye_calibration', 'quaternion.py'),
               'return np.str(self.q)',
               'return str(self.q)')
    # headless matplotlib + stub for the unused ROS1 `tf` import
    import matplotlib
    matplotlib.use('Agg')
    sys.modules.setdefault('tf', types.ModuleType('tf'))
    sys.path.insert(0, ETH_PY)
    from hand_eye_calibration.dual_quaternion import DualQuaternion
    from hand_eye_calibration.dual_quaternion_hand_eye_calibration import (
        compute_hand_eye_calibration_RANSAC, HandEyeConfig)
    return DualQuaternion, compute_hand_eye_calibration_RANSAC, HandEyeConfig


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session', help='session folder (calibration/phone_<ts>)')
    ap.add_argument('--intrinsics', default='calibration/intrinsics.yaml')
    ap.add_argument('--rigid-body', default='phone')
    ap.add_argument('--clock-shows', action='append', default=[],
                    metavar='HH:MM:SS.mmm')
    ap.add_argument('--clock-offset', type=float, default=None, metavar='SEC')
    ap.add_argument('--min-corners', type=int, default=12)
    ap.add_argument('--out', default=None,
                    help='output YAML (default: '
                         'calibration/<session>/hand_eye_eth.yaml)')
    args = ap.parse_args()

    DualQuaternion, ransac, HandEyeConfig = import_eth()

    session = args.session.rstrip('/')
    pairs = collect_pairs(session, args.intrinsics, args.rigid_body,
                          args.clock_shows, args.clock_offset,
                          args.min_corners)
    resolve_ambiguity(pairs)
    n = len(pairs['used'])
    if n < 5:
        raise SystemExit(f'only {n} usable photos')

    # ETH convention: dq_B_H = hand (phone body) pose in base (world);
    # dq_W_E = eye (camera) pose in its "world" (the board frame) — the
    # inverse of solvePnP's target-in-camera transform.
    dq_B_H, dq_W_E = [], []
    for i in range(n):
        t = pairs['t_g2b'][i]
        q = R_to_quat(pairs['R_g2b'][i])
        dq_B_H.append(DualQuaternion.from_pose(t[0], t[1], t[2], *q))
        R_ct = pairs['R_t2c'][i].T
        t_ct = -R_ct @ pairs['t_t2c'][i]
        q_ct = R_to_quat(R_ct)
        dq_W_E.append(DualQuaternion.from_pose(*t_ct, *q_ct))

    config = HandEyeConfig()
    config.visualize = False
    # our sessions are small (10-40 pairs): keep every pose (RANSAC still
    # rejects outliers) and scale the inlier requirement to the set
    config.prefilter_poses_enabled = False
    config.min_num_inliers = max(4, n // 2)
    config.ransac_max_number_iterations = 80
    (success, dq_H_E, rmse, n_inliers, n_filtered,
     runtime, *_ ) = ransac(dq_B_H, dq_W_E, config)
    if not success or dq_H_E is None:
        raise SystemExit('ETH solver did not converge')

    pose = dq_H_E.to_pose()                 # [x, y, z, qx, qy, qz, qw]
    t_eth = np.array(pose[:3])
    q_eth = np.array(pose[3:])
    R_eth = quat_to_R(q_eth)

    print(f'\nETH dual-quaternion RANSAC result (camera in phone-body frame):')
    print(f'  t = {np.round(t_eth * 1000, 2).tolist()} mm')
    print(f'  quat = {np.round(q_eth, 5).tolist()}')
    print(f'  inliers {n_inliers}/{n_filtered} (of {n} pairs), '
          f'rmse pos {rmse[0]:.4f} / orient {rmse[1]:.3f}, '
          f'{runtime:.2f} s')

    print('\ncomparison with the OpenCV solvers on the SAME pose pairs:')
    results = solve_all_methods(pairs['R_g2b'], pairs['t_g2b'],
                                pairs['R_t2c'], pairs['t_t2c'])
    for name, (R, t) in results.items():
        d = np.linalg.norm(t - t_eth) * 1000
        a = rot_angle_deg(R.T @ R_eth)
        print(f'  vs {name:11s} Δt = {d:6.1f} mm, Δrot = {a:5.2f} deg')

    out = args.out or (f'calibration/{os.path.basename(session)}/'
                       f'hand_eye_eth.yaml')
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    with open(out, 'w') as f:
        yaml.safe_dump({
            'solver': 'ethz-asl/hand_eye_calibration '
                      '(dual quaternion, RANSAC)',
            'session': session,
            'intrinsics_file': args.intrinsics,
            'camera_in_body': {
                'translation_m': [float(v) for v in t_eth],
                'quaternion_xyzw': [float(v) for v in q_eth],
            },
            'n_pairs': n,
            'n_inliers': int(n_inliers),
            'rmse': {'position': float(rmse[0]),
                     'orientation': float(rmse[1])},
        }, f, sort_keys=False)
    print(f'\nwrote {out}')


if __name__ == '__main__':
    main()
