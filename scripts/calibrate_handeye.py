#!/usr/bin/env python3
"""Hand-eye calibration: `phone` marker body → lens optical center (AX = XB).

    pixi run calibrate-handeye -- datasets/phone_<ts> \
        --clock-shows 14:03:21.132 --clock-shows 14:41:07.850
    python scripts/calibrate_handeye.py --self-test

Session requirements (this is the CLASSIC variant — cv2.calibrateHandEye):
  - the printed ChArUco board STATIC in the scene (no markers needed on it);
  - `pixi run record-poses` running, clock photos at start/end (as always);
  - ~20–40 ProRAW photos of the board HANDHELD from varied orientations —
    the phone must rotate about at least two different axes across the set
    (a fixed camera + rotating board is mathematically degenerate here);
  - hold still ~2 s per shot (pose accuracy then doesn't depend on clock
    precision), one lens, the session anchor focus, exposure locked.

Inputs per photo: the body pose in the world (interpolated from the bag at
the photo's corrected EXIF time) and the board pose in the camera (PnP with
the intrinsics from calibrate_intrinsics.py — run that FIRST; its YAML also
carries the board spec).

Output: calibration/hand_eye.yaml — the camera pose in the `phone` body
frame (translation + xyzw quaternion), with per-method agreement and a
reprojection validation report. Consumed later by match_photos --hand-eye.

Self-test: synthetic body/board poses with a known hand-eye; checks that
every OpenCV solver recovers it through our exact frame conventions.
"""

import argparse
import math
import os
import sys
from datetime import datetime
from glob import glob

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from match_photos import (read_dng_exif, clock_offset_from, load_pose_track,
                          interp_pose)
from calibrate_intrinsics import make_board, load_image, detect_board

METHODS = {
    'TSAI': cv2.CALIB_HAND_EYE_TSAI,
    'PARK': cv2.CALIB_HAND_EYE_PARK,
    'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
    'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF,
    'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]])


def R_to_quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-6:
        return np.array([(R[2, 1] - R[1, 2]) / (4 * w),
                         (R[0, 2] - R[2, 0]) / (4 * w),
                         (R[1, 0] - R[0, 1]) / (4 * w), w])
    # w ~ 0: fall back via largest diagonal element
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1 + R[i, i] - R[j, j] - R[k, k])) * 2
    q = np.zeros(4)
    q[i] = s / 4
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


def rot_angle_deg(R):
    return math.degrees(math.acos(min(1.0, max(-1.0, (np.trace(R) - 1) / 2))))


def solve_all_methods(R_g2b, t_g2b, R_t2c, t_t2c):
    """Run every OpenCV hand-eye solver; return {name: (R, t)}."""
    out = {}
    for name, flag in METHODS.items():
        try:
            R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=flag)
            out[name] = (R, t.ravel())
        except cv2.error as e:
            print(f'  {name}: solver failed ({str(e).splitlines()[-1]})',
                  file=sys.stderr)
    return out


def self_test():
    rng = np.random.default_rng(4)
    # ground truth: camera pose in the body frame (the hand-eye X)
    R_x, _ = cv2.Rodrigues(np.array([0.2, -0.3, 0.15]))
    t_x = np.array([0.012, -0.058, 0.021])
    # board pose in the world
    R_wt, _ = cv2.Rodrigues(np.array([0.1, 0.05, -0.3]))
    t_wt = np.array([0.4, -0.2, 0.9])

    R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
    for _ in range(15):
        R_wb, _ = cv2.Rodrigues(rng.uniform(-0.9, 0.9, 3))   # rotation-diverse
        t_wb = np.array([0.3, -0.1, 1.0]) + rng.uniform(-0.25, 0.25, 3)
        R_wc = R_wb @ R_x                       # camera in world
        t_wc = R_wb @ t_x + t_wb
        R_tc = R_wc.T @ R_wt                    # target in camera frame
        t_tc = R_wc.T @ (t_wt - t_wc)
        R_g2b.append(R_wb)
        t_g2b.append(t_wb)
        R_t2c.append(R_tc)
        t_t2c.append(t_tc)

    ok = True
    for name, (R, t) in solve_all_methods(R_g2b, t_g2b, R_t2c, t_t2c).items():
        rot_err = rot_angle_deg(R.T @ R_x)
        tr_err = np.linalg.norm(t - t_x) * 1000
        good = rot_err < 0.01 and tr_err < 0.1
        ok &= good
        print(f'  {"PASS" if good else "FAIL"}  {name:10s} '
              f'rot err {rot_err:.4f} deg, trans err {tr_err:.4f} mm')
    print('self-test ' + ('PASSED' if ok else 'FAILED'))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session', nargs='?',
                    help='session folder (bag + photos/ of the board)')
    ap.add_argument('--intrinsics', default='calibration/intrinsics.yaml')
    ap.add_argument('--rigid-body', default='phone')
    ap.add_argument('--clock-shows', action='append', default=[],
                    metavar='HH:MM:SS.mmm')
    ap.add_argument('--clock-offset', type=float, default=None, metavar='SEC')
    ap.add_argument('--min-corners', type=int, default=12)
    ap.add_argument('--out', default='calibration/hand_eye.yaml')
    ap.add_argument('--self-test', action='store_true')
    args = ap.parse_args()

    if args.self_test:
        sys.exit(self_test())
    if not args.session:
        ap.error('session folder required (or --self-test)')

    with open(args.intrinsics) as f:
        intr = yaml.safe_load(f)
    K = np.array(intr['camera_matrix'])
    dist = np.array(intr['distortion'])
    board = make_board(intr['board'])
    print(f'intrinsics: {args.intrinsics} (rms {intr["rms_px"]:.3f} px, '
          f'board {intr["board"]["squares_x"]}x{intr["board"]["squares_y"]} '
          f'@ {intr["board"]["square_mm"]:g} mm)')

    # clock offset (same convention as match_photos)
    session = args.session.rstrip('/')
    if args.clock_offset is not None:
        offset = args.clock_offset
    elif args.clock_shows:
        clock_files = sorted(
            glob(os.path.join(session, 'clock', '*.dng')) +
            glob(os.path.join(session, 'clock', '*.DNG')),
            key=lambda p: read_dng_exif(p)['epoch_s'])
        offsets = [clock_offset_from(s, read_dng_exif(c)['epoch_s'],
                                     read_dng_exif(c)['tz'])
                   for s, c in zip(args.clock_shows, clock_files)]
        offset = float(np.mean(offsets))
        print('clock offset: ' + ', '.join(f'{o:+.3f} s' for o in offsets))
    else:
        offset = 0.0
        print('WARNING: no clock offset given — assuming NTP-synced clocks',
              file=sys.stderr)

    track = load_pose_track(session, args.rigid_body)

    photos = sorted(glob(os.path.join(session, 'photos', '*.dng')) +
                    glob(os.path.join(session, 'photos', '*.DNG')))
    if not photos:
        raise SystemExit(f'no DNGs in {session}/photos/')

    R_g2b, t_g2b, R_t2c, t_t2c, img_pts, obj_pts, used = [], [], [], [], [], [], []
    for pf in photos:
        name = os.path.basename(pf)
        ex = read_dng_exif(pf)
        t_ns = int(round((ex['epoch_s'] + offset) * 1e9))
        pose, reason = interp_pose(track, t_ns)
        if pose is None:
            print(f'  SKIP {name}: {reason}')
            continue
        if pose['pos_spread_mm'] > 2.0:
            print(f'  SKIP {name}: moving at shutter '
                  f'({pose["pos_spread_mm"]:.1f} mm in ±50 ms) — hold still')
            continue
        det = detect_board(load_image(pf), board, args.min_corners)
        if det is None:
            print(f'  SKIP {name}: board not detected')
            continue
        obj, imgp = det
        ok, rvec, tvec = cv2.solvePnP(obj, imgp, K, dist,
                                      flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            print(f'  SKIP {name}: PnP failed')
            continue
        rvec, tvec = cv2.solvePnPRefineLM(obj, imgp, K, dist, rvec, tvec)
        R_t2c.append(cv2.Rodrigues(rvec)[0])
        t_t2c.append(tvec.ravel())
        R_g2b.append(quat_to_R(pose['quat']))
        t_g2b.append(np.asarray(pose['pos']))
        obj_pts.append(obj)
        img_pts.append(imgp)
        used.append(name)
        print(f'  ok   {name}: {len(imgp)} corners, '
              f'spread {pose["pos_spread_mm"]:.2f} mm')

    n = len(used)
    if n < 8:
        raise SystemExit(f'only {n} usable photos — need >= 8 (aim for 20+)')

    # rotation diversity: hand-eye is unsolvable without it
    max_rel = max(rot_angle_deg(R_g2b[i].T @ R_g2b[j])
                  for i in range(n) for j in range(i + 1, n))
    print(f'\n{n} pose pairs, max relative body rotation {max_rel:.1f} deg')
    if max_rel < 20:
        print('WARNING: little rotation diversity — solution will be poorly '
              'constrained. Re-shoot with more varied phone orientations.',
              file=sys.stderr)

    results = solve_all_methods(R_g2b, t_g2b, R_t2c, t_t2c)
    print('\nmethod results (camera in phone-body frame):')
    for name, (R, t) in results.items():
        print(f'  {name:10s} t = {np.round(t * 1000, 2).tolist()} mm, '
              f'quat = {np.round(R_to_quat(R), 5).tolist()}')
    R_ref, t_ref = results['PARK']
    spread_rot = max(rot_angle_deg(R.T @ R_ref) for R, _ in results.values())
    spread_tr = max(np.linalg.norm(t - t_ref) * 1000
                    for _, t in results.values())
    print(f'method agreement: rot within {spread_rot:.3f} deg, '
          f'trans within {spread_tr:.2f} mm '
          f'{"(good)" if spread_rot < 0.5 and spread_tr < 3 else "(POOR — check data)"}')

    # validation: board pose in world implied by each shot must agree, and
    # reprojection through mocap + hand-eye must match the detections
    boards_w = []
    for i in range(n):
        R_wc = R_g2b[i] @ R_ref
        t_wc = R_g2b[i] @ t_ref + t_g2b[i]
        boards_w.append((R_wc @ R_t2c[i], R_wc @ t_t2c[i] + t_wc))
    t_boards = np.array([b[1] for b in boards_w])
    t_bw = t_boards.mean(axis=0)
    q_mean = np.array([R_to_quat(b[0]) for b in boards_w])
    q_mean = np.where((q_mean @ q_mean[0])[:, None] < 0, -q_mean, q_mean).mean(0)
    R_bw = quat_to_R(q_mean / np.linalg.norm(q_mean))
    print(f'board-in-world consistency: '
          f'{np.linalg.norm(t_boards - t_bw, axis=1).max() * 1000:.2f} mm max spread')

    errs = []
    for i in range(n):
        R_wc = R_g2b[i] @ R_ref
        t_wc = R_g2b[i] @ t_ref + t_g2b[i]
        R_pred = R_wc.T @ R_bw
        t_pred = R_wc.T @ (t_bw - t_wc)
        proj, _ = cv2.projectPoints(obj_pts[i], cv2.Rodrigues(R_pred)[0],
                                    t_pred, K, dist)
        errs.append(float(np.sqrt(np.mean((proj - img_pts[i]) ** 2))))
    print(f'reprojection via mocap + hand-eye: mean {np.mean(errs):.2f} px, '
          f'max {np.max(errs):.2f} px')

    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    with open(args.out, 'w') as f:
        yaml.safe_dump({
            'date': datetime.now().isoformat(timespec='seconds'),
            'session': session,
            'rigid_body': args.rigid_body,
            'intrinsics_file': args.intrinsics,
            'camera_in_body': {
                'translation_m': [float(v) for v in t_ref],
                'quaternion_xyzw': [float(v) for v in R_to_quat(R_ref)],
            },
            'method': 'PARK',
            'n_photos': n,
            'max_relative_rotation_deg': float(max_rel),
            'method_agreement': {'rot_deg': float(spread_rot),
                                 'trans_mm': float(spread_tr)},
            'reprojection_px': {'mean': float(np.mean(errs)),
                                'max': float(np.max(errs))},
        }, f, sort_keys=False)
    print(f'\nwrote {args.out}')


if __name__ == '__main__':
    main()
