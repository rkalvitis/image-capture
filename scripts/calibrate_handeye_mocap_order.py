#!/usr/bin/env python3
"""Hand-eye from a FREEDRIVEN board run: mocap phone-body poses, ORDER-matched.

For the case where the arm was moved by hand (no pose replay, no CSV): the
hand poses come from /optitrack/phone still windows in the session bag, and
photo k (sorted) is matched to still window k - no clocks, no EXIF. Requires
the phone body tracked through the run (markers on) and a hold at every shot.

Solves T_body<-camera (cv2.calibrateHandEye), and additionally chains it to
the FLANGE using the 20260826 registration's body<->flange transform
(R_flange_from_body + marker_body_origin_in_flange_m) - valid as long as the
phone sits in the cradle the same way and the markers/body are unchanged
since that session.

    pixi run python scripts/calibrate_handeye_mocap_order.py \\
        datasets/phone_<ts> --images <dir> \\
        --intrinsics calibration/intrinsics_20260831.yaml \\
        [--registration datasets/phone_20260826_113325/registration.yaml]
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
                               resolve_ambiguity, solve_all_methods)
from calibrate_intrinsics import detect_board, load_image, make_board
from link_robot_mocap import load_bag_topic, quat_mat, still_windows
from match_photos import read_dng_exif


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('session', help='folder with the board-run .db3')
    ap.add_argument('--images', required=True)
    ap.add_argument('--intrinsics', required=True)
    ap.add_argument('--registration', default=None,
                    help='registration.yaml carrying R_flange_from_body + '
                         'marker_body_origin_in_flange_m to also output '
                         'camera_in_flange')
    ap.add_argument('--min-corners', type=int, default=12)
    ap.add_argument('--method', default='PARK', choices=sorted(METHODS))
    ap.add_argument('--still-thresh', type=float, default=0.0025)
    ap.add_argument('--min-dur', type=float, default=1.5)
    ap.add_argument('--out', default='calibration/handeye_robot.yaml')
    args = ap.parse_args()

    intr = yaml.safe_load(open(args.intrinsics))
    K = np.array(intr['camera_matrix'])
    dist = np.array(intr['distortion'])
    board = make_board(intr['board'])

    bag = sorted(glob(os.path.join(args.session, '*.db3')))[0]
    t0_abs, ph = load_bag_topic(bag, '/optitrack/phone')
    t = t0_abs - t0_abs[0]
    stills = still_windows(t, ph[:, :3], ph[:, 3:7],
                           thresh=args.still_thresh, min_dur=args.min_dur)
    files = sorted(f for f in glob(os.path.join(args.images, '*'))
                   if f.lower().endswith(('.dng', '.jpg', '.jpeg', '.png')))
    print(f'{len(stills)} still windows, {len(files)} photos')

    # --- pair by TIME: search the iPhone<->mocap clock offset that places
    # every photo inside a hold (freedriving creates pseudo-stills, so order
    # pairing is unreliable). Photo epochs vs bag epochs, 1-D search.
    epochs = np.array([read_dng_exif(f)['epoch_s'] for f in files])
    rel = epochs - t0_abs[0]
    # per-photo: nearest still start (stills store (t_start, pos, quat));
    # score an offset by summed distance from each shifted photo time to the
    # nearest still start (photos are taken during the hold, near its start)
    starts = np.array([sw[0] for sw in stills])
    cands = np.arange(-60.0, 60.0, 0.05)
    def score(d):
        idx = np.clip(np.searchsorted(starts, rel + d) - 1, 0, len(starts) - 1)
        # distance INTO the hold (photo should be shortly after a start)
        into = (rel + d) - starts[idx]
        bad = (into < 0) | (into > 20)
        return np.sum(np.where(bad, 20.0, 0.0)) + np.sum(np.abs(np.clip(into, 0, 20)))
    best = min(cands, key=score)
    idx = np.clip(np.searchsorted(starts, rel + best) - 1, 0, len(starts) - 1)
    print(f'clock offset iPhone->bag: {best:+.2f} s; '
          f'{len(set(idx))} distinct holds for {len(files)} photos')

    img_kw = dict()
    pairs = dict(R_g2b=[], t_g2b=[], R_t2c=[], t_t2c=[], branches=[], used=[])
    for k, f in enumerate(files):
        img = load_image(f)
        det = detect_board(img, board, args.min_corners)
        if det is None:
            print(f'  SKIP {os.path.basename(f)}: board not detected')
            continue
        obj, imgp = det
        nsol, rvecs, tvecs, errs = cv2.solvePnPGeneric(
            obj, imgp, K, dist, flags=cv2.SOLVEPNP_IPPE)
        if nsol == 0:
            continue
        branch = [(cv2.Rodrigues(rvecs[i])[0], tvecs[i].reshape(3),
                   float(errs[i])) for i in range(nsol)]
        _, c, q = stills[idx[k]]
        pairs['R_g2b'].append(quat_mat(q))
        pairs['t_g2b'].append(c)
        pairs['R_t2c'].append(branch[0][0])
        pairs['t_t2c'].append(branch[0][1])
        pairs['branches'].append(branch)
        pairs['used'].append(os.path.basename(f))
    print(f"{len(pairs['used'])} usable pairs")
    if len(pairs['used']) < 5:
        sys.exit('not enough pairs')

    # Branch selection by the rotation-angle INVARIANT: for photos i,j the
    # relative body rotation angle must equal the relative PnP rotation
    # angle. Label branches greedily for global consistency; the mirrored
    # global labeling is also self-consistent for planar targets, so solve
    # BOTH and keep the one whose static-board world pose is tighter.
    def rot_angle(R):
        return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))

    def label(seed_branch):
        Rb = pairs['R_g2b']
        br = pairs['branches']
        lab = [None] * len(br)
        lab[0] = min(seed_branch, len(br[0]) - 1)
        for i in range(1, len(br)):
            costs = []
            for c in range(len(br[i])):
                tot = 0.0
                for j in range(i):
                    aB = rot_angle(Rb[j].T @ Rb[i])
                    aC = rot_angle(br[j][lab[j]][0].T @ br[i][c][0])
                    tot += abs(aC - aB)
                costs.append(tot)
            lab[i] = int(np.argmin(costs))
        return lab

    def spread(lab, X):
        devs = []
        Ps = []
        for i in range(len(lab)):
            Twb = np.eye(4)
            Twb[:3, :3] = pairs['R_g2b'][i]
            Twb[:3, 3] = pairs['t_g2b'][i]
            Tct = np.eye(4)
            Tct[:3, :3] = pairs['branches'][i][lab[i]][0]
            Tct[:3, 3] = pairs['branches'][i][lab[i]][1]
            Ps.append(Twb @ X @ Tct)
        P = np.array([T[:3, 3] for T in Ps])
        return float(np.median(np.linalg.norm(P - np.median(P, 0), axis=1)))

    best_choice = None
    for seed in (0, 1):
        lab = label(seed)
        Rt = [pairs['branches'][i][lab[i]][0] for i in range(len(lab))]
        tt = [pairs['branches'][i][lab[i]][1] for i in range(len(lab))]
        res = solve_all_methods(pairs['R_g2b'], pairs['t_g2b'], Rt, tt)
        R0, t0 = res[args.method]
        X0 = np.eye(4)
        X0[:3, :3] = R0
        X0[:3, 3] = t0
        sp = spread(lab, X0)
        nflip = sum(1 for v in lab if v != 0)
        print(f'  seed branch {seed}: {nflip} flipped, '
              f'board-in-world median spread {sp * 1000:.1f} mm')
        if best_choice is None or sp < best_choice[0]:
            best_choice = (sp, lab)
    _, lab = best_choice
    for i in range(len(lab)):
        pairs['R_t2c'][i] = pairs['branches'][i][lab[i]][0]
        pairs['t_t2c'][i] = pairs['branches'][i][lab[i]][1]
    kept = pairs['used']
    results = solve_all_methods(pairs['R_g2b'], pairs['t_g2b'],
                                pairs['R_t2c'], pairs['t_t2c'])
    R_pick, t_pick = results[args.method]
    rots, trans = [], []
    for name, (R, tt) in results.items():
        dr = np.degrees(np.arccos(np.clip((np.trace(R.T @ R_pick) - 1) / 2,
                                          -1, 1)))
        dt = np.linalg.norm(tt - t_pick) * 1000
        rots.append(dr)
        trans.append(dt)
        print(f'{name:10s} vs {args.method}: rot {dr:6.3f} deg, '
              f'trans {dt:6.2f} mm')
    print(f'\ncamera in PHONE BODY: t {np.round(t_pick, 5)}, '
          f'axis {np.round(R_pick @ [0, 0, 1], 4)}')

    # VALIDATION: the board is static, so T_world<-board implied by every
    # pair must coincide. Spread = pairing + PnP + solution quality.
    X = np.eye(4)
    X[:3, :3] = R_pick
    X[:3, 3] = t_pick
    Tb_list = []
    for i in range(len(kept)):
        Twb = np.eye(4)
        Twb[:3, :3] = pairs['R_g2b'][i]
        Twb[:3, 3] = pairs['t_g2b'][i]
        Tct = np.eye(4)
        Tct[:3, :3] = pairs['R_t2c'][i]
        Tct[:3, 3] = pairs['t_t2c'][i]
        Tb_list.append(Twb @ X @ Tct)   # t2c IS T_cam<-target: no inverse
    Pb = np.array([T[:3, 3] for T in Tb_list])
    ctr = np.median(Pb, axis=0)
    dpos = np.linalg.norm(Pb - ctr, axis=1) * 1000
    Rref = Tb_list[int(np.argmin(dpos))][:3, :3]
    dang = [np.degrees(np.arccos(np.clip((np.trace(Rref.T @ T[:3, :3]) - 1) / 2,
                                         -1, 1))) for T in Tb_list]
    print('[validate] board-in-world consistency per pair:')
    for i in range(len(kept)):
        print(f'  {kept[i]}: pos dev {dpos[i]:7.1f} mm, rot dev {dang[i]:6.1f} deg')
    print(f'[validate] pos dev median {np.median(dpos):.1f} / max {dpos.max():.1f} mm; '
          f'rot dev median {np.median(dang):.1f} / max {max(dang):.1f} deg '
          f'(good calibration: a few mm / a few deg)')

    out = {
        'date': datetime.now().isoformat(timespec='seconds'),
        'variant': 'mocap-order (freedriven arm, phone body tracked)',
        'method': args.method,
        'n_pairs': len(kept),
        'pairs': kept,
        'intrinsics_file': args.intrinsics,
        'bag': bag,
        'camera_in_body': {
            'translation_m': t_pick.tolist(),
            'quaternion_xyzw': R_to_quat(R_pick).tolist(),
        },
        'method_agreement': {'rot_deg_max': float(max(rots)),
                             'trans_mm_max': float(max(trans))},
    }
    if args.registration:
        reg = yaml.safe_load(open(args.registration))
        Rx = np.array(reg['R_flange_from_body'])
        r = np.array(reg['marker_body_origin_in_flange_m'])
        R_fc = Rx @ R_pick
        t_fc = Rx @ t_pick + r
        axis_fl = R_fc @ np.array([0, 0, 1.0])
        print(f'camera in FLANGE (via 20260826 body<->flange): '
              f't {np.round(t_fc, 5)}, axis {np.round(axis_fl, 4)}')
        print('  cross-check: cased-lens estimate was '
              '(-0.0188, -0.0497, 0.1158), axis ~(-0.65, -0.15, 0.74)')
        out['camera_in_flange'] = {
            'translation_m': t_fc.tolist(),
            'quaternion_xyzw': R_to_quat(R_fc).tolist(),
            'via': args.registration,
            'note': 'chained through the 20260826 AX=XB body<->flange; valid '
                    'while the phone/markers/cradle seating are unchanged '
                    'since that session (~1 deg / few mm chain uncertainty)',
        }
        out['optical_axis_in_flange'] = axis_fl.tolist()
    with open(args.out, 'w') as f:
        yaml.safe_dump(out, f, sort_keys=False)
    print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
