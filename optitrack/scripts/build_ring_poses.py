#!/usr/bin/env python3
"""Per-image camera poses for a RING-protocol dataset (aug31_20deg_9vertical).

Protocol: for each robot height (pose 1..9), the insect does a full 360 in
~10 deg steps (~36 photos per ring), then the arm moves down. Both bodies
are tracked; the camera pose RELATIVE TO THE INSECT is measured per photo:

    T_insect<-cam(k) = inv(T_world<-popillia(t_k)) @ T_world<-phone(t_k)
                       @ T_body_from_cam

with t_k from the photo's EXIF time (clock offset auto-solved), poses as
medians over +-0.35 s, and T_body_from_cam from
calibrations/rig_transforms.yaml. No fold assumptions - the insect rotation
comes from the mocap, exactly as recorded.

Outputs into the dataset folder:
  camera_poses.csv   one row per image: ring, angle, T_insect<-cam
                     (t + quat), phone/popillia hold quality, sharpness
                     (Laplacian variance of the embedded preview - low =
                     defocused; ring 8 is the known focus-problem ring but
                     is scored per image, not excluded)
  ring_report.txt    per-ring summary

Usage:
  PYTHONPATH=scripts pixi run python scripts/build_ring_poses.py \\
      datasets/aug31_20deg_9vertical
"""

import argparse
import csv
import glob
import os

import cv2
import numpy as np
import yaml

from calibrate_intrinsics import load_image
from link_robot_mocap import load_bag_topic, mat_quat, quat_mat
from match_photos import read_dng_exif


def pose_at(t_arr, vals, t, half=0.35):
    i0, i1 = np.searchsorted(t_arr, [t - half, t + half])
    if i1 - i0 < 5:
        return None, None, 1e9, i1 - i0
    seg = vals[i0:i1]
    pos = np.median(seg[:, :3], axis=0)
    q = np.median(seg[:, 3:7], axis=0)
    q = q / np.linalg.norm(q)
    jitter = float(np.linalg.norm(seg[:, :3].std(axis=0)))
    return pos, q, jitter, i1 - i0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dataset')
    ap.add_argument('--no-sharpness', action='store_true')
    ap.add_argument('--axis-only', action='store_true',
                    help='only (re)fit the insect axis and exit')
    args = ap.parse_args()
    ds = args.dataset.rstrip('/')
    rt = yaml.safe_load(open(os.path.join(ds, 'calibrations',
                                          'rig_transforms.yaml')))
    R_bc = np.array(rt['T_body_from_cam']['rotation_matrix'])
    p_bc = np.array(rt['T_body_from_cam']['translation_m'])

    bag = sorted(glob.glob(os.path.join(ds, '*.db3')))[0]
    t_ph, ph = load_bag_topic(bag, '/optitrack/phone')
    t_po, po = load_bag_topic(bag, '/optitrack/popillia')
    files = sorted(glob.glob(os.path.join(ds, 'images', '*', '*.DNG')))
    epochs = np.array([read_dng_exif(f)['epoch_s'] for f in files])
    print(f'{len(files)} photos, bag {(t_ph[-1]-t_ph[0])/60:.1f} min')

    # clock offset: photos happen while BOTH bodies are still -> minimize
    # local motion around shifted photo times
    def motion_score(d):
        s = 0.0
        for e in epochs[::4]:
            t = e + d
            for ta, va in ((t_ph, ph), (t_po, po)):
                i0, i1 = np.searchsorted(ta, [t - 0.3, t + 0.3])
                if i1 - i0 < 5:
                    s += 0.05
                    continue
                s += float(np.linalg.norm(va[i0:i1, :3].std(axis=0)))
        return s
    cands = np.arange(-15.0, 15.0, 0.25)
    best = min(cands, key=motion_score)
    fine = np.arange(best - 0.3, best + 0.3, 0.05)
    best = min(fine, key=motion_score)
    print(f'clock offset iPhone->bag: {best:+.2f} s')

    def bridge_gap(t_arr, vals, t, tol_m=0.002):
        """Phone lost at t: the robot is PARKED during a ring, so if the
        tracked pose just before and just after the gap agree (<= tol), the
        photo's pose is their average. Returns (pos, quat, jitter_proxy) or
        None if the robot moved across the gap."""
        i = np.searchsorted(t_arr, t)
        i0, i1 = max(0, i - 60), min(len(t_arr), i + 60)
        before = vals[max(0, i - 300):i]
        after = vals[i:i + 300]
        if len(before) < 20 or len(after) < 20:
            return None
        pb = np.median(before[-200:, :3], axis=0)
        pa = np.median(after[:200, :3], axis=0)
        if np.linalg.norm(pb - pa) > tol_m:
            return None
        seg = np.vstack([before[-200:], after[:200]])
        q = np.median(seg[:, 3:7], axis=0)
        q = q / np.linalg.norm(q)
        return (pb + pa) / 2, q, float(np.linalg.norm(pb - pa))

    # ---- insect rotation axis, from RELATIVE step motions: each ~10 deg
    # step of the popillia body is a rotation about the rod axis, and that
    # axis is a CONSTANT line in the BODY frame - extract it as the screw
    # axis of every consecutive-step relative transform and average.
    # (A whole-session world circle fit fails here: the centroid orbit is
    # tiny and the fit is destroyed by drift / stage adjustments.)
    def pose_T(t):
        pos, q, j, n = pose_at(t_po, po, t)
        if pos is None:
            return None
        T = np.eye(4)
        T[:3, :3] = quat_mat(q)
        T[:3, 3] = pos
        return T
    axes, pts, angs = [], [], []
    for k in range(len(epochs) - 1):
        Ti = pose_T(epochs[k] + best)
        Tj = pose_T(epochs[k + 1] + best)
        if Ti is None or Tj is None:
            continue
        A = np.linalg.inv(Ti) @ Tj
        ang = np.degrees(np.arccos(np.clip((np.trace(A[:3, :3]) - 1) / 2, -1, 1)))
        if not (5 < ang < 15):
            continue
        w, V = np.linalg.eig(A[:3, :3])
        a = np.real(V[:, np.argmin(np.abs(w - 1))])
        a = a / np.linalg.norm(a)
        if axes and a @ axes[0] < 0:
            a = -a
        # point on the axis: (I - R) p = t, solved in the plane normal to a
        M = np.eye(3) - A[:3, :3]
        p_axis, *_ = np.linalg.lstsq(M + np.outer(a, a), A[:3, 3], rcond=None)
        p_axis = p_axis - (p_axis @ a) * a + 0  # any point on the line; project
        axes.append(a)
        pts.append(p_axis)
        angs.append(ang)
    axes = np.array(axes)
    pts = np.array(pts)
    a_dir = axes.mean(0)
    a_dir = a_dir / np.linalg.norm(a_dir)
    # points projected into the plane through origin normal to a_dir
    pts_pl = pts - np.outer(pts @ a_dir, a_dir)
    a_pt = np.median(pts_pl, axis=0)
    spread_dir = np.degrees(np.arccos(np.clip((axes @ a_dir).min(), -1, 1)))
    spread_pt = np.linalg.norm((pts_pl - a_pt), axis=1)
    print(f'[axis] from {len(axes)} step motions: dir {np.round(a_dir, 4)} '
          f'(worst dev {spread_dir:.2f} deg), point {np.round(a_pt, 4)} '
          f'(lateral spread median {np.median(spread_pt)*1000:.1f} mm), '
          f'step angle median {np.median(angs):.2f} deg')
    print(f'[axis] body-origin (marker centroid) orbit radius: '
          f'{np.linalg.norm(a_pt)*1000:.1f} mm')
    with open(os.path.join(ds, 'insect_axis.yaml'), 'w') as fo:
        yaml.safe_dump({
            'axis_point_insect_frame_m': a_pt.tolist(),
            'axis_dir_insect_frame': a_dir.tolist(),
            'centroid_orbit_radius_m': float(np.linalg.norm(a_pt)),
            'n_step_motions': int(len(axes)),
            'note': 'true rod/rotation axis in the popillia BODY frame, from '
                    'the screw axes of the ~10 deg step motions; the body '
                    'origin is the marker centroid and orbits this axis',
        }, fo, sort_keys=False)
    print('wrote insect_axis.yaml')
    if getattr(args, 'axis_only', False):
        return

    rows = []
    n_bridged = 0
    for k, f in enumerate(files):
        t = epochs[k] + best
        php, phq, phj, phn = pose_at(t_ph, ph, t)
        pop, poq, poj, pon = pose_at(t_po, po, t)
        if php is None:
            br = bridge_gap(t_ph, ph, t)
            if br is not None:
                php, phq, phj = br
                n_bridged += 1
                print(f'  BRIDGED {os.path.basename(f)}: phone lost at t, '
                      f'parked-pose before/after agree to {phj*1000:.2f} mm')
        if pop is None:
            br = bridge_gap(t_po, po, t)
            if br is not None:
                pop, poq, poj = br
        if php is None or pop is None:
            print(f'  DROP {os.path.basename(f)}: no mocap around photo time '
                  f'and the pose moved across the gap')
            continue
        T_wb = np.eye(4)
        T_wb[:3, :3] = quat_mat(phq)
        T_wb[:3, 3] = php
        T_wp = np.eye(4)
        T_wp[:3, :3] = quat_mat(poq)
        T_wp[:3, 3] = pop
        T_cam_w = T_wb.copy()
        T_cam_w[:3, :3] = T_wb[:3, :3] @ R_bc
        T_cam_w[:3, 3] = T_wb[:3, :3] @ p_bc + T_wb[:3, 3]
        T_ic = np.linalg.inv(T_wp) @ T_cam_w      # insect <- cam
        sharp = -1.0
        if not args.no_sharpness:
            img = load_image(f)
            g = cv2.cvtColor(cv2.resize(img, (1008, 756)), cv2.COLOR_BGR2GRAY)
            sharp = float(cv2.Laplacian(g, cv2.CV_64F).var())
        yaw = np.degrees(np.arctan2(T_wp[1, 0], T_wp[0, 0]))
        rows.append({
            'image': os.path.basename(f)[:-4],
            't_bag_s': round(float(t - t_ph[0]), 2),
            'cam_x': round(float(T_ic[0, 3]), 5),
            'cam_y': round(float(T_ic[1, 3]), 5),
            'cam_z': round(float(T_ic[2, 3]), 5),
            'quat_xyzw': ' '.join(f'{v:.6f}' for v in mat_quat(T_ic[:3, :3] @ np.eye(3))),
            'qx': 0,
            'popillia_yaw_deg': round(float(yaw), 2),
            'phone_jitter_mm': round(phj * 1000, 2),
            'pop_jitter_mm': round(poj * 1000, 2),
            'sharpness': round(sharp, 1),
            'phone_world': ' '.join(f'{v:.4f}' for v in php),
        })
    # fix quaternion field (of the insect<-cam rotation)
    for r in rows:
        r.pop('qx', None)

    # ring segmentation: consecutive photos share a phone WORLD pose; a jump
    # > 15 mm starts a new ring
    ring = 0
    prev = None
    for r in rows:
        pw = np.array([float(v) for v in r['phone_world'].split()])
        if prev is not None and np.linalg.norm(pw - prev) > 0.015:
            ring += 1
        r['ring'] = ring + 1
        prev = pw
    nrings = ring + 1
    print(f'{len(rows)} posed photos in {nrings} rings '
          f'({n_bridged} recovered across tracking gaps)')

    out = os.path.join(ds, 'camera_poses.csv')
    hdr = ['image', 'ring', 't_bag_s', 'cam_x', 'cam_y', 'cam_z',
           'quat_xyzw', 'popillia_yaw_deg', 'phone_jitter_mm',
           'pop_jitter_mm', 'sharpness', 'phone_world']
    with open(out, 'w', newline='') as fo:
        w = csv.DictWriter(fo, fieldnames=hdr)
        w.writeheader()
        w.writerows(rows)
    rep = [f'clock offset {best:+.2f} s; {len(rows)} photos, {nrings} rings']
    for g in range(1, nrings + 1):
        rs = [r for r in rows if r['ring'] == g]
        sh = np.array([r['sharpness'] for r in rs])
        yaws = np.array([r['popillia_yaw_deg'] for r in rs])
        steps = np.diff(np.unwrap(np.radians(yaws)))
        rep.append(
            f'ring {g:2d}: {len(rs):3d} photos, phone z '
            f'{np.mean([float(r["phone_world"].split()[2]) for r in rs]):.3f}, '
            f'median step {np.degrees(np.median(steps)) if len(steps) else 0:+.1f} deg, '
            f'sharpness median {np.median(sh):.0f} '
            f'min {sh.min():.0f}')
    with open(os.path.join(ds, 'ring_report.txt'), 'w') as fo:
        fo.write('\n'.join(rep) + '\n')
    print('\n'.join(rep))
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
