#!/usr/bin/env python3
"""Per-hold stability statistics of the /optitrack/phone stream.

For every still hold in the photo-session bag (matched to its CSV waypoint),
reports how trustworthy the mocap was there:

  n / expected   samples received vs expected at the stream rate -> dropout %
                 (dropout = markers occluded / body not solved = the
                 tracking-quality signal)
  pos std        per-axis + norm jitter of the body position (mm)
  rot std        orientation jitter (deg, angle to the hold's median)
  drift          |median(2nd half) - median(1st half)| (mm) - a slow slide
                 the std alone would hide

Prints a table and writes <dataset>/pose_stability.csv.

Usage:
  PYTHONPATH=scripts pixi run python scripts/pose_stability.py \\
      datasets/phone_20260826_113325
"""

import argparse
import csv
import os

import numpy as np

from link_robot_mocap import (load_bag_topic, load_run_csv, quat_mat,
                              rot_angle_deg)


def still_windows_idx(t, p, lag=1.0, thresh=0.0025, min_dur=2.0):
    il = np.searchsorted(t, t - lag)
    moving = np.linalg.norm(p - p[il], axis=1) > thresh
    out = []
    i, n = 0, len(moving)
    while i < n:
        j = i
        while j < n and moving[j] == moving[i]:
            j += 1
        if not moving[i] and t[j - 1] - t[i] >= min_dur:
            if out and np.linalg.norm(np.median(p[i:j], 0)
                                      - np.median(p[out[-1][0]:out[-1][1]], 0)) < 0.002:
                out[-1] = (out[-1][0], j)
            else:
                out.append((i, j))
        i = j
    return out


def match_by_angle(T_wp, q_stills):
    ang_wp = np.array([rot_angle_deg(T_wp[0][:3, :3].T @ T[:3, :3]) for T in T_wp])
    R0 = quat_mat(q_stills[0])
    out = [(0, 0)]
    for k in range(1, len(q_stills)):
        a = rot_angle_deg(R0.T @ quat_mat(q_stills[k]))
        w = int(np.argmin(np.abs(ang_wp - a)))
        if abs(ang_wp[w] - a) < 3.0 and w not in (m[1] for m in out):
            out.append((k, w))
    return dict(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dataset')
    args = ap.parse_args()
    ds = args.dataset.rstrip('/')
    bag = sorted(f for f in os.listdir(ds) if f.endswith('.db3'))[0]
    t, ph = load_bag_topic(os.path.join(ds, bag), '/optitrack/phone')
    t = t - t[0]
    # stream period from the densest part of the stream
    period = np.median(np.diff(t))
    rate = 1.0 / period
    print(f'stream: {len(t)} msgs, nominal rate {rate:.1f} Hz')

    csvs = sorted(f for f in os.listdir(os.path.join(ds, 'phone_poses_robot'))
                  if f.startswith('manual_poses_2026'))
    rows = []
    for bi, (lo, hi) in enumerate([(0, 336), (336, 615)]):
        m = (t >= lo) & (t < hi)
        tb, pb, qb = t[m], ph[m, :3], ph[m, 3:7]
        wins = still_windows_idx(tb, pb)
        T_wp, _ = load_run_csv(os.path.join(ds, 'phone_poses_robot', csvs[bi]))
        med_q = []
        for i, j in wins:
            q = np.median(qb[i:j], axis=0)
            med_q.append(q / np.linalg.norm(q))
        wp_of = match_by_angle(T_wp, med_q)
        for k, (i, j) in enumerate(wins):
            dur = tb[j - 1] - tb[i]
            n = j - i
            expected = dur / period + 1
            drop = max(0.0, 1 - n / expected)
            pos = pb[i:j]
            med = np.median(pos, axis=0)
            std = pos.std(axis=0) * 1000
            h = (i + j) // 2
            drift = np.linalg.norm(np.median(pos[:h - i], 0)
                                   - np.median(pos[h - i:], 0)) * 1000
            Rm = quat_mat(med_q[k])
            angs = [rot_angle_deg(Rm.T @ quat_mat(qq)) for qq in qb[i:j:max(1, n // 200)]]
            rows.append({
                'block': bi + 1,
                'waypoint': wp_of.get(k, -1),
                't_start_s': round(float(tb[i]), 1),
                'dur_s': round(float(dur), 1),
                'n': n,
                'dropout_pct': round(100 * drop, 1),
                'std_x_mm': round(float(std[0]), 3),
                'std_y_mm': round(float(std[1]), 3),
                'std_z_mm': round(float(std[2]), 3),
                'std_mm': round(float(np.linalg.norm(std)), 3),
                'rot_std_deg': round(float(np.std(angs)), 3),
                'rot_max_deg': round(float(np.max(angs)), 3),
                'drift_mm': round(float(drift), 3),
            })

    hdr = ['block', 'waypoint', 't_start_s', 'dur_s', 'n', 'dropout_pct',
           'std_x_mm', 'std_y_mm', 'std_z_mm', 'std_mm', 'rot_std_deg',
           'rot_max_deg', 'drift_mm']
    print(' | '.join(f'{h:>11}' for h in hdr))
    for r in rows:
        print(' | '.join(f'{r[h]:>11}' for h in hdr))
    out = os.path.join(ds, 'pose_stability.csv')
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=hdr)
        w.writeheader()
        w.writerows(rows)
    print(f'\nwrote {out}')


if __name__ == '__main__':
    main()
