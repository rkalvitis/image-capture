#!/usr/bin/env python3
"""Physical ground truth for the lens position from a marker on the camera.

Procedure (done 2026-07-13): a marker ball is rested centered on the
ultra-wide lens ring, temporarily added to the `phone` rigid body in
Motive, and captured via `pixi run dump-bodies` (marker_6 in
calibration/rigid_bodies.yaml).

This script walks from the ball center down the lens axis to the optical
center:

    ball center
      − (ball-center height above the module face)   [from the plate plane:
              ball·n − phone thickness − module bump]
      − (module face → entrance pupil depth)         [teardown estimate]
    = ground-truth lens position  (± error bars)

and writes calibration/lens_ground_truth.yaml, which plot_handeye.py draws
as a star and every solver can be judged against.

    pixi run -- python scripts/lens_ground_truth.py --ball-diameter-mm 20
"""

import argparse
import os
from datetime import datetime

import numpy as np
import yaml

PHONE_T, BUMP = 8.25, 4.2          # mm — iPhone 15 Pro body + camera module
PLANE_MARKERS = ('marker_2', 'marker_3', 'marker_4')


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ball-diameter-mm', type=float, required=True,
                    help='measured diameter of the lens marker ball')
    ap.add_argument('--marker-name', default='marker_6',
                    help='the lens ball in rigid_bodies.yaml (default marker_6)')
    ap.add_argument('--pupil-depth-mm', type=float, default=2.0,
                    help='module face -> optical center depth (teardown '
                         'estimate, default 2.0)')
    ap.add_argument('--markers', default='calibration/rigid_bodies.yaml')
    ap.add_argument('--out', default='calibration/lens_ground_truth.yaml')
    args = ap.parse_args()

    with open(args.markers) as f:
        body = yaml.safe_load(f)['bodies']['phone']
    m = {mk['name']: np.array(mk['position_m']) for mk in body['markers']}
    ball = m[args.marker_name]
    p = [m[k] for k in PLANE_MARKERS]
    n = np.cross(p[1] - p[0], p[2] - p[0])
    n /= np.linalg.norm(n)
    if np.dot(ball - p[0], n) < 0:              # normal points to the phone side
        n = -n

    r = args.ball_diameter_mm / 2.0
    h_ball = float(np.dot(ball - p[0], n)) * 1000          # above plate, mm
    h_face = PHONE_T + BUMP                                # module face, mm
    h_cm = h_ball - h_face                                 # center above face
    if not 0 < h_cm <= r + 0.5:
        print(f'WARNING: ball-center height above module face = {h_cm:.1f} mm '
              f'not in (0, r={r:.1f}] — geometry assumptions may be off')
    rim = float(np.sqrt(max(0.0, r ** 2 - h_cm ** 2)))
    drop = (h_cm + args.pupil_depth_mm) / 1000.0
    gt = ball - drop * n

    print(f'ball center:            {(ball * 1000).round(2).tolist()} mm (body frame)')
    print(f'  above plate plane:    {h_ball:.2f} mm')
    print(f'  above module face:    {h_cm:.2f} mm '
          f'(consistent with resting on a ~{2 * rim:.1f} mm dia lens ring)')
    print(f'drop to optical center: {drop * 1000:.2f} mm along the lens axis')
    print(f'GROUND TRUTH lens:      {(gt * 1000).round(2).tolist()} mm')
    print(f'  above plate plane:    {h_ball - drop * 1000:.2f} mm')
    print('  error bars: ~1 mm lateral (ball centering), '
          '~2 mm axial (pupil depth)')

    # judge every stored solution
    print('\nsolver vs ground truth:')
    for root, _, files in sorted(os.walk('calibration')):
        for fn in sorted(files):
            if fn in ('hand_eye.yaml', 'hand_eye_eth.yaml'):
                path = os.path.join(root, fn)
                d = yaml.safe_load(open(path))
                t = np.array(d['camera_in_body']['translation_m'])
                dv = (t - gt) * 1000
                ax = float(np.dot(dv, n))
                lat = float(np.sqrt(max(0.0, np.dot(dv, dv) - ax ** 2)))
                label = d.get('method', d.get('solver', '?'))
                print(f'  {path:55s} [{label:>10s}]  '
                      f'axial {ax:+6.1f} mm, lateral {lat:5.1f} mm')

    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    with open(args.out, 'w') as f:
        yaml.safe_dump({
            'date': datetime.now().isoformat(timespec='seconds'),
            'method': 'marker ball on the ultra-wide lens ring '
                      '(marker_6 of the phone body)',
            'ball_diameter_mm': args.ball_diameter_mm,
            'pupil_depth_mm': args.pupil_depth_mm,
            'lens_position_m': [float(v) for v in gt],
            'sigma_mm': {'lateral': 1.0, 'axial': 2.0},
        }, f, sort_keys=False)
    print(f'\nwrote {args.out}')


if __name__ == '__main__':
    main()
