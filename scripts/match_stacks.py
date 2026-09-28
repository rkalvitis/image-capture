#!/usr/bin/env python3
"""Match organized focus stacks to OptiTrack poses → stacks.csv.

Consumes a session folder whose images were organized by
scripts/organize_focus_stacks.py:

    datasets/phone_<ts>/
      <rosbag files>                 poses-only bag from `pixi run record-poses`
      images/<n>/<stacked>.jpg       one folder per stack, n = 1..N
      images/<n>/focus_stack/*.dng   the focus-sweep frames of that stack

One row is written per STACK (not per photo): the pose of the camera body and
the object body is the median over the stack's whole capture window (first to
last frame EXIF time — the rig is still for the entire sweep), and the camera
pose is re-expressed in the object body frame (rel_* columns). A rotating
object + fixed/arced camera thus becomes a virtual camera constellation
around the object at the origin.

Timestamps come from the focus-sweep DNGs (the composite JPG's EXIF time is a
copy of the last DNG's, not a 13th capture). With no clock photos, pass
--clock-offset if the iPhone↔Mac NTP offset was measured; the default 0 is
fine when both are NTP-synced — verify by checking that <body>_spread_mm
stays small (the rig is still around every stack, so a wrong offset shows up
as motion).

Salvage mode (--reconstruct-missing) for stacks shot BEFORE the bag started
(recorder started late). Requires the session's capture pattern to be
"blocks": the camera repeats the same --stacks-per-block poses in every
block, and the object only rotates BETWEEN blocks. Then, for a stack with no
bag coverage:
  - the CAMERA pose is copied from the same in-block position one block later
    (validated 2026-07-22: repetition within ~3 mm / 1.6 deg);
  - the OBJECT pose is the median over the bag's initial still window
    (auto-detected), valid when recording started before the first object
    rotation. The *_source columns record exactly what was reconstructed.
"""

import argparse
import csv
import math
import os
import sys
from glob import glob

import numpy as np

from match_photos import (read_dng_exif, load_pose_track, _quat_mean,
                          _quat_angle, _q_mul, _q_conj, _q_rot)

POSE_COLS = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
             'spread_mm', 'rot_spread_deg', 'gap_ms', 'source')


def discover_stacks(session):
    """[{n, stacked_image, frame_times}] from the organized images/ tree."""
    images_dir = os.path.join(session, 'images')
    dirs = sorted((d for d in glob(os.path.join(images_dir, '*'))
                   if os.path.isdir(d) and os.path.basename(d).isdigit()),
                  key=lambda d: int(os.path.basename(d)))
    if not dirs:
        raise SystemExit(f'no numbered stack folders in {images_dir} — '
                         'run scripts/organize_focus_stacks.py first')
    stacks = []
    for d in dirs:
        frames = sorted(glob(os.path.join(d, 'focus_stack', '*.dng')) +
                        glob(os.path.join(d, 'focus_stack', '*.DNG')))
        jpgs = sorted(glob(os.path.join(d, '*.jpg')) +
                      glob(os.path.join(d, '*.JPG')))
        if not frames or not jpgs:
            raise SystemExit(f'{d}: missing focus_stack frames or stacked jpg')
        times = sorted(read_dng_exif(f)['epoch_s'] for f in frames)
        stacks.append({'n': int(os.path.basename(d)),
                       'stacked_image': os.path.relpath(jpgs[0], session),
                       'n_frames': len(frames),
                       't_first': times[0], 't_mid': times[len(times) // 2],
                       't_last': times[-1]})
    return stacks


def median_pose(track, t0_s, t1_s, min_samples=10):
    """Median pose + quality metrics over [t0_s, t1_s] (unix seconds)."""
    i0 = np.searchsorted(track.times, int(t0_s * 1e9))
    i1 = np.searchsorted(track.times, int(t1_s * 1e9))
    if i1 - i0 < min_samples:
        return None
    P, Q = track.pos[i0:i1], track.quat[i0:i1]
    med = np.median(P, axis=0)
    qm = _quat_mean(Q)
    return {'pos': med, 'quat': qm,
            'spread_mm': float(np.max(np.linalg.norm(P - med, axis=1))) * 1e3,
            'rot_spread_deg': math.degrees(max(_quat_angle(q, qm) for q in Q)),
            'gap_ms': float(np.max(np.diff(track.times[i0:i1]))) / 1e6}


def initial_still_pose(track, max_pos_m=0.015, max_rot_rad=math.radians(3.0),
                       ref_s=5.0):
    """Median object pose over the bag's initial still window.

    The window runs from the first sample until the pose deviates from the
    initial ref_s-second median by more than the thresholds for a sustained
    stretch (0.5 s — single mocap noise spikes don't end the window). That is
    when the object was first moved/rotated.
    """
    t0 = track.times[0] / 1e9
    ref = median_pose(track, t0, t0 + ref_s)
    sustain_ns, out_since, end = int(0.5e9), None, len(track.times)
    for k in range(len(track.times)):
        out = (np.linalg.norm(track.pos[k] - ref['pos']) > max_pos_m
               or _quat_angle(track.quat[k], ref['quat']) > max_rot_rad)
        if not out:
            out_since = None
        elif out_since is None:
            out_since = k
        elif track.times[k] - track.times[out_since] > sustain_ns:
            end = out_since
            break
    t_end = track.times[max(end - 1, 1)] / 1e9
    res = median_pose(track, t0, t_end)
    res['window_s'] = t_end - t0
    return res


def relative_pose(cam, obj):
    """Camera pose in the object body frame: T_obj⁻¹ ∘ T_cam."""
    q_inv = _q_conj(obj['quat'])
    return {'pos': _q_rot(q_inv, cam['pos'] - obj['pos']),
            'quat': _q_mul(q_inv, cam['quat'])}


def pose_row(res, source):
    return [*(round(float(v), 6) for v in res['pos']),
            *(round(float(v), 6) for v in res['quat']),
            round(res['spread_mm'], 2), round(res['rot_spread_deg'], 3),
            round(res.get('gap_ms', 0.0), 1), source]


def load_lens_offset(path):
    """lens position in the camera body frame from a lens marker estimate
    (calibration/lens_marker_estimate.yaml — position only, no rotation)."""
    import yaml
    with open(path) as f:
        doc = yaml.safe_load(f)
    t = np.asarray(doc['lens_position_m'], dtype=float)
    print(f'hand-eye (marker estimate {path}): camera positions are the LENS '
          f'({np.round(t * 1e3, 1)} mm in the {doc.get("method", "?")!r} '
          f'body frame); orientation stays the marker body’s')
    return t


def process(args):
    session = args.session.rstrip('/')
    stacks = discover_stacks(session)
    print(f'{len(stacks)} stacks discovered in {session}/images/')

    lens_t = load_lens_offset(args.lens_marker) if args.lens_marker else None
    cam_track = load_pose_track(session, args.camera_body)
    obj_track = load_pose_track(session, args.object_body)

    # first pass: measured poses where the bag covers the stack window
    measured = {}
    for s in stacks:
        t0, t1 = s['t_first'] + args.clock_offset, s['t_last'] + args.clock_offset
        cam = median_pose(cam_track, t0, t1)
        obj = median_pose(obj_track, t0, t1)
        if cam is not None and obj is not None:
            if lens_t is not None:
                cam['pos'] = cam['pos'] + _q_rot(cam['quat'], lens_t)
            measured[s['n']] = (cam, obj)

    missing = [s['n'] for s in stacks if s['n'] not in measured]
    if missing:
        print(f'stacks with no bag coverage: {missing}')

    obj_start = None
    if missing and args.reconstruct_missing:
        obj_start = initial_still_pose(obj_track)
        print(f"object fallback: median over the bag's initial "
              f"{obj_start['window_s']:.0f} s still window "
              f"(spread {obj_start['spread_mm']:.1f} mm / "
              f"{obj_start['rot_spread_deg']:.1f} deg)")
    elif missing:
        print('(--reconstruct-missing not set — these stacks are skipped)',
              file=sys.stderr)

    rows, skipped = [], []
    for s in stacks:
        n = s['n']
        if n in measured:
            cam, obj = measured[n]
            cam_src = obj_src = 'measured'
        elif args.reconstruct_missing:
            donor = next((n + k * args.stacks_per_block
                          for k in range(1, 1 + len(stacks) // args.stacks_per_block)
                          if n + k * args.stacks_per_block in measured), None)
            if donor is None:
                skipped.append((n, 'no measured donor stack in any later block'))
                continue
            cam = measured[donor][0]
            cam_src = f'copied_from_stack_{donor}'
            obj, obj_src = obj_start, 'bag_start_window'
        else:
            skipped.append((n, 'no pose data in bag for stack window'))
            continue
        rel = relative_pose(cam, obj)
        rows.append([n, s['stacked_image'], s['n_frames'],
                     int(round((s['t_mid'] + args.clock_offset) * 1e9))]
                    + pose_row(cam, cam_src) + pose_row(obj, obj_src)
                    + [round(float(v), 6) for v in (*rel['pos'], *rel['quat'])])

    out = os.path.join(session, 'stacks.csv')
    with open(out, 'w', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(['stack', 'stacked_image', 'n_frames', 't_mid_ns']
                    + [f'cam_{c}' for c in POSE_COLS]
                    + [f'obj_{c}' for c in POSE_COLS]
                    + [f'rel_{c}' for c in ('x', 'y', 'z',
                                            'qx', 'qy', 'qz', 'qw')])
        wr.writerows(rows)
    print(f'wrote {len(rows)}/{len(stacks)} stacks → {out}')
    for n, why in skipped:
        print(f'  SKIPPED stack {n}: {why}', file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session', help='session folder (datasets/phone_<ts>)')
    ap.add_argument('--camera-body', default='phone',
                    help='rigid body carrying the camera (default: phone)')
    ap.add_argument('--object-body', default='popillia_mount',
                    help='rigid body rotating with the object '
                         '(default: popillia_mount)')
    ap.add_argument('--lens-marker', metavar='YAML',
                    help='lens marker estimate yaml (e.g. calibration/'
                         'lens_marker_estimate.yaml); shifts the camera '
                         'position from the marker-body origin to the lens '
                         '(hand-eye translation; orientation unchanged)')
    ap.add_argument('--clock-offset', type=float, default=0.0, metavar='SEC',
                    help='iPhone→Mac clock offset in seconds (default 0; '
                         'both NTP-synced)')
    ap.add_argument('--reconstruct-missing', action='store_true',
                    help='salvage stacks shot before the bag started (see '
                         'docstring; needs the block capture pattern)')
    ap.add_argument('--stacks-per-block', type=int, default=8,
                    help='camera poses per block for --reconstruct-missing '
                         '(default 8)')
    process(ap.parse_args())


if __name__ == '__main__':
    main()
