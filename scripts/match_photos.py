#!/usr/bin/env python3
"""Match iPhone photos to OptiTrack poses via EXIF timestamps.

Consumes a session folder produced per docs/IPHONE_IMPLEMENTATION_PLAN.md:

    datasets/phone_<ts>/
      <rosbag files>       poses-only bag from `pixi run record-poses`
      photos/*.dng         the dataset shots (ProRAW)
      clock/*.dng          clock photos (session start + end)

and writes frames.csv (one row per matched photo: median pose over the still
window around its capture time) plus rejected.csv (photos with no still
window — the phone was moving, or pose data is missing).

Clock offset: the iPhone clock is not the Mac clock. Photograph the
`pixi run ms-clock` display at session start/end and pass the time visible
in each clock photo, in EXIF order:

    pixi run match -- datasets/phone_<ts> \
        --clock-shows 14:03:21.132 --clock-shows 14:41:07.850

The offset (displayed Mac time minus the clock photo's own EXIF time) is
applied to every photo timestamp. Two clock photos also expose clock drift.
Alternatively pass --clock-offset SECONDS directly; with neither, 0 is used
(fine for stop-and-shoot if both devices are NTP-synced, but measure it).

Output frame (default): the camera pose is expressed **in the insect's
body frame** (relative pose = object_pose⁻¹ ∘ camera_pose, both bodies
interpolated at the photo instant). This supports the turntable method —
fixed camera, insect rotated between shots — where a rotating insect with a
fixed camera is equivalent to a camera orbiting a fixed insect. It is also
correct for the older handheld-orbit sessions (there the object simply
barely moves). --world switches back to raw world-frame camera poses.

Matching (default): the pose is interpolated at each photo's corrected EXIF
timestamp — no shooting protocol is enforced. Per-row quality columns let
you filter afterwards: 'gap_ms' (spacing of the bracketing pose samples;
large = bridged across a tracking dropout) and 'pos_spread_mm' /
'rot_spread_deg' (relative camera↔object motion within ±50 ms of the shot —
the pose uncertainty a tens-of-ms timestamp error causes). Keeping both
bodies still around each shot (turntable at rest) makes the pose immune to
clock error.

--require-still (opt-in strict mode): only accept photos with a still
window around them (largest of ±1.5/±1.0/±0.6/±0.3 s whose pose spread is
below --pos-thresh-mm / --rot-thresh-deg); its median pose is the answer.

Until the Stage 2 hand-eye calibration exists, poses are of the `phone`
marker body, not the lens optical center.

Self-test (no lab, no files):  python scripts/match_photos.py --self-test
"""

import argparse
import csv
import math
import mmap
import os
import struct
import sys
from bisect import bisect_left, bisect_right
from datetime import datetime, timedelta, timezone
from glob import glob

import numpy as np

# ─── EXIF capture time from a DNG (stdlib TIFF walk, no exif library) ───────

_TYPESIZE = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 6: 1, 7: 1, 8: 2, 9: 4, 10: 8}


def _ifd_entries(mm, bo, off):
    """Yield (tag, type, count, value_offset) for each entry of the IFD at off."""
    (n,) = struct.unpack_from(bo + 'H', mm, off)
    for i in range(n):
        tag, typ, cnt = struct.unpack_from(bo + 'HHI', mm, off + 2 + i * 12)
        voff = off + 2 + i * 12 + 8
        if _TYPESIZE.get(typ, 1) * cnt > 4:
            (voff,) = struct.unpack_from(bo + 'I', mm, voff)
        yield tag, typ, cnt, voff


def _ascii(mm, voff, cnt):
    return bytes(mm[voff:voff + cnt]).split(b'\0')[0].decode('ascii', 'replace')


def read_dng_exif(path):
    """Return {'epoch_s', 'iso', 'exposure_s'} from a DNG's EXIF block.

    epoch_s combines DateTimeOriginal + SubSecTimeOriginal +
    OffsetTimeOriginal into a float unix timestamp (the *iPhone's* clock).
    """
    with open(path, 'rb') as f:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
    try:
        bo = '<' if mm[:2] == b'II' else '>'
        (ifd0,) = struct.unpack_from(bo + 'I', mm, 4)
        exif_off = None
        for tag, typ, cnt, voff in _ifd_entries(mm, bo, ifd0):
            if tag == 34665:                     # ExifIFD pointer (inline LONG)
                exif_off = struct.unpack_from(bo + 'I', mm, voff)[0]
        if exif_off is None:
            raise ValueError(f'{path}: no EXIF IFD')

        dto = subsec = offset = None
        iso = exposure = None
        for tag, typ, cnt, voff in _ifd_entries(mm, bo, exif_off):
            if tag == 36867:                     # DateTimeOriginal
                dto = _ascii(mm, voff, cnt)
            elif tag == 37521:                   # SubSecTimeOriginal
                subsec = _ascii(mm, voff, cnt)
            elif tag == 36881:                   # OffsetTimeOriginal
                offset = _ascii(mm, voff, cnt)
            elif tag == 34855:                   # ISO
                iso = struct.unpack_from(bo + 'H', mm, voff)[0]
            elif tag == 33434:                   # ExposureTime (rational)
                num, den = struct.unpack_from(bo + 'II', mm, voff)
                exposure = num / den if den else 0.0
        if dto is None:
            raise ValueError(f'{path}: no DateTimeOriginal')

        dt = datetime.strptime(dto, '%Y:%m:%d %H:%M:%S')
        dt = dt.replace(tzinfo=_parse_tz(offset))
        frac = float('0' + '.' + subsec) if subsec and subsec.isdigit() else 0.0
        return {'epoch_s': dt.timestamp() + frac, 'iso': iso,
                'exposure_s': exposure, 'tz': dt.tzinfo}
    finally:
        mm.close()


def _parse_tz(offset_str):
    """EXIF '+02:00' → tzinfo. Missing/malformed → local timezone (warn)."""
    if offset_str and len(offset_str) >= 6 and offset_str[0] in '+-':
        sign = 1 if offset_str[0] == '+' else -1
        h, m = int(offset_str[1:3]), int(offset_str[4:6])
        return timezone(sign * timedelta(hours=h, minutes=m))
    print('WARNING: photo has no EXIF timezone; assuming Mac local time',
          file=sys.stderr)
    return datetime.now().astimezone().tzinfo


def clock_offset_from(shown_str, clock_exif_epoch, tz):
    """Offset (s) to ADD to iPhone EXIF times to get Mac time.

    shown_str is the 'HH:MM:SS.mmm' visible in the clock photo (Mac local
    time); its date is taken from the clock photo itself, disambiguated to
    the nearest day.
    """
    hms, _, frac = shown_str.partition('.')
    h, m, s = (int(x) for x in hms.split(':'))
    photo_dt = datetime.fromtimestamp(clock_exif_epoch, tz)
    shown_dt = photo_dt.replace(hour=h, minute=m, second=s, microsecond=0)
    shown_epoch = shown_dt.timestamp() + (float('0.' + frac) if frac else 0.0)
    off = shown_epoch - clock_exif_epoch
    # nearest-day disambiguation (clock photo near midnight)
    if off > 43200:
        off -= 86400
    elif off < -43200:
        off += 86400
    return off


# ─── Pose track + stillness matching ─────────────────────────────────────────

class PoseTrack:
    """Time-sorted PoseStamped samples of one rigid body."""

    def __init__(self, times_ns, pos, quat):
        order = np.argsort(times_ns)
        self.times = np.asarray(times_ns, dtype=np.int64)[order]
        self.pos = np.asarray(pos, dtype=float)[order]
        self.quat = np.asarray(quat, dtype=float)[order]   # xyzw


def _quat_mean(Q):
    """Sign-aligned normalized mean of xyzw quaternions (fine for a still window)."""
    q0 = Q[0]
    aligned = np.where((Q @ q0)[:, None] < 0, -Q, Q)
    m = aligned.mean(axis=0)
    return m / np.linalg.norm(m)


def _quat_angle(q, qm):
    return 2.0 * math.acos(min(1.0, abs(float(np.dot(q, qm)))))


def _q_mul(a, b):
    """Hamilton product of two xyzw quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz])


def _q_conj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def _q_rot(q, v):
    """Rotate vector v by xyzw quaternion q."""
    return _q_mul(_q_mul(q, np.array([v[0], v[1], v[2], 0.0])), _q_conj(q))[:3]


def build_relative_track(cam, obj):
    """Camera poses re-expressed in the (possibly rotating) object body frame.

    For each camera sample, the object pose is interpolated at that instant
    and the relative transform T_obj→cam = T_world→obj⁻¹ ∘ T_world→cam is
    taken. Camera samples outside the object track's coverage are skipped.
    A rotating insect + fixed camera thus becomes a camera orbit around a
    fixed insect at the origin.
    """
    times, pos, quat = [], [], []
    for k in range(len(cam.times)):
        t = int(cam.times[k])
        i = bisect_left(obj.times, t)
        if i == 0 or i == len(obj.times):
            continue
        t0, t1 = int(obj.times[i - 1]), int(obj.times[i])
        u = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        p_obj = (1.0 - u) * obj.pos[i - 1] + u * obj.pos[i]
        q_obj = _slerp(obj.quat[i - 1], obj.quat[i], u)
        q_inv = _q_conj(q_obj)
        times.append(t)
        pos.append(_q_rot(q_inv, cam.pos[k] - p_obj))
        quat.append(_q_mul(q_inv, cam.quat[k]))
    if not times:
        raise SystemExit('camera and object tracks do not overlap in time')
    return PoseTrack(times, pos, quat)


def _slerp(q0, q1, u):
    """Spherical interpolation between two xyzw quaternions, u in [0, 1]."""
    dot = float(np.dot(q0, q1))
    if dot < 0.0:            # take the short arc
        q1, dot = -q1, -dot
    if dot > 0.9995:         # nearly parallel — lerp is fine and stable
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = math.acos(max(-1.0, min(1.0, dot)))
    s = math.sin(theta)
    return (math.sin((1.0 - u) * theta) * q0 + math.sin(u * theta) * q1) / s


def interp_pose(track, t_ns):
    """Pose interpolated at t_ns (the default matching mode).

    No stillness or gap requirement — every photo inside the bag's time span
    gets a pose. 'gap_ms' reports the spacing of the bracketing samples
    (~10 ms normally; large values mean the pose was bridged across a
    tracking dropout — filter on it downstream). Returns
    (result_dict, None) or (None, reason_str).
    """
    i = bisect_left(track.times, t_ns)
    if i == 0 or i == len(track.times):
        return None, 'no pose data around timestamp (check clock offset / bag)'
    t0, t1 = int(track.times[i - 1]), int(track.times[i])
    u = (t_ns - t0) / (t1 - t0) if t1 > t0 else 0.0
    pos = (1.0 - u) * track.pos[i - 1] + u * track.pos[i]
    quat = _slerp(track.quat[i - 1], track.quat[i], u)
    # quality metric: how much the camera moved within ±50 ms of the shot —
    # roughly the pose uncertainty from a ±tens-of-ms timestamp error
    j0 = bisect_left(track.times, t_ns - 50_000_000)
    j1 = bisect_right(track.times, t_ns + 50_000_000)
    P, Q = track.pos[j0:j1], track.quat[j0:j1]
    if len(P) == 0:                      # dropout: no samples near the shot
        P, Q = track.pos[i - 1:i + 1], track.quat[i - 1:i + 1]
    med = np.median(P, axis=0)
    return {'pos': pos, 'quat': quat, 'still_ms': 0.0,
            'gap_ms': (t1 - t0) / 1e6,
            'pos_spread_mm': float(np.max(np.linalg.norm(P - med, axis=1))) * 1e3,
            'rot_spread_deg': math.degrees(
                max(_quat_angle(q, _quat_mean(Q)) for q in Q))}, None


_WINDOWS_S = (1.5, 1.0, 0.6, 0.3)


def match_still_pose(track, t_ns, pos_thresh_m, rot_thresh_rad, min_samples=20):
    """Median pose over the largest still window around t_ns.

    Returns (result_dict, None) or (None, reason_str).
    """
    best = None   # (pos_spread, rot_spread, span_ms) of the best failing try
    for w in _WINDOWS_S:
        half = int(w * 1e9)
        i0 = bisect_left(track.times, t_ns - half)
        i1 = bisect_right(track.times, t_ns + half)
        if i1 - i0 < min_samples:
            continue
        P, Q = track.pos[i0:i1], track.quat[i0:i1]
        med = np.median(P, axis=0)
        pos_spread = float(np.max(np.linalg.norm(P - med, axis=1)))
        qm = _quat_mean(Q)
        rot_spread = max(_quat_angle(q, qm) for q in Q)
        span_ms = (track.times[i1 - 1] - track.times[i0]) / 1e6
        if pos_spread <= pos_thresh_m and rot_spread <= rot_thresh_rad:
            return {'pos': med, 'quat': qm, 'still_ms': span_ms,
                    'gap_ms': float(np.max(np.diff(track.times[i0:i1]))) / 1e6,
                    'pos_spread_mm': pos_spread * 1e3,
                    'rot_spread_deg': math.degrees(rot_spread)}, None
        if best is None or pos_spread < best[0]:
            best = (pos_spread, rot_spread, span_ms)
    if best is None:
        return None, 'no pose data around timestamp (check clock offset / bag)'
    return None, (f'moving: best window spread {best[0]*1e3:.1f} mm / '
                  f'{math.degrees(best[1]):.2f} deg over {best[2]:.0f} ms')


# ─── Bag loading (lazy ROS imports — self-test runs without the overlay) ────

def load_pose_track(bag_dir, rigid_body, allow_fallback=True):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import PoseStamped

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    topics = [t.name for t in reader.get_all_topics_and_types()
              if t.name.startswith('/optitrack/')]
    want = f'/optitrack/{rigid_body}'
    if want not in topics:
        if allow_fallback and len(topics) == 1:
            want = topics[0]
            print(f'NOTE: rigid body {rigid_body!r} not in bag; '
                  f'using the only body present: {want}', file=sys.stderr)
        else:
            raise SystemExit(f'rigid body topic {want} not in bag; '
                             f'available: {topics}')

    times, pos, quat = [], [], []
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic != want:
            continue
        m = deserialize_message(raw, PoseStamped)
        times.append(m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec)
        p, o = m.pose.position, m.pose.orientation
        pos.append((p.x, p.y, p.z))
        quat.append((o.x, o.y, o.z, o.w))
    if not times:
        raise SystemExit(f'no messages on {want} in {bag_dir}')
    print(f'{want}: {len(times)} poses, '
          f'{(max(times) - min(times)) / 1e9:.1f} s span')
    return PoseTrack(times, pos, quat)


# ─── Session processing ──────────────────────────────────────────────────────

def process_session(args):
    session = args.session.rstrip('/')
    photo_files = sorted(glob(os.path.join(session, 'photos', '*.dng')) +
                         glob(os.path.join(session, 'photos', '*.DNG')))
    if not photo_files:
        raise SystemExit(f'no DNGs in {session}/photos/')

    # clock offset
    if args.clock_offset is not None:
        offset = args.clock_offset
        print(f'clock offset (given): {offset:+.3f} s')
    elif args.clock_shows:
        clock_files = sorted(
            glob(os.path.join(session, 'clock', '*.dng')) +
            glob(os.path.join(session, 'clock', '*.DNG')),
            key=lambda p: read_dng_exif(p)['epoch_s'])
        if len(clock_files) < len(args.clock_shows):
            raise SystemExit(f'{len(args.clock_shows)} --clock-shows values but '
                             f'only {len(clock_files)} DNGs in {session}/clock/')
        offsets = []
        for shown, cf in zip(args.clock_shows, clock_files):
            ex = read_dng_exif(cf)
            offsets.append(clock_offset_from(shown, ex['epoch_s'], ex['tz']))
        offset = float(np.mean(offsets))
        print('clock offset: ' + ', '.join(f'{o:+.3f} s' for o in offsets)
              + f' → using {offset:+.3f} s')
        if len(offsets) > 1 and max(offsets) - min(offsets) > 0.05:
            print(f'WARNING: clock offset drifted '
                  f'{(max(offsets) - min(offsets)) * 1e3:.0f} ms during the '
                  f'session — timestamps may be degraded', file=sys.stderr)
    else:
        offset = 0.0
        print('WARNING: no --clock-shows/--clock-offset — assuming iPhone and '
              'Mac clocks agree (NTP). Measure it next session.',
              file=sys.stderr)

    track = load_pose_track(session, args.rigid_body)
    if not args.world:
        try:
            obj = load_pose_track(session, args.object_body,
                                  allow_fallback=False)
        except SystemExit as e:
            print(f'WARNING: {e} — falling back to world-frame camera poses',
                  file=sys.stderr)
        else:
            track = build_relative_track(track, obj)
            print(f'poses expressed in the {args.object_body!r} body frame '
                  f'({len(track.times)} relative samples)')

    matched, rejected = [], []
    for pf in photo_files:
        rel = os.path.relpath(pf, session)
        try:
            ex = read_dng_exif(pf)
        except (ValueError, struct.error) as e:
            rejected.append((rel, f'EXIF parse failed: {e}'))
            continue
        t_ns = int(round((ex['epoch_s'] + offset) * 1e9))
        if args.require_still:
            res, reason = match_still_pose(
                track, t_ns,
                pos_thresh_m=args.pos_thresh_mm * 1e-3,
                rot_thresh_rad=math.radians(args.rot_thresh_deg))
        else:
            res, reason = interp_pose(track, t_ns)
        if res is None:
            rejected.append((rel, reason))
            continue
        p, q = res['pos'], res['quat']
        matched.append([rel, t_ns,
                        *(round(float(v), 6) for v in p),
                        *(round(float(v), 6) for v in q),
                        round(res['still_ms'], 1),
                        round(res['gap_ms'], 1),
                        round(res['pos_spread_mm'], 2),
                        round(res['rot_spread_deg'], 3),
                        ex['iso'], ex['exposure_s']])

    frames_csv = os.path.join(session, 'frames.csv')
    with open(frames_csv, 'w', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(['image_file', 't_capture_ns', 'x', 'y', 'z',
                     'qx', 'qy', 'qz', 'qw', 'still_ms', 'gap_ms',
                     'pos_spread_mm', 'rot_spread_deg', 'iso', 'exposure_s'])
        wr.writerows(sorted(matched, key=lambda r: r[1]))

    rej_csv = os.path.join(session, 'rejected.csv')
    with open(rej_csv, 'w', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(['image_file', 'reason'])
        wr.writerows(rejected)

    print(f'\nmatched {len(matched)}/{len(photo_files)} photos → {frames_csv}')
    for rel, reason in rejected:
        print(f'  REJECTED {rel}: {reason}', file=sys.stderr)
    if rejected:
        print(f'({len(rejected)} rejected → {rej_csv})', file=sys.stderr)


# ─── Self-test (synthetic track + fabricated timestamps, no ROS, no files) ──

def self_test():
    rng = np.random.default_rng(0)
    hz, t0 = 120, 1_700_000_000_000_000_000        # some epoch, ns
    times, pos, quat = [], [], []

    def q_z(deg):
        a = math.radians(deg) / 2
        return np.array([0.0, 0.0, math.sin(a), math.cos(a)])

    A, B = np.array([1.0, 2.0, 3.0]), np.array([2.0, 2.0, 3.0])
    qA, qB = q_z(0), q_z(30)
    for i in range(10 * hz):                        # 10 s @ 120 Hz
        t = i / hz
        if t < 3.0:      p, q = A, qA               # still at A
        elif t < 4.0:                                # moving A → B
            u = t - 3.0
            p, q = A + u * (B - A), q_z(30 * u)
        elif t < 7.0:    p, q = B, qB               # still at B
        else:                                        # moving again
            p, q = B + (t - 7.0) * np.array([0.3, 0, 0]), qB
        times.append(t0 + int(t * 1e9))
        pos.append(p + rng.normal(0, 0.0002, 3))    # 0.2 mm mocap noise
        quat.append(q / np.linalg.norm(q))
    track = PoseTrack(times, pos, quat)

    def t_at(sec):
        return t0 + int(sec * 1e9)

    ok = True

    def check(name, cond):
        nonlocal ok
        print(f'  {"PASS" if cond else "FAIL"}  {name}')
        ok &= cond

    kw = dict(pos_thresh_m=0.002, rot_thresh_rad=math.radians(0.3))
    resA, _ = match_still_pose(track, t_at(1.5), **kw)
    check('still shot at A matches', resA is not None)
    check('pose A correct (<1 mm)',
          resA is not None and np.linalg.norm(resA['pos'] - A) < 0.001)

    resB, _ = match_still_pose(track, t_at(5.5), **kw)
    check('still shot at B matches', resB is not None)
    check('pose B correct (<1 mm, ~30 deg)',
          resB is not None and np.linalg.norm(resB['pos'] - B) < 0.001
          and abs(_quat_angle(resB['quat'], q_z(30))) < math.radians(0.1))

    resM, reasonM = match_still_pose(track, t_at(3.5), **kw)
    check('moving shot rejected', resM is None and reasonM.startswith('moving'))

    resN, reasonN = match_still_pose(track, t_at(60.0), **kw)
    check('shot outside pose data rejected',
          resN is None and reasonN.startswith('no pose data'))

    resI, _ = interp_pose(track, t_at(3.5))
    mid = (A + B) / 2
    check('interp: moving shot pose = midpoint (~15 deg)',
          resI is not None and np.linalg.norm(resI['pos'] - mid) < 0.001
          and abs(_quat_angle(resI['quat'], q_z(15))) < math.radians(0.1))
    resI2, reasonI2 = interp_pose(track, t_at(60.0))
    check('interp: outside pose data rejected',
          resI2 is None and reasonI2.startswith('no pose data'))

    # turntable scenario: camera FIXED, object rotating 45 deg/s about z.
    # In the object frame the camera must orbit: at t the relative position
    # is R(-45t°) @ cam_pos and the relative orientation is q_z(-45t°).
    cam_p = np.array([0.5, 0.0, 0.2])
    tt, tp, tq = [], [], []
    ot, op, oq = [], [], []
    for i in range(8 * hz):
        t = i / hz
        tt.append(t0 + int(t * 1e9)); tp.append(cam_p); tq.append(q_z(0))
        ot.append(t0 + int(t * 1e9)); op.append(np.zeros(3))
        oq.append(q_z(45.0 * t))
    rel = build_relative_track(PoseTrack(tt, tp, tq), PoseTrack(ot, op, oq))
    resR, _ = interp_pose(rel, t_at(2.0))          # object rotated 90 deg
    expect_p = np.array([0.0, -0.5, 0.2])          # R(-90°) @ (0.5, 0, 0.2)
    check('turntable: fixed camera orbits in object frame',
          resR is not None and np.linalg.norm(resR['pos'] - expect_p) < 0.001
          and abs(_quat_angle(resR['quat'], q_z(-90))) < math.radians(0.1))
    check('turntable: orbit radius constant',
          abs(np.linalg.norm(rel.pos[:, :2], axis=1).max() - 0.5) < 1e-6
          and abs(np.linalg.norm(rel.pos[:, :2], axis=1).min() - 0.5) < 1e-6)

    # clock-offset math: photo EXIF says 10:00:00.000, screen showed 10:00:00.500
    tz = timezone(timedelta(hours=2))
    exif_epoch = datetime(2026, 7, 7, 10, 0, 0, tzinfo=tz).timestamp()
    off = clock_offset_from('10:00:00.500', exif_epoch, tz)
    check('clock offset = +0.500 s', abs(off - 0.5) < 1e-6)
    off = clock_offset_from('23:59:59.900',
                            datetime(2026, 7, 7, 0, 0, 0, tzinfo=tz).timestamp(), tz)
    check('midnight wrap disambiguated', abs(off - (-0.1)) < 1e-6)

    print('self-test ' + ('PASSED' if ok else 'FAILED'))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session', nargs='?',
                    help='session folder (datasets/phone_<ts>)')
    ap.add_argument('--rigid-body', default='phone',
                    help="Motive rigid body of the iPhone (default: phone)")
    ap.add_argument('--object-body', default='popillia',
                    help="Motive rigid body of the photographed object; "
                         "camera poses are output in ITS body frame, so a "
                         "rotating object + fixed camera becomes a camera "
                         "orbit (default: popillia)")
    ap.add_argument('--world', action='store_true',
                    help='output raw world-frame camera poses instead of '
                         'object-frame relative poses')
    ap.add_argument('--clock-shows', action='append', default=[],
                    metavar='HH:MM:SS.mmm',
                    help='time visible in a clock photo; repeat per clock photo '
                         '(EXIF order: session start first)')
    ap.add_argument('--clock-offset', type=float, default=None, metavar='SEC',
                    help='iPhone→Mac clock offset in seconds (overrides '
                         '--clock-shows)')
    ap.add_argument('--pos-thresh-mm', type=float, default=2.0,
                    help='max position spread to count as still (default 2.0)')
    ap.add_argument('--rot-thresh-deg', type=float, default=0.5,
                    help='max rotation spread to count as still (default 0.5; '
                         'Motive orientation noise on small bodies measured '
                         '~0.3-0.4 deg peak, lab test 2026-07-08)')
    ap.add_argument('--require-still', action='store_true',
                    help='opt-in strict mode: only accept photos with a '
                         'still window around them (default: interpolate '
                         'the pose at each photo\'s EXIF timestamp, no '
                         'protocol enforced)')
    ap.add_argument('--self-test', action='store_true',
                    help='run the synthetic matcher test (no session needed)')
    args = ap.parse_args()

    if args.self_test:
        sys.exit(self_test())
    if not args.session:
        ap.error('session folder required (or --self-test)')
    process_session(args)


if __name__ == '__main__':
    main()
