#!/usr/bin/env python3
"""3D view of a stack session's camera constellation (from stacks.csv).

Reads the stacks.csv written by scripts/match_stacks.py and shows the camera
positions in the insect (object body) frame: each arm pose traces a closed
RING around the specimen (one point per object rotation), rings stacked from
the top of the arc down — the "simulated 360°" constellation.

The stand's rotation axis and the specimen are fitted from the data itself —
no hand-eye calibration needed:
  - axis: the world axis is the circle fit through the object body origins
    (the marker centroid orbits it); re-expressed in the object body frame it
    is constant across rotations.
  - specimen: the point that stays FIXED in the camera body frame across all
    poses. A rig that keeps the object centred in frame while orbiting it
    holds the object at one constant location relative to the camera, so
    that fixed point is where the object was (on the 2026-07-22 session it
    lands 0.8 mm off the rotation axis, spread 4.7 mm rms). Note this rig
    property also makes the hand-eye translation unobservable from these
    poses — and a hand-eye from an older session does NOT transfer once the
    phone body was redefined in Motive, so camera points are the marker
    body, not the lens.
  - facing arrows + lens markers are anchored on the capture protocol: arm
    pose 1 shoots straight down from directly above the specimen. That
    anchor fixes the optical axis in the body frame, and --working-distance
    MM puts the lens MM millimetres above the specimen at pose 1; the rigid
    body-frame offset then places lens and facing at every other pose from
    the measured orientations. (Aim-at-the-object fitting is deliberately
    NOT used: with an ultra-wide lens at ~25 mm the object need not be
    centred in frame.)

Reconstructed stacks (cam_source != measured, see --reconstruct-missing in
match_stacks.py) are drawn as hollow markers.

    pixi run python scripts/plot_stacks.py datasets/phone_<ts> \
        [--frame insect|world] [--working-distance 25] [--save out.png]

Only needs numpy + matplotlib (no ROS) — runs outside the lab.
"""

import argparse
import csv
import math
import os

import numpy as np

# one-hue ordinal ramp (light→dark = ring 1→8), colorblind-safe step gaps
RING_COLORS = ['#86b6ef', '#719dd3', '#5d85b7', '#496e9c',
               '#375882', '#254269', '#132e50', '#041a39']
SPECIMEN_COLOR = '#eb6834'


def rotm(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


def load(session):
    path = os.path.join(session.rstrip('/'), 'stacks.csv')
    if not os.path.exists(path):
        raise SystemExit(f'{path} not found — run scripts/match_stacks.py first')
    with open(path) as f:
        rows = list(csv.DictReader(f))
    vec = lambda r, p: np.array([float(r[p + c]) for c in ('x', 'y', 'z')])
    quat = lambda r, p: np.array([float(r[p + 'q' + c]) for c in 'xyzw'])
    return [{'n': int(r['stack']),
             'rel': vec(r, 'rel_'), 'rel_q': quat(r, 'rel_'),
             'cam': vec(r, 'cam_'),
             'obj': vec(r, 'obj_'), 'obj_q': quat(r, 'obj_'),
             'measured': r['cam_source'] == 'measured'} for r in rows]


def fit_circle_xy(pts):
    """Least-squares (Kåsa) circle through xy of pts → (cx, cy, r)."""
    x, y = pts[:, 0], pts[:, 1]
    u, v = x - x.mean(), y - y.mean()
    A = np.array([[(u*u).sum(), (u*v).sum()], [(u*v).sum(), (v*v).sum()]])
    b = np.array([((u**3) + u*v*v).sum() / 2, ((v**3) + v*u*u).sum() / 2])
    uc, vc = np.linalg.solve(A, b)
    r = math.sqrt(uc*uc + vc*vc + (u*u + v*v).mean())
    return x.mean() + uc, y.mean() + vc, r


def fit_geometry(stacks, spb):
    """Rotation axis + specimen position, both in the object body frame."""
    blocks = sorted({(s['n'] - 1) // spb for s in stacks})
    firsts = [next(s for s in stacks if (s['n'] - 1) // spb == b)
              for b in blocks]
    cx, cy, orbit_r = fit_circle_xy(np.array([s['obj'] for s in firsts]))

    axis_pts, axis_dirs = [], []
    for s in firsts:
        R, p = rotm(s['obj_q']), s['obj']
        axis_pts.append(R.T @ (np.array([cx, cy, p[2]]) - p))
        axis_dirs.append(R.T @ np.array([0.0, 0.0, 1.0]))
    ap = np.mean(axis_pts, axis=0)
    ad = np.mean(axis_dirs, axis=0)
    ad /= np.linalg.norm(ad)

    # specimen = the point most nearly FIXED in the camera body frame:
    # minimize the spread of s_i = R_i^T (S - p_i) over S — linear LS.
    # (rel_q is the camera body orientation in the object frame.)
    A = np.stack([rotm(s['rel_q']).T for s in stacks])
    b = np.stack([-A[i] @ s['rel'] for i, s in enumerate(stacks)])
    Am, bm = A.mean(0), b.mean(0)
    M, v = np.zeros((3, 3)), np.zeros(3)
    for i in range(len(stacks)):
        D = A[i] - Am
        M += D.T @ D
        v -= D.T @ (b[i] - bm)
    spec = np.linalg.solve(M, v)
    s_body = np.einsum('nij,j->ni', A, spec) + b
    spread = np.linalg.norm(s_body - s_body.mean(0), axis=1)
    off_axis = np.linalg.norm(np.cross(spec - ap, ad))
    print(f'axis (object frame): point {np.round(ap * 1e3, 1)} mm, '
          f'dir {np.round(ad, 3)}')
    print(f'specimen (body-frame fixed point): {np.round(spec * 1e3, 1)} mm, '
          f'{off_axis * 1e3:.1f} mm off the rotation axis, '
          f'spread rms {np.sqrt((spread**2).mean()) * 1e3:.1f} mm')

    # camera facing + lens, anchored on the capture protocol: ARM POSE 1
    # SHOOTS STRAIGHT DOWN FROM DIRECTLY ABOVE THE SPECIMEN. That anchor
    # fixes the optical axis in the body frame (o_b = R1^T · down); the arm
    # then pitches the phone through its semicircle, so ring-by-ring facing
    # follows from the measured orientations alone. (An aim-at-the-object
    # fit is NOT usable here: with an ultra-wide lens at ~25 mm the object
    # need not be centred in frame.)
    pose1 = [s for s in stacks if (s['n'] - 1) % spb == 0]
    o_bs = np.stack([rotm(s['rel_q']).T @ (-ad) for s in pose1])
    o_b = o_bs.mean(0)
    o_b /= np.linalg.norm(o_b)
    sweep = [math.degrees(math.asin(max(-1.0, min(1.0,
             (rotm(s['rel_q']) @ o_b) @ ad)))) for s in stacks]
    print(f'optical axis (body frame, pose-1-down anchor): {np.round(o_b, 3)}')
    print(f'facing elevation sweep: {min(sweep):.1f} deg (down) to '
          f'{max(sweep):.1f} deg (up)')
    return ap, ad, spec, o_b, {'cx': cx, 'cy': cy, 'r': orbit_r}


def align_heading(stacks, spb, ap, ad, spec, camera_azimuth_deg):
    """Rotate the insect-frame data about the rotation axis so azimuth 0° is
    the insect's heading: the block-1 cameras end up at camera_azimuth_deg
    (CCW about the axis, seen from above). Geometry is unchanged — this only
    re-references where azimuth zero points."""
    e1 = np.array([1.0, 0.0, 0.0])
    e1 -= (e1 @ ad) * ad
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(ad, e1)
    blk1 = np.mean([s['rel'] for s in stacks if s['n'] <= spb], axis=0) - ap
    az_now = math.atan2(blk1 @ e2, blk1 @ e1)
    a = math.radians(camera_azimuth_deg) - az_now
    K = np.array([[0, -ad[2], ad[1]], [ad[2], 0, -ad[0]], [-ad[1], ad[0], 0]])
    R = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)
    qa = np.array([*(math.sin(a / 2) * ad), math.cos(a / 2)])   # xyzw
    for s in stacks:
        s['rel'] = ap + R @ (s['rel'] - ap)
        x, y, z, w = s['rel_q']
        ax_, ay, az_, aw = qa
        s['rel_q'] = np.array([          # q_align ⊗ q (orientations rotate too)
            aw * x + ax_ * w + ay * z - az_ * y,
            aw * y - ax_ * z + ay * w + az_ * x,
            aw * z + ax_ * y - ay * x + az_ * w,
            aw * w - ax_ * x - ay * y - az_ * z])
    return ap + R @ (spec - ap), e1


def plot_insect(ax, stacks, spb, ap, ad, spec, look=None,
                working_distance_m=None, heading_dir=None,
                show_facing=False):
    mm = 1e3
    n_rings = spb

    # camera position per stack: the LENS when the pose-1 anchor is usable
    # (rigid body-frame offset placing the lens working_distance directly
    # above the specimen at arm pose 1), else the marker-body origin
    if working_distance_m:
        pose1 = [s for s in stacks if (s['n'] - 1) % spb == 0]
        t_b = np.mean([rotm(s['rel_q']).T
                       @ (spec + working_distance_m * ad - s['rel'])
                       for s in pose1], axis=0)
        for s in stacks:
            s['pt'] = s['rel'] + rotm(s['rel_q']) @ t_b
        dists = [np.linalg.norm(s['pt'] - spec) for s in stacks]
        print(f'lens offset (body frame, anchored): {np.round(t_b * 1e3, 1)} '
              f'mm; lens-to-specimen across all stacks '
              f'{min(dists) * 1e3:.1f}–{max(dists) * 1e3:.1f} mm')
        what = 'lens (pose-1 anchor)'
    else:
        for s in stacks:
            s['pt'] = s['rel']
        what = 'marker body'

    # basis perpendicular to the rotation axis, for drawing ring circles
    e1 = np.array([1.0, 0.0, 0.0])
    e1 -= (e1 @ ad) * ad
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(ad, e1)
    circle_t = np.linspace(0, 2 * np.pi, 73)

    # the arm's physical path: one semicircle per insect rotation
    blocks = sorted({(s['n'] - 1) // spb for s in stacks})
    for i, blk in enumerate(blocks):
        path = np.array([s['pt'] for s in stacks
                         if (s['n'] - 1) // spb == blk]) * mm
        ax.plot(path[:, 0], path[:, 1], path[:, 2], color='0.55', lw=1.0,
                alpha=0.7, label='arm semicircle (per rotation)'
                if i == 0 else None)

    for k in range(1, n_rings + 1):
        ring = sorted((s for s in stacks if (s['n'] - 1) % spb + 1 == k),
                      key=lambda s: s['n'])
        pts = np.array([s['pt'] for s in ring]) * mm
        c = RING_COLORS[(k - 1) % len(RING_COLORS)]
        # the circle this arm pose traces around the stand axis as the
        # insect rotates (the 6 points are samples of it)
        hh = np.mean([(s['pt'] - ap) @ ad for s in ring])
        rr = np.mean([np.linalg.norm(np.cross(s['pt'] - ap, ad))
                      for s in ring])
        circ = (ap + hh * ad
                + rr * np.outer(np.cos(circle_t), e1)
                + rr * np.outer(np.sin(circle_t), e2)) * mm
        ax.plot(circ[:, 0], circ[:, 1], circ[:, 2], color=c, lw=0.9,
                ls=':', alpha=0.5)
        meas = np.array([s['measured'] for s in ring])
        ax.scatter(*pts[meas].T, color=c, s=36, depthshade=False,
                   label=f'ring {k} (arm pose {k})')
        if (~meas).any():
            ax.scatter(*pts[~meas].T, facecolors='none', edgecolors=c,
                       s=36, depthshade=False)
        if show_facing and look is not None:
            # facing arrows: the anchored optical axis rotated by each
            # pose's measured orientation
            dirs = np.array([rotm(s['rel_q']) @ look for s in ring]) * 10
            ax.quiver(pts[:, 0], pts[:, 1], pts[:, 2],
                      dirs[:, 0], dirs[:, 1], dirs[:, 2],
                      color=c, lw=1.4, alpha=0.9, arrow_length_ratio=0.35,
                      label='facing' if k == 1 else None)
    ax.set_title(f'Camera positions in the insect frame — {what}\n'
                 '(8 arm poses × 6 insect rotations; hollow = reconstructed)')
    ax.scatter(*spec * mm, color=SPECIMEN_COLOR, marker='*', s=340,
               depthshade=False, label='specimen (fitted)')
    if heading_dir is not None:
        head = np.stack([spec, spec + 0.045 * heading_dir]) * mm
        ax.plot(*head.T, color=SPECIMEN_COLOR, lw=1.6)
        ax.text(*head[1], '  insect heading 0°', color=SPECIMEN_COLOR,
                fontsize=8)
    hs = [(s['pt'] - ap) @ ad for s in stacks]
    t = np.array([min(hs) - 0.015, max(hs) + 0.015])
    axis_line = (ap[None] + t[:, None] * ad[None]) * mm
    ax.plot(*axis_line.T, color='0.6', lw=0.8, ls='--', label='rotation axis')


def plot_world(ax, stacks, spb, orbit):
    mm = 1e3
    cams = [s for s in stacks if spb < s['n'] <= 2 * spb]     # block 2: measured
    for i, s in enumerate(cams):
        c = RING_COLORS[i % len(RING_COLORS)]
        ax.scatter(*s['cam'] * mm, color=c, s=42, depthshade=False,
                   label=f'arm pose {i + 1}')
    arc = np.array([s['cam'] for s in cams]) * mm
    ax.plot(arc[:, 0], arc[:, 1], arc[:, 2], color='0.6', lw=1.0, alpha=0.8)
    firsts = {}
    for s in stacks:
        firsts.setdefault((s['n'] - 1) // spb, s)
    objs = np.array([s['obj'] for s in firsts.values()]) * mm
    ax.scatter(objs[:, 0], objs[:, 1], objs[:, 2], color='0.35', s=40,
               depthshade=False, label='mount origin (6 rotations)')
    t = np.linspace(0, 2 * np.pi, 73)
    ax.plot(orbit['cx'] * mm + orbit['r'] * mm * np.cos(t),
            orbit['cy'] * mm + orbit['r'] * mm * np.sin(t),
            np.full_like(t, objs[:, 2].mean()), color='0.8', lw=0.8)
    zl = np.array([objs[:, 2].mean() - 100, arc[:, 2].max() + 60])
    ax.plot([orbit['cx'] * mm] * 2, [orbit['cy'] * mm] * 2, zl,
            color='0.6', lw=0.8, ls='--', label='stand rotation axis')
    ax.set_title('World frame: the actual rig\n'
                 '(fixed camera arc, insect mount orbiting the stand axis)')


def main():
    ap_ = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap_.add_argument('session', help='session folder containing stacks.csv')
    ap_.add_argument('--frame', choices=('insect', 'world'), default='insect')
    ap_.add_argument('--stacks-per-block', type=int, default=8,
                     help='camera poses per rotation block (default 8)')
    ap_.add_argument('--working-distance', type=float, metavar='MM',
                     help='estimated lens-to-object distance in mm; marks '
                          'the lens on each sight line (insect frame only)')
    ap_.add_argument('--camera-azimuth-deg', type=float, metavar='DEG',
                     help='azimuth (CCW about the rotation axis, seen from '
                          'above) at which the block-1 cameras sat relative '
                          'to the insect\'s 0° heading; rotates the '
                          'insect-frame view so azimuth 0° = insect heading '
                          '(phone_20260722_143911: 270)')
    ap_.add_argument('--facing', action='store_true',
                     help='draw a facing arrow per camera (insect frame)')
    ap_.add_argument('--save', metavar='PNG',
                     help='write the figure instead of opening a window')
    args = ap_.parse_args()

    if args.save:
        import matplotlib
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    stacks = load(args.session)
    spb = args.stacks_per_block
    ap, ad, spec, look, orbit = fit_geometry(stacks, spb)

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(projection='3d')
    if args.frame == 'insect':
        wd = args.working_distance * 1e-3 if args.working_distance else None
        heading = None
        if args.camera_azimuth_deg is not None:
            spec, heading = align_heading(stacks, spb, ap, ad, spec,
                                          args.camera_azimuth_deg)
        plot_insect(ax, stacks, spb, ap, ad, spec, look, wd, heading,
                    show_facing=args.facing)
    else:
        plot_world(ax, stacks, spb, orbit)
    ax.set_xlabel('x (mm)'); ax.set_ylabel('y (mm)'); ax.set_zlabel('z (mm)')
    ax.legend(loc='upper left', fontsize=8, framealpha=0.9)
    # equal aspect so rings are round
    lims = np.array([getattr(ax, f'get_{a}lim')() for a in 'xyz'])
    ctr, half = lims.mean(1), (lims[:, 1] - lims[:, 0]).max() / 2
    for a, c in zip('xyz', ctr):
        getattr(ax, f'set_{a}lim')(c - half, c + half)
    fig.tight_layout()

    if args.save:
        fig.savefig(args.save, dpi=160)
        print(f'saved → {args.save}')
    else:
        # matplotlib 3D has no scroll zoom out of the box — add it
        def on_scroll(ev):
            f = 0.85 if ev.button == 'up' else 1 / 0.85
            for get, set_ in ((ax.get_xlim, ax.set_xlim),
                              (ax.get_ylim, ax.set_ylim),
                              (ax.get_zlim, ax.set_zlim)):
                lo, hi = get()
                c, h = (lo + hi) / 2, (hi - lo) / 2 * f
                set_(c - h, c + h)
            fig.canvas.draw_idle()
        fig.canvas.mpl_connect('scroll_event', on_scroll)
        print('controls: left-drag = rotate · scroll (or right-drag) = zoom')
        plt.show()


if __name__ == '__main__':
    main()
