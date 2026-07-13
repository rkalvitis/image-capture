#!/usr/bin/env python3
"""3D view of the hand-eye result inside the `phone` rigid-body frame.

    pixi run -- python scripts/plot_handeye.py datasets/phone_<ts> \
        [--intrinsics calibration/intrinsics.yaml] [--clock-shows ...]

Re-processes the hand-eye session and renders an interactive HTML
(calibration/hand_eye_view.html) showing, all in the phone BODY frame (mm):

  - the body origin with its x/y/z axes (this is the point OptiTrack streams);
  - one gray dot PER PHOTO: the camera position implied by that single shot
    (via the mean board pose) — the scatter of this cloud IS the calibration
    uncertainty, visualized;
  - the five solver results (Tsai/Park/Horaud/Andreff/Daniilidis) as
    diamonds — tight clustering = healthy solve;
  - the camera's optical axis (look direction) from the PARK solution.

Hover any dot for the photo it came from.
"""

import argparse
import os
import sys
import webbrowser

import numpy as np
import plotly.graph_objects as go
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibrate_handeye import (collect_pairs, resolve_ambiguity,
                               solve_all_methods, R_to_quat, quat_to_R)

# categorical palette (fixed order); gray for the per-shot cloud
CAT = ['#2a78d6', '#1baf7a', '#eda100', '#008300', '#4a3aa7']
AXIS_COLORS = {'x': '#e34948', 'y': '#1baf7a', 'z': '#2a78d6'}

# iPhone 15 Pro schematic (mm): body 146.6 x 70.6 x 8.25 with ~12 mm
# rounded corners; camera module ~36.5 mm rounded square, ~4.2 mm proud of
# the back, in the top-left corner (ultra-wide lens = its lower-left spot).
# Physical setup (2026-07-13): the phone rests flat on the plate whose
# plane is defined by markers 2/3/4 of the `phone` body — the sketch is
# seated on that plane when the marker file is available, anchored in-plane
# at the projected lens position. Lens placement on the body is approximate.
PHONE_L, PHONE_W, PHONE_T = 146.6, 70.6, 8.25
PHONE_R = 12.0                                 # body corner radius
# The sketch is anchored at the ULTRA-WIDE lens (the camera used, per EXIF):
# the UPPER-most lens of the module, ~17 mm from the top edge, ~14 mm from
# the side. All module/lens offsets below are relative to it (a = toward
# the phone bottom, b = away from the near side edge).
LENS_FROM_TOP, LENS_FROM_EDGE = 17.0, 14.0
CAM_BOX = dict(a0=-8.0, a1=28.5, b0=-8.0, b1=28.5, r=9.0, bump=4.2)
LENS_R = 7.5
OTHER_LENSES = ((19.0, 0.0), (9.5, 19.0))      # main (wide), telephoto
PHONE_PLANE_MARKERS = ('marker_2', 'marker_3', 'marker_4')


def _rounded_rect(a0, a1, b0, b1, r, n=6):
    """Closed (a, b) outline of a rounded rectangle."""
    import math
    pts = []
    for ca, cb, start in ((a1 - r, b1 - r, 0), (a0 + r, b1 - r, 90),
                          (a0 + r, b0 + r, 180), (a1 - r, b0 + r, 270)):
        for k in range(n + 1):
            ang = math.radians(start + 90 * k / n)
            pts.append((ca + r * math.cos(ang), cb + r * math.sin(ang)))
    pts.append(pts[0])
    return pts


def _circle(ca, cb, r, n=24):
    """Closed (a, b) outline of a circle."""
    import math
    return [(ca + r * math.cos(2 * math.pi * k / n),
             cb + r * math.sin(2 * math.pi * k / n)) for k in range(n + 1)]


def _phone_sketch(origin, u, v, nvec, side):
    """Wireframe polylines (mm) of the schematic phone in an (u, v, n) frame.

    origin = the point on the SCREEN plane below the ultra-wide lens; the
    body slab extends side*thickness along n (the screen rests at c = 0),
    the camera module protrudes further to side*(thickness + bump), and the
    lens circles sit on the module face. Returns (paths, uw_circle).
    """
    def to3(a, b, c):
        return (origin + (a * u + b * v + c * nvec) / 1000.0) * 1000

    c_back = side * PHONE_T
    c_mod = side * (PHONE_T + CAM_BOX['bump'])              # module outer face
    a0, a1 = -LENS_FROM_TOP, PHONE_L - LENS_FROM_TOP
    b0, b1 = -LENS_FROM_EDGE, PHONE_W - LENS_FROM_EDGE
    body = _rounded_rect(a0, a1, b0, b1, PHONE_R)
    cam = _rounded_rect(CAM_BOX['a0'], CAM_BOX['a1'],
                        CAM_BOX['b0'], CAM_BOX['b1'], CAM_BOX['r'])
    paths = [
        [to3(a, b, 0.0) for a, b in body],                  # screen face
        [to3(a, b, c_back) for a, b in body],               # back face
        [to3(a, b, c_back) for a, b in cam],                # camera module
        [to3(a, b, c_mod) for a, b in cam],                 # module top
    ]
    for ca, cb in OTHER_LENSES:                             # main + telephoto
        paths.append([to3(a, b, c_mod) for a, b in _circle(ca, cb, LENS_R)])
    # connectors between the two body faces (at the four side midpoints)
    for a, b in (( (a0 + a1) / 2, b0), ((a0 + a1) / 2, b1),
                 (a0, (b0 + b1) / 2), (a1, (b0 + b1) / 2)):
        paths.append([to3(a, b, 0.0), to3(a, b, c_back)])
    xs, ys, zs = [], [], []
    for path in paths:
        for p in path:
            xs.append(p[0]); ys.append(p[1]); zs.append(p[2])
        xs.append(None); ys.append(None); zs.append(None)
    # ultra-wide lens circle separately, on the module face at the anchor
    uw = [to3(a, b, c_mod) for a, b in _circle(0.0, 0.0, LENS_R)]
    uw_xyz = ([p[0] for p in uw], [p[1] for p in uw], [p[2] for p in uw])
    return (xs, ys, zs), uw_xyz


def phone_wireframe(R_x, t_x):
    """Fallback sketch in the camera frame (no marker plane available), mm."""
    look = R_x[:, 2]
    origin = t_x - (PHONE_T + CAM_BOX['bump']) / 1000.0 * look
    return _phone_sketch(origin, R_x[:, 0], R_x[:, 1], look, side=1.0)


def phone_wireframe_on_plane(R_x, t_x, marker_pos):
    """Sketch seated on the plane of markers 2/3/4 (the mounting plate), mm.

    The phone occupies the side of the plate the camera LOOKS toward (the
    board it photographed was in front of the lens — this is the provable
    side, independent of the noisy translation). Physically the lens should
    then sit ~+(thickness+bump) from the plate on that side; the returned
    signed lens-to-plane distance is printed as a validation number.
    """
    p = [marker_pos[name] for name in PHONE_PLANE_MARKERS]
    n = np.cross(p[1] - p[0], p[2] - p[0])
    n /= np.linalg.norm(n)
    look = R_x[:, 2]
    side = 1.0 if np.dot(look, n) > 0 else -1.0   # the side the camera views
    d_lens = float(np.dot(t_x - p[0], n))
    anchor = t_x - d_lens * n                   # lens projected onto the plane
    u = R_x[:, 0] - np.dot(R_x[:, 0], n) * n    # camera x projected in-plane
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    sketch, uw = _phone_sketch(anchor, u, v, n, side)
    return sketch, uw, d_lens, side


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session')
    ap.add_argument('--intrinsics', default='calibration/intrinsics.yaml')
    ap.add_argument('--rigid-body', default='phone')
    ap.add_argument('--clock-shows', action='append', default=[],
                    metavar='HH:MM:SS.mmm')
    ap.add_argument('--clock-offset', type=float, default=None, metavar='SEC')
    ap.add_argument('--min-corners', type=int, default=12)
    ap.add_argument('--markers', default='calibration/rigid_bodies.yaml',
                    help='marker constellation from scripts/dump_rigid_bodies.py '
                         '(skipped if the file does not exist)')
    ap.add_argument('--method', default=None,
                    help='which solver anchors the sketch/look direction: '
                         'ETH, TSAI, PARK, HORAUD, ANDREFF or DANIILIDIS '
                         '(default: ETH when hand_eye_eth.yaml exists, '
                         'else PARK)')
    ap.add_argument('--out', default=None,
                    help='output HTML; default: '
                         'calibration/<session>/hand_eye_view.html')
    ap.add_argument('--no-open', action='store_true',
                    help="don't open the result in the browser")
    args = ap.parse_args()

    pairs = collect_pairs(args.session, args.intrinsics, args.rigid_body,
                          args.clock_shows, args.clock_offset, args.min_corners)
    resolve_ambiguity(pairs)
    R_g2b, t_g2b = pairs['R_g2b'], pairs['t_g2b']
    R_t2c, t_t2c = pairs['R_t2c'], pairs['t_t2c']
    used = pairs['used']
    n = len(used)
    if n < 3:
        raise SystemExit(f'only {n} usable photos')

    results = solve_all_methods(R_g2b, t_g2b, R_t2c, t_t2c)

    # ETH result (independent solver), if calibrate-handeye-eth has been run
    eth_path = (f'calibration/{os.path.basename(args.session.rstrip("/"))}/'
                f'hand_eye_eth.yaml')
    eth = None
    if os.path.exists(eth_path):
        with open(eth_path) as f:
            eth = yaml.safe_load(f)

    # anchor for the sketch / look direction / lens circle: ETH when
    # available (most robust on our data so far), else the stored method
    method = args.method
    if method is None:
        method = 'ETH' if eth else 'PARK'
    if method == 'ETH':
        if eth is None:
            raise SystemExit(f'--method ETH needs {eth_path} — run '
                             'calibrate-handeye-eth first')
        t_x = np.array(eth['camera_in_body']['translation_m'])
        R_x = quat_to_R(np.array(eth['camera_in_body']['quaternion_xyzw']))
    elif method in results:
        R_x, t_x = results[method]
    else:
        raise SystemExit(f'method {method!r} not available '
                         f'(got ETH + {list(results)})')
    print(f'sketch/look anchored to method: {method}')

    # mean board pose in world (via the PARK solution), then per-shot camera
    # position in the body frame: X_i = T_w2b_i^-1 . T_w2board . T_t2c_i^-1
    boards_t, boards_q = [], []
    for i in range(n):
        R_wc = R_g2b[i] @ R_x
        t_wc = R_g2b[i] @ t_x + t_g2b[i]
        boards_t.append(R_wc @ t_t2c[i] + t_wc)
        boards_q.append(R_to_quat(R_wc @ R_t2c[i]))
    t_bw = np.mean(boards_t, axis=0)
    q = np.array(boards_q)
    q = np.where((q @ q[0])[:, None] < 0, -q, q).mean(0)
    R_bw = quat_to_R(q / np.linalg.norm(q))

    cloud = []
    for i in range(n):
        R_ct = R_t2c[i].T                      # camera pose in board frame
        t_ct = -R_ct @ t_t2c[i]
        t_wc_i = R_bw @ t_ct + t_bw            # camera in world (this shot)
        cloud.append(R_g2b[i].T @ (t_wc_i - t_g2b[i]))   # -> body frame
    cloud = np.array(cloud) * 1000             # mm

    spread = np.linalg.norm(cloud - cloud.mean(0), axis=1)
    print(f'\nper-shot camera-position cloud: {n} points, '
          f'mean scatter {spread.mean():.1f} mm, max {spread.max():.1f} mm')

    fig = go.Figure()
    ax_len = max(30.0, 1.3 * float(np.abs(cloud).max()))
    for k, name in enumerate('xyz'):           # body axes triad
        v = np.zeros(3)
        v[k] = ax_len
        fig.add_trace(go.Scatter3d(
            x=[0, v[0]], y=[0, v[1]], z=[0, v[2]], mode='lines+text',
            line=dict(color=AXIS_COLORS[name], width=5),
            text=['', f'body {name}'], textfont=dict(color=AXIS_COLORS[name]),
            name=f'body {name} axis', hoverinfo='skip', showlegend=False))
    fig.add_trace(go.Scatter3d(
        x=[0], y=[0], z=[0], mode='markers',
        marker=dict(size=7, color='#55524c', symbol='x'),
        name='body origin (streamed pose)',
        hovertemplate='phone rigid-body origin<extra></extra>'))
    fig.add_trace(go.Scatter3d(
        x=cloud[:, 0], y=cloud[:, 1], z=cloud[:, 2], mode='markers',
        marker=dict(size=5, color='#8a8781', opacity=0.65),
        name=f'per-shot estimates (n={n})', customdata=used,
        hovertemplate='<b>%{customdata}</b><br>'
                      'x=%{x:.1f} y=%{y:.1f} z=%{z:.1f} mm<extra></extra>'))
    for k, (name, (R, t)) in enumerate(results.items()):
        t = t * 1000
        fig.add_trace(go.Scatter3d(
            x=[t[0]], y=[t[1]], z=[t[2]], mode='markers',
            marker=dict(size=8, color=CAT[k % len(CAT)], symbol='diamond'),
            name=f'{name}  ({t[0]:.0f},{t[1]:.0f},{t[2]:.0f}) mm',
            hovertemplate=f'{name}<br>x=%{{x:.1f}} y=%{{y:.1f}} '
                          f'z=%{{z:.1f}} mm<extra></extra>'))
    # ETH-ASL dual-quaternion result (independent solver)
    if eth is not None:
        t = np.array(eth['camera_in_body']['translation_m']) * 1000
        fig.add_trace(go.Scatter3d(
            x=[t[0]], y=[t[1]], z=[t[2]], mode='markers',
            marker=dict(size=10, color='#e87ba4', symbol='diamond-open',
                        line=dict(color='#e87ba4', width=4)),
            name=f'ETH DQ-RANSAC  ({t[0]:.0f},{t[1]:.0f},{t[2]:.0f}) mm',
            hovertemplate=(f'ETH DQ-RANSAC '
                           f'({eth["n_inliers"]}/{eth["n_pairs"]} inliers)'
                           f'<br>x=%{{x:.1f}} y=%{{y:.1f}} z=%{{z:.1f}} mm'
                           f'<extra></extra>')))
        print(f'ETH result included from {eth_path}')
    else:
        print(f'NOTE: {eth_path} not found — run calibrate-handeye-eth to '
              'include the ETH solver in the plot')
    look = R_x @ np.array([0, 0, 1.0])         # camera optical axis (cam +z)
    tip = t_x * 1000 + look * ax_len * 0.6
    fig.add_trace(go.Scatter3d(
        x=[t_x[0] * 1000, tip[0]], y=[t_x[1] * 1000, tip[1]],
        z=[t_x[2] * 1000, tip[2]], mode='lines',
        line=dict(color='#55524c', width=6), name=f'look direction ({method})',
        hoverinfo='skip'))

    # rigid-body markers (from Motive, via scripts/dump_rigid_bodies.py)
    marker_pos = None
    if os.path.exists(args.markers):
        with open(args.markers) as f:
            bodies = yaml.safe_load(f)['bodies']
        body = bodies.get(args.rigid_body)
        if body:
            marker_pos = {m['name']: np.array(m['position_m'])
                          for m in body['markers']}
            pts = np.array(list(marker_pos.values())) * 1000
            names = list(marker_pos.keys())
            fig.add_trace(go.Scatter3d(
                x=pts[:, 0], y=pts[:, 1], z=pts[:, 2], mode='markers+text',
                marker=dict(size=9, color='#eb6834', opacity=0.9),
                text=[n.replace('marker_', 'm') for n in names],
                textposition='top center', textfont=dict(size=11,
                                                         color='#8a5a2b'),
                name=f'{args.rigid_body} markers (n={len(pts)})',
                customdata=names,
                hovertemplate='<b>%{customdata}</b><br>x=%{x:.1f} '
                              'y=%{y:.1f} z=%{z:.1f} mm<extra></extra>'))
            print(f'markers: {len(pts)} from {args.markers}')
        else:
            print(f'NOTE: body {args.rigid_body!r} not in {args.markers}')
    else:
        print(f'NOTE: {args.markers} not found — run `pixi run dump-bodies` '
              'in the lab to include the marker constellation')

    # schematic iPhone outline — seated on the marker-2/3/4 plate plane when
    # the constellation is known; camera-frame fallback otherwise
    if marker_pos and all(k in marker_pos for k in PHONE_PLANE_MARKERS):
        (xs, ys, zs), uw, d_lens, side = phone_wireframe_on_plane(
            R_x, t_x, marker_pos)
        expect = side * (PHONE_T + CAM_BOX['bump'])
        print(f'solved lens vs marker-2/3/4 plate plane: {d_lens * 1000:+.1f} mm '
              f'(physically expect ≈ {expect:+.1f} mm on the side the camera '
              f'looks toward; wrong sign or large deviation = translation error)')
        sketch_name = 'iPhone 15 Pro (on m2/m3/m4 plane, approx.)'
    else:
        (xs, ys, zs), uw = phone_wireframe(R_x, t_x)
        sketch_name = 'iPhone 15 Pro (schematic, approx.)'
    fig.add_trace(go.Scatter3d(
        x=xs, y=ys, z=zs, mode='lines',
        line=dict(color='#8a8781', width=3),
        name=sketch_name, hoverinfo='skip'))
    fig.add_trace(go.Scatter3d(
        x=uw[0], y=uw[1], z=uw[2], mode='lines',
        line=dict(color='#2a78d6', width=5),
        name='ultra-wide lens (anchored at solved camera)',
        hoverinfo='skip'))

    fig.update_layout(
        title=f'Camera in the phone rigid-body frame — {args.session}',
        scene=dict(xaxis_title='x (mm)', yaxis_title='y (mm)',
                   zaxis_title='z (mm)', aspectmode='data'),
        template='plotly_white', height=750,
        legend=dict(orientation='h', yanchor='bottom', y=1.02))

    out = args.out or (f'calibration/'
                       f'{os.path.basename(args.session.rstrip("/"))}/'
                       f'hand_eye_view.html')
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    fig.write_html(out, include_plotlyjs=True)
    print(f'wrote {out}')
    if not args.no_open:
        webbrowser.open('file://' + os.path.abspath(out))


if __name__ == '__main__':
    main()
