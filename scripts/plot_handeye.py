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
from calibrate_handeye import (collect_pairs, solve_all_methods, R_to_quat,
                               quat_to_R)

# categorical palette (fixed order); gray for the per-shot cloud
CAT = ['#2a78d6', '#1baf7a', '#eda100', '#008300', '#4a3aa7']
AXIS_COLORS = {'x': '#e34948', 'y': '#1baf7a', 'z': '#2a78d6'}

# iPhone 15 Pro schematic (mm): body 146.6 x 70.6 x 8.25, drawn in the
# CAMERA frame (x = image right ~ phone long axis, y = image down,
# z = viewing direction out of the back glass) and anchored to the solved
# lens position. Where the lens sits on the body is approximate — this is
# a sketch for orientation, not a measurement.
PHONE_L, PHONE_W, PHONE_T = 146.6, 70.6, 8.25
LENS_FROM_TOP, LENS_FROM_EDGE = 20.0, 13.0


def phone_wireframe(R_x, t_x):
    """12 box edges of the schematic phone, transformed into the body frame (mm)."""
    x0, x1 = -LENS_FROM_TOP, PHONE_L - LENS_FROM_TOP
    y0, y1 = -LENS_FROM_EDGE, PHONE_W - LENS_FROM_EDGE
    z0, z1 = -PHONE_T, 0.0                      # back glass through the lens
    corners_cam = np.array([[x, y, z] for x in (x0, x1)
                            for y in (y0, y1) for z in (z0, z1)]) / 1000.0
    corners = (R_x @ corners_cam.T).T + t_x     # -> body frame (m)
    corners *= 1000                             # mm
    edges = [(0, 1), (2, 3), (4, 5), (6, 7),    # along z
             (0, 2), (1, 3), (4, 6), (5, 7),    # along y
             (0, 4), (1, 5), (2, 6), (3, 7)]    # along x
    xs, ys, zs = [], [], []
    for a, b in edges:
        xs += [corners[a, 0], corners[b, 0], None]
        ys += [corners[a, 1], corners[b, 1], None]
        zs += [corners[a, 2], corners[b, 2], None]
    return xs, ys, zs


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
    ap.add_argument('--out', default='calibration/hand_eye_view.html')
    ap.add_argument('--no-open', action='store_true',
                    help="don't open the result in the browser")
    args = ap.parse_args()

    pairs = collect_pairs(args.session, args.intrinsics, args.rigid_body,
                          args.clock_shows, args.clock_offset, args.min_corners)
    R_g2b, t_g2b = pairs['R_g2b'], pairs['t_g2b']
    R_t2c, t_t2c = pairs['R_t2c'], pairs['t_t2c']
    used = pairs['used']
    n = len(used)
    if n < 3:
        raise SystemExit(f'only {n} usable photos')

    results = solve_all_methods(R_g2b, t_g2b, R_t2c, t_t2c)
    R_x, t_x = results['PARK']

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
    look = R_x @ np.array([0, 0, 1.0])         # camera optical axis (cam +z)
    tip = t_x * 1000 + look * ax_len * 0.6
    fig.add_trace(go.Scatter3d(
        x=[t_x[0] * 1000, tip[0]], y=[t_x[1] * 1000, tip[1]],
        z=[t_x[2] * 1000, tip[2]], mode='lines',
        line=dict(color='#55524c', width=6), name='look direction (PARK)',
        hoverinfo='skip'))

    # rigid-body markers (from Motive, via scripts/dump_rigid_bodies.py)
    if os.path.exists(args.markers):
        with open(args.markers) as f:
            bodies = yaml.safe_load(f)['bodies']
        body = bodies.get(args.rigid_body)
        if body:
            pts = np.array([m['position_m'] for m in body['markers']]) * 1000
            names = [m['name'] for m in body['markers']]
            fig.add_trace(go.Scatter3d(
                x=pts[:, 0], y=pts[:, 1], z=pts[:, 2], mode='markers',
                marker=dict(size=9, color='#eb6834', opacity=0.9),
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

    # schematic iPhone outline anchored to the solved lens pose
    xs, ys, zs = phone_wireframe(R_x, t_x)
    fig.add_trace(go.Scatter3d(
        x=xs, y=ys, z=zs, mode='lines',
        line=dict(color='#8a8781', width=3),
        name='iPhone 15 Pro (schematic, approx.)', hoverinfo='skip'))

    fig.update_layout(
        title=f'Camera in the phone rigid-body frame — {args.session}',
        scene=dict(xaxis_title='x (mm)', yaxis_title='y (mm)',
                   zaxis_title='z (mm)', aspectmode='data'),
        template='plotly_white', height=750,
        legend=dict(orientation='h', yanchor='bottom', y=1.02))

    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    fig.write_html(args.out, include_plotlyjs=True)
    print(f'wrote {args.out}')
    if not args.no_open:
        webbrowser.open('file://' + os.path.abspath(args.out))


if __name__ == '__main__':
    main()
