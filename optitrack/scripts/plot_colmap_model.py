#!/usr/bin/env python3
"""Interactive 3D view of a COLMAP TXT model (WebGL: drag to rotate).

Writes a self-contained local HTML next to the model and opens it in the
browser (fully offline, no upload). Shows the sparse point cloud with its
real RGB and every camera as a dot with a short line along its optical
axis, colored by pose (via camera_poses.csv) when available.

Controls in the browser: drag = rotate, scroll = zoom, right-drag/shift-
drag = pan, double-click = reset. Click legend entries to hide/show poses.

Usage:
  pixi run python scripts/plot_colmap_model.py <model_dir> [options]

  --points N     max points to draw (default 80000)
  --axis-len L   optical-axis line length in model units (default auto)
  --csv PATH     camera_poses.csv for ring colors
  --no-open      just write the HTML, don't open the browser

Examples:
  pixi run python scripts/plot_colmap_model.py \\
      datasets/aug31_20deg_9vertical/colmap_models/free_181/0
  pixi run python scripts/plot_colmap_model.py \\
      datasets/aug31_20deg_9vertical/colmap_models/perimage_362/0
"""

import argparse
import csv
import os
import webbrowser

import numpy as np


def qmat(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_model(mdir):
    cams = []
    for l in open(os.path.join(mdir, 'images.txt')):
        t = l.split()
        if l.startswith('#') or len(t) < 10 or \
                not t[-1].lower().endswith(('.jpg', '.png', '.dng')):
            continue
        q = list(map(float, t[1:5]))
        tv = np.array(list(map(float, t[5:8])))
        R = qmat(*q)
        cams.append({'name': t[9], 'C': -R.T @ tv, 'axis': R.T @ [0, 0, 1]})
    pts, rgb = [], []
    for l in open(os.path.join(mdir, 'points3D.txt')):
        if l.startswith('#') or not l.strip():
            continue
        t = l.split()
        pts.append([float(t[1]), float(t[2]), float(t[3])])
        rgb.append([int(t[4]), int(t[5]), int(t[6])])
    return cams, np.array(pts), np.array(rgb)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('model_dir')
    ap.add_argument('--points', type=int, default=80000)
    ap.add_argument('--axis-len', type=float, default=None)
    ap.add_argument('--csv', default='datasets/aug31_20deg_9vertical/'
                                     'camera_poses.csv')
    ap.add_argument('--no-open', action='store_true')
    args = ap.parse_args()

    cams, pts, rgb = load_model(args.model_dir)
    print(f'{len(cams)} cameras, {len(pts)} points from {args.model_dir}')

    ring_of = {}
    if os.path.exists(args.csv):
        for r in csv.DictReader(open(args.csv)):
            ring_of[r['image']] = int(r['ring'])

    C = np.array([c['C'] for c in cams])
    A = np.array([c['axis'] for c in cams])
    if len(pts):
        med = np.median(pts, axis=0)
        scene = np.linalg.norm(pts.std(axis=0))
        keep = np.linalg.norm(pts - med, axis=1) < 6 * scene
        pts, rgb = pts[keep], rgb[keep]
        if len(pts) > args.points:
            sel = np.random.default_rng(0).choice(len(pts), args.points,
                                                  replace=False)
            pts, rgb = pts[sel], rgb[sel]
    else:
        # poses-only model (e.g. mocap): centre the view on the cameras
        med = C.mean(axis=0)
    alen = args.axis_len or float(np.median(
        np.linalg.norm(C - med, axis=1))) * 0.25

    import plotly.graph_objects as go
    fig = go.Figure()
    if len(pts):
        fig.add_trace(go.Scatter3d(
            x=pts[:, 0], y=pts[:, 1], z=pts[:, 2], mode='markers',
            marker=dict(size=1.6, color=[f'rgb({r},{g},{b})' for r, g, b in rgb]),
            name=f'points ({len(pts)})', hoverinfo='skip'))

    import plotly.express as px
    palette = px.colors.sample_colorscale(
        'viridis', np.linspace(0, 1, 10).tolist())
    groups = sorted({ring_of.get(c['name'].split('.')[0], 0) for c in cams})
    for gi, g in enumerate(groups):
        idx = [i for i, c in enumerate(cams)
               if ring_of.get(c['name'].split('.')[0], 0) == g]
        col = palette[groups.index(g) % 10] if len(groups) > 1 else '#d95f02'
        label = f'pose {g-1} ({len(idx)})' if g else f'cameras ({len(idx)})'
        fig.add_trace(go.Scatter3d(
            x=C[idx, 0], y=C[idx, 1], z=C[idx, 2], mode='markers',
            marker=dict(size=5, color=col), name=label,
            text=[cams[i]['name'] for i in idx],
            hovertemplate='%{text}<extra>' + label + '</extra>'))
        lx, ly, lz = [], [], []
        for i in idx:
            e = C[i] + A[i] * alen
            lx += [C[i, 0], e[0], None]
            ly += [C[i, 1], e[1], None]
            lz += [C[i, 2], e[2], None]
        fig.add_trace(go.Scatter3d(
            x=lx, y=ly, z=lz, mode='lines',
            line=dict(color=col, width=2), showlegend=False,
            hoverinfo='skip'))

    fig.update_layout(
        title=os.path.abspath(args.model_dir),
        scene=dict(aspectmode='data',
                   xaxis_title='x', yaxis_title='y', zaxis_title='z'),
        legend=dict(itemsizing='constant'),
        margin=dict(l=0, r=0, t=40, b=0))

    out = os.path.join(args.model_dir, 'model_view.html')
    fig.write_html(out, include_plotlyjs=True)
    print('wrote', out)
    if not args.no_open:
        webbrowser.open('file://' + os.path.abspath(out))


if __name__ == '__main__':
    main()
