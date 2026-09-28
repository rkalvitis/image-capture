#!/usr/bin/env python3
"""Audit COLMAP camera models against a reference model + one overlay HTML.

Reference = the model you trust (e.g. colmap_models/perimage_362/0, the free
per-image SfM). Every TEST model (mocap poses, corrected mocap, a guided-BA
result, ...) is compared with it:

  * sim(3) Umeyama alignment of the REFERENCE onto the test model's frame (a
    metric / insect-frame test model keeps its frame), then position and
    orientation residuals, plus the constant camera-frame rotation that best
    explains the orientation residual (= a hand-eye-like error)
  * per-ring circle fits: radius, ring normal, planarity (ring = camera_poses.csv)
  * PIXEL test: the reference 3D points, aligned into the test frame, projected
    with the TEST poses + TEST intrinsics onto the reference's own 2D
    observations -> reprojection error of the test poses in px. This is what
    COLMAP's point_triangulator / bundle_adjuster see when given those poses.

The HTML is the same offline plotly view as plot_colmap_model.py (drag =
rotate, scroll = zoom, double-click = reset), with the reference cameras as
open diamonds + its cloud, the test cameras as dots (colored by pose/ring),
red residual lines test -> reference, and one button per test model.

Usage:
  pixi run python scripts/compare_colmap_models.py \\
      --ref datasets/aug31_20deg_9vertical/colmap_models/perimage_362/0 \\
      mocap=datasets/aug31_20deg_9vertical/gs_upload/sparse/0 \\
      exp6=~/Downloads/exp8_viz/exp6 exp8=~/Downloads/exp8_viz/exp8 \\
      -o datasets/aug31_20deg_9vertical/colmap_models/compare_vs_perimage_362.html

Test models must share one frame (all metric insect-frame here); the
reference is aligned onto the FIRST test model and drawn in that frame.
"""

import argparse
import csv
import os
import webbrowser

import numpy as np

IMG_EXT = ('.jpg', '.jpeg', '.png')


# ---------------------------------------------------------------- COLMAP IO
def qmat(w, x, y, z):
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def read_model(mdir, with_obs=False, with_points=False):
    """-> dict: images {name: (R_cw, t_cw, cam_id, obs[N,3] or None)},
    cameras {id: (model, params)}, points {id: xyz}, rgb {id: rgb}"""
    cams = {}
    for l in open(os.path.join(mdir, 'cameras.txt')):
        t = l.split()
        if not t or t[0].startswith('#'):
            continue
        cams[int(t[0])] = (t[1], np.array([float(v) for v in t[4:]]))
    images = {}
    with open(os.path.join(mdir, 'images.txt')) as f:
        lines = f.readlines()
    k = 0
    while k < len(lines):
        t = lines[k].split()
        if len(t) >= 10 and not t[0].startswith('#') \
                and t[9].lower().endswith(IMG_EXT):
            obs = None
            if with_obs and k + 1 < len(lines):
                o = lines[k + 1].split()
                if len(o) % 3 == 0 and o:
                    a = np.array(o, dtype=float).reshape(-1, 3)
                    obs = a[a[:, 2] >= 0]
            images[t[9]] = (qmat(*map(float, t[1:5])),
                            np.array(list(map(float, t[5:8]))), int(t[8]), obs)
            k += 2
        else:
            k += 1
    pts, rgb = {}, {}
    if with_points:
        for l in open(os.path.join(mdir, 'points3D.txt')):
            if l.startswith('#') or not l.strip():
                continue
            t = l.split()
            pts[int(t[0])] = np.array([float(t[1]), float(t[2]), float(t[3])])
            rgb[int(t[0])] = (int(t[4]), int(t[5]), int(t[6]))
    return {'images': images, 'cameras': cams, 'points': pts, 'rgb': rgb}


def centres(m, names):
    return np.array([-m['images'][n][0].T @ m['images'][n][1] for n in names])


def rot_wc(m, names):
    return np.array([m['images'][n][0].T for n in names])


# ---------------------------------------------------------------- geometry
def umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) * len(src) / (xs ** 2).sum()
    return s, R, mu_d - s * R @ mu_s


def ang(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def chordal_mean(Rs):
    U, _, Vt = np.linalg.svd(sum(Rs))
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def fit_circle(P):
    """PCA plane + algebraic circle. -> centre, unit normal, radius,
    rms out-of-plane, rms radial."""
    c0 = P.mean(0)
    _, _, Vt = np.linalg.svd(P - c0)
    n = Vt[2] if Vt[2, 2] >= 0 else -Vt[2]
    xy = (P - c0) @ np.stack([Vt[0], Vt[1]], 1)
    A = np.column_stack([xy[:, 0], xy[:, 1], np.ones(len(xy))])
    sol, *_ = np.linalg.lstsq(A, -(xy ** 2).sum(1), rcond=None)
    cx, cy = -sol[0] / 2, -sol[1] / 2
    r = np.sqrt(max(cx * cx + cy * cy - sol[2], 0))
    centre = c0 + cx * Vt[0] + cy * Vt[1]
    oop = np.sqrt((((P - c0) @ n) ** 2).mean())
    rad = np.sqrt(((np.linalg.norm(xy - [cx, cy], axis=1) - r) ** 2).mean())
    return centre, n, r, oop, rad


def pct(x, f='{:.2f}'):
    return (f'p50 {f} p90 {f} max {f}').format(
        np.percentile(x, 50), np.percentile(x, 90), x.max())


def project(cam, X):
    """PINHOLE / SIMPLE_PINHOLE (distortion of other models ignored)."""
    model, p = cam
    if model == 'SIMPLE_PINHOLE' or model == 'SIMPLE_RADIAL' or model == 'RADIAL':
        fx = fy = p[0]
        cx, cy = p[1], p[2]
    else:
        fx, fy, cx, cy = p[:4]
    ok = X[:, 2] > 1e-9
    u = fx * X[:, 0] / np.where(ok, X[:, 2], 1) + cx
    v = fy * X[:, 1] / np.where(ok, X[:, 2], 1) + cy
    return np.stack([u, v], 1), ok


# ---------------------------------------------------------------- audit
def audit(ref, test, names, ring_of, unit, ufac, label):
    Cr, Ct = centres(ref, names), centres(test, names)
    s, Ra, ta = umeyama(Cr, Ct)
    Cra = s * (Ra @ Cr.T).T + ta
    Rra = np.array([Ra @ R for R in rot_wc(ref, names)])
    Rt = rot_wc(test, names)
    dpos = np.linalg.norm(Cra - Ct, axis=1) * ufac
    dR = [Rt[i].T @ Rra[i] for i in range(len(names))]
    dRc = chordal_mean(dR)
    rot = np.array([ang(R) for R in dR])
    rot_c = np.array([ang(dRc.T @ R) for R in dR])
    print(f'\n=== {label}: {len(names)} common cameras; reference sim3 scale {s:.5f} ===')
    print(f'position residual [{unit}]      {pct(dpos)}')
    print(f'orientation residual [deg]     {pct(rot)}')
    print(f'  after best constant camera-frame rotation ({ang(dRc):.2f} deg): {pct(rot_c)}')

    rings = sorted({ring_of.get(n, 0) for n in names})
    if len(rings) > 1:
        print('ring  n   pos p50   rot p50 | radius test/ref  normal angle  out-of-plane rms test/ref  centre offset')
        rg = np.array([ring_of.get(n, 0) for n in names])
        for g in rings:
            i = rg == g
            if i.sum() < 5:
                continue
            ct, nt, rt_, ot, _ = fit_circle(Ct[i])
            cr, nr, rr_, orr, _ = fit_circle(Cra[i])
            print(f'{g:3d} {i.sum():3d}   {np.median(dpos[i]):6.2f}   {np.median(rot[i]):5.2f} | '
                  f'{rt_*ufac:6.1f}/{rr_*ufac:6.1f}      {np.degrees(np.arccos(np.clip(abs(nt@nr),-1,1))):5.2f}       '
                  f'{ot*ufac:5.2f}/{orr*ufac:5.2f}                 {np.linalg.norm(ct-cr)*ufac:5.1f}')

    # pixel test
    if ref['points'] and any(ref['images'][n][3] is not None for n in names):
        errs, offs = [], []
        for n in names:
            obs = ref['images'][n][3]
            if obs is None or not len(obs):
                continue
            X = np.array([ref['points'][int(p)] for p in obs[:, 2] if int(p) in ref['points']])
            uv = np.array([obs[k, :2] for k in range(len(obs)) if int(obs[k, 2]) in ref['points']])
            if not len(X):
                continue
            Xa = s * (Ra @ X.T).T + ta
            R, t, cid, _ = test['images'][n]
            Xc = (R @ Xa.T).T + t
            p, ok = project(test['cameras'][cid], Xc)
            if ok.sum() < 5:
                errs.append(np.inf)
                continue
            e = p[ok] - uv[ok]
            errs.append(np.linalg.norm(e, axis=1).mean())
            offs.append(np.linalg.norm(e.mean(0)))
        errs = np.array(errs)
        fin = np.isfinite(errs)
        print(f'PIXEL test (reference structure through test poses+intrinsics): '
              f'mean error per image [px] {pct(errs[fin], "{:.1f}")}'
              + (f'; {(~fin).sum()} images with points behind the camera' if (~fin).any() else ''))
    return s, Ra, ta, Cra, Rra, Ct, Rt, dpos


# ---------------------------------------------------------------- html
def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument('--ref', required=True, help='reference model dir (TXT)')
    ap.add_argument('tests', nargs='+', help='NAME=model_dir (TXT)')
    ap.add_argument('-o', '--out', default=None, help='output HTML (default: next to the first test model)')
    ap.add_argument('--csv', default='datasets/aug31_20deg_9vertical/camera_poses.csv')
    ap.add_argument('--points', type=int, default=40000)
    ap.add_argument('--unit', default='mm', help='label for residuals')
    ap.add_argument('--unit-factor', type=float, default=1000.0,
                    help='multiply test-frame lengths by this for the tables (m -> mm: 1000)')
    ap.add_argument('--force', action='store_true', help='allow overwriting an existing output HTML')
    ap.add_argument('--no-open', action='store_true')
    args = ap.parse_args()
    out = args.out or os.path.join(
        os.path.dirname(os.path.expanduser(args.tests[0].split('=', 1)[1])), 'compare_view.html')
    if os.path.exists(out) and not args.force:
        raise SystemExit(f'refusing to overwrite {out} (choose another -o or pass --force)')

    ref = read_model(os.path.expanduser(args.ref), with_obs=True, with_points=True)
    tests = []
    for spec in args.tests:
        name, path = spec.split('=', 1)
        tests.append((name, read_model(os.path.expanduser(path))))
    ring_of = {}
    if os.path.exists(args.csv):
        for r in csv.DictReader(open(args.csv)):
            ring_of[r['image'] + '.jpg'] = int(r['ring'])
    print(f'reference {args.ref}: {len(ref["images"])} images, {len(ref["points"])} points')

    import plotly.graph_objects as go
    import plotly.express as px
    palette = px.colors.sample_colorscale('viridis', np.linspace(0, 1, 10).tolist())
    fig = go.Figure()
    groups = []          # trace indices per test model
    frame_s = None
    for ti, (label, test) in enumerate(tests):
        names = sorted(set(ref['images']) & set(test['images']))
        if len(names) < 3:
            print(f'{label}: only {len(names)} common images, skipped')
            continue
        s, Ra, ta, Cra, Rra, Ct, Rt, dpos = audit(ref, test, names, ring_of, args.unit,
                                                  args.unit_factor, label)
        if frame_s is None:      # cloud drawn once, in the first test model's frame
            frame_s = (s, Ra, ta)
            P = np.array(list(ref['points'].values()))
            rgb = np.array(list(ref['rgb'].values()))
            Pa = s * (Ra @ P.T).T + ta
            med = np.median(Pa, axis=0)
            keep = np.linalg.norm(Pa - med, axis=1) < 6 * np.linalg.norm(Pa.std(axis=0))
            Pa, rgb = Pa[keep], rgb[keep]
            if len(Pa) > args.points:
                sel = np.random.default_rng(0).choice(len(Pa), args.points, replace=False)
                Pa, rgb = Pa[sel], rgb[sel]
            fig.add_trace(go.Scatter3d(
                x=Pa[:, 0], y=Pa[:, 1], z=Pa[:, 2], mode='markers',
                marker=dict(size=1.6, color=[f'rgb({r},{g},{b})' for r, g, b in rgb]),
                name=f'reference cloud ({len(Pa)})', hoverinfo='skip'))
            alen = float(np.median(np.linalg.norm(Ct - med, axis=1))) * 0.25
        idx0 = len(fig.data)
        At = np.array([R @ [0, 0, 1] for R in Rt])
        Ar = np.array([R @ [0, 0, 1] for R in Rra])
        rings = sorted({ring_of.get(n, 0) for n in names})
        for g in rings:
            ii = [i for i, n in enumerate(names) if ring_of.get(n, 0) == g]
            col = palette[rings.index(g) % 10] if len(rings) > 1 else '#d95f02'
            lab = f'pose {g-1} ({len(ii)})' if g else f'cameras ({len(ii)})'
            fig.add_trace(go.Scatter3d(
                x=Ct[ii, 0], y=Ct[ii, 1], z=Ct[ii, 2], mode='markers',
                marker=dict(size=5, color=col), name=f'{label} {lab}',
                text=[f'{names[i]}<br>residual {dpos[i]:.1f} {args.unit}' for i in ii],
                hovertemplate='%{text}<extra>' + label + '</extra>', legendgroup=label))
            fig.add_trace(go.Scatter3d(
                x=Cra[ii, 0], y=Cra[ii, 1], z=Cra[ii, 2], mode='markers',
                marker=dict(size=5, color=col, symbol='diamond-open'),
                name=f'reference {lab}', text=[names[i] for i in ii],
                hovertemplate='%{text}<extra>reference</extra>', legendgroup='ref'))
            lx, ly, lz = [], [], []
            for i in ii:
                for C, A in ((Ct, At), (Cra, Ar)):
                    e = C[i] + A[i] * alen
                    lx += [C[i, 0], e[0], None]; ly += [C[i, 1], e[1], None]; lz += [C[i, 2], e[2], None]
            fig.add_trace(go.Scatter3d(x=lx, y=ly, z=lz, mode='lines',
                                       line=dict(color=col, width=2), showlegend=False,
                                       hoverinfo='skip', legendgroup=label))
        lx, ly, lz = [], [], []
        for i in range(len(names)):
            lx += [Ct[i, 0], Cra[i, 0], None]; ly += [Ct[i, 1], Cra[i, 1], None]; lz += [Ct[i, 2], Cra[i, 2], None]
        fig.add_trace(go.Scatter3d(x=lx, y=ly, z=lz, mode='lines',
                                   line=dict(color='red', width=3),
                                   name=f'{label} -> reference residual', hoverinfo='skip',
                                   legendgroup=label))
        groups.append((label, list(range(idx0, len(fig.data)))))

    n_tr = len(fig.data)
    buttons = []
    for k, (label, idxs) in enumerate(groups):
        vis = [i == 0 or i in idxs for i in range(n_tr)]
        buttons.append(dict(label=label, method='update', args=[{'visible': vis}]))
    for i in range(n_tr):
        fig.data[i].visible = (i == 0) or (groups and i in groups[0][1])
    fig.update_layout(
        title=f'reference {os.path.abspath(args.ref)}  (open diamonds + cloud)  vs  test models (dots)',
        scene=dict(aspectmode='data', xaxis_title='x', yaxis_title='y', zaxis_title='z'),
        legend=dict(itemsizing='constant'),
        updatemenus=[dict(type='buttons', direction='right', x=0.0, y=1.0, xanchor='left',
                          yanchor='bottom', buttons=buttons, showactive=True)],
        margin=dict(l=0, r=0, t=70, b=0))
    fig.write_html(out, include_plotlyjs=True)
    print('\nwrote', out)
    if not args.no_open:
        webbrowser.open('file://' + os.path.abspath(out))


if __name__ == '__main__':
    main()
