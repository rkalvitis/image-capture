#!/usr/bin/env python3
"""Hand-eye (phone body -> camera) refit from a free SfM model + the mocap poses,
and a 3D view of the CURRENT vs REFITTED hand-eye in the phone body frame.

Idea: the free SfM of a capture session gives hundreds of accurate camera poses
T_sfm<-cam(k) (up to one sim3); the mocap gives the phone BODY pose in the
insect frame for the same photos, T_insect<-body(k). They are linked by ONE
unknown hand-eye X = T_body<-cam and ONE unknown sim3 S (insect <-> SfM):

    T_sfm<-cam(k) = S @ T_insect<-body(k) @ X          (robot-world hand-eye, AX = ZB)

Both are solved jointly by least squares over all photos. Compared with a
board-based hand-eye (19 pairs, board distance band) this uses 300+ poses at
the real working distance and lens/focus contract. Optionally fit one X per
ring GROUP: if the groups disagree, the phone moved relative to the markers
between them (the mount is not rigid) and no single hand-eye can be right.

Caveat: a free-focal SfM leaves each camera's position along its own optical
axis slightly ambiguous (focal <-> distance), so the component of the
translation correction along the optical axis (camera z) is the least
certain part of the refit.

The body poses are recovered from camera_poses.csv (which already contains
the current hand-eye) by removing calibrations/rig_transforms.yaml's
T_body_from_cam, so the script needs nothing but the dataset folder.

Usage:
  PYTHONPATH=scripts pixi run python scripts/calibrate_handeye_sfm.py \\
      [datasets/aug31_20deg_9vertical] [--sfm colmap_models/perimage_362/0] \\
      [--split 6] [--exclude-rings 1] [--w-rot 2] [-o handeye_sfm.html] [--yaml out.yaml]
"""

import argparse
import csv
import os
import webbrowser
from datetime import date

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

DEFAULT_DS = 'datasets/aug31_20deg_9vertical'
DEFAULT_SFM = 'colmap_models/perimage_362/0'
IMG_W, IMG_H = 4032, 3024


def read_colmap_images(path):
    """images.txt -> {name: (R_cw, t_cw)} (cam_from_world)."""
    out = {}
    for line in open(path):
        t = line.split()
        if len(t) < 10 or t[0].startswith('#') or not t[9].lower().endswith(('.jpg', '.png', '.jpeg')):
            continue
        q = np.array(list(map(float, t[1:5])))
        out[t[9]] = (Rot.from_quat([q[1], q[2], q[3], q[0]]).as_matrix(),
                     np.array(list(map(float, t[5:8]))))
    return out


def umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    U, D, Vt = np.linalg.svd(xd.T @ xs / len(src))
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) * len(src) / (xs ** 2).sum()
    return s, R, mu_d - s * R @ mu_s


def ang(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def pct(x, f='{:.2f}'):
    return ('p50 ' + f + '  p90 ' + f + '  max ' + f).format(
        np.percentile(x, 50), np.percentile(x, 90), x.max())


def load_stl(path, grid_mm=2.5):
    raw = open(path, 'rb').read()
    n = int.from_bytes(raw[80:84], 'little')
    rec = np.frombuffer(raw[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    tris = rec[:, 12:48].copy().view('<f4').reshape(n * 3, 3).astype(np.float64)
    keys = np.round(tris / grid_mm).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    verts = np.zeros((len(uniq), 3))
    counts = np.bincount(inv, minlength=len(uniq)).astype(float)
    for k in range(3):
        verts[:, k] = np.bincount(inv, weights=tris[:, k], minlength=len(uniq)) / counts
    faces = inv.reshape(n, 3)
    good = ((faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2]))
    return verts * 0.001, np.unique(np.sort(faces[good], axis=1), axis=0)


def fit(Cb, Rb, Cs, Rs, groups, R_bc, p_bc, w_rot):
    """Joint sim3 (SfM -> insect) + per-group hand-eye X_g = T_bc @ [dR_g | dt_g].
    Cb, Rb: body centres/rotations in the insect frame; Cs, Rs: SfM camera
    centres/rotations (SfM frame).  Returns dict per group + residuals."""
    N, G = len(Cb), groups.max() + 1
    # initial sim3 from the CURRENT hand-eye camera centres
    C_cur = Cb + np.einsum('kij,j->ki', Rb, p_bc)
    s0, Ra0, ta0 = umeyama(Cs, C_cur)
    Rc_cur = np.einsum('kij,jl->kil', Rb, R_bc)            # current camera rotations

    def unpack(p):
        return p[0], Rot.from_rotvec(p[1:4]).as_matrix(), p[4:7], p[7:].reshape(G, 6)

    def predict(p):
        s, Ra, ta, cr = unpack(p)
        Cs_i = s * (Ra @ Cs.T).T + ta
        Rs_i = np.einsum('ij,kjl->kil', Ra, Rs)
        Cp = np.empty_like(Cb); Rp = np.empty_like(Rb)
        for g in range(G):
            i = groups == g
            dR = Rot.from_rotvec(cr[g, :3]).as_matrix()
            Cp[i] = C_cur[i] + np.einsum('kij,j->ki', Rc_cur[i], cr[g, 3:])
            Rp[i] = np.einsum('kij,jl->kil', Rc_cur[i], dR)
        return Cs_i, Rs_i, Cp, Rp, cr

    def resid(p):
        Cs_i, Rs_i, Cp, Rp, _ = predict(p)
        rp = (Cs_i - Cp).ravel() * 1000.0
        rr = Rot.from_matrix(np.einsum('kji,kjl->kil', Rp, Rs_i)).as_rotvec()
        return np.concatenate([rp, np.degrees(rr).ravel() * w_rot])

    p0 = np.concatenate([[s0], Rot.from_matrix(Ra0).as_rotvec(), ta0, np.zeros(6 * G)])
    sol = least_squares(resid, p0, method='trf', x_scale='jac', max_nfev=5000)
    Cs_i, Rs_i, Cp, Rp, cr = predict(sol.x)
    dpos = np.linalg.norm(Cs_i - Cp, axis=1) * 1000.0
    drot = np.array([ang(Rp[k].T @ Rs_i[k]) for k in range(N)])
    # residual BEFORE the correction (current hand-eye, same sim3 refit)
    out = {'s': sol.x[0], 'dpos': dpos, 'drot': drot, 'groups': []}
    for g in range(G):
        dR = Rot.from_rotvec(cr[g, :3]).as_matrix(); dt = cr[g, 3:]
        out['groups'].append({'dR': dR, 'dt': dt, 'R_bc': R_bc @ dR, 'p_bc': p_bc + R_bc @ dt})
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument('dataset', nargs='?', default=DEFAULT_DS)
    ap.add_argument('--sfm', default=None, help=f'SfM model dir (TXT); default <dataset>/{DEFAULT_SFM}')
    ap.add_argument('--split', type=int, default=6,
                    help='also fit one hand-eye per group: rings <= SPLIT vs > SPLIT (0 = off)')
    ap.add_argument('--exclude-rings', type=int, nargs='*', default=[],
                    help='rings left out of the fits (e.g. 1 = the tiny top ring)')
    ap.add_argument('--w-rot', type=float, default=2.0, help='weight of 1 deg orientation residual in mm')
    ap.add_argument('--handeye', default=None,
                    help='start from an alternative hand-eye yaml (handeye_robot.yaml layout: camera_in_body '
                         'translation_m + quaternion_xyzw) instead of rig_transforms.yaml T_body_from_cam')
    ap.add_argument('--tag', default='sfm_refit',
                    help='name tag of the outputs: handeye_<tag>.html, calibrations/handeye_<tag>.yaml, '
                         'camera_poses_<tag>.csv (default sfm_refit)')
    ap.add_argument('-o', '--out', default=None, help='HTML view (default <dataset>/handeye_<tag>.html)')
    ap.add_argument('--write-poses', action='store_true',
                    help='also write calibrations/handeye_<tag>.yaml and camera_poses_<tag>.csv (the mocap '
                         'poses recomputed with the refitted hand-eye); the originals are never touched')
    ap.add_argument('--force', action='store_true', help='allow overwriting an existing output file')
    ap.add_argument('--no-open', action='store_true')
    args = ap.parse_args()
    ds = args.dataset.rstrip('/')

    def guard(path):
        if os.path.exists(path) and not args.force:
            raise SystemExit(f'refusing to overwrite {path} (choose another --tag/-o or pass --force)')
        return path
    sfm_dir = args.sfm or os.path.join(ds, DEFAULT_SFM)
    rt = yaml.safe_load(open(os.path.join(ds, 'calibrations', 'rig_transforms.yaml')))
    R_bc = np.array(rt['T_body_from_cam']['rotation_matrix'])
    p_bc = np.array(rt['T_body_from_cam']['translation_m'])
    handeye_src = os.path.join(ds, 'calibrations/rig_transforms.yaml') + ' (T_body_from_cam)'
    # camera_poses.csv was built with rig_transforms.yaml's hand-eye: that one is
    # removed to recover the body poses; --handeye then supplies the hand-eye to evaluate
    R_bc_csv, p_bc_csv = R_bc, p_bc
    if args.handeye:
        he = yaml.safe_load(open(args.handeye))
        cib = he.get('camera_in_body', he.get('T_body_from_cam', he))
        R_bc = Rot.from_quat(cib['quaternion_xyzw']).as_matrix()
        p_bc = np.array(cib['translation_m'])
        handeye_src = f'{args.handeye} (camera_in_body)'

    # --- poses --------------------------------------------------------------
    rows = list(csv.DictReader(open(os.path.join(ds, 'camera_poses.csv'))))
    sfm = read_colmap_images(os.path.join(sfm_dir, 'images.txt'))
    rows = [r for r in rows if r['image'] + '.jpg' in sfm]
    ring = np.array([int(r['ring']) for r in rows])
    C_ic = np.array([[float(r['cam_x']), float(r['cam_y']), float(r['cam_z'])] for r in rows])
    R_ic = np.array([Rot.from_quat(list(map(float, r['quat_xyzw'].split()))).as_matrix() for r in rows])
    # body pose = camera pose with the csv's hand-eye removed; then the hand-eye
    # under evaluation is applied again (identical to the csv unless --handeye)
    Rb = np.einsum('kij,lj->kil', R_ic, R_bc_csv)          # R_ib = R_ic R_bc^T
    Cb = C_ic - np.einsum('kij,j->ki', Rb, p_bc_csv)
    R_ic = np.einsum('kij,jl->kil', Rb, R_bc)
    C_ic = Cb + np.einsum('kij,j->ki', Rb, p_bc)
    Cs = np.array([-sfm[r['image'] + '.jpg'][0].T @ sfm[r['image'] + '.jpg'][1] for r in rows])
    Rs = np.array([sfm[r['image'] + '.jpg'][0].T for r in rows])
    keep = ~np.isin(ring, args.exclude_rings)
    print(f'{len(rows)} photos with SfM poses; fitting on {keep.sum()} '
          f'(excluded rings {args.exclude_rings or "none"}); hand-eye under test: {handeye_src}')

    def report(res, label, groups_desc):
        print(f'\n=== {label} ===')
        print(f'residual vs SfM: position [mm] {pct(res["dpos"][keep])}   orientation [deg] {pct(res["drot"][keep])}')
        for g, gr in enumerate(res['groups']):
            rv = np.degrees(Rot.from_matrix(gr['dR']).as_rotvec())
            print(f'  {groups_desc[g]}: correction in the CAMERA frame: rot {np.linalg.norm(rv):.2f} deg '
                  f'(rotvec {rv.round(2)}), t {np.round(gr["dt"]*1000, 1)} mm '
                  f'-> camera moves {np.linalg.norm(gr["dt"])*1000:.1f} mm, turns {ang(gr["dR"]):.2f} deg')
        print('  per ring pos p50 [mm]: ' + ' '.join(f'{np.median(res["dpos"][ring == g]):5.1f}' for g in sorted(set(ring))))
        print('  per ring rot p50 [deg]: ' + ' '.join(f'{np.median(res["drot"][ring == g]):5.1f}' for g in sorted(set(ring))))

    # baseline: sim3 only (current hand-eye)
    s0, Ra0, ta0 = umeyama(Cs[keep], C_ic[keep])
    d0 = np.linalg.norm(s0 * (Ra0 @ Cs.T).T + ta0 - C_ic, axis=1) * 1000
    r0 = np.array([ang(R_ic[k].T @ Ra0 @ Rs[k]) for k in range(len(rows))])
    print(f'\n=== current hand-eye (sim3 alignment only) ===\nresidual vs SfM: position [mm] {pct(d0[keep])}   orientation [deg] {pct(r0[keep])}')

    one = fit(Cb[keep], Rb[keep], Cs[keep], Rs[keep], np.zeros(keep.sum(), int), R_bc, p_bc, args.w_rot)
    # expand residuals back to all rows for the per-ring print
    full = {'dpos': np.full(len(rows), np.nan), 'drot': np.full(len(rows), np.nan), 'groups': one['groups']}
    full['dpos'][keep] = one['dpos']; full['drot'][keep] = one['drot']
    report(full, 'ONE hand-eye for the whole session', ['all rings'])
    variants = [('refit (all rings)', one['groups'][0])]

    if args.split:
        grp = np.where(ring[keep] <= args.split, 0, 1)
        two = fit(Cb[keep], Rb[keep], Cs[keep], Rs[keep], grp, R_bc, p_bc, args.w_rot)
        full2 = {'dpos': np.full(len(rows), np.nan), 'drot': np.full(len(rows), np.nan), 'groups': two['groups']}
        full2['dpos'][keep] = two['dpos']; full2['drot'][keep] = two['drot']
        report(full2, f'TWO hand-eyes: rings <= {args.split} / rings > {args.split} (mount rigidity check)',
               [f'rings <= {args.split}', f'rings > {args.split}'])
        gA, gB = two['groups']
        print(f'  the two groups differ by {np.linalg.norm(gA["p_bc"] - gB["p_bc"])*1000:.1f} mm and '
              f'{ang(gA["R_bc"].T @ gB["R_bc"]):.2f} deg -> if this is well above the residual noise, '
              f'the phone moved relative to the markers between the groups')
        variants += [(f'refit rings <= {args.split}', gA), (f'refit rings > {args.split}', gB)]

    # --- yaml-ready output ---------------------------------------------------
    g = one['groups'][0]
    q = Rot.from_matrix(g['R_bc']).as_quat()
    block = {
        'T_body_from_cam_sfm_refit': {
            'date': str(date.today()),
            'fitted_from': f'{sfm_dir} ({keep.sum()} photos) + camera_poses.csv body poses',
            'note': 'T_bc_new = T_bc_old @ [dR|dt]; the dt component along camera z is the least certain (SfM focal/distance ambiguity)',
            'translation_m': [round(float(v), 6) for v in g['p_bc']],
            'quaternion_xyzw': [round(float(v), 6) for v in q],
            'rotation_matrix': [[round(float(v), 8) for v in row] for row in g['R_bc']],
            'correction_camera_frame': {
                'rotvec_deg': [round(float(v), 3) for v in np.degrees(Rot.from_matrix(g['dR']).as_rotvec())],
                'translation_mm': [round(float(v), 2) for v in g['dt'] * 1000]},
            'residual_vs_sfm': {'pos_mm_p50': round(float(np.median(one['dpos'])), 2),
                                'rot_deg_p50': round(float(np.median(one['drot'])), 2)},
        }}
    print('\n' + yaml.safe_dump(block, sort_keys=False))
    if args.write_poses:
        ypath = guard(os.path.join(ds, 'calibrations', f'handeye_{args.tag}.yaml'))
        cpath = guard(os.path.join(ds, f'camera_poses_{args.tag}.csv'))
        yaml.safe_dump(block, open(ypath, 'w'), sort_keys=False)
        # mocap camera poses recomputed with the refitted hand-eye (same columns as camera_poses.csv)
        Rn = np.einsum('kij,jl->kil', Rb, g['R_bc']); Cn = Cb + np.einsum('kij,j->ki', Rb, g['p_bc'])
        with open(cpath, 'w', newline='') as fo:
            w = csv.DictWriter(fo, fieldnames=list(rows[0].keys()))
            w.writeheader()
            for k, r in enumerate(rows):
                r = dict(r)
                r['cam_x'], r['cam_y'], r['cam_z'] = (f'{v:.5f}' for v in Cn[k])
                r['quat_xyzw'] = ' '.join(f'{v:.6f}' for v in Rot.from_matrix(Rn[k]).as_quat())
                w.writerow(r)
        print(f'wrote {ypath}\nwrote {cpath} ({len(rows)} rows; ring/pose/sharpness columns copied from camera_poses.csv)')

    # --- 3D view in the body frame --------------------------------------------
    import plotly.graph_objects as go
    cal = os.path.join(ds, 'calibrations')
    fig = go.Figure()
    try:
        R_bm = np.array(rt['T_body_from_phone_model']['rotation_matrix'])
        t_bm = np.array(rt['T_body_from_phone_model']['translation_m'])
        pv, pf = load_stl(os.path.join(cal, rt['meshes']['phone_model']))
        pv = (R_bm @ pv.T).T + t_bm
        fig.add_trace(go.Mesh3d(x=pv[:, 0], y=pv[:, 1], z=pv[:, 2], i=pf[:, 0], j=pf[:, 1], k=pf[:, 2],
                                color='#8b897f', opacity=0.45, flatshading=True, hoverinfo='skip',
                                name='iPhone model (placed by the CURRENT hand-eye)', showlegend=True))
        ring_b = R_bm @ np.array(rt['ultrawide_ring_phone_model_m']) + t_bm
        fig.add_trace(go.Scatter3d(x=[ring_b[0]], y=[ring_b[1]], z=[ring_b[2]], mode='markers',
                                   marker=dict(size=6, color='#52514e', symbol='circle-open'),
                                   name='ultra-wide ring on the model', hoverinfo='name'))
        R_fb = np.array(rt['T_flange_from_body']['rotation_matrix']); t_fb = np.array(rt['T_flange_from_body']['translation_m'])
        bv, bf = load_stl(os.path.join(cal, rt['meshes']['base']))
        b2l = rt['base_mesh_to_link8']; a = np.radians(b2l['yaw_deg'])
        Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
        bv = bv @ Rz.T; bv[:, 2] += b2l['z_offset_m']; bv = (R_fb.T @ (bv - t_fb).T).T
        fig.add_trace(go.Mesh3d(x=bv[:, 0], y=bv[:, 1], z=bv[:, 2], i=bf[:, 0], j=bf[:, 1], k=bf[:, 2],
                                color='#7f9db9', opacity=0.35, flatshading=True, hoverinfo='skip',
                                name='mount base (STL chain)', showlegend=True))
    except (KeyError, FileNotFoundError) as e:
        print('meshes skipped:', e)
    M = np.array([rt['markers_in_body_m'][f'm{i}']['position'] for i in range(1, 6)])
    fig.add_trace(go.Scatter3d(x=M[:, 0], y=M[:, 1], z=M[:, 2], mode='markers+text',
                               marker=dict(size=7, color='#eda100'), text=[f'm{i}' for i in range(1, 6)],
                               textposition='top center', name='markers (phone body)',
                               hovertemplate='%{text}<extra></extra>'))
    for axis, col in zip(np.eye(3) * 0.03, ('#e34948', '#1baf7a', '#2a78d6')):
        fig.add_trace(go.Scatter3d(x=[0, axis[0]], y=[0, axis[1]], z=[0, axis[2]], mode='lines',
                                   line=dict(color=col, width=5), showlegend=False, hoverinfo='skip'))
    fig.add_trace(go.Scatter3d(x=[0], y=[0], z=[0], mode='markers', marker=dict(size=4, color='#0b0b0b'),
                               name='body origin (marker centroid) + xyz axes'))

    fx = fy = 1976.0
    def camera_traces(R, p, color, label, dash=None):
        c = p
        zax = R @ [0, 0, 1]
        fig.add_trace(go.Scatter3d(x=[c[0]], y=[c[1]], z=[c[2]], mode='markers',
                                   marker=dict(size=7, color=color, symbol='x'), name=label,
                                   hovertemplate=f'{label}<br>(%{{x:.4f}}, %{{y:.4f}}, %{{z:.4f}}) m<extra></extra>'))
        e = c + 0.06 * zax
        fig.add_trace(go.Scatter3d(x=[c[0], e[0]], y=[c[1], e[1]], z=[c[2], e[2]], mode='lines',
                                   line=dict(color=color, width=5, dash=dash or 'solid'), showlegend=False, hoverinfo='skip'))
        for k, col in zip(range(3), ('#e34948', '#1baf7a', '#2a78d6')):
            e = c + 0.015 * (R @ np.eye(3)[k])
            fig.add_trace(go.Scatter3d(x=[c[0], e[0]], y=[c[1], e[1]], z=[c[2], e[2]], mode='lines',
                                       line=dict(color=col, width=3), showlegend=False, hoverinfo='skip'))
        d = 0.03
        corners = np.array([[sx * IMG_W / 2 / fx * d, sy * IMG_H / 2 / fy * d, d]
                            for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
        cb = (R @ corners.T).T + c
        xs, ys, zs = [], [], []
        for k in range(4):
            for P in (c, cb[k], cb[(k + 1) % 4]):
                xs.append(P[0]); ys.append(P[1]); zs.append(P[2])
            xs.append(None); ys.append(None); zs.append(None)
        fig.add_trace(go.Scatter3d(x=xs, y=ys, z=zs, mode='lines', line=dict(color=color, width=2, dash=dash or 'solid'),
                                   showlegend=False, hoverinfo='skip'))

    camera_traces(R_bc, p_bc, '#e87ba4', 'camera: CURRENT hand-eye (rig_transforms.yaml)')
    cols = ('#2a78d6', '#1baf7a', '#eda100')
    for (label, gr), col in zip(variants, cols):
        camera_traces(gr['R_bc'], gr['p_bc'], col,
                      f'camera: {label}  (+{np.linalg.norm(gr["dt"])*1000:.1f} mm, {ang(gr["dR"]):.1f} deg)', dash='dash')
        fig.add_trace(go.Scatter3d(x=[p_bc[0], gr['p_bc'][0]], y=[p_bc[1], gr['p_bc'][1]], z=[p_bc[2], gr['p_bc'][2]],
                                   mode='lines', line=dict(color=col, width=4), showlegend=False, hoverinfo='skip'))
    fig.update_layout(
        title='Hand-eye in the phone BODY frame (m): current vs SfM-refitted camera; '
              'frustum depth 30 mm, optical axis 60 mm',
        scene=dict(aspectmode='data', xaxis_title='x (m)', yaxis_title='y (m)', zaxis_title='z (m)'),
        legend=dict(itemsizing='constant'), margin=dict(l=0, r=0, t=50, b=0))
    out = guard(args.out or os.path.join(ds, f'handeye_{args.tag}.html'))
    fig.write_html(out, include_plotlyjs=True)
    print('wrote', out)
    if not args.no_open:
        webbrowser.open('file://' + os.path.abspath(out))


if __name__ == '__main__':
    main()
