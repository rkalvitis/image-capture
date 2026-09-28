#!/usr/bin/env python3
"""COLMAP triangulation + pose refinement guided by the CSV-derived poses.

Builds a COLMAP workspace inside the dataset and runs the local colmap CLI:

  1. images/  = symlinks to masking/rgb (sensor-native JPEGs)
     masks/   = beetle masks rotated back to sensor orientation + dilated,
     so SIFT sees only the insect (the static background contradicts the
     beetle-frame poses on a turntable and MUST be masked out)
  2. sparse_prior/ = PINHOLE camera (solved focal) + per-image pose priors
     in the beetle frame: R_cam<-obj = R_cf @ R_flange^T @ R_block^T,
     C = lens position folded by +60 deg/block (same math as the 3D plot)
  3. feature_extractor -> exhaustive_matcher -> point_triangulator with the
     prior poses held fixed -> bundle_adjuster (intrinsics fixed, poses
     free) -> sim(3) Umeyama back onto the prior centres (BA has a free
     gauge; metric scale + beetle frame come from the priors)
  4. reports triangulation stats and the per-camera pose correction —
     which is the accuracy audit of the robot-pose chain

Output: <dataset>/colmap/{sparse_tri,sparse_ba,sparse_refined}/0 (text
models; sparse_refined is the one to use downstream) + report lines.

Usage:
  PYTHONPATH=scripts pixi run python scripts/colmap_guided.py \\
      datasets/phone_20260826_113325
"""

import argparse
import os
import shutil
import subprocess

import numpy as np
import yaml
from PIL import Image, ImageFilter

from link_robot_mocap import load_run_csv, mat_quat

WS_SUB = 'colmap'
WSIZE, HSIZE = 4032, 3024


def quat_wxyz(R):
    x, y, z, w = mat_quat(R)
    return w, x, y, z


def build_poses(ds):
    """-> {image_name: (R_world2cam, C_world)} in the beetle frame + focal."""
    reg = yaml.safe_load(open(os.path.join(ds, 'registration.yaml')))
    lens_fl = np.array(reg['lens_offset_in_flange_corrected_m'])
    fold = reg.get('fold_axis_robot', reg['turntable_axis_robot'])
    axis_pt = np.array(fold['point'])
    axis_dir = np.array(fold['dir'])
    axis_dir = axis_dir / np.linalg.norm(axis_dir)
    Rcf = np.array(reg['camera_axis_solution']['R_cam_from_flange'])
    f_px = float(reg['camera_axis_solution']['focal_px'])
    cx = float(reg['camera_axis_solution'].get('cx_px', WSIZE / 2))
    cy = float(reg['camera_axis_solution'].get('cy_px', HSIZE / 2))

    def axis_rot(deg):
        a = np.radians(deg)
        k = axis_dir
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * (K @ K)

    csvs = sorted(f for f in os.listdir(os.path.join(ds, 'phone_poses_robot'))
                  if f.startswith('manual_poses_2026'))
    images = sorted(i for i in os.listdir(os.path.join(ds, 'images'))
                    if i.startswith('IMG'))
    poses = {}
    for b, fn in enumerate(csvs):
        T_wp, _ = load_run_csv(os.path.join(ds, 'phone_poses_robot', fn))
        Rb = axis_rot(b * 60.0)
        for w in range(1, len(T_wp)):
            R_fl, p_fl = T_wp[w][:3, :3], T_wp[w][:3, 3]
            C = axis_pt + Rb @ (p_fl + R_fl @ lens_fl - axis_pt)
            R_w2c = Rcf @ R_fl.T @ Rb.T
            poses[images[b * (len(T_wp) - 1) + (w - 1)] + '.jpg'] = (R_w2c, C)
    return poses, f_px, (cx, cy)


def prepare_workspace(ds, poses, f_px, dilate_px):
    ws = os.path.join(ds, WS_SUB)
    img_dir = os.path.join(ws, 'images')
    mask_dir = os.path.join(ws, 'masks')
    prior_dir = os.path.join(ws, 'sparse_prior')
    for d in (img_dir, mask_dir, prior_dir):
        os.makedirs(d, exist_ok=True)

    for name in poses:
        stem = name[:-4]
        dst = os.path.join(img_dir, name)
        if not os.path.exists(dst):
            os.symlink(os.path.abspath(os.path.join(ds, 'masking/rgb', name)), dst)
        if os.path.exists(os.path.join(mask_dir, name + '.png')):
            continue   # masks are deterministic - never rebuild existing ones
        # mask: displayed orientation -> sensor orientation, then dilate
        m = Image.open(os.path.join(ds, 'masking/masks', stem + '.png')).convert('L')
        flag = Image.open(os.path.join(ds, 'masking/rgb', name)).getexif().get(274)
        if flag == 6:          # displayed = sensor rotated 90 CW -> undo (90 CCW)
            m = m.transpose(Image.ROTATE_90)
        elif flag == 3:
            m = m.transpose(Image.ROTATE_180)
        elif flag == 8:
            m = m.transpose(Image.ROTATE_270)
        if m.size != (WSIZE, HSIZE):
            raise SystemExit(f'mask {stem}: unexpected size {m.size} after unrotate')
        # fast dilation: box blur spreads any nonzero into the radius, then
        # threshold (MaxFilter on 12 MP is ~10 s/image; this is ~0.1 s)
        m = m.filter(ImageFilter.BoxBlur(dilate_px // 2))
        m = m.point(lambda v: 255 if v > 0 else 0)
        m.save(os.path.join(mask_dir, name + '.png'))

    return ws


def write_prior_model(ws, poses, f_px, db, pp=None):
    """Write the prior model AFTER feature extraction, using the image ids
    (and camera id) the database assigned — COLMAP 4.x requires the model
    ids to match the database exactly."""
    import sqlite3
    con = sqlite3.connect(db)
    rows = con.execute('SELECT image_id, name, camera_id FROM images').fetchall()
    con.close()
    prior_dir = os.path.join(ws, 'sparse_prior')
    cam_id = rows[0][2]
    cx, cy = pp if pp else (WSIZE / 2, HSIZE / 2)
    with open(os.path.join(prior_dir, 'cameras.txt'), 'w') as f:
        f.write(f'{cam_id} PINHOLE {WSIZE} {HSIZE} {f_px:.4f} {f_px:.4f} '
                f'{cx:.4f} {cy:.4f}\n')
    with open(os.path.join(prior_dir, 'images.txt'), 'w') as f:
        for image_id, name, cid in sorted(rows):
            R, C = poses[name]
            t = -R @ C
            qw, qx, qy, qz = quat_wxyz(R)
            f.write(f'{image_id} {qw:.9f} {qx:.9f} {qy:.9f} {qz:.9f} '
                    f'{t[0]:.9f} {t[1]:.9f} {t[2]:.9f} {cid} {name}\n\n')
    open(os.path.join(prior_dir, 'points3D.txt'), 'w').close()


def run(cmd):
    print('>>', ' '.join(cmd))
    subprocess.run(cmd, check=True)


def read_images_txt(path):
    out = {}
    expect_pose = True
    for line in open(path):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        if expect_pose:
            p = line.split()
            q = np.array([float(v) for v in p[1:5]])   # wxyz
            t = np.array([float(v) for v in p[5:8]])
            w, x, y, z = q / np.linalg.norm(q)
            R = np.array([
                [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
            out[p[9]] = (R, -R.T @ t)   # (R_w2c, centre)
        expect_pose = not expect_pose
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dataset')
    ap.add_argument('--dilate', type=int, default=31,
                    help='mask dilation kernel in px (default 31)')
    ap.add_argument('--skip-extract', action='store_true',
                    help='reuse existing database (features+matches)')
    ap.add_argument('--dense', action='store_true',
                    help='denser features: full-res SIFT, more keypoints, '
                         'lower peak threshold (use when track length ~2)')
    ap.add_argument('--aliked', action='store_true',
                    help='ALIKED learned features + learned brute-force '
                         'matching - best for shiny/low-texture subjects. '
                         'Needs a COLMAP build with ONNX support; the '
                         'Homebrew macOS build only registers SIFT and '
                         'rejects this at runtime (use the server CUDA '
                         'container for it)')
    args = ap.parse_args()
    ds = args.dataset.rstrip('/')

    poses, f_px, (cx, cy) = build_poses(ds)
    ws = prepare_workspace(ds, poses, f_px, args.dilate)
    db = os.path.join(ws, 'database.db')
    print(f'workspace {ws}: {len(poses)} images, f = {f_px:.0f} px')

    if not args.skip_extract:
        if os.path.exists(db):
            os.remove(db)
        run(['colmap', 'feature_extractor',
             '--database_path', db,
             '--image_path', os.path.join(ws, 'images'),
             '--ImageReader.mask_path', os.path.join(ws, 'masks'),
             '--ImageReader.camera_model', 'PINHOLE',
             '--ImageReader.camera_params',
             f'{f_px:.4f},{f_px:.4f},{cx:.4f},{cy:.4f}',
             '--ImageReader.single_camera', '1',
             '--FeatureExtraction.use_gpu', '0']
            # dense: full-res + more features, but NO affine shape / DSP and
            # few threads — that combination at 12 MP gets OOM-killed
            + (['--FeatureExtraction.type', 'ALIKED',
                '--AlikedExtraction.max_num_features', '8192'] if args.aliked else
               ['--FeatureExtraction.max_image_size', '4032',
                '--FeatureExtraction.num_threads', '4',
                '--SiftExtraction.max_num_features', '16384',
                '--SiftExtraction.peak_threshold', '0.004'] if args.dense else
               ['--SiftExtraction.estimate_affine_shape', '1',
                '--SiftExtraction.domain_size_pooling', '1']))
        run(['colmap', 'exhaustive_matcher',
             '--database_path', db,
             '--FeatureMatching.use_gpu', '0',
             '--FeatureMatching.guided_matching', '1']
            + (['--FeatureMatching.type', 'ALIKED_BRUTEFORCE'] if args.aliked else
               ['--SiftMatching.max_ratio', '0.85'] if args.dense else []))

    write_prior_model(ws, poses, f_px, db, (cx, cy))
    tri = os.path.join(ws, 'sparse_tri', '0')
    os.makedirs(tri, exist_ok=True)
    run(['colmap', 'point_triangulator',
         '--database_path', db,
         '--image_path', os.path.join(ws, 'images'),
         '--input_path', os.path.join(ws, 'sparse_prior'),
         '--output_path', tri,
         '--Mapper.tri_ignore_two_view_tracks', '0',
         '--Mapper.filter_max_reproj_error', '8'])

    ba = os.path.join(ws, 'sparse_ba', '0')
    os.makedirs(ba, exist_ok=True)
    run(['colmap', 'bundle_adjuster',
         '--input_path', tri,
         '--output_path', ba,
         '--BundleAdjustment.refine_focal_length', '0',
         '--BundleAdjustment.refine_principal_point', '0',
         '--BundleAdjustment.refine_extra_params', '0'])
    for d in (tri, ba):
        run(['colmap', 'model_converter', '--input_path', d,
             '--output_path', d, '--output_type', 'TXT'])

    # sim(3) align BA result back onto the prior centres (gauge fix)
    prior_C = {n: C for n, (R, C) in poses.items()}
    ba_poses = read_images_txt(os.path.join(ba, 'images.txt'))
    names = sorted(set(prior_C) & set(ba_poses))
    A = np.array([ba_poses[n][1] for n in names])       # BA centres
    B = np.array([prior_C[n] for n in names])           # prior centres
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, s_, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    Rs = Vt.T @ D @ U.T
    scale = np.trace(np.diag(s_) @ D) / ((A - ca) ** 2).sum()
    ts = cb - scale * Rs @ ca
    corr = np.array([np.linalg.norm(scale * Rs @ ba_poses[n][1] + ts - prior_C[n])
                     for n in names])
    print(f'\n[align] {len(names)} cameras, BA scale vs prior: {scale:.5f}')
    print(f'[audit] per-camera correction after sim(3): '
          f'mean {corr.mean()*1000:.2f} mm, median {np.median(corr)*1000:.2f} mm, '
          f'max {corr.max()*1000:.2f} mm ({names[int(np.argmax(corr))]})')

    # write refined model: BA poses mapped into the metric beetle frame
    ref = os.path.join(ws, 'sparse_refined', '0')
    os.makedirs(ref, exist_ok=True)
    shutil.copy(os.path.join(ba, 'cameras.txt'), os.path.join(ref, 'cameras.txt'))
    # transform points3D
    with open(os.path.join(ba, 'points3D.txt')) as fi, \
         open(os.path.join(ref, 'points3D.txt'), 'w') as fo:
        for line in fi:
            if line.startswith('#') or not line.strip():
                fo.write(line)
                continue
            p = line.split()
            X = scale * Rs @ np.array([float(v) for v in p[1:4]]) + ts
            fo.write(' '.join([p[0], f'{X[0]:.9f}', f'{X[1]:.9f}', f'{X[2]:.9f}']
                              + p[4:]) + '\n')
    # rewrite pose lines, keep each image's observation line verbatim (the
    # world transform does not touch image-local keypoints)
    with open(os.path.join(ba, 'images.txt')) as fi, \
         open(os.path.join(ref, 'images.txt'), 'w') as fo:
        expect_pose = True
        for line in fi:
            s = line.strip()
            if not s or s.startswith('#'):
                fo.write(line)
                continue
            if expect_pose:
                p = s.split()
                n = p[9]
                R, C = ba_poses[n]
                Rn = R @ Rs.T
                Cn = scale * Rs @ C + ts
                tn = -Rn @ Cn
                qw, qx, qy, qz = quat_wxyz(Rn)
                fo.write(f'{p[0]} {qw:.9f} {qx:.9f} {qy:.9f} {qz:.9f} '
                         f'{tn[0]:.9f} {tn[1]:.9f} {tn[2]:.9f} {p[8]} {n}\n')
            else:
                fo.write(line)
            expect_pose = not expect_pose
    print(f'[out] refined model -> {ref}')
    run(['colmap', 'model_analyzer', '--path', ref])


if __name__ == '__main__':
    main()
