#!/usr/bin/env python3
"""Refine the mocap camera poses with COLMAP (the plan's §7 hybrid).

The mocap+anchor poses carry a systematic few-mm error — at a 25 mm working
distance that is 100–800 px of reprojection error, which smears the splat.
This script runs structure-from-motion on the beetle pixels only and keeps
the mocap for what it is good at (metric scale, insect frame):

  1. SIFT features on the ORIGINAL composites, restricted to the insect by
     COLMAP native masks (dilated so silhouette features survive)
  2. exhaustive matching + incremental SfM with the calibrated intrinsics
     held FIXED (PINHOLE, no refinement)
  3. the SfM result — internally consistent to sub-px — is sim(3)-aligned
     (Umeyama) onto the mocap camera centres: metric scale + insect frame
     from mocap, relative geometry from SfM
  4. writes <dataset>/<out>/0/{cameras,images,points3D}.txt — poses refined,
     points3D = the real triangulated beetle cloud (better GS init than the
     random ball)

The per-camera correction it reports IS the mocap pose error audit.

    pixi run -e masking python scripts/refine_poses_colmap.py \
        /path/to/popillia_masked [--out sparse_refined]
"""

import argparse
import os
import shutil
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from export_gs import mat_to_quat_wxyz


def read_model_poses(sparse_dir):
    """images.txt → {name: (R_wc, t_wc)}."""
    out = {}
    with open(os.path.join(sparse_dir, 'images.txt')) as f:
        for line in f:
            t = line.strip().split()
            if len(t) >= 10 and t[0].isdigit():
                w, x, y, z = (float(v) for v in t[1:5])
                R = quat_wxyz_to_mat(np.array([w, x, y, z]))
                out[t[9]] = (R, np.array([float(v) for v in t[5:8]]))
    return out


def quat_wxyz_to_mat(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


def umeyama(src, dst):
    """Similarity transform (s, R, t) minimising ||s R src + t − dst||."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) * len(src) / (xs ** 2).sum()
    t = mu_d - s * R @ mu_s
    return s, R, t


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dataset', help='dataset dir (images/, masks/, sparse/0)')
    ap.add_argument('--out', default='sparse_refined')
    ap.add_argument('--workdir', default=None,
                    help='COLMAP scratch dir (default <dataset>/colmap_work)')
    ap.add_argument('--mask-dilate-px', type=int, default=20)
    ap.add_argument('--max-image-size', type=int, default=3200)
    args = ap.parse_args()

    import cv2
    import pycolmap

    ds = args.dataset.rstrip('/')
    work = args.workdir or os.path.join(ds, 'colmap_work')
    os.makedirs(work, exist_ok=True)
    db = os.path.join(work, 'database.db')
    img_dir = os.path.join(ds, 'images')

    # COLMAP masks: <image_name>.png (e.g. stack_01.jpg.png), white = use
    cmask_dir = os.path.join(work, 'colmap_masks')
    os.makedirs(cmask_dir, exist_ok=True)
    kernel = np.ones((args.mask_dilate_px, args.mask_dilate_px), np.uint8)
    names = sorted(n for n in os.listdir(img_dir) if n.endswith('.jpg'))
    for name in names:
        m = cv2.imread(os.path.join(ds, 'masks', name[:-4] + '.png'),
                       cv2.IMREAD_GRAYSCALE)
        m = cv2.dilate((m > 127).astype(np.uint8) * 255, kernel)
        cv2.imwrite(os.path.join(cmask_dir, name + '.png'), m)
    print(f'{len(names)} COLMAP masks (dilated {args.mask_dilate_px} px)')

    # intrinsics from the existing model — held fixed through SfM
    with open(os.path.join(ds, 'sparse/0/cameras.txt')) as f:
        cam_line = [l for l in f if not l.startswith('#')][0].split()
    W, H = int(cam_line[2]), int(cam_line[3])
    fx, fy, cx, cy = (float(v) for v in cam_line[4:8])

    if os.path.exists(db):
        os.remove(db)
    reader = pycolmap.ImageReaderOptions()
    reader.mask_path = cmask_dir
    reader.camera_model = 'PINHOLE'
    reader.camera_params = f'{fx},{fy},{cx},{cy}'
    sift = pycolmap.SiftExtractionOptions()
    sift.max_image_size = args.max_image_size
    print('extracting features (masked SIFT)…')
    pycolmap.extract_features(db, img_dir,
                              camera_mode=pycolmap.CameraMode.SINGLE,
                              reader_options=reader, sift_options=sift)
    print('matching (exhaustive)…')
    pycolmap.match_exhaustive(db)

    opts = pycolmap.IncrementalPipelineOptions()
    opts.ba_refine_focal_length = False
    opts.ba_refine_principal_point = False
    opts.ba_refine_extra_params = False
    print('incremental mapping…')
    sfm_dir = os.path.join(work, 'sfm')
    os.makedirs(sfm_dir, exist_ok=True)
    recs = pycolmap.incremental_mapping(db, img_dir, sfm_dir, options=opts)
    if not recs:
        raise SystemExit('COLMAP failed to reconstruct — nothing registered')
    rec = max(recs.values(), key=lambda r: r.num_reg_images())
    print(f'reconstruction: {rec.num_reg_images()}/{len(names)} images, '
          f'{rec.num_points3D()} points, '
          f'mean track {rec.compute_mean_track_length():.1f}, '
          f'mean reproj err {rec.compute_mean_reprojection_error():.2f} px')

    # ---- sim(3) alignment onto mocap camera centres ----
    mocap = read_model_poses(os.path.join(ds, 'sparse/0'))
    sfm_centers, mocap_centers, reg_names = [], [], []
    for image in rec.images.values():
        if image.name in mocap:
            cw = image.cam_from_world()
            R_wc = cw.rotation.matrix()
            C = -R_wc.T @ np.array(cw.translation)
            Rm, tm = mocap[image.name]
            sfm_centers.append(C)
            mocap_centers.append(-Rm.T @ tm)
            reg_names.append(image.name)
    sfm_centers = np.array(sfm_centers)
    mocap_centers = np.array(mocap_centers)
    s, Ra, ta = umeyama(sfm_centers, mocap_centers)
    resid = s * (Ra @ sfm_centers.T).T + ta - mocap_centers
    rn = np.linalg.norm(resid, axis=1)
    print(f'sim3: scale {s:.4f}; centre residual vs mocap '
          f'p50 {np.percentile(rn, 50) * 100:.1f} mm, '
          f'max {rn.max() * 100:.1f} mm  (= mocap pose error audit)')

    # ---- write refined model ----
    out_dir = os.path.join(ds, args.out, '0')
    os.makedirs(out_dir, exist_ok=True)
    shutil.copy2(os.path.join(ds, 'sparse/0/cameras.txt'),
                 os.path.join(out_dir, 'cameras.txt'))
    corrections = []
    with open(os.path.join(out_dir, 'images.txt'), 'w') as f:
        f.write('# IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME\n')
        for k, image in enumerate(sorted(rec.images.values(),
                                         key=lambda im: im.name), 1):
            if image.name not in mocap:
                continue
            cw = image.cam_from_world()
            R_wc = cw.rotation.matrix()
            t_wc = np.array(cw.translation)
            # world transform x' = s Ra x + ta  ⇒  R' = R_wc Raᵀ,
            # C' = s Ra C + ta, t' = −R' C'
            C = -R_wc.T @ t_wc
            Cn = s * Ra @ C + ta
            Rn_ = R_wc @ Ra.T
            tn = -Rn_ @ Cn
            q = mat_to_quat_wxyz(Rn_)
            f.write(f'{k} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f} '
                    f'{tn[0]:.9f} {tn[1]:.9f} {tn[2]:.9f} 1 {image.name}\n\n')
            Rm, tm = mocap[image.name]
            corrections.append((np.linalg.norm(Cn - (-Rm.T @ tm)) * 100,
                                np.degrees(np.arccos(np.clip(
                                    (np.trace(Rn_ @ Rm.T) - 1) / 2, -1, 1)))))
    with open(os.path.join(out_dir, 'points3D.txt'), 'w') as f:
        f.write('# POINT3D_ID X Y Z R G B ERROR TRACK[]\n')
        for pid, p in rec.points3D.items():
            xyz = s * Ra @ np.array(p.xyz) + ta
            col = np.array(p.color)
            f.write(f'{pid} {xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f} '
                    f'{col[0]} {col[1]} {col[2]} '
                    f'{p.error:.3f}\n')
    corr = np.array(corrections)
    print(f'per-camera correction applied: position p50 '
          f'{np.percentile(corr[:, 0], 50):.1f} mm max {corr[:, 0].max():.1f} '
          f'mm; rotation p50 {np.percentile(corr[:, 1], 50):.2f}° '
          f'max {corr[:, 1].max():.2f}°')
    missing = [n for n in names if n not in reg_names]
    if missing:
        print(f'WARNING: {len(missing)} images not registered by SfM and '
              f'left OUT of the refined model: {missing}')
    print(f'→ {out_dir}')


if __name__ == '__main__':
    main()
