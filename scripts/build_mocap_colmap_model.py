#!/usr/bin/env python3
"""Rebuild the mocap camera poses with a chosen hand-eye + per-image intrinsics
into a NEW COLMAP text model (prior / init for COLMAP and 3DGS).

Chain (see dataset README):  T_insect<-cam(k) = T_insect<-body(k) @ T_body<-cam
The body poses T_insect<-body(k) are recovered from camera_poses.csv by removing
the hand-eye that built it (calibrations/rig_transforms.yaml, T_body_from_cam);
the hand-eye given with --handeye is then applied instead. Intrinsics come from
a per-image csv (image, fx, fy, cx, cy — e.g. intrinsics_factory_aug31.csv) or
one fixed --K. Nothing existing is modified; outputs are tagged new files:

    <dataset>/camera_poses_<tag>.csv                 same columns as camera_poses.csv
    <dataset>/gs_upload/sparse_<tag>/0/cameras.txt   one PINHOLE camera per image
    <dataset>/gs_upload/sparse_<tag>/0/images.txt    cam_from_world, ids 1..N by name
    <dataset>/gs_upload/sparse_<tag>/0/points3D.txt  empty (COLMAP triangulates)

Usage:
  pixi run python scripts/build_mocap_colmap_model.py [datasets/aug31_20deg_9vertical] \\
      --handeye calibrations/handeye_factoryK_20260831.yaml --tag mocap_factoryK \\
      [--intrinsics-csv intrinsics_factory_aug31.csv | --K fx,fy,cx,cy] [--scale 1]
"""

import argparse
import csv
import os
from datetime import date

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as Rot

DEFAULT_DS = 'datasets/aug31_20deg_9vertical'
W, H = 4032, 3024


def load_handeye(path):
    """-> (R_bc, p_bc), from camera_in_body / T_body_from_cam / any block with
    translation_m + quaternion_xyzw."""
    d = yaml.safe_load(open(path))
    for key in ('camera_in_body', 'T_body_from_cam'):
        if key in d:
            d = d[key]
            break
    else:
        blocks = [v for v in d.values() if isinstance(v, dict) and 'translation_m' in v and 'quaternion_xyzw' in v]
        if len(blocks) != 1:
            raise SystemExit(f'{path}: cannot find one hand-eye block (translation_m + quaternion_xyzw)')
        d = blocks[0]
    return Rot.from_quat(d['quaternion_xyzw']).as_matrix(), np.array(d['translation_m'], float)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument('dataset', nargs='?', default=DEFAULT_DS)
    ap.add_argument('--handeye', required=True, help='hand-eye yaml (T_body<-cam)')
    ap.add_argument('--tag', required=True)
    ap.add_argument('--intrinsics-csv', default=None,
                    help='per-image intrinsics csv with columns image,fx,fy,cx,cy (default <dataset>/intrinsics_factory_aug31.csv)')
    ap.add_argument('--K', default=None, help='fixed intrinsics fx,fy,cx,cy instead of the csv')
    ap.add_argument('--scale', type=float, default=1.0, help='multiply all lengths (1 = metres)')
    ap.add_argument('--force', action='store_true')
    args = ap.parse_args()
    ds = args.dataset.rstrip('/')

    rt = yaml.safe_load(open(os.path.join(ds, 'calibrations', 'rig_transforms.yaml')))
    R_csv = np.array(rt['T_body_from_cam']['rotation_matrix']); p_csv = np.array(rt['T_body_from_cam']['translation_m'])
    R_bc, p_bc = load_handeye(args.handeye)
    print(f'hand-eye {args.handeye}: differs from rig_transforms.yaml by '
          f'{np.degrees(np.linalg.norm(Rot.from_matrix(R_csv.T @ R_bc).as_rotvec())):.2f} deg, {np.linalg.norm(p_bc - p_csv)*1000:.2f} mm')

    if args.K:
        fx, fy, cx, cy = (float(v) for v in args.K.split(','))
        K_of = lambda stem: (fx, fy, cx, cy)   # noqa: E731
        k_src = f'fixed K {args.K}'
    else:
        kpath = args.intrinsics_csv or os.path.join(ds, 'intrinsics_factory_aug31.csv')
        ktab = {r['image']: (float(r['fx']), float(r['fy']), float(r['cx']), float(r['cy'])) for r in csv.DictReader(open(kpath))}
        K_of = lambda stem: ktab[stem]          # noqa: E731
        k_src = f'per-image K from {kpath}'

    rows = list(csv.DictReader(open(os.path.join(ds, 'camera_poses.csv'))))
    rows.sort(key=lambda r: r['image'])
    out_csv = os.path.join(ds, f'camera_poses_{args.tag}.csv')
    out_dir = os.path.join(ds, 'gs_upload', f'sparse_{args.tag}', '0')
    for p in (out_csv, out_dir):
        if os.path.exists(p) and not args.force:
            raise SystemExit(f'refusing to overwrite {p} (choose another --tag or pass --force)')
    os.makedirs(out_dir, exist_ok=True)

    s = args.scale
    with open(out_csv, 'w', newline='') as fc, \
            open(os.path.join(out_dir, 'cameras.txt'), 'w') as fk, \
            open(os.path.join(out_dir, 'images.txt'), 'w') as fi:
        wcsv = csv.DictWriter(fc, fieldnames=list(rows[0].keys())); wcsv.writeheader()
        fk.write(f'# Camera list: CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]  ({k_src}; one camera per image)\n')
        fi.write(f'# Image list: IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME  (mocap body poses + hand-eye '
                 f'{os.path.basename(args.handeye)}, scale {s:g}, built {date.today()} by build_mocap_colmap_model.py)\n')
        for k, r in enumerate(rows, 1):
            C = np.array([float(r['cam_x']), float(r['cam_y']), float(r['cam_z'])])
            R_ic = Rot.from_quat([float(v) for v in r['quat_xyzw'].split()]).as_matrix()
            R_ib = R_ic @ R_csv.T                       # body pose in the insect frame
            C_b = C - R_ib @ p_csv
            R_new = R_ib @ R_bc                          # new camera pose
            C_new = C_b + R_ib @ p_bc
            r2 = dict(r)
            r2['cam_x'], r2['cam_y'], r2['cam_z'] = (f'{v:.5f}' for v in C_new)
            r2['quat_xyzw'] = ' '.join(f'{v:.6f}' for v in Rot.from_matrix(R_new).as_quat())
            wcsv.writerow(r2)
            fx, fy, cx, cy = K_of(r['image'])
            fk.write(f'{k} PINHOLE {W} {H} {fx:.4f} {fy:.4f} {cx:.4f} {cy:.4f}\n')
            R_cw = R_new.T; t = -R_cw @ (C_new * s)
            q = Rot.from_matrix(R_cw).as_quat()      # xyzw
            fi.write(f'{k} {q[3]:.9f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {t[0]:.9f} {t[1]:.9f} {t[2]:.9f} {k} {r["image"]}.jpg\n\n')
    open(os.path.join(out_dir, 'points3D.txt'), 'w').write('# empty: triangulate with COLMAP\n')
    print(f'{len(rows)} cameras -> {out_dir} and {out_csv}')


if __name__ == '__main__':
    main()
