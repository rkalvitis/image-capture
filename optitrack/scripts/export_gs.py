#!/usr/bin/env python3
"""Export a masked stack session as a 3DGS training dataset (COLMAP layout).

Builds, in the output directory:

    images/stack_<n>.jpg     masked composites, white-padded so the
                             principal point sits at the image centre (the
                             Inria 3DGS reader ignores cx/cy)
    sparse/0/                COLMAP text model — PRIMARY lens transform
    sparse_anchored/0/       same poses with the rig-anchored lens offset
    README.md                provenance + train command

Poses: rig-consistency averaged. Each of the 8 arm poses is averaged over
its measured revisits across the 6 rotation blocks (world frame, ~sqrt(6)
noise reduction; block 1's copied poses are excluded from the mean and then
served by it). The camera-in-insect pose is recomposed per stack from the
averaged arm pose and the block's object pose.

Lens transforms (phone body -> camera):
  primary   --lens-yaml calibration/lens_marker_estimate.yaml position +
            --handeye-yaml (ETH) rotation — the 07-13 calibration pair, AS
            REQUESTED. NB: the phone body was redefined in Motive after
            that session; on this dataset this puts the lens 117–134 mm
            from the specimen (photos were shot at ~25 mm).
  anchored  the pose-1-down rig anchor (25 mm working distance), the
            transform consistent with this dataset.

World frame: the insect (object body) frame, translated so the fitted
specimen point is the origin, scaled by --scale (default 10: 1 unit =
100 mm) so the scene extent is O(1) for 3DGS learning rates.

    pixi run -e masking python scripts/export_gs.py \
        datasets/phone_<ts> /path/to/3DGS/<scene> [--roll 270]
"""

import argparse
import csv
import os
import shutil
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mask_sam_geom import camera_basis
from plot_stacks import fit_geometry, load, rotm

# landscape-sensor camera -> portrait-image camera (90° cw display
# rotation, the convention verified for this dataset in mask_sam_geom):
# x_p = -y_c, y_p = x_c, z_p = z_c   (columns = portrait axes in cam coords)
M_PORTRAIT = np.array([[0.0, 1.0, 0.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0]])


def quat_mean(qs):
    qs = np.array(qs, dtype=float)
    qs[np.sum(qs * qs[0], axis=1) < 0] *= -1
    q = qs.mean(0)
    return q / np.linalg.norm(q)


def mat_to_quat_wxyz(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-8:
        x = (R[2, 1] - R[1, 2]) / (4 * w)
        y = (R[0, 2] - R[2, 0]) / (4 * w)
        z = (R[1, 0] - R[0, 1]) / (4 * w)
    else:                                     # w ~ 0: pick largest diagonal
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(max(0.0, 1 + R[i, i] - R[j, j] - R[k, k])) * 2
        v = [0.0, 0.0, 0.0]
        v[i] = s / 4
        v[j] = (R[j, i] + R[i, j]) / s
        v[k] = (R[k, i] + R[i, k]) / s
        w = (R[k, j] - R[j, k]) / s
        x, y, z = v
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def load_rows(session):
    with open(os.path.join(session, 'stacks.csv')) as f:
        rows = list(csv.DictReader(f))
    vec = lambda r, p: np.array([float(r[p + c]) for c in ('x', 'y', 'z')])
    quat = lambda r, p: np.array([float(r[p + 'q' + c]) for c in 'xyzw'])
    return [{'n': int(r['stack']),
             'cam': vec(r, 'cam_'), 'cam_q': quat(r, 'cam_'),
             'obj': vec(r, 'obj_'), 'obj_q': quat(r, 'obj_'),
             'measured': r['cam_source'] == 'measured'} for r in rows]


def averaged_rel_poses(rows, spb):
    """Rig-consistency averaging → {stack: (R_rel, t_rel)} camera body in
    the object frame."""
    by_k = {}
    for r in rows:
        if r['measured']:
            by_k.setdefault((r['n'] - 1) % spb, []).append(r)
    cam_avg = {}
    for k, rs in by_k.items():
        cam_avg[k] = (np.mean([r['cam'] for r in rs], axis=0),
                      quat_mean([r['cam_q'] for r in rs]))
        spread = np.linalg.norm(
            np.array([r['cam'] for r in rs]) - cam_avg[k][0], axis=1)
        print(f'arm pose {k + 1}: {len(rs)} revisits, spread '
              f'{spread.max() * 1e3:.1f} mm max')
    rel = {}
    for r in rows:
        k = (r['n'] - 1) % spb
        p, q = cam_avg[k]
        R_obj, R_cam = rotm(r['obj_q']), rotm(q)
        rel[r['n']] = (R_obj.T @ R_cam, R_obj.T @ (p - r['obj']))
    return rel


def write_colmap(model_dir, cam_line, images, points):
    os.makedirs(model_dir, exist_ok=True)
    with open(os.path.join(model_dir, 'cameras.txt'), 'w') as f:
        f.write('# CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]\n' + cam_line)
    with open(os.path.join(model_dir, 'images.txt'), 'w') as f:
        f.write('# IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME\n')
        for i, (name, q, t) in enumerate(images, 1):
            f.write(f'{i} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f} '
                    f'{t[0]:.9f} {t[1]:.9f} {t[2]:.9f} 1 {name}\n\n')
    with open(os.path.join(model_dir, 'points3D.txt'), 'w') as f:
        f.write('# POINT3D_ID X Y Z R G B ERROR TRACK[]\n')
        for i, (p, c) in enumerate(points, 1):
            f.write(f'{i} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} '
                    f'{c[0]} {c[1]} {c[2]} 1.0\n')


def main():
    ap_ = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap_.add_argument('session')
    ap_.add_argument('out', help='dataset dir to create (e.g. .../3DGS/popillia)')
    ap_.add_argument('--masked', default='masked',
                     help='masked-image dir under the session')
    ap_.add_argument('--intrinsics', default='calibration/intrinsics.yaml')
    ap_.add_argument('--lens-yaml',
                     default='calibration/lens_marker_estimate.yaml')
    ap_.add_argument('--handeye-yaml',
                     default='calibration/phone_20260713_134547/'
                             'hand_eye_eth.yaml')
    ap_.add_argument('--working-distance', type=float, default=25.0)
    ap_.add_argument('--roll', type=float, default=270.0)
    ap_.add_argument('--scale', type=float, default=10.0,
                     help='world scale: units per metre (default 10 → '
                          '1 unit = 100 mm)')
    ap_.add_argument('--n-init-points', type=int, default=30000)
    ap_.add_argument('--init-radius-mm', type=float, default=8.0)
    ap_.add_argument('--stacks-per-block', type=int, default=8)
    args = ap_.parse_args()

    from PIL import Image
    session = args.session.rstrip('/')
    spb = args.stacks_per_block

    stacks = load(session)
    ap, ad, spec, o_b, _ = fit_geometry(stacks, spb)
    rows = load_rows(session)
    rel = averaged_rel_poses(rows, spb)

    # lens transforms (camera pose in the phone body frame)
    with open(args.lens_yaml) as f:
        C_marker = np.array(yaml.safe_load(f)['lens_position_m'])
    with open(args.handeye_yaml) as f:
        he = yaml.safe_load(f)['camera_in_body']
    R_he = rotm(np.array(he['quaternion_xyzw']))
    wd = args.working_distance * 1e-3
    pose1 = [s for s in stacks if (s['n'] - 1) % spb == 0]
    t_anch = np.mean([rotm(s['rel_q']).T @ (spec + wd * ad - s['rel'])
                      for s in pose1], axis=0)
    R_anch = camera_basis(o_b, args.roll).T          # camera axes as columns
    variants = {
        'sparse': (R_he @ M_PORTRAIT, C_marker),
        'sparse_anchored': (R_anch @ M_PORTRAIT, t_anch),
    }

    # intrinsics → portrait + white padding so the pp is centred
    with open(args.intrinsics) as f:
        calib = yaml.safe_load(f)
    K = np.array(calib['camera_matrix'])
    Wl, Hl = calib['image_size']                     # 4032, 3024 landscape
    fx_p, fy_p = K[1, 1], K[0, 0]
    cx_p, cy_p = (Hl - 1) - K[1, 2], K[0, 2]
    Wp = int(round(2 * cx_p / 2) * 2)
    Hp = int(round(2 * cy_p / 2) * 2)
    pad_r, pad_b = Wp - Hl, Hp - Wl                  # portrait is Hl × Wl
    print(f'portrait intrinsics: fx {fx_p:.1f} fy {fy_p:.1f} '
          f'pp ({cx_p:.1f}, {cy_p:.1f}) → padded {Wp}×{Hp} '
          f'(right {pad_r}, bottom {pad_b})')
    cam_line = f'1 PINHOLE {Wp} {Hp} {fx_p:.6f} {fy_p:.6f} ' \
               f'{Wp / 2:.1f} {Hp / 2:.1f}\n'

    os.makedirs(os.path.join(args.out, 'images'), exist_ok=True)
    names = {}
    for n in sorted(rel):
        src = os.path.join(session, args.masked, f'stack_{n:02d}.jpg')
        if not os.path.exists(src):
            print(f'stack {n:2d}: no masked image, skipped')
            continue
        img = Image.open(src)
        canvas = Image.new('RGB', (Wp, Hp), (255, 255, 255))
        canvas.paste(img, (0, 0))
        names[n] = f'stack_{n:02d}.jpg'
        canvas.save(os.path.join(args.out, 'images', names[n]), quality=95)

    rng = np.random.default_rng(0)
    v = rng.normal(size=(args.n_init_points, 3))
    v = (v / np.linalg.norm(v, axis=1, keepdims=True)
         * rng.random((args.n_init_points, 1)) ** (1 / 3)
         * args.init_radius_mm * 1e-3 * args.scale)
    cols = rng.integers(60, 160, size=(args.n_init_points, 3))
    points = list(zip(v, cols))

    for sub, (R_cam_body, C_body) in variants.items():
        images = []
        dists = []
        for n in sorted(names):
            R_rel, t_rel = rel[n]
            R_cw = R_rel @ R_cam_body                # camera → insect world
            C = (t_rel + R_rel @ C_body - spec) * args.scale
            dists.append(np.linalg.norm(C) / args.scale * 1e3)
            R_wc = R_cw.T
            images.append((names[n], mat_to_quat_wxyz(R_wc), -R_wc @ C))
        write_colmap(os.path.join(args.out, sub, '0'), cam_line,
                     images, points)
        print(f'{sub}: camera-to-origin {min(dists):.1f}–{max(dists):.1f} mm')

    with open(os.path.join(args.out, 'README.md'), 'w') as f:
        f.write(f"""# popillia — mocap-posed 3DGS dataset

Source: optitrack `{session}` (48 focus-stack composites, masked to the
insect on white by `masks/hybrid_text_sam`, exported by
`scripts/export_gs.py`).

- World frame: insect body frame, specimen at origin, **1 unit = 100 mm**
  (scale {args.scale:g}).
- Poses: rig-consistency averaged (8 arm poses × measured revisits).
- `sparse/0`: lens transform = 07-13 marker estimate + ETH hand-eye
  rotation (the 07-13 phone body was later redefined in Motive — on this
  dataset this puts the lens 117–134 mm from the specimen; kept per
  explicit request).
- `sparse_anchored/0`: rig-anchored lens offset (pose-1-down anchor,
  {args.working_distance:g} mm working distance) — the transform consistent
  with this dataset (lens 24–33 mm from specimen). To train with it:
  swap the directories or pass the images dir with this sparse model.
- Images are white-padded to {Wp}×{Hp} so the principal point is centred
  (this repo's COLMAP reader ignores cx/cy). Intrinsics are the 07-13
  ultra-wide calibration; distortion treated as negligible (ProRAW bakes
  the correction in).
- `points3D.txt` is a random init cloud (r = {args.init_radius_mm:g} mm
  ball at origin) — there is no COLMAP reconstruction.

Train (white background matters — the masks baked white):

    python train.py -s <this dir> -m output/popillia -w --eval
""")
    print(f'\ndataset → {args.out}')


if __name__ == '__main__':
    main()
