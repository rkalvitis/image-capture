#!/usr/bin/env python3
"""Package the ring-protocol dataset for server-side masking + 3DGS training.

Creates <dataset>/gs_upload/ ready to rsync to rhea, everything in ONE data
folder (Masked-3DGS trains with -s <this folder>):

  images/           362 photos, sensor-native pixels, EXIF orientation tag
                    STRIPPED (poses/hand-eye are sensor-native; a viewer
                    applying the tag would lie)
  masks/            binary beetle masks, sensor-native. Whatever the local
                    masking run has already produced is converted in
                    (display->sensor + threshold); the REST are created on
                    the server by masking/mask_beetle_server.py, which
                    skips existing files - the two runs meet in the middle.
  masking/          the server-side masking script (GPU, resume-safe)
  sparse/0/         COLMAP TXT model: PINHOLE macro-band intrinsics
                    (calibration/intrinsics.yaml, 2026-07-13, fx~1976 -
                    the ring photos are at the same ~5 cm macro focus),
                    poses = the per-photo mocap chain from
                    camera_poses.csv (insect frame, METRIC - requires the
                    near-cull-patched masked-3dgs.sif; use --scale 20 for
                    a stock container), points3D = random-ball init at the
                    beetle position on the fitted rotation axis
  camera_poses.csv, insect_axis.yaml, rig_transforms.yaml, ring_report.txt
                    provenance copies
  README_server.md  the exact command sequence (upload, masking, training)

Re-run any time before uploading: it only converts what is missing, so a
second run scoops up masks the local masking finished in the meantime.

Usage:
  PYTHONPATH=scripts pixi run python scripts/prepare_ring_gs.py \\
      datasets/aug31_20deg_9vertical
"""

import argparse
import csv
import os
import shutil

import numpy as np
import yaml
from PIL import Image

from link_robot_mocap import quat_mat

WSIZE, HSIZE = 4032, 3024
REMOTE = '/media/white/nanodrones/roberts.kalvitis/3dgs/popillia/362img_01_09_masked/data'


def to_sensor(im, flag):
    """Undo an EXIF orientation: display-oriented image -> sensor pixels."""
    if flag == 6:
        return im.transpose(Image.ROTATE_90)
    if flag == 3:
        return im.transpose(Image.ROTATE_180)
    if flag == 8:
        return im.transpose(Image.ROTATE_270)
    return im


def mat_to_qwxyz(R):
    """Rotation matrix -> (w, x, y, z), COLMAP order."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s,
                         (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = np.empty(4)
    q[0] = (R[k, j] - R[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (R[j, i] + R[i, j]) / s
    q[1 + k] = (R[k, i] + R[i, k]) / s
    return q


SERVER_PATCHES = [
    # outputs live in gs_upload/ (the parent of masking/), not in masking/
    ("HERE = os.path.dirname(os.path.abspath(__file__))",
     "HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))"),
    # flat pre-converted images, resume-safe: skip photos already masked
    ("""    images = convert_dngs(os.path.join(args.session, 'images'),
                          os.path.join(HERE, 'rgb'))""",
     """    images = [(f[:-4], os.path.join(HERE, 'images', f))
              for f in sorted(os.listdir(os.path.join(HERE, 'images')))
              if f.endswith('.jpg')]
    images = [(n, p) for n, p in images if not os.path.exists(
        os.path.join(HERE, 'masks', n + '.png'))]"""),
    # training masks are binary (the packaged local ones are thresholded too)
    ("        save_mask(m, os.path.join(HERE, 'masks', name + '.png'))",
     "        save_mask((m >= 0.5).astype(np.float32),\n"
     "                  os.path.join(HERE, 'masks', name + '.png'))"),
    # the container's py3.8 transformers predates the grounded-detection
    # name; the old API returns the same dict (and still exists upstream)
    ("post_process_grounded_object_detection",
     "post_process_object_detection"),
]


def write_server_mask_script(local_script, dst):
    src = open(local_script).read()
    for old, new in SERVER_PATCHES:
        if old not in src:
            raise SystemExit(f'server patch anchor not found:\n{old}')
        src = src.replace(old, new)
    with open(dst, 'w') as f:
        f.write(src)


README = f"""# popillia ring dataset 2026-09-01 — 362 photos, mocap poses (Masked-3DGS)

One data folder, trainable as-is once masks/ is complete: images/ (sensor-
native, EXIF tag stripped), masks/ (binary), sparse/0 (PINHOLE fx~1976
macro-band intrinsics; poses = the measured per-photo mocap chain in the
INSECT frame, pose column verified against the robot joint log).

SCENE IS METRIC (working distances 40-60 mm). This requires the
masked-3dgs.sif with the 0.01 near-cull patch (the one rebuilt for the
26.08 runs). With a stock rasterizer (0.2 cull) NOTHING renders — re-make
the upload with `--scale 20` instead.

points3D.txt is a random-ball initialisation at the beetle's position on
the fitted rotation axis (no COLMAP involved). The loader converts it to
points3D.ply on first run — the bind must be writable; delete any stale
points3D.ply if you re-upload sparse models.

## 0. upload (Mac side)

    rsync -av --progress datasets/aug31_20deg_9vertical/gs_upload/ \\
        rhea:{REMOTE}/

## 1. finish the masks on the GPU (rhea)

One-time: the 3dgs container lacks the HF stack — install to your user site:

    singularity exec --nv ~/containers/3dgs.sif \\
        python -m pip install --user transformers timm einops kornia

Run (resume-safe — it only processes photos with no mask yet, so it
coexists with masks made on the Mac; ~2-4 s/photo on GPU):

    DATA={REMOTE}
    singularity exec --nv --bind $DATA:$DATA ~/containers/3dgs.sif \\
        env CUDA_VISIBLE_DEVICES=1 \\
        python $DATA/masking/mask_beetle_server.py

Sanity when done (expect 362 = 362):

    ls $DATA/images/*.jpg | wc -l; ls $DATA/masks/*.png | wc -l

Spot-check $DATA/preview/*.jpg (green contour on dimmed background) and
$DATA/report.csv (agree_iou < 0.8 rows deserve a look).

## 2. train (rhea, GPU 1)

    unset SINGULARITYENV_CUDA_VISIBLE_DEVICES
    export CODE_DIR=/home/robertsk/3dgs-masked
    export DATA={REMOTE}
    export OUT=$(dirname $DATA)/output
    mkdir -p $OUT
    S="singularity exec --nv --cleanenv --contain \\
       --bind $CODE_DIR:/workspace --bind $DATA:/data --bind $OUT:/output"

    # sanity: should print 362 images RGB 362 masks
    $S ~/containers/masked-3dgs.sif python -c "from PIL import Image; import glob; i=sorted(glob.glob('/data/images/*.jpg')); m=sorted(glob.glob('/data/masks/*.png')); print(len(i),'images; mode:',Image.open(i[0]).mode,';',len(m),'masks')"

    $S ~/containers/masked-3dgs.sif python /workspace/train.py \\
        -s /data -m /output --masks masks --lambda_mask 0.1 \\
        --eval --disable_viewer --data_device cpu --device 1 -r 1

    $S ~/containers/masked-3dgs.sif python /workspace/render.py  -m /output --device 1
    $S ~/containers/masked-3dgs.sif python /workspace/metrics.py -m /output --device 1

(362 full-res images: --data_device cpu keeps VRAM sane; use -r 2 if
training is slow. metrics.py: skip LPIPS as before if the VGG download
is blocked — set TORCH_HOME to a writable cache.)

## 3. fetch results back (Mac side)

    rsync -av rhea:$DATA/masks/ datasets/aug31_20deg_9vertical/gs_upload/masks/
    rsync -av rhea:$DATA/report.csv rhea:$DATA/preview \\
        datasets/aug31_20deg_9vertical/gs_upload/ 2>/dev/null || true
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dataset')
    ap.add_argument('--scale', type=float, default=1.0,
                    help='scene scale (default 1 = metric, needs the '
                         'near-cull-patched container; 20 for stock)')
    args = ap.parse_args()
    ds = args.dataset.rstrip('/')
    up = os.path.join(ds, 'gs_upload')
    for d in ('images', 'masks', 'masking', os.path.join('sparse', '0')):
        os.makedirs(os.path.join(up, d), exist_ok=True)

    # 1. images: sensor-native, EXIF tag stripped ------------------------
    names = sorted(n for n in os.listdir(os.path.join(ds, 'masking/rgb'))
                   if n.endswith('.jpg'))
    assert names, 'no rgb images found - run the masking DNG conversion first'
    flags = {}
    new_img = 0
    for n in names:
        src = os.path.join(ds, 'masking/rgb', n)
        flags[n] = Image.open(src).getexif().get(274)
        dst = os.path.join(up, 'images', n)
        if not os.path.exists(dst):
            im = Image.open(src).convert('RGB')  # pixels are sensor-native
            if im.size != (WSIZE, HSIZE):
                raise SystemExit(f'{n}: unexpected size {im.size}')
            im.save(dst, quality=97)             # PIL save drops the tag
            new_img += 1

    # 2. masks made locally so far: display -> sensor, binarize ----------
    new_masks = 0
    for n in names:
        msrc = os.path.join(ds, 'masking/masks', n[:-4] + '.png')
        mdst = os.path.join(up, 'masks', n[:-4] + '.png')
        if os.path.exists(msrc) and not os.path.exists(mdst):
            m = to_sensor(Image.open(msrc).convert('L'), flags[n])
            m.point(lambda v: 255 if v > 127 else 0).save(mdst)
            new_masks += 1

    # 3. server masking script ------------------------------------------
    write_server_mask_script(os.path.join(ds, 'masking/mask_popillia.py'),
                             os.path.join(up, 'masking/mask_beetle_server.py'))

    # 4. sparse/0 --------------------------------------------------------
    sdir = os.path.join(up, 'sparse', '0')
    intr = yaml.safe_load(open('calibration/intrinsics.yaml'))
    K = np.array(intr['camera_matrix'])
    s = args.scale
    with open(os.path.join(sdir, 'cameras.txt'), 'w') as f:
        f.write('# Camera list: CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]\n')
        f.write(f'1 PINHOLE {WSIZE} {HSIZE} {K[0,0]:.4f} {K[1,1]:.4f} '
                f'{K[0,2]:.4f} {K[1,2]:.4f}\n')

    rows = list(csv.DictReader(open(os.path.join(ds, 'camera_poses.csv'))))
    centers = []
    with open(os.path.join(sdir, 'images.txt'), 'w') as f:
        f.write('# Image list: IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID '
                'NAME\n#   (empty points2D line follows each)\n')
        for i, r in enumerate(sorted(rows, key=lambda r: r['image']), 1):
            C = np.array([float(r['cam_x']), float(r['cam_y']),
                          float(r['cam_z'])])
            centers.append(C)
            R_ic = quat_mat([float(v) for v in r['quat_xyzw'].split()])
            R = R_ic.T                       # world(insect) -> cam
            t = -R @ (C * s)
            q = mat_to_qwxyz(R)
            f.write(f'{i} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f} '
                    f'{t[0]:.9f} {t[1]:.9f} {t[2]:.9f} 1 '
                    f'{r["image"]}.jpg\n\n')

    # beetle sits on the fitted rotation axis at the dome's aim height
    ax = yaml.safe_load(open(os.path.join(ds, 'insect_axis.yaml')))
    a_pt = np.array(ax['axis_point_insect_frame_m'])
    a_dir = np.array(ax['axis_dir_insect_frame'], float)
    a_dir /= np.linalg.norm(a_dir)
    cloud = np.mean(centers, axis=0)
    beetle = a_pt + float((cloud - a_pt) @ a_dir) * a_dir
    rng = np.random.default_rng(0)
    pts = rng.normal(size=(20000, 3))
    pts /= np.linalg.norm(pts, axis=1, keepdims=True)
    pts = beetle + pts * (0.012 * rng.random((20000, 1)) ** (1 / 3))
    with open(os.path.join(sdir, 'points3D.txt'), 'w') as f:
        f.write('# 3D point list: POINT3D_ID X Y Z R G B ERROR TRACK[]\n')
        for i, p in enumerate(pts * s, 1):
            g = int(rng.integers(90, 160))
            f.write(f'{i} {p[0]:.7f} {p[1]:.7f} {p[2]:.7f} '
                    f'{g} {g} {g} 1.0\n')
    ply = os.path.join(sdir, 'points3D.ply')
    if os.path.exists(ply):
        os.remove(ply)   # a stale ply silently overrides the txt

    # 5. experiment tooling + provenance + README ------------------------
    os.makedirs(os.path.join(up, 'tools'), exist_ok=True)
    for fn in ('refine_on_server.sh', 'align_sfm_to_mocap.py'):
        shutil.copy(os.path.join('scripts', fn),
                    os.path.join(up, 'tools', fn))
    # run_experiments.sh lives in gs_upload/ directly (hand-maintained)
    for fn in ('camera_poses.csv', 'insect_axis.yaml', 'ring_report.txt'):
        shutil.copy(os.path.join(ds, fn), os.path.join(up, fn))
    shutil.copy(os.path.join(ds, 'calibrations/rig_transforms.yaml'),
                os.path.join(up, 'rig_transforms.yaml'))
    with open(os.path.join(up, 'README_server.md'), 'w') as f:
        f.write(README)

    done_masks = len(os.listdir(os.path.join(up, 'masks')))
    total = sum(os.path.getsize(os.path.join(r, fn))
                for r, _, fs in os.walk(up) for fn in fs)
    print(f'prepared {up}: {len(names)} images ({new_img} new), '
          f'{done_masks}/{len(names)} masks packaged ({new_masks} new), '
          f'scale x{s:g}, ~{total / 1e6:.0f} MB')
    print(f'upload:  rsync -av --progress {up}/ rhea:{REMOTE}/')


if __name__ == '__main__':
    main()
