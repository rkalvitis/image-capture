#!/usr/bin/env python3
"""Package the 20260826 dataset for server-side 3DGS training, two variants.

Creates <dataset>/gs_upload/ ready to rsync to the server:

  images/            48 ORIGINAL photos, sensor-native pixels with the
                     EXIF orientation tag stripped (Masked-3DGS reads raw
                     pixels; a viewer applying the tag would lie)
  masks/             tight binary beetle masks, sensor-native — the
                     Masked-3DGS hybrid loss (--masks) suppresses
                     everything outside them, so images stay unmasked
  prior/sparse/0     poses = the robot-CSV guidance priors, points3D =
                     the cloud triangulated AT those poses (colmap/sparse_tri)
  refined/sparse/0   poses = COLMAP bundle-adjusted, sim(3)-aligned back
                     to the metric beetle frame (colmap/sparse_refined)
  prior/{images,masks}, refined/{images,masks}
                     relative symlinks -> ../images, ../masks (Masked-3DGS
                     resolves -s <src>/images and --masks masks against
                     the source path)
  registration.yaml  provenance copy
  README_server.md   exact training + comparison commands

Both variants share identical images, masks, and intrinsics — the ONLY
difference is the camera poses, so any quality difference between the two
trainings is attributable to the poses.

Usage:
  PYTHONPATH=scripts pixi run python scripts/prepare_gs_upload.py \\
      datasets/phone_20260826_113325
"""

import argparse
import os
import shutil

from PIL import Image

WSIZE, HSIZE = 4032, 3024
# The 3DGS CUDA rasterizer hard-culls gaussians closer than 0.2 units to the
# camera (auxiliary.h: p_view.z <= 0.2f). This macro scene has 3.4-6.6 cm
# working distances, so at metric scale NOTHING ever rasterizes (opacities
# stay at the reset floor, renders come out black). Scale the whole scene up;
# GS is scale-agnostic. Divide model coordinates by SCENE_SCALE to get metres.
SCENE_SCALE = 20.0


def to_sensor(im, flag):
    if flag == 6:
        return im.transpose(Image.ROTATE_90)
    if flag == 3:
        return im.transpose(Image.ROTATE_180)
    if flag == 8:
        return im.transpose(Image.ROTATE_270)
    return im


README = """# popillia 48img 2026-08-26 — prior vs refined pose comparison (Masked-3DGS)

Two COLMAP models over IDENTICAL images/masks/intrinsics; only the poses
differ:

  prior/    robot-CSV chain poses (registration.yaml pipeline), points
            triangulated at those poses
  refined/  COLMAP bundle-adjusted poses, sim(3)-aligned back to the
            metric beetle frame (local audit: mean 1.65 mm from prior)

images/ are the ORIGINAL photos (sensor-native 4032x3024, EXIF tag
stripped), RGB jpg; masks/ are tight binary beetle masks (separate files,
NOT alpha-embedded) — hence --masks masks --lambda_mask 0.1.
Camera: PINHOLE, fx=fy=1796.88, cx=2016, cy=1512; intrinsics fixed in BA.
sparse/0 is TXT; the loader converts points3D.txt -> points3D.ply on the
first run (the bind must be writable).

SCENE IS SCALED x20 (divide by 20 for metres): the CUDA rasterizer culls
gaussians closer than 0.2 units to the camera, and the true 3.4-6.6 cm
working distances sit entirely inside that -- at metric scale the model
never receives a gradient (that is the black-renders failure). If you
re-upload sparse models, delete any stale points3D.ply next to them.

The variant dirs use RELATIVE symlinks (images -> ../images,
masks -> ../masks), so ALWAYS bind the whole dataset root — never a
variant subdir alone (with --contain the symlink would point outside the
bind and the loader would find 0 images).

## Train both on rhea (GPU 1)

    unset SINGULARITYENV_CUDA_VISIBLE_DEVICES   # would remap ordinals and break --device 1
    export CODE_DIR=/home/robertsk/3dgs-masked
    export DATA_ROOT=/media/white/nanodrones/roberts.kalvitis/3dgs/popillia/48img_2608_prior_vs_refined
    export OUTPUT_ROOT=/media/white/nanodrones/roberts.kalvitis/3dgs/popillia/48img_2608_output
    SIF=~/containers/masked-3dgs.sif

    for V in prior refined; do
      mkdir -p $OUTPUT_ROOT/$V
      S="singularity exec --nv --cleanenv --contain \
         --bind $CODE_DIR:/workspace --bind $DATA_ROOT:/data \
         --bind $OUTPUT_ROOT/$V:/output"

      # sanity: should print 48 images RGB 48 masks
      $S $SIF python -c "from PIL import Image, ImageOps; import glob; i=sorted(glob.glob('/data/$V/images/*.jpg')); m=sorted(glob.glob('/data/$V/masks/*.png')); print(len(i),'images; mode:',Image.open(i[0]).mode,';',len(m),'masks')"

      $S $SIF python /workspace/train.py \
          -s /data/$V -m /output \
          --masks masks --lambda_mask 0.1 --eval --disable_viewer \
          --data_device cpu --device 1 -r 1

      $S $SIF python /workspace/render.py  -m /output --device 1
      $S $SIF python /workspace/metrics.py -m /output --device 1
    done

## Compare

metrics.py writes results into each -m dir: diff $OUTPUT_ROOT/prior vs
$OUTPUT_ROOT/refined (PSNR/SSIM/LPIPS) and flip between the two
test-view render folders for a visual A/B. Identical seeds and config,
so the gap is attributable to the poses.
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dataset')
    ap.add_argument('--scale', type=float, default=SCENE_SCALE,
                    help='scene scale factor (default 20 for the stock '
                         'rasterizer 0.2-unit near cull; use 1 once the '
                         'container is rebuilt with the 1 cm cull patch)')
    args = ap.parse_args()
    ds = args.dataset.rstrip('/')
    up = os.path.join(ds, 'gs_upload')
    img_dir = os.path.join(up, 'images')
    mask_dir = os.path.join(up, 'masks')
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(mask_dir, exist_ok=True)

    names = sorted(n for n in os.listdir(os.path.join(ds, 'masking/rgb'))
                   if n.endswith('.jpg'))
    assert names, 'no rgb images found'
    for n in names:
        src = os.path.join(ds, 'masking/rgb', n)
        flag = Image.open(src).getexif().get(274)
        dst = os.path.join(img_dir, n)
        if not os.path.exists(dst):
            im = Image.open(src).convert('RGB')   # pixels are sensor-native
            if im.size != (WSIZE, HSIZE):
                raise SystemExit(f'{n}: unexpected size {im.size}')
            im.save(dst, quality=97)              # PIL save drops the EXIF tag
        mdst = os.path.join(mask_dir, n[:-4] + '.png')
        if not os.path.exists(mdst):
            m = Image.open(os.path.join(ds, 'masking/masks', n[:-4] + '.png')).convert('L')
            m = to_sensor(m, flag)                # masks ARE display-oriented
            m.point(lambda v: 255 if v > 127 else 0).save(mdst)

    for variant, src in [('prior', 'sparse_tri'), ('refined', 'sparse_refined')]:
        vdir = os.path.join(up, variant, 'sparse', '0')
        os.makedirs(vdir, exist_ok=True)
        sdir = os.path.join(ds, 'colmap', src, '0')
        shutil.copy(os.path.join(sdir, 'cameras.txt'),
                    os.path.join(vdir, 'cameras.txt'))
        # images.txt: scale the translation (t = -R C scales with the world)
        with open(os.path.join(sdir, 'images.txt')) as fi, \
             open(os.path.join(vdir, 'images.txt'), 'w') as fo:
            pose_line = True
            for line in fi:
                t_ = line.strip()
                if not t_ or t_.startswith('#'):
                    fo.write(line)
                    continue
                if pose_line:
                    q = t_.split()
                    for k in (5, 6, 7):
                        q[k] = f'{float(q[k]) * args.scale:.9f}'
                    fo.write(' '.join(q) + '\n')
                else:
                    fo.write(line)
                pose_line = not pose_line
        # points3D.txt: scale xyz
        with open(os.path.join(sdir, 'points3D.txt')) as fi, \
             open(os.path.join(vdir, 'points3D.txt'), 'w') as fo:
            for line in fi:
                if line.startswith('#') or not line.strip():
                    fo.write(line)
                    continue
                q = line.split()
                for k in (1, 2, 3):
                    q[k] = f'{float(q[k]) * args.scale:.9f}'
                fo.write(' '.join(q) + '\n')
        # stale ply from a previous run would silently override the txt
        ply = os.path.join(vdir, 'points3D.ply')
        if os.path.exists(ply):
            os.remove(ply)
        for what in ('images', 'masks'):
            link = os.path.join(up, variant, what)
            if not os.path.islink(link):
                os.symlink('../' + what, link)

    shutil.copy(os.path.join(ds, 'registration.yaml'),
                os.path.join(up, 'registration.yaml'))
    with open(os.path.join(up, 'README_server.md'), 'w') as f:
        f.write(README)

    total = sum(os.path.getsize(os.path.join(r, f))
                for r, _, fs in os.walk(up) for f in fs)
    print(f'prepared {up}: {len(names)} images, ~{total/1e6:.0f} MB')
    print('sync with, e.g.:')
    print(f'  rsync -av --progress {up}/ '
          'rhea:/media/white/nanodrones/roberts.kalvitis/3dgs/popillia/'
          '48img_2608_prior_vs_refined/')


if __name__ == '__main__':
    main()
