#!/usr/bin/env python3
"""Experiment 1 — off-the-shelf background removal with BiRefNet.

Masks the insect in every focus-stack composite of a session using
ZhengPeng7/BiRefNet (high-resolution dichotomous segmentation) with NO
prompts and NO knowledge of the rig — the pure "just run a salient-object
model" baseline. Compare against scripts/mask_sam_geom.py (experiment 2)
with scripts/compare_masks.py.

    pixi run -e masking mask-birefnet -- datasets/phone_<ts> [--limit N]

Writes, under <session>/masks/birefnet/:
    stack_<n>.png       full-res 8-bit soft mask (255 = insect)
    preview/<n>.jpg     downscaled overlay (background dimmed, contour drawn)
"""

import argparse
import os
import sys
import time

import numpy as np
import torch
from torchvision import transforms

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mask_common import (find_stacks, load_image, overlay_preview, pick_device,
                         save_mask)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session', help='session folder (has stacks.csv, images/)')
    ap.add_argument('--model', default='ZhengPeng7/BiRefNet',
                    help='HF model id (default ZhengPeng7/BiRefNet)')
    ap.add_argument('--res', type=int, default=1024,
                    help='inference resolution (square, default 1024)')
    ap.add_argument('--out', default=None,
                    help='output dir (default <session>/masks/birefnet)')
    ap.add_argument('--limit', type=int, default=None,
                    help='only process the first N stacks (for testing)')
    args = ap.parse_args()

    out_dir = args.out or os.path.join(args.session.rstrip('/'),
                                       'masks', 'birefnet')
    stacks = find_stacks(args.session)
    if args.limit:
        stacks = stacks[:args.limit]
    print(f'{len(stacks)} composites → {out_dir}')

    from transformers import AutoModelForImageSegmentation
    device = pick_device()
    print(f'loading {args.model} on {device} …')
    model = AutoModelForImageSegmentation.from_pretrained(
        args.model, trust_remote_code=True)
    # the remote code half-precisions some weights; keep everything f32
    model.to(device).float().eval()

    prep = transforms.Compose([
        transforms.Resize((args.res, args.res)),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])

    for n, path in stacks:
        t0 = time.time()
        img = load_image(path)
        x = prep(img).unsqueeze(0).to(device)
        with torch.no_grad():
            pred = model(x)[-1].sigmoid().cpu()          # (1, 1, res, res)
        m = transforms.functional.resize(pred[0], (img.size[1], img.size[0]),
                                         antialias=True)[0].numpy()
        save_mask(m, os.path.join(out_dir, f'stack_{n:02d}.png'))
        overlay_preview(img, m, os.path.join(out_dir, 'preview', f'{n:02d}.jpg'))
        frac = float((m >= 0.5).mean())
        print(f'stack {n:2d}: mask {frac * 100:5.2f}% of frame '
              f'({time.time() - t0:.1f} s)')

    print('done.')


if __name__ == '__main__':
    main()
