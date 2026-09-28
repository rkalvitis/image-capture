#!/usr/bin/env python3
"""Apply background masks to a stack session's composites.

Renders each focus-stack composite with the background replaced (white by
default) and the insect kept in place at full resolution. The mask edge is
feathered a few pixels so the cutout blends instead of aliasing.

    pixi run -e masking apply-masks -- datasets/phone_<ts> \
        [--masks masks/hybrid_text_sam] [--background white|black|alpha] \
        [--feather 3] [--limit N]

Writes <session>/masked/stack_<n>.jpg (PNG with real alpha when
--background alpha — that variant is what a GS trainer with an alpha loss
should consume) and a masked_gallery.html contact sheet next to them.
"""

import argparse
import base64
import html
import io
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mask_common import find_stacks, load_image, load_mask

BG = {'white': (255, 255, 255), 'black': (0, 0, 0)}


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session')
    ap.add_argument('--masks', default='masks/hybrid_text_sam',
                    help='mask dir under the session '
                         '(default masks/hybrid_text_sam — the best set)')
    ap.add_argument('--background', default='white',
                    choices=('white', 'black', 'alpha'))
    ap.add_argument('--feather', type=float, default=3.0,
                    help='edge feather sigma in px at full res (default 3)')
    ap.add_argument('--out', default='masked')
    ap.add_argument('--quality', type=int, default=95)
    ap.add_argument('--limit', type=int, default=None)
    args = ap.parse_args()

    import cv2
    from PIL import Image
    session = args.session.rstrip('/')
    out_dir = os.path.join(session, args.out)
    os.makedirs(out_dir, exist_ok=True)
    composites = find_stacks(session)
    if args.limit:
        composites = composites[:args.limit]

    thumbs = []
    for n, path in composites:
        mp = os.path.join(session, args.masks, f'stack_{n:02d}.png')
        if not os.path.exists(mp):
            print(f'stack {n:2d}: no mask, skipped')
            continue
        img = np.asarray(load_image(path)).astype(np.float32)
        m = load_mask(mp)
        if args.feather > 0:
            k = int(args.feather * 3) | 1
            m = cv2.GaussianBlur(m, (k, k), args.feather)
        a = np.clip(m, 0, 1)[..., None]

        if args.background == 'alpha':
            out = np.dstack([img, a[..., 0] * 255]).astype(np.uint8)
            fn = os.path.join(out_dir, f'stack_{n:02d}.png')
            Image.fromarray(out, 'RGBA').save(fn)
        else:
            bg = np.array(BG[args.background], dtype=np.float32)
            out = (img * a + bg * (1 - a)).astype(np.uint8)
            fn = os.path.join(out_dir, f'stack_{n:02d}.jpg')
            Image.fromarray(out).save(fn, quality=args.quality)

        thumb = out[..., :3] if out.shape[-1] == 4 else out
        sc = 360 / max(thumb.shape[:2])
        th = cv2.resize(thumb, (round(thumb.shape[1] * sc),
                                round(thumb.shape[0] * sc)))
        buf = io.BytesIO()
        Image.fromarray(th).save(buf, format='JPEG', quality=70)
        thumbs.append((n, base64.b64encode(buf.getvalue()).decode()))
        print(f'stack {n:2d}: → {fn}')

    gallery = os.path.join(out_dir, 'masked_gallery.html')
    cells = ''.join(
        f'<figure><img loading="lazy" alt="stack {n}" '
        f'src="data:image/jpeg;base64,{b}"><figcaption>stack {n}'
        '</figcaption></figure>' for n, b in thumbs)
    with open(gallery, 'w') as f:
        f.write(
            '<title>Beetle Cutout Gallery</title><style>'
            '*{margin:0;padding:0;box-sizing:border-box}'
            'body{background:#f4f4f2;color:#2a2c30;font:15px/1.5 '
            '"Avenir Next","Segoe UI",system-ui,sans-serif;'
            'padding:2.2rem clamp(1rem,4vw,3rem)}'
            'h1{font-size:1.5rem;font-weight:600}'
            'p{color:#71757c;margin:.3rem 0 1.4rem;max-width:65ch}'
            '.grid{display:grid;gap:.8rem;'
            'grid-template-columns:repeat(auto-fill,minmax(11rem,1fr))}'
            'figure{background:#fff;border:1px solid #e0e0dc;'
            'border-radius:5px;padding:.4rem}'
            'img{width:100%;display:block;border-radius:3px}'
            'figcaption{font-size:.75rem;color:#71757c;margin-top:.3rem;'
            'text-transform:uppercase;letter-spacing:.05em}</style>'
            f'<h1>Beetle Cutout Gallery</h1><p>{html.escape(session)} — '
            f'{len(thumbs)} composites over {args.background} background, '
            f'masks: {html.escape(args.masks)}, feather '
            f'{args.feather:g} px.</p><div class="grid">'
            + cells + '</div>')
    print(f'\n{len(thumbs)} images → {out_dir}\ngallery → {gallery}')


if __name__ == '__main__':
    main()
