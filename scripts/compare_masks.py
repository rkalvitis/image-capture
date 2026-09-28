#!/usr/bin/env python3
"""Compare N background-mask sets of a stack session.

Computes per-stack agreement metrics between the masking experiments —
there is no ground truth, so the numbers measure agreement, not accuracy —
and writes:

    <session>/masks/comparison.csv    per-stack metrics
    <session>/masks/report.html      self-contained visual report
                                      (composite · one overlay per method ·
                                       consensus map)

    pixi run -e masking compare-masks -- datasets/phone_<ts> \
        [--dirs masks/birefnet masks/sam_geom masks/textprompt] \
        [--names BiRefNet SAM+geometry "OWLv2+SAM (text)"]

In the consensus map, full brightness = every method keeps the pixel;
tinted = only a subset does (tint mixes the claiming methods' colours).
"""

import argparse
import base64
import csv
import html
import io
import itertools
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mask_common import find_stacks, load_image, load_mask

# report palette (dark light-table; method hues double as map encoding)
C_A = '#d08a52'          # copper — the beetle's elytra
C_B = '#86b06b'          # green — the beetle's head
C_C = '#6f9fc8'          # steel blue
C_D = '#b98bc9'          # lilac
C_E = '#c9b25e'          # brass
COLORS = [C_A, C_B, C_C, C_D, C_E]


def b64_jpeg(arr, quality=72):
    from PIL import Image
    buf = io.BytesIO()
    Image.fromarray(arr).save(buf, format='JPEG', quality=quality)
    return base64.b64encode(buf.getvalue()).decode()


def hex_rgb(h):
    return np.array([int(h[i:i + 2], 16) for i in (1, 3, 5)], dtype=np.float32)


def panels(img, masks, colors, max_side=440):
    """Downscaled (composite, overlay per mask, consensus map) uint8."""
    import cv2
    sc = max_side / max(img.size)
    small = np.asarray(img.resize((round(img.size[0] * sc),
                                   round(img.size[1] * sc)))).astype(np.float32)
    bins = [cv2.resize(m, (small.shape[1], small.shape[0])) >= 0.5
            for m in masks]

    def overlay(m, color):
        out = small * (0.22 + 0.78 * m[..., None])
        cont, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_LIST,
                                   cv2.CHAIN_APPROX_SIMPLE)
        out = out.astype(np.uint8).copy()
        cv2.drawContours(out, cont, -1, tuple(int(v) for v in hex_rgb(color)), 2)
        return out

    count = np.sum(bins, axis=0)
    consensus = small * 0.22
    full = count == len(bins)
    consensus[full] = small[full]
    part = (count > 0) & ~full
    if part.any():
        mix = np.zeros((*count.shape, 3), np.float32)
        for m, c in zip(bins, colors):
            mix[m] += hex_rgb(c)
        mix[part] /= count[part][..., None]
        consensus[part] = 0.45 * small[part] + 0.55 * mix[part]
    return ([small.astype(np.uint8)]
            + [overlay(b, c) for b, c in zip(bins, colors)]
            + [consensus.astype(np.uint8)])


def metrics(masks):
    bins = [m >= 0.5 for m in masks]
    inter_all = np.logical_and.reduce(bins).sum()
    union_all = np.logical_or.reduce(bins).sum()
    out = {'consensus_iou': inter_all / union_all if union_all
           else float('nan')}
    for i, b in enumerate(bins):
        out[f'area_{i}_pct'] = 100 * b.sum() / b.size
    for i, j in itertools.combinations(range(len(bins)), 2):
        u = (bins[i] | bins[j]).sum()
        out[f'iou_{i}{j}'] = (bins[i] & bins[j]).sum() / u if u \
            else float('nan')
    return out


CSS = """
:root { --ink:#14161a; --panel:#1c2025; --line:#2c3138; --text:#e9e6df;
  --muted:#9aa0a8; --warn:#e0b352; }
* { margin:0; padding:0; box-sizing:border-box; }
body { background:var(--ink); color:var(--text); font:15px/1.55 "Avenir Next",
  "Segoe UI", system-ui, sans-serif; padding:2.5rem clamp(1rem,4vw,3rem); }
.mono { font-family:"SF Mono", ui-monospace, Menlo, monospace;
  font-variant-numeric:tabular-nums; }
header { max-width:74rem; margin:0 auto 2.2rem; }
h1 { font-size:1.7rem; font-weight:600; letter-spacing:.01em;
  text-wrap:balance; }
.sub { color:var(--muted); margin-top:.3rem; max-width:65ch; }
.methods { display:flex; gap:1.6rem; margin-top:1rem; flex-wrap:wrap; }
.methods span { display:inline-flex; align-items:center; gap:.45em;
  font-size:.88rem; color:var(--muted); }
.dot { width:.7em; height:.7em; border-radius:50%; display:inline-block; }
.stats { display:flex; gap:2.2rem; margin-top:1.3rem; flex-wrap:wrap; }
.stat b { display:block; font-size:1.45rem; font-weight:600; }
.stat span { font-size:.78rem; text-transform:uppercase;
  letter-spacing:.07em; color:var(--muted); }
main { max-width:74rem; margin:0 auto; display:flex; flex-direction:column;
  gap:1.1rem; }
.row { background:var(--panel); border:1px solid var(--line);
  border-radius:6px; padding:.9rem 1rem; }
.row.flag { border-color:var(--warn); }
.rowhead { display:flex; gap:1.4rem; align-items:baseline; flex-wrap:wrap;
  margin-bottom:.7rem; }
.rowhead .id { font-weight:600; font-size:1.02rem; }
.rowhead .m { color:var(--muted); font-size:.85rem; }
.rowhead .m b { color:var(--text); font-weight:500; }
.flagnote { color:var(--warn); font-size:.85rem; }
.imgs { display:flex; gap:.6rem; overflow-x:auto; }
figure { flex:1 1 0; min-width:8rem; }
figure img { width:100%; display:block; border-radius:3px; }
figcaption { font-size:.74rem; color:var(--muted); margin-top:.3rem;
  text-transform:uppercase; letter-spacing:.06em; }
footer { max-width:74rem; margin:2.2rem auto 0; color:var(--muted);
  font-size:.83rem; }
"""


def build_report(session, rows, names, colors, out_html):
    cons = np.array([r['consensus_iou'] for r in rows])
    flags = sum(c < 0.7 for c in cons)
    pair_stats = ''.join(
        f'<div class="stat"><b class="mono">'
        f'{np.nanmean([r[f"iou_{i}{j}"] for r in rows]):.3f}</b>'
        f'<span>mean IoU {chr(65 + i)}·{chr(65 + j)}</span></div>'
        for i, j in itertools.combinations(range(len(names)), 2))
    parts = [
        '<title>Beetle Mask Face-off</title>',
        f'<style>{CSS}</style>',
        '<header>',
        '<h1>Beetle Mask Face-off</h1>',
        f'<p class="sub">Background masks for {html.escape(session)} — '
        f'{len(rows)} focus-stack composites, {len(names)} methods, no '
        'ground truth: the maps show where the methods disagree. In the '
        'consensus panel, full brightness = every method keeps the pixel; '
        'tinted = only a subset does (tint mixes the claiming methods\' '
        'colours).</p>',
        '<div class="methods">' + ''.join(
            f'<span><i class="dot" style="background:{c}"></i>'
            f'{chr(65 + i)} · {html.escape(nm)}</span>'
            for i, (nm, c) in enumerate(zip(names, colors))) + '</div>',
        '<div class="stats">'
        f'<div class="stat"><b class="mono">{np.nanmean(cons):.3f}</b>'
        '<span>mean consensus IoU</span></div>'
        f'{pair_stats}'
        f'<div class="stat"><b class="mono">{flags}</b>'
        '<span>stacks below 0.7</span></div>'
        '</div></header><main>',
    ]
    for r in rows:
        flag = ' flag' if r['consensus_iou'] < 0.7 else ''
        note = ('<span class="flagnote">low agreement — inspect</span>'
                if flag else '')
        pair_m = ''.join(
            f'<span class="m">{chr(65 + i)}·{chr(65 + j)} <b class="mono">'
            f'{r[f"iou_{i}{j}"]:.3f}</b></span>'
            for i, j in itertools.combinations(range(len(names)), 2))
        area_m = ' · '.join(
            f'{chr(65 + i)} <b class="mono">{r[f"area_{i}_pct"]:.2f}%</b>'
            for i in range(len(names)))
        caps = ['composite'] + [f'{chr(65 + i)} · {html.escape(nm)}'
                                for i, nm in enumerate(names)] + ['consensus']
        parts.append(
            f'<section class="row{flag}"><div class="rowhead">'
            f'<span class="id">Stack {r["stack"]}</span>'
            f'<span class="m">consensus <b class="mono">'
            f'{r["consensus_iou"]:.3f}</b></span>'
            f'{pair_m}<span class="m">area {area_m}</span>{note}</div>'
            '<div class="imgs">' +
            ''.join(f'<figure><img loading="lazy" alt="{cap}" '
                    f'src="data:image/jpeg;base64,{b64}">'
                    f'<figcaption>{cap}</figcaption></figure>'
                    for cap, b64 in zip(caps, r['panels'])) +
            '</div></section>')
    parts.append(
        '</main><footer>Generated by scripts/compare_masks.py · masks '
        'binarised at 0.5 · consensus IoU = intersection / union across all '
        'methods.</footer>')
    with open(out_html, 'w') as f:
        f.write('\n'.join(parts))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session')
    ap.add_argument('--dirs', nargs='+',
                    default=['masks/birefnet', 'masks/sam_geom',
                             'masks/textprompt'])
    ap.add_argument('--names', nargs='+',
                    default=['BiRefNet', 'SAM+geometry', 'OWLv2+SAM (text)'])
    args = ap.parse_args()
    if len(args.dirs) != len(args.names):
        raise SystemExit('--dirs and --names must have the same length')
    if len(args.dirs) > len(COLORS):
        raise SystemExit(f'at most {len(COLORS)} methods supported')
    colors = COLORS[:len(args.dirs)]

    session = args.session.rstrip('/')
    rows = []
    for n, path in find_stacks(session):
        paths = [os.path.join(session, d, f'stack_{n:02d}.png')
                 for d in args.dirs]
        if not all(os.path.exists(p) for p in paths):
            print(f'stack {n:2d}: missing mask, skipped')
            continue
        masks = [load_mask(p) for p in paths]
        m = metrics(masks)
        img = load_image(path)
        rows.append({'stack': n, **m,
                     'panels': [b64_jpeg(p)
                                for p in panels(img, masks, colors)]})
        print(f'stack {n:2d}: consensus {m["consensus_iou"]:.3f}  ' +
              '  '.join(f'{chr(65 + i)} {m[f"area_{i}_pct"]:5.2f}%'
                        for i in range(len(masks))))

    csv_path = os.path.join(session, 'masks', 'comparison.csv')
    fields = [k for k in rows[0] if k != 'panels']
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)

    out_html = os.path.join(session, 'masks', 'report.html')
    build_report(session, rows, args.names, colors, out_html)
    cons = [r['consensus_iou'] for r in rows]
    print(f'\n{len(rows)} stacks · mean consensus IoU {np.nanmean(cons):.3f}'
          f' · median {np.nanmedian(cons):.3f} · min {np.nanmin(cons):.3f}')
    print(f'→ {csv_path}\n→ {out_html}')


if __name__ == '__main__':
    main()
