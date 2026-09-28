#!/usr/bin/env python3
"""Experiment 2 — SAM prompted from the fitted rig geometry.

No text, no clicks: the prompts come from the rig itself. plot_stacks.py's
fit (rotation axis + specimen point + pose-1-down optical-axis anchor +
working-distance lens offset) plus the calibrated intrinsics project, for
every stack composite:

  - the specimen centre        → positive point prompt + bounding-box prompt
    (a sphere of --object-radius-mm around the specimen)
  - points down the mounting rod (specimen − t·axis, t = --rod-neg-mm)
    → negative point prompts, so the pin is cut away (rod = background)

and hand them to SAM (facebook/sam-vit-*). The mask is cleaned to the
connected component containing the positive point.

Two discrete conventions cannot come from the fit and are set by flags,
resolved once per rig with --scan-roll (draws all four roll candidates on
the first composites; pick the one whose points land on the beetle/rod):

  --roll {0,90,180,270}   in-plane rotation of the camera frame around the
                          optical axis (which phone edge is image-x)
  --orient {cw,ccw}       how the landscape sensor maps to the portrait JPG

    pixi run -e masking mask-sam-geom -- datasets/phone_<ts> \
        [--roll 0 --orient cw] [--scan-roll] [--limit N]

Writes, under <session>/masks/sam_geom/:
    stack_<n>.png       full-res binary mask (255 = insect)
    preview/<n>.jpg     downscaled overlay (background dimmed, contour drawn)
    debug/<n>.jpg       prompt visualisation (box, +point green, −points red)
"""

import argparse
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mask_common import (find_stacks, load_image, overlay_preview, pick_device,
                         save_mask)
from plot_stacks import fit_geometry, load, rotm


def camera_basis(o_b, roll_deg):
    """Rows of R (body→camera): x_c, y_c, z_c=optical axis, in body coords."""
    z = o_b / np.linalg.norm(o_b)
    e = min(np.eye(3), key=lambda v: abs(v @ z))
    u0 = e - (e @ z) * z
    u0 /= np.linalg.norm(u0)
    th = np.radians(roll_deg)
    x = np.cos(th) * u0 + np.sin(th) * np.cross(z, u0)
    y = np.cross(z, x)
    return np.stack([x, y, z])


def project(P_obj, s, R_bc, t_b, K, size_land, orient):
    """Object-frame 3D point → (u, v) in the upright portrait JPG, or None."""
    R_ob = rotm(s['rel_q'])                       # body → object
    lens = s['rel'] + R_ob @ t_b
    p_cam = R_bc @ (R_ob.T @ (P_obj - lens))
    if p_cam[2] <= 1e-4:
        return None, p_cam[2]
    u = K[0, 0] * p_cam[0] / p_cam[2] + K[0, 2]
    v = K[1, 1] * p_cam[1] / p_cam[2] + K[1, 2]
    W, H = size_land                              # 4032, 3024
    if orient == 'cw':                            # sensor → portrait, 90° cw
        up, vp = H - 1 - v, u
    else:                                         # 90° ccw
        up, vp = v, W - 1 - u
    return (up, vp), p_cam[2]


def sharpness_recenter(img, pos, half, search_scale=2.5):
    """Refine the projected centre using the focus-stack physics: the insect
    is the only sharp content (background sits far outside the focus sweep),
    so the peak of local Laplacian energy inside the geometric search window
    is the insect. Returns the refined (u, v)."""
    import cv2
    ds = 4
    g = cv2.cvtColor(np.asarray(img)[::ds, ::ds], cv2.COLOR_RGB2GRAY)
    lap = cv2.Laplacian(g.astype(np.float32), cv2.CV_32F, ksize=3) ** 2
    k = max(9, int(half / ds) | 1)                # ~object-size window
    e = cv2.boxFilter(lap, -1, (k, k))
    h, w = e.shape
    r = int(half * search_scale / ds)
    x0, y0 = int(pos[0] / ds), int(pos[1] / ds)
    win = np.zeros_like(e, dtype=bool)
    win[max(0, y0 - r):min(h, y0 + r), max(0, x0 - r):min(w, x0 + r)] = True
    e[~win] = 0
    yy, xx = np.unravel_index(int(np.argmax(e)), e.shape)
    return float(xx * ds), float(yy * ds)


def keep_component_at(mask, pt):
    """Connected component of `mask` containing pt (fallback: largest)."""
    import cv2
    n, lab = cv2.connectedComponents(mask.astype(np.uint8))
    if n <= 2:
        return mask
    x, y = int(round(pt[0])), int(round(pt[1]))
    h, w = mask.shape
    tgt = lab[min(max(y, 0), h - 1), min(max(x, 0), w - 1)]
    if tgt == 0:                                  # +point missed the mask
        areas = [(lab == i).sum() for i in range(1, n)]
        tgt = 1 + int(np.argmax(areas))
    return (lab == tgt)


def draw_debug(img, box, pos, negs, out_path, extra=None, max_side=1000):
    import cv2
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    sc = max_side / max(img.size)
    rgb = np.asarray(img.resize((round(img.size[0] * sc),
                                 round(img.size[1] * sc)))).copy()
    if box is not None:
        x0, y0, x1, y1 = (np.array(box) * sc).astype(int)
        cv2.rectangle(rgb, (x0, y0), (x1, y1), (255, 220, 40), 3)
    if pos is not None:
        cv2.circle(rgb, tuple((np.array(pos) * sc).astype(int)), 10,
                   (40, 255, 40), -1)
    for p in negs:
        cv2.circle(rgb, tuple((np.array(p) * sc).astype(int)), 8,
                   (255, 50, 50), -1)
    for label, p, color in (extra or []):
        q = tuple((np.array(p) * sc).astype(int))
        cv2.circle(rgb, q, 9, color, 2)
        cv2.putText(rgb, label, (q[0] + 10, q[1]), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, color, 2)
    from PIL import Image
    Image.fromarray(rgb).save(out_path, quality=88)


def main():
    ap_ = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap_.add_argument('session')
    ap_.add_argument('--intrinsics', default='calibration/intrinsics.yaml')
    ap_.add_argument('--model', default='facebook/sam-vit-large')
    ap_.add_argument('--working-distance', type=float, default=25.0,
                     metavar='MM', help='pose-1 lens height (default 25)')
    ap_.add_argument('--object-radius-mm', type=float, default=8.0,
                     help='prompt-box sphere radius; the beetle is ~10 mm '
                          'long (default 8)')
    ap_.add_argument('--crop-scale', type=float, default=1.8,
                     help='SAM runs on a crop of this × the prompt-box '
                          'half-size around the projection, so the insect '
                          'fills the model input (default 1.8)')
    ap_.add_argument('--rod-neg-mm', default='22,32,45,70,100',
                     help='distances below the specimen (along −axis) for '
                          'negative rod/mount points; the far ones land on '
                          'the mount hub in top-down views '
                          '(default 22,32,45,70,100)')
    ap_.add_argument('--hub-ring-mm', default='28,100',
                     help='"radius,depth" of the mount hub\'s white ring '
                          'below the specimen — 8 negative points are laid '
                          'on it (default 28,100; empty string disables)')
    ap_.add_argument('--roll', type=float, default=0.0,
                     choices=(0.0, 90.0, 180.0, 270.0))
    ap_.add_argument('--no-refocus', action='store_true',
                     help='disable the sharpness-peak recentring of the '
                          'projected specimen point (pure geometry)')
    ap_.add_argument('--orient', choices=('cw', 'ccw'), default='cw')
    ap_.add_argument('--stacks-per-block', type=int, default=8)
    ap_.add_argument('--out', default=None)
    ap_.add_argument('--limit', type=int, default=None)
    ap_.add_argument('--scan-roll', action='store_true',
                     help='no SAM: draw all four roll candidates on the '
                          'first composites and exit')
    args = ap_.parse_args()

    out_dir = args.out or os.path.join(args.session.rstrip('/'),
                                       'masks', 'sam_geom')
    stacks = load(args.session)
    spb = args.stacks_per_block
    ap, ad, spec, o_b, _ = fit_geometry(stacks, spb)

    # lens offset in the body frame — same pose-1 anchor as plot_stacks
    wd = args.working_distance * 1e-3
    pose1 = [s for s in stacks if (s['n'] - 1) % spb == 0]
    t_b = np.mean([rotm(s['rel_q']).T @ (spec + wd * ad - s['rel'])
                   for s in pose1], axis=0)
    print(f'lens offset (body frame): {np.round(t_b * 1e3, 1)} mm')

    with open(args.intrinsics) as f:
        calib = yaml.safe_load(f)
    K = np.array(calib['camera_matrix'])
    size_land = calib['image_size']               # [4032, 3024]

    by_n = {s['n']: s for s in stacks}
    composites = find_stacks(args.session)
    if args.limit:
        composites = composites[:args.limit]

    rod_ts = [float(t) * 1e-3 for t in args.rod_neg_mm.split(',')]

    if args.scan_roll:
        colors = {0: (40, 255, 40), 90: (255, 220, 40),
                  180: (60, 160, 255), 270: (255, 60, 255)}
        for n, path in composites[:4]:
            img, s = load_image(path), by_n[n]
            extra = []
            for r, col in colors.items():
                R_bc = camera_basis(o_b, r)
                pt, _ = project(spec, s, R_bc, t_b, K, size_land, args.orient)
                if pt is not None:
                    extra.append((f'r{r}', pt, col))
                rp, _ = project(spec - rod_ts[-1] * ad, s, R_bc, t_b, K,
                                size_land, args.orient)
                if rp is not None:
                    extra.append((f'r{r}rod', rp, col))
            draw_debug(img, None, None, [],
                       os.path.join(out_dir, 'debug', f'scan_{n:02d}.jpg'),
                       extra=extra)
            print(f'stack {n}: → {out_dir}/debug/scan_{n:02d}.jpg')
        return

    import torch
    from transformers import SamModel, SamProcessor
    device = pick_device()
    print(f'loading {args.model} on {device} …')
    processor = SamProcessor.from_pretrained(args.model)
    model = SamModel.from_pretrained(args.model).to(device).eval()

    R_bc = camera_basis(o_b, args.roll)
    for n, path in composites:
        img, s = load_image(path), by_n[n]
        W, H = img.size

        pos, z = project(spec, s, R_bc, t_b, K, size_land, args.orient)
        if pos is None:
            print(f'stack {n:2d}: specimen behind camera?! skipped')
            continue
        half = K[0, 0] * args.object_radius_mm * 1e-3 / z * 1.1
        raw_pos = pos
        if not args.no_refocus:
            pos = sharpness_recenter(img, pos, half)
        shift_uv = np.array(pos) - np.array(raw_pos)
        box = [max(0, pos[0] - half), max(0, pos[1] - half),
               min(W - 1, pos[0] + half), min(H - 1, pos[1] + half)]
        negs = []
        neg_pts_3d = [spec - t * ad for t in rod_ts]
        # the mount hub's white ring: a circle around the axis at hub depth
        if args.hub_ring_mm:
            r_mm, d_mm = (float(v) for v in args.hub_ring_mm.split(','))
            e1 = np.array([1.0, 0, 0]) - ad[0] * ad
            e1 /= np.linalg.norm(e1)
            e2 = np.cross(ad, e1)
            for a in np.linspace(0, 2 * np.pi, 8, endpoint=False):
                neg_pts_3d.append(spec - d_mm * 1e-3 * ad
                                  + r_mm * 1e-3 * (np.cos(a) * e1
                                                   + np.sin(a) * e2))
        # occlusion test: a negative hidden behind the beetle would project
        # ONTO it and poison the prompt — drop rays passing within 6 mm of
        # the specimen centre that are farther away than it
        R_ob = rotm(s['rel_q'])
        lens = s['rel'] + R_ob @ t_b
        spec_c = R_bc @ (R_ob.T @ (spec - lens))
        for P in neg_pts_3d:
            pc = R_bc @ (R_ob.T @ (P - lens))
            if pc[2] <= 1e-4:
                continue
            u = pc / np.linalg.norm(pc)
            if (np.linalg.norm(np.cross(spec_c, u)) < 6e-3
                    and np.linalg.norm(pc) > np.linalg.norm(spec_c)):
                continue
            p, _ = project(P, s, R_bc, t_b, K, size_land, args.orient)
            if p is None:
                continue
            # the projection error is a common image-space drift — apply the
            # sharpness-refined shift to the negatives too
            p = (p[0] + shift_uv[0], p[1] + shift_uv[1])
            if 0 <= p[0] < W and 0 <= p[1] < H:
                negs.append([float(p[0]), float(p[1])])

        # SAM sees a crop centred on the projection (the beetle is ~1–2 % of
        # the frame; full-frame SAM wastes its 1024 px input on background)
        ch = half * args.crop_scale
        cx0 = int(max(0, min(pos[0] - ch, W - 2 * ch)))
        cy0 = int(max(0, min(pos[1] - ch, H - 2 * ch)))
        cx1, cy1 = int(min(W, cx0 + 2 * ch)), int(min(H, cy0 + 2 * ch))
        crop = img.crop((cx0, cy0, cx1, cy1))
        cbox = [box[0] - cx0, box[1] - cy0, box[2] - cx0, box[3] - cy0]
        cpts = [[pos[0] - cx0, pos[1] - cy0]] + \
               [[p[0] - cx0, p[1] - cy0] for p in negs
                if cx0 <= p[0] < cx1 and cy0 <= p[1] < cy1]
        labels = [1] + [0] * (len(cpts) - 1)

        inputs = processor(crop, input_boxes=[[cbox]],
                           input_points=[[cpts]], input_labels=[[labels]],
                           return_tensors='pt')
        # MPS has no float64 — the processor emits points/boxes as f64
        inputs = {k: (v.to(torch.float32) if v.dtype == torch.float64 else v)
                  .to(device) for k, v in inputs.items()}
        with torch.no_grad():
            out = model(**inputs)
        cands = processor.image_processor.post_process_masks(
            out.pred_masks.cpu(), inputs['original_sizes'].cpu(),
            inputs['reshaped_input_sizes'].cpu())[0][0]     # (3, h, w) bool
        scores = out.iou_scores[0, 0].cpu().numpy()

        # candidate choice. Three priors: the projected area is predictable
        # from the geometry; a candidate must not swallow known-background
        # (negative) points; and — the focus-stack physics again — the
        # right candidate is SHARP inside (background is defocused mush)
        import cv2
        a_exp = np.pi * (K[0, 0] * args.object_radius_mm * 1e-3 / (2 * z))**2
        areas = np.array([float(c.sum()) for c in cands])
        nneg = np.array([sum(bool(c[min(int(p[1]), c.shape[0] - 1),
                                    min(int(p[0]), c.shape[1] - 1)])
                             for p in cpts[1:]) for c in cands])
        gray = cv2.cvtColor(np.asarray(crop)[::4, ::4], cv2.COLOR_RGB2GRAY)
        lap = cv2.Laplacian(gray.astype(np.float32), cv2.CV_32F, ksize=3)**2
        sharp = np.array([
            float(lap[cv2.resize(c.numpy().astype(np.uint8), lap.shape[::-1])
                      > 0].mean()) if c.any() else 0.0 for c in cands])
        ok = (areas > 0.15 * a_exp) & (areas < 6 * a_exp)
        rank = (sharp / (sharp.max() + 1e-9) + 0.5 * scores - 1.0 * nneg
                - np.where(ok, 0.0, 10.0))
        best = (int(np.argmax(rank)) if (rank > -5).any()
                else int(np.argmin(np.abs(np.log(areas / a_exp)))))

        # negatives are certain background: punch them out before taking the
        # connected component, so a thin rod bridge can't drag the mount in
        cm8 = cands[best].numpy().astype(np.uint8)
        r_px = max(8, int(K[0, 0] * 1.5e-3 / z))            # ~1.5 mm
        for p in cpts[1:]:
            cv2.circle(cm8, (int(p[0]), int(p[1])), r_px, 0, -1)
        cm = keep_component_at(cm8, (pos[0] - cx0, pos[1] - cy0))
        m = np.zeros((H, W), dtype=np.float32)
        m[cy0:cy1, cx0:cx1] = cm.astype(np.float32)

        save_mask(m, os.path.join(out_dir, f'stack_{n:02d}.png'))
        overlay_preview(img, m, os.path.join(out_dir, 'preview',
                                             f'{n:02d}.jpg'))
        draw_debug(img, box, pos, negs,
                   os.path.join(out_dir, 'debug', f'{n:02d}.jpg'),
                   extra=[('geom', raw_pos, (210, 210, 210))])
        print(f'stack {n:2d}: z {z * 1e3:5.1f} mm, '
              f'refocus {np.hypot(*shift_uv):4.0f} px, '
              f'{len(negs)} neg points, iou {scores[best]:.3f}, '
              f'mask {m.mean() * 100:5.2f}% of frame')

    print('done.')


if __name__ == '__main__':
    main()
