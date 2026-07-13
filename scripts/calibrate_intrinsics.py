#!/usr/bin/env python3
"""Camera intrinsics calibration (Zhang's method) from ChArUco board photos.

    pixi run calibrate-intrinsics -- <image_folder> --square-mm 7.98
    python scripts/calibrate_intrinsics.py --self-test

Input: a folder of photos of the board (from scripts/make_charuco_board.py)
taken at the SESSION ANCHOR FOCUS with the locked Reeflex configuration —
intrinsics are only valid for the exact lens/zoom/focus they were shot at.
~30–50 shots: near/far within the working band, tilts up to ~35°, board
pushed into every corner of the frame.

Formats: .jpg/.png read directly; .dng ProRAW is supported by extracting
the embedded full-size JPEG preview (identical pixel geometry to the raw).

--square-mm must be the CALIPER-MEASURED printed square size (printers
rescale); the marker size is scaled proportionally from the nominal ratio.

Output: a YAML file (default calibration/intrinsics.yaml) with K, distortion
coefficients, image size, RMS reprojection error, and the board spec —
consumed by calibrate_handeye.py.

Self-test: renders the board into ~15 synthetic views with a known camera
matrix, runs the full detection + calibration path, and checks the recovered
focal length / principal point / distortion against ground truth.
"""

import argparse
import mmap
import os
import struct
import sys
from datetime import datetime
from glob import glob

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from match_photos import _ifd_entries  # stdlib TIFF walk (DNG preview extraction)

DEFAULT_BOARD = dict(squares_x=8, squares_y=6, square_mm=8.0, marker_mm=6.0,
                     dictionary='DICT_5X5_100')


def make_board(spec):
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, spec['dictionary']))
    return cv2.aruco.CharucoBoard(
        (spec['squares_x'], spec['squares_y']),
        spec['square_mm'] / 1000.0, spec['marker_mm'] / 1000.0, dictionary)


def load_image(path):
    """cv2 image from .jpg/.png directly, or the embedded preview of a .dng."""
    if not path.lower().endswith('.dng'):
        img = cv2.imread(path)
        if img is None:
            raise ValueError(f'{path}: unreadable')
        return img
    with open(path, 'rb') as f:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
    try:
        bo = '<' if mm[:2] == b'II' else '>'
        (ifd0,) = struct.unpack_from(bo + 'I', mm, 4)
        off = cnt = None
        for tag, typ, c, voff in _ifd_entries(mm, bo, ifd0):
            if tag == 273:                      # StripOffsets (preview JPEG)
                off = struct.unpack_from(bo + 'I', mm, voff)[0]
            elif tag == 279:                    # StripByteCounts
                cnt = struct.unpack_from(bo + 'I', mm, voff)[0]
        if off is None or cnt is None:
            raise ValueError(f'{path}: no embedded preview found')
        img = cv2.imdecode(np.frombuffer(mm[off:off + cnt], np.uint8),
                           cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError(f'{path}: preview decode failed')
        return img
    finally:
        mm.close()


def detect_board(img, board, min_corners=12):
    """Return (obj_points, img_points) for one image, or None."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    detector = cv2.aruco.CharucoDetector(board)
    ch_corners, ch_ids, _, _ = detector.detectBoard(gray)
    if ch_ids is None or len(ch_ids) < min_corners:
        return None
    obj, imgp = board.matchImagePoints(ch_corners, ch_ids)
    return obj, imgp


def calibrate(images, board, min_corners=12, verbose=True, flags=None):
    obj_all, img_all, used, size = [], [], [], None
    for name, img in images:
        if size is None:
            size = (img.shape[1], img.shape[0])
        elif (img.shape[1], img.shape[0]) != size:
            raise SystemExit(f'{name}: image size differs from the rest — '
                             'one resolution per calibration')
        det = detect_board(img, board, min_corners)
        if det is None:
            if verbose:
                print(f'  SKIP {name}: board not detected / too few corners')
            continue
        obj_all.append(det[0])
        img_all.append(det[1])
        used.append(name)
        if verbose:
            print(f'  ok   {name}: {len(det[1])} corners')
    if len(obj_all) < 8:
        raise SystemExit(f'only {len(obj_all)} usable images — need >= 8 '
                         '(aim for 30+)')
    # k3 fixed to 0 by default: iPhone output is already distortion-corrected
    # on-device, so the residual is small and an unconstrained k3 overfits.
    if flags is None:
        flags = cv2.CALIB_FIX_K3
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_all, img_all, size, None, None, flags=flags)
    per_view = []
    for o, i, r, t in zip(obj_all, img_all, rvecs, tvecs):
        proj, _ = cv2.projectPoints(o, r, t, K, dist)
        per_view.append(float(np.sqrt(np.mean((proj - i) ** 2))))
    return dict(rms=float(rms), K=K, dist=dist, size=size,
                per_view=per_view, used=used)


def save_yaml(result, board_spec, out, source):
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    data = {
        'date': datetime.now().isoformat(timespec='seconds'),
        'source': source,
        'image_size': list(result['size']),
        'camera_matrix': [[float(v) for v in row] for row in result['K']],
        'distortion': [float(v) for v in result['dist'].ravel()],
        'rms_px': result['rms'],
        'n_images': len(result['used']),
        'board': board_spec,
    }
    with open(out, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    print(f'\nwrote {out}')


def self_test():
    board_spec = dict(DEFAULT_BOARD)
    board = make_board(board_spec)
    px_per_m = 600 / 25.4 * 1000
    w_px = round(board_spec['squares_x'] * board_spec['square_mm'] / 1000 * px_per_m)
    h_px = round(board_spec['squares_y'] * board_spec['square_mm'] / 1000 * px_per_m)
    pattern = board.generateImage((w_px, h_px), marginSize=0)

    W, H = 1920, 1440
    K_true = np.array([[1500.0, 0, W / 2 - 7], [0, 1500.0, H / 2 + 4], [0, 0, 1]])
    rng = np.random.default_rng(1)

    images = []
    for i in range(15):
        rvec = np.deg2rad(rng.uniform(-25, 25, 3))
        rvec[2] = np.deg2rad(rng.uniform(-40, 40))
        R, _ = cv2.Rodrigues(rvec)
        t = np.array([rng.uniform(-0.035, 0.035), rng.uniform(-0.025, 0.025),
                      rng.uniform(0.22, 0.38)])
        # center the board (origin at its corner) in front of the camera
        t[:2] -= (R[:2, 0] * board_spec['squares_x'] +
                  R[:2, 1] * board_spec['squares_y']) * board_spec['square_mm'] / 2000
        H_plane = K_true @ np.column_stack([R[:, 0], R[:, 1], t])
        S = np.diag([1 / px_per_m, 1 / px_per_m, 1.0])     # board px -> meters
        warped = cv2.warpPerspective(pattern, H_plane @ S, (W, H),
                                     borderValue=255)
        images.append((f'synthetic_{i:02d}', warped))

    # ground truth has zero distortion (pure homography warp), so calibrate
    # with the distortion model matched to it — this makes the self-test a
    # clean test of detection + geometry conventions, without the pp<->k
    # correlation that under-covered synthetic views cannot constrain.
    zero_dist = (cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3
                 | cv2.CALIB_ZERO_TANGENT_DIST)
    res = calibrate(images, board, verbose=False, flags=zero_dist)
    fx, fy = res['K'][0, 0], res['K'][1, 1]
    cx, cy = res['K'][0, 2], res['K'][1, 2]
    ok = True

    def check(name, cond):
        nonlocal ok
        print(f'  {"PASS" if cond else "FAIL"}  {name}')
        ok &= cond

    check(f'used {len(res["used"])}/15 synthetic views', len(res['used']) >= 12)
    check(f'rms {res["rms"]:.3f} px < 0.5', res['rms'] < 0.5)
    check(f'fx {fx:.1f} within 1% of 1500', abs(fx - 1500) / 1500 < 0.01)
    check(f'fy {fy:.1f} within 1% of 1500', abs(fy - 1500) / 1500 < 0.01)
    check(f'principal point ({cx:.1f},{cy:.1f}) within 5 px',
          abs(cx - (W / 2 - 7)) < 5 and abs(cy - (H / 2 + 4)) < 5)
    print('self-test ' + ('PASSED' if ok else 'FAILED'))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('images', nargs='?', help='folder of board photos')
    ap.add_argument('--square-mm', type=float, default=DEFAULT_BOARD['square_mm'],
                    help='CALIPER-MEASURED printed square size (mm)')
    ap.add_argument('--squares-x', type=int, default=DEFAULT_BOARD['squares_x'])
    ap.add_argument('--squares-y', type=int, default=DEFAULT_BOARD['squares_y'])
    ap.add_argument('--marker-mm', type=float, default=None,
                    help='marker size; default scales the nominal 6/8 ratio '
                         'by the measured square size')
    ap.add_argument('--dict', default=DEFAULT_BOARD['dictionary'])
    ap.add_argument('--out', default='calibration/intrinsics.yaml')
    ap.add_argument('--min-corners', type=int, default=12)
    ap.add_argument('--self-test', action='store_true')
    args = ap.parse_args()

    if args.self_test:
        sys.exit(self_test())
    if not args.images:
        ap.error('image folder required (or --self-test)')

    marker_mm = args.marker_mm or (
        args.square_mm * DEFAULT_BOARD['marker_mm'] / DEFAULT_BOARD['square_mm'])
    board_spec = dict(squares_x=args.squares_x, squares_y=args.squares_y,
                      square_mm=args.square_mm, marker_mm=marker_mm,
                      dictionary=args.dict)
    board = make_board(board_spec)

    files = sorted(f for f in glob(os.path.join(args.images, '*'))
                   if f.lower().endswith(('.jpg', '.jpeg', '.png', '.dng')))
    if not files:
        raise SystemExit(f'no images in {args.images}')
    print(f'{len(files)} images from {args.images}')
    images = ((os.path.basename(f), load_image(f)) for f in files)

    res = calibrate(images, board, args.min_corners)
    print(f'\nRMS reprojection error: {res["rms"]:.3f} px '
          f'({len(res["used"])} images)   '
          f'{"OK" if res["rms"] < 0.5 else "WARNING: above 0.5 px target"}')
    print(f'fx={res["K"][0,0]:.1f}  fy={res["K"][1,1]:.1f}  '
          f'cx={res["K"][0,2]:.1f}  cy={res["K"][1,2]:.1f}')
    print(f'distortion: {np.round(res["dist"].ravel(), 5).tolist()}')
    worst = sorted(zip(res['per_view'], res['used']))[-3:]
    print('worst views: ' + ', '.join(f'{n} ({e:.2f} px)' for e, n in worst))
    save_yaml(res, board_spec, args.out, source=args.images)


if __name__ == '__main__':
    main()
