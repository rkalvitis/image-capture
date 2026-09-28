#!/usr/bin/env python3
"""Generate a print-ready ChArUco calibration board.

Defaults produce a SMALL board for the ~15–25 cm working distance of the
insect rig: 8×6 squares of 8 mm → 64×48 mm pattern (credit-card sized) on
a white margin. Output is a PNG with embedded DPI so any print dialog at
"100 % / actual size" reproduces the physical dimensions.

    pixi run make-board
    pixi run make-board -- --square-mm 10 --squares-x 10 --squares-y 7

PRINTING RULES (accuracy depends on them):
  1. Print at 100 % scale — disable "fit to page".
  2. Glue perfectly flat to something rigid (glass/acrylic/aluminum).
  3. Measure the printed square size with calipers (average over several
     squares: measure N squares, divide by N). Pass THAT value as
     --square-mm to calibrate_intrinsics.py — printers rescale silently.

The same board parameters (squares, dictionary) must be passed to
calibrate_intrinsics.py; the intrinsics file then carries them onward to
calibrate_handeye.py automatically.
"""

import argparse
import os

import cv2
from PIL import Image


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--squares-x', type=int, default=8)
    ap.add_argument('--squares-y', type=int, default=6)
    ap.add_argument('--square-mm', type=float, default=8.0,
                    help='nominal square size (default 8 mm)')
    ap.add_argument('--marker-mm', type=float, default=6.0,
                    help='ArUco marker size inside a square (default 6 mm)')
    ap.add_argument('--dict', default='DICT_5X5_100',
                    help='ArUco dictionary (default DICT_5X5_100)')
    ap.add_argument('--dpi', type=int, default=600)
    ap.add_argument('--margin-mm', type=float, default=8.0,
                    help='white margin around the pattern')
    ap.add_argument('--out', default=None,
                    help='output PNG (default calibration/charuco_<spec>.png)')
    args = ap.parse_args()

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dict))
    board = cv2.aruco.CharucoBoard(
        (args.squares_x, args.squares_y),
        args.square_mm / 1000.0, args.marker_mm / 1000.0, dictionary)

    px_per_mm = args.dpi / 25.4
    w_mm = args.squares_x * args.square_mm
    h_mm = args.squares_y * args.square_mm
    size_px = (round(w_mm * px_per_mm), round(h_mm * px_per_mm))
    margin_px = round(args.margin_mm * px_per_mm)
    img = board.generateImage(
        (size_px[0] + 2 * margin_px, size_px[1] + 2 * margin_px),
        marginSize=margin_px)

    out = args.out or (f'calibration/charuco_{args.squares_x}x{args.squares_y}'
                       f'_{args.square_mm:g}mm_{args.dict}.png')
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    Image.fromarray(img).save(out, dpi=(args.dpi, args.dpi))

    print(f'wrote {out}')
    print(f'  pattern: {args.squares_x}x{args.squares_y} squares, '
          f'{args.square_mm:g} mm square / {args.marker_mm:g} mm marker, {args.dict}')
    print(f'  physical size incl. margin: '
          f'{w_mm + 2 * args.margin_mm:.0f} x {h_mm + 2 * args.margin_mm:.0f} mm '
          f'@ {args.dpi} dpi')
    print('  PRINT AT 100% SCALE, glue flat, then CALIPER-MEASURE the real '
          'square size\n  and pass it to calibrate_intrinsics.py as --square-mm.')


if __name__ == '__main__':
    main()
