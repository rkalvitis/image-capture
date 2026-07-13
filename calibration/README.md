# Calibration boards — print sheet

Five sizes, print them all once (100 % scale / "actual size", never
"fit to page"; laser or photo-quality print — check the small markers are
crisp under a magnifier). Glue the chosen one dead-flat to something rigid,
then **caliper-measure the real square size** (measure a full row of
squares, divide by the count) and use it as `--square-mm`.

**Label each print on the back** — the calibration command must match the
exact board, and the patterns look alike at a glance.

| File | Pattern | Size (with margin) | Best for |
|---|---|---|---|
| `charuco_7x5_6mm_DICT_5X5_100.png` | 7×5 @ 6 mm | 54×42 mm | ~10–20 cm distances; "same size, fewer squares" variant |
| `charuco_14x10_3mm_DICT_5X5_100.png` | 14×10 @ 3 mm | 58×46 mm | ~5–10 cm |
| `charuco_14x10_2.5mm_DICT_4X4_100.png` | 14×10 @ 2.5 mm | 45×35 mm | ~4–8 cm |
| `charuco_12x8_2mm_DICT_4X4_50.png` | 12×8 @ 2 mm | 34×26 mm | ~2–4 cm (the 2 cm macro candidate) |
| `charuco_10x7_1.5mm_DICT_4X4_50.png` | 10×7 @ 1.5 mm | 23×18 mm | ~2 cm, experimental — 1.1 mm markers are at the edge of print resolution |

Pick the board whose pattern roughly **fills the frame at your anchor-focus
distance**. Then calibrate with the matching flags (replace `--square-mm`
with your measured value):

```bash
# 7x5 @ 6 mm
pixi run calibrate-intrinsics -- <folder> --squares-x 7  --squares-y 5  \
    --square-mm 6.0 --marker-mm 4.5 --dict DICT_5X5_100

# 14x10 @ 3 mm  (defaults are 8x6 — flags still required!)
pixi run calibrate-intrinsics -- <folder> --squares-x 14 --squares-y 10 \
    --square-mm 3.0 --marker-mm 2.2 --dict DICT_5X5_100

# 14x10 @ 2.5 mm
pixi run calibrate-intrinsics -- <folder> --squares-x 14 --squares-y 10 \
    --square-mm 2.5 --marker-mm 1.8 --dict DICT_4X4_100

# 12x8 @ 2 mm
pixi run calibrate-intrinsics -- <folder> --squares-x 12 --squares-y 8  \
    --square-mm 2.0 --marker-mm 1.5 --dict DICT_4X4_50

# 10x7 @ 1.5 mm
pixi run calibrate-intrinsics -- <folder> --squares-x 10 --squares-y 7  \
    --square-mm 1.5 --marker-mm 1.1 --dict DICT_4X4_50
```

When measuring, scale `--marker-mm` by the same factor as the squares
(e.g. squares measured 2.94 mm instead of 3.0 → markers 2.2 × 2.94/3.0 =
2.156). The hand-eye step reads the board spec from `intrinsics.yaml`
automatically — no board flags needed there.

Note: the small boards use 4×4 dictionaries so the printed marker bits stay
above ~0.25 mm; if the 1.5 mm board's markers smear in print, fall back to
the 2 mm one.
