# Recording an insect dataset — full procedure

Every command and step, in order, from a fresh setup to a matched, posed,
calibrated dataset. Phases 0–3 are done **once** (repeat only when marked);
phase 4 is the per-dataset routine.

```
Phase 0  one-time setup (env, Motive, Reeflex, board)
Phase 1  intrinsics calibration        → calibration/intrinsics.yaml
Phase 2  hand-eye calibration          → calibration/hand_eye.yaml
Phase 3  insect pivot fix (in Motive)
Phase 4  dataset session (repeat per dataset)
```

---

## Phase 0 — one-time setup

### Environment
```bash
pixi install          # resolves ROS 2 + OpenCV + everything else
```

### Motive (Windows PC)
- Streaming Engine ON, Unicast, Rigid Bodies ON, ports 1510/1511.
- Rigid bodies: **`phone`** (camera stand/case markers) and **`popillia`**
  (markers that ROTATE WITH the insect mount, ≥3 visible through 360°).

### Reeflex Pro Camera (the consistency contract — never change mid-project)
- **Main (24 mm) lens** selected explicitly — never the lens auto-switch.
- **ProRAW, one fixed resolution** (48 MP recommended).
- **Manual focus, ONE fixed anchor value** for everything below (dataset
  shots, calibration shots) — do NOT let autofocus re-pick it per viewpoint.
  Choose it once at the working distance with the focus magnifier.
- Exposure locked. Focus stacking: anchor ±5 steps → 10 images (dataset
  shots only; calibration shots are single frames).
- iOS Settings → Camera → Lens Correction: leave at its default, never touch.

If any of these change → redo Phase 1 (and Phase 2 if the phone moved in
its mount).

### Calibration board
```bash
pixi run make-board                     # calibration/charuco_8x6_8mm_DICT_5X5_100.png
# custom size: pixi run make-board -- --square-mm 10 --squares-x 10 --squares-y 7
```
1. Print the PNG at **100 % scale** ("actual size" — never "fit to page").
2. Glue dead-flat onto something rigid (glass / acrylic / aluminum plate).
3. **Caliper-measure the printed square size** (measure a row of N squares,
   divide by N) — printers rescale silently. Write the number down; it is
   the `--square-mm` value used below.

---

## Phase 1 — intrinsics calibration

*(Repeat whenever lens / resolution / anchor focus / Lens Correction change.)*

1. Phone in its normal mount config, **anchor focus locked**.
2. Take **30–50 single photos** of the board: distances across the working
   band, tilts up to ~35°, and the board pushed into **every corner of the
   frame** (corner coverage constrains distortion). Parts of the board
   leaving the depth of field is fine — soft corners just drop out.
3. AirDrop into a folder, e.g. `calibration/intrinsics_photos/` (DNG works
   directly — the script uses the embedded previews; JPG/PNG also fine).
4. ```bash
   pixi run calibrate-intrinsics -- calibration/intrinsics_photos \
       --square-mm 7.98        # ← YOUR caliper-measured value
   ```
5. **Accept if RMS < 0.5 px.** Output: `calibration/intrinsics.yaml`
   (also carries the board spec forward to Phase 2).

---

## Phase 2 — hand-eye calibration (phone body → lens)

*(Repeat whenever the phone is remounted / dropped / the case changes.)*

1. Board **static** in the tracked volume (on the table; it needs no
   markers). OptiTrack tracking `phone`.
2. ```bash
   pixi run record-poses          # terminal 1 — leave running
   pixi run ms-clock              # terminal 2
   ```
3. Photograph the ms-clock screen (clock photo #1).
4. **Handheld**, take **20–40 single photos** of the board from varied
   positions AND orientations — tilt and roll the phone deliberately
   (rotation about ≥2 axes is mathematically required). **Hold still ~2 s
   for each shot.** Anchor focus, single frames (no stacking).
5. Photograph the ms-clock screen again (clock photo #2). Ctrl-C terminal 1.
6. Import into the session folder:
   ```bash
   SESSION=datasets/phone_<timestamp>
   mkdir -p $SESSION/photos $SESSION/clock
   mv ~/Downloads/IMG_*/IMG_*.DNG $SESSION/photos/     # AirDrop folder form
   mv $SESSION/photos/IMG_<first>.DNG $SESSION/photos/IMG_<last>.DNG $SESSION/clock/
   ```
7. ```bash
   pixi run calibrate-handeye -- $SESSION \
       --clock-shows HH:MM:SS.mmm --clock-shows HH:MM:SS.mmm
   ```
8. Read the report: method agreement should be within ~0.5° / 3 mm and the
   mocap-reprojection error a few px. Output: `calibration/hand_eye.yaml`.

---

## Phase 3 — insect pivot fix (Motive, ~10 minutes)

Place a spare marker exactly where the insect sits (pin tip). In Motive,
select the `popillia` rigid body and **move its pivot point to that
marker**. Remove the marker, remount the insect. The streamed pose origin
now IS the insect — no offsets anywhere downstream.

---

## Phase 4 — dataset session (per dataset)

1. **Terminal 1:** `pixi run record-poses` → `datasets/phone_<ts>/`
   (note the timestamp). **Terminal 2:** `pixi run ms-clock`.
2. 📸 Photograph the ms-clock screen (clock photo #1).
3. Shoot ring by ring, camera untouched on its stand:
   - rotate the insect one step (~18° → ~20 stacks per revolution),
   - let it settle a moment,
   - shoot one **focus stack** (anchor ±5 → 10 frames),
   - repeat to a full revolution;
   - raise the camera to the next height (**clearly separated, 5–10 cm**)
     and repeat. Aim for 5–10 rings.
   - Keep the insect **centered on the turntable's rotation axis** (constant
     camera–insect radius keeps framing and focus valid everywhere).
4. 📸 Photograph the ms-clock screen again (clock photo #2). Ctrl-C.
5. Import (same pattern as Phase 2 step 6): DNGs → `photos/`, the two clock
   shots → `clock/`.
6. Match photos to poses (times read from inside the clock photos):
   ```bash
   pixi run match -- datasets/phone_<ts> \
       --clock-shows HH:MM:SS.mmm --clock-shows HH:MM:SS.mmm
   ```
   Output: `frames.csv` (camera pose per photo, in the insect body frame)
   + quality columns (`gap_ms` = tracking dropouts, `pos_spread_mm` =
   motion at shutter). Nothing is rejected except photos outside the bag's
   time span.
7. Inspect: open `notebooks/camera_positions.ipynb` (pixi kernel), set
   `SESSION` — rings should be clean circles of evenly spaced dots at each
   height level.

### Quality checklist before calling a session good
- match rate ≈ 100 %, start/end clock offsets agree within ~50 ms;
- `gap_ms` ≈ 10 everywhere (no tracking dropouts at any turntable angle);
- `pos_spread_mm` sub-millimeter (nothing moved at shutter time);
- rings in the notebook are round and evenly spaced;
- EXIF `LensModel` identical on every shot (no silent lens switch).

### Not yet automated (known gaps)
- Stack grouping in `frames.csv` (one row per stack, anchor flagged) and
  `--hand-eye` application in the matcher — planned, see
  `docs/IPHONE_IMPLEMENTATION_PLAN.md` Stage 2.5.
- Focus-stack merging (tool TBD; geometry must be verified against anchor
  frames — see `thesis/optitrack/focus-problem.md`).
