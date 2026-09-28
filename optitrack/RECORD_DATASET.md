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

### Alternative import + matching for focus-stack sessions (done for
### `datasets/phone_20260722_143911`)

When the session was shot as focus stacks (N sweep DNGs + 1 composite JPG per
shot, consecutive `IMG_` numbers), organize the flat AirDrop dump first —
**this replaces steps 5–6 above**:

```bash
python3 scripts/organize_focus_stacks.py datasets/phone_<ts>/images [--dry-run]
    --sweep-count 21          # frames per focus sweep
    --stacked-ext .jpg .jpeg  # extension(s) of the stacked image
    --frame-ext .dng          # extension(s) of the focus frames
```

Result: `images/<n>/<stacked>.jpg` + `images/<n>/focus_stack/*.dng` per stack
(n = shooting order). Nothing is moved unless the whole folder validates
against the N+1 pattern (prune aborted sweeps first). Then match **per
stack** (median pose over each stack's full capture window; the composite
JPG's EXIF time is a copy of the last DNG's, so times come from the DNGs):

```bash
pixi run python scripts/match_stacks.py datasets/phone_<ts> \
    [--camera-body phone --object-body popillia_mount] \
    [--lens-marker calibration/lens_marker_estimate.yaml] \
    [--clock-offset SEC] [--reconstruct-missing --stacks-per-block 8]
```

`--lens-marker` applies the hand-eye translation from a lens marker
estimate (marker ball on the lens ring): camera positions become the LENS,
not the marker-body origin; orientation stays the body's. **Only valid if
the estimate was made against the SAME rigid-body definition** — redefining
the `phone` body in Motive (new markers/cradle) invalidates every earlier
hand-eye and lens estimate, because they are expressed in a body frame that
no longer exists. This is why the 2026-07-13 hand-eye / 2026-07-16 marker
estimate must NOT be applied to `phone_20260722_143911` (verified: applying
them moved poses ~44 mm the wrong way). Also note: a rig that keeps the
object centred in frame while orbiting it holds the object at one fixed
point in the camera body frame, which makes the hand-eye translation
unobservable from the dataset itself — it must come from a dedicated
calibration made against the current body definition.

Output: `stacks.csv` — one row per stack with world poses of both bodies,
quality columns, and `rel_*` = camera pose in the object body frame.
`--reconstruct-missing` salvages stacks shot before the recorder was started
(the phone_20260722_143911 mistake): the camera pose is copied from the same
in-block arm pose one block later (robot repeatability measured ~2 mm /
1.6°), the object pose from the bag's initial still window — only valid for
the block-pattern protocol (same camera poses every block, object rotated
between blocks) with recording started before the first object rotation.
`*_source` columns record what was reconstructed.

Visualize the result (no ROS needed):

```bash
pixi run python scripts/plot_stacks.py datasets/phone_<ts> \
    [--frame insect|world] [--working-distance 25] \
    [--camera-azimuth-deg 270] [--save out.png]
```

`--camera-azimuth-deg` re-references azimuth so 0° is the insect's heading
(the stand's degree-map zero): pass the azimuth the block-1 cameras sat at
(270 for phone_20260722_143911, i.e. when the map read 0° the camera was at
the 270° mark, CCW from above — pass 90 instead if the map runs clockwise).

Insect frame: one ring per arm pose around the specimen, stand rotation
axis dashed, hollow markers = reconstructed, plus a facing arrow and an
estimated lens dot per camera. The specimen is fitted as the point that
stays FIXED in the camera body frame across all poses (2026-07-22 session:
0.8 mm off the rotation axis, 4.7 mm rms spread). Facing + lens are
anchored on the capture protocol — **arm pose 1 shoots straight down from
directly above the specimen** — which fixes the optical axis and (with
`--working-distance MM`, the lens-to-object distance) the lens offset in
the body frame; measured orientations place both at every other pose. Do
NOT fit "camera aims at the object" instead: with the ultra-wide at ~25 mm
the object need not be centred in frame (that fit gave a wrong, shallow
sweep). Verified on 2026-07-22: the arm pitches the phone through a true
semicircle — body positions on a 62 mm circle (1 mm residuals) spanning
143.7°, equal to the 143.0° orientation sweep (no roll); facing runs −90°
(straight down, ring 1) to +53° (up from under the insect, ring 8, near
the rod); the anchored lens lands 24–33 mm from the specimen at ALL 48
stacks with only pose 1 pinned — independently confirming the ~2.5 cm
working distance. World frame: the actual rig (fixed camera arc + mount
origins orbiting the stand axis).

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

---

## Robot ↔ mocap registration from the continuous trajectory (2026-08-31)

`scripts/link_robot_mocap.py` registers the robot base frame to the mocap
frame. By default it matches **still windows** to the `manual_poses_*.csv`
waypoints (~9–18 pairs, and the quality gate throws away exactly the low
poses where marker visibility is worst). With a **`/joint_states` recording**
it instead uses the entire continuous trajectory: FK turns the joint stream
into flange poses, the robot-PC↔Mac clock offset is solved automatically by
cross-correlating the move/hold patterns (both sides stamp at capture time,
so there is no lag bias and the machines need no synchronized clocks), and
every mocap sample becomes a time-synced pair — thousands instead of a dozen
— with per-frame weights (slow-motion preference, local dropout, jitter,
Huber IRLS) replacing the hard gates.

During the robot session, on the **robot PC** in its own terminal:

```bash
rosrun panda_moveit_ctrl record_joint_states.py     # Ctrl-C when done
```

(the recorder lives in the robo_arm repo; start it before `record-poses`
stops mattering — any overlap with the bag works, the offset solver aligns
them). Copy the resulting `joint_states_<ts>.db` (sqlite, table
`joint_states`: stamp + the 7 joints, robot-PC clock) into the session folder
next to the pose logs (`<dataset>/phone_poses_robot/`), then:

```bash
pixi run python scripts/link_robot_mocap.py datasets/phone_<ts> \
    --spin-bag ... --csv manual_poses_*.csv \
    --joints joint_states_<ts>.db
```

The report shows the FK self-validation against the CSV flange poses
(must be sub-mm), the solved clock offset with how many correlation
candidates were disambiguated geometrically, and the weighted-rms
comparison against the still-window solve it replaces. Without `--joints`
the still-window behavior is unchanged. `registration.yaml` gains
`registration_method` and a `trajectory` block (offset, pair counts,
effective weight). Verified on synthetic data built from a real session's
waypoints: offset recovered to ~2 ms, registration to 0.03° / 0.3 mm with
a 95%-dropout window and a 6 mm jitter burst present but down-weighted.
