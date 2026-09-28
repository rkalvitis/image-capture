# Measured facts (fill in from the real phone / rig)

| item | value | date |
|---|---|---|
| macOS / Xcode version | macOS 15.7.7, Xcode 26.3 (17C529), xcodegen 2.46, pymobiledevice3 11.10.1 (pipx) | 2026-09-08 |
| iOS version, device | iPhone 15 Pro, iOS ? | |
| ProRAW offered on the physical ultra-wide in the app (`format` in `ready`) | YES — "ProRAW 4032x3024", DNG 15.4 MB, is_raw true | 2026-09-08 |
| `calib_available` (photo-level calibration data) | false (as expected: single physical camera) — sidecar `calibration` = null | 2026-09-08 |
| video intrinsics reference dimensions (w×h) and fx at lens 0.3 / 0.5 / 0.7 | 1572×1178 video frame, scaled ×(4032/1572) to the photo. Sweep (fx px @ lens): 1779@0.2, 1747@0.3, 1722@0.4, 1692@0.5, 1666@0.6, 1637@0.7, 1611@0.8; cx,cy constant ≈ (2009, 1510). CAVEAT: the scale assumes the video frame covers the same field of view as the photo — verify against ChArUco (Stage 3) | 2026-09-08 |
| scale factor video→photo confirmed against ChArUco? | | |
| lens position range that focuses a 40–60 mm subject | not measured yet (one-shot AF on a far scene returns 0.0; do the AF on the beetle at working distance to get the value) | 2026-09-08 |
| USB round trip / clock offset (`mac_client.py ping`) | 19 ms round trip, offset −30…−41 ms (phone behind Mac) | 2026-09-08 |
| ProRAW capture time per frame | ~1.9 s from `stack` to `done` (0.3 s settle + ~0.3 s focus/capture + ~1.35 s ProRAW processing) | 2026-09-08 |
| thermal state after 1 h on the cable | | |
| stillness thresholds chosen (mm / s) | | |
| robot PC host:port | | |

## Bench / acceptance test log
(see docs/APP_BUILD_PROMPT.md §7 — record pass/fail with evidence)

## Findings 2026-09-08 (bench)
- **EXIF DateTimeOriginal on ProRAW is ~0.93 s LATE** (stamped at end of
  processing): app capture_time = +0.57 s after the shoot command, EXIF =
  +1.44 s. Use the app's `capture_time` (from `AVCapturePhoto.timestamp`),
  never EXIF, for mocap windows. (The old EXIF-based pipeline was saved by
  its 3 s still windows.)
- Apple's per-frame intrinsics put the principal point at the image centre
  (2004, 1508) whereas the ChArUco calibration gave (2037, 1421). To be
  resolved in Stage 3 with a board at the lens positions used.
- fx from Apple at lens 0.45 = 1705 px (scaled); ChArUco at the macro band
  gave 1976. Different focus positions — the per-position sweep will tell.
- Battery/charging reported −1/false in the first build: battery monitoring
  was not enabled (fixed in AppState.start()).
- One-shot autofocus returned the same value every time in the first build:
  the wait loop checked `isAdjustingFocus` before the scan started (fixed).
- Autofocus (fixed build): returns 0.0 for a far scene, i.e. changes with the
  scene; use `mac_client.py focus auto` with the beetle in view to pick the
  per-pose lens positions.
- Battery/charging OK in the fixed build (0.80, charging over USB-C).

## Findings 2026-09-15 (the 9 Sep rig stacks merged, preparation of the full dataset)
- **Merge bug found and fixed**: rawpy ignores `half_size` for ProRAW (the DNG
  is an already demosaiced "Stack" raw), so `decode_dng(half=True)` returned
  the full 4032x3024 while `align_stack` was told the greys were at scale 0.5
  -> every translation of the chain doubled -> the 9 Sep merges came out
  smeared (multi-edged beetle, "max dev from intrinsics 106 px"). The bench
  stack of 9 Sep had the same bug (attributed to hand motion then).
  `decode_dng` now shrinks itself; stop 1 re-merged: max dev 5.3 px, cc >= 0.9996.
- Focus breathing on the rig (stop 1, ref lens 0.255): measured scale 0.9725
  at lens 0.0078 .. 1.0316 at 0.5098; Apple's fx ratio predicts 0.9650 ..
  1.0425 (overpredicts by ~25 %); frame-centre shift <= 3.6 px. Beetle (4-5 cm)
  and foam background scale within 0.7 % of each other -> one similarity per
  frame is enough, no parallax handling needed.
- 9 stops merged (pyramid, 3 jobs): 105-264 s each under contention (90 s
  alone); alignment dev from the intrinsics 5-9 px on stops 0-5, 14-44 px on
  stops 6-8 (stop 8 min cc 0.994) — still fine visually, worth a look.
- Timing 9 Sep: 79 s per stop from the focus confirmation to stop_ok (50
  frames + pull), 85 s per stop including the robot move -> 324 stops ~7.7 h.
  Thermal over the 450 frames: 200 nominal, 250 fair, never serious.
- Exposure: `locked` metered the foam to 0.14 linear (p50), beetle median
  0.022 linear (sRGB 0.16), p99.9 0.43 -> nothing near clipping; +1 EV clips
  0.1 %, +1.5 EV 8 % (foam under the lamp), +2 EV 22 %. Apple's own rendering
  (Preview) applies BaselineExposure -0.41 EV, a ProfileToneCurve and a
  ProfileGainTableMap (local tone map) -> foam at sRGB 0.52 vs 0.41 for the
  plain linear->sRGB render; the beetle is dark in both. Decision: expose
  more at capture (exposure check at the session start, +1.5 EV suggested,
  shutter first: the rig is static) and encode the master TIFF in sRGB (a
  linear 16-bit TIFF opens black-ish in every viewer).
- dtcwt (dual-tree complex wavelet, 6 levels) on a 12 MP float32 channel:
  0.73 s forward, 0.77 s inverse; wavelet fusion = 4 forward transforms per
  frame (grey for the decision, RGB for the gather) -> ~4-5 min per 50-frame
  stop, 3 parallel jobs keep up with the 85 s capture cycle (12 cores, 19 GB).
- A vs B on the 9 Sep stops (stop 8 excluded: the rig shook there, user):
  visually identical beetles at stops 1, 4, 6 (1:1 crops), foam noise std
  4.89 (A) vs 4.69 (B) vs 3.75 (a single frame) in 8-bit units. Time with 3
  jobs in parallel: A 105-264 s per stop (~50 s effective), B 318-481 s
  (~120 s effective) -> B cannot keep up with the 85 s capture cycle; the
  plan runs A inline (B later from the phone's raw copies if wanted).
- Beetle finder (`exposure.find_subject`, threshold 0.6 x foam p75 at 1008 px,
  opening 1.5 % of the width, compact + surrounded-by-white ring): correct
  box on 10 of 12 test frames (fails: stop 0 k20, stop 8 k20), 7 ms per call;
  used for the exposure statistics and the per-frame sharpness in the merge
  (`stacks.csv` best_focus_index/best_focus_lens = the measured best focus
  vs the aug31 anchor).
- Mac buffer (user): at most 5 unmerged stacks (250 frames) on the Mac —
  `merge_max_pending`; the next pull waits, the phone keeps the frames.
- Dry run of plan.sep15 (fake phone/robot, synthetic mocap, 2 rotations x 9
  stops, 3 frames): exposure check applied +1.5 EV (0.01 s -> 0.0283 s), 18
  stops, 18 wavelet merges, raw deleted, stacks.csv/alignment.csv, robot
  joints, mocap.csv per stop, calibrations copied; resume test: stop ids
  continue, captured (step, pose) skipped, yaw0 and exposure restored.

## Dataset session 2026-09-16/17 (sep15_10deg_9vertical_FS): what went wrong, what was recovered
- 324 stops captured and merged (17:08 -> 02:03), 18 GB of TIFF masters
  (copied to ~/Desktop/dataset_17_09_2026).
- **Mocap gap**: the driver launched by the coordinator died at start ("Address
  already in use": a driver from the one-loop test at 16:09 was still running);
  that stray driver stopped delivering at 17:39:29 because the Motive PC
  became unreachable; nothing until the user started record-poses at 00:28.
  Result: rotations 3-29 + s02_p7/p8 (245 stops) have no mocap. Lesson: kill
  every earlier record-poses before a session (the launcher must do it), and
  the coordinator must STOP when samples stop arriving (it only checked at
  start).
- **The arm turns the platter** on its move from pose 5 to pose 6 in some
  runs (rotation 1: -18 deg, rotation 2: -11 deg, exactly while the phone
  moves; rotations 0 and 30-35: no jump). Path-dependent (randomized
  planner). Fix before the next session: larger keep-out for the robot run.
- **Parallel merges lost CSV rows** (3 subprocesses rewrote stacks.csv from
  stale snapshots): fixed with a file lock + `--rebuild-csv`; the per-stop
  stack.json is the complete record.
- **Popillia tracking noise at rest** 0.1-0.28 mm, phone 0.02-0.25 mm.
- **Motion inside a stack** (residual frame-centre shift, still baseline 5-8
  px): s03_p4 95 px, s11_p4 52, s18_p5 34, s01_p0 29, five more 10-18 px. The
  ECC chain absorbed it (merged images clean), the pose of those stops is
  uncertain by that amount.
- **Recovery** (tools/fill_poses.py, derived/ only, originals untouched):
  camera body per robot pose from the mocap rotations (repeatability 0.8-1.1
  mm std), popillia of rotations 3-29 by one index step per rotation about the
  insect axis (-9.948 deg/step fitted between rotations 2 and 30; single-step
  validation 0.4-1.8 deg). User's decision: assume 10 deg per rotation.

## Robot reflex 2026-09-15 (debugging the arm without the Mac)
- Symptom: every run moved to the initial pose and stopped on the move to
  pose 1 ("Run aborted ..."), in keyboard and socket mode alike. The three
  scripts on the robot PC were byte-identical to the Mac copies
  (md5 checked), VELOCITY_SCALE = ACCELERATION_SCALE = 0.05, one trajectory
  controller, allowed_start_tolerance 0.01, no second commander.
- Cause: `/franka_state_controller/franka_states/robot_mode` = 4 (Reflex),
  `last_motion_errors` = joint_motion_generator_velocity_limits_violation +
  velocity_discontinuity + acceleration_discontinuity = a jump in the
  commanded position at the start of the trajectory (stale controller state
  after an earlier reflex). Every later attempt fails immediately until the
  arm is recovered.
- Fix that worked: restart T1 (franka_control) and T2 together, then
  `rostopic pub -1 /franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal "{}"`,
  robot_mode back to 2, runs fine.
- Script additions (robo_arm + robot/ copy, scp'd): `_confirm_via:=none`
  (no prompts, `_wait_between_points` pause), `_obstacles:=false`, and the
  run aborts only on an explicit refusal of the confirmation (never on a
  missing return value).
- Settle time (user): the arm's move swings the popillia -> 10 s idle after
  every movement before the first frame (`settle_after_move_s`, coordinator
  side, also after the insect turn), the mocap stillness gate on top.

## Findings 2026-09-09 (merge + export, bench)
- **The bench6 40-frame stack (2026-09-08) was shot HANDHELD and not mocap
  tracked** — the camera moved between frames. It is useless for measuring
  the lens behaviour (it showed up to ~115 px of "shift" and a scale that
  disagreed with Apple's fx by ~1/3; both are hand motion). Do NOT tune the
  merge against it; the first stack from the rig is the reference case.
- ProRAW decode: rawpy reads the DNG as an already demosaiced linear
  16-bit RGB image (RawType.Stack, 3 colours, white level 65535, no CFA);
  ~0.4 s per frame full size, `user_flip=0` keeps the sensor-native
  4032x3024 orientation (rawpy would otherwise rotate by the EXIF flag).
  rawpy 0.27 added to the optitrack pixi env (`pixi add --pypi rawpy`).
- Merge cost: ~65 s per 40-frame stop at full resolution (alignment at
  half res + fusion), ~2 GB RAM; `--jobs 4` for post-session merges.
- Alignment: ECC directly between frames far apart in focus diverges
  (blur too different) — neighbour-chained ECC with the intrinsics ratio as
  the seed converges (cc > 0.98 on every pair of the bench stack). Whether
  the intrinsics alone are enough on the rig is what `merged/alignment.csv`
  answers (scale_pred vs scale_meas, shift_meas).
- OptiTrack driver publishes Z-up (Motive Y-up converted in
  `optitrack_ros.py pose_msg()`), metres, xyzw — the coordinator's yaw check
  (body x-axis about world z) matches `build_ring_poses.py`.
- `record.launch.py` records only `phone phone_mount popillia_mount` by
  default: start it with `RECORD_BODIES="phone popillia"` for the bag.

## Lens position in Apple's metadata (2026-09-09)
- Every iPhone DNG carries the lens position in the Apple MakerNote tag
  `FocusPosition` (0..255): **lens_position = 1 − FocusPosition/255**,
  exact to the last digit against the app's sidecars (255/222/189/156/127
  at lens 0/0.129/0.259/0.388/0.502). Read with exiftool (brew).
- So the previous dataset (aug31_20deg_9vertical, manual tap-AF in Reeflex)
  gives the focus per robot pose: `tools/focus_from_dataset.py` →
  medians pose 0..9 = 0.322 0.271 0.357 0.220 0.153 0.220 0.341 0.149 0.012
  0.241 (spread within a pose up to ±0.1 — not noise: the focus depends on
  the insect's rotation too, decided 2026-09-09). Hence `focus_mode: table`
  (per pose AND yaw, `coordinator/focus_aug31.yaml`, 326 entries for the 9
  used poses 0,1,2,3,4,5,6,7,9; pose 8 dropped) with the per-pose medians
  as `lens_position[]` fallback. Old session order (36 rotations per pose)
  vs new (9 poses per rotation) is irrelevant for the yaw lookup. The 3
  duplicate images (re-shots ~4-12 s apart, same yaw, same focus) resolve
  to the latest.
- AF comparison at ~8–10 cm on the bench: Reeflex 0.31 vs the app's
  centre-point AF 0.42–0.51 (scatter ±0.05) — the app's AF window sees
  mostly foam behind the small beetle; at 3 cm both agree (0.05 / 0.06).
  With `focus_mode: pose` the AF is not needed during the session.

## Autofocus (2026-09-09, bench with the beetle)
- One-shot `.autoFocus` does NOT scan at macro distance: at ~3 cm it returned
  the start position unchanged (0.50 -> 0.50 in 0.8 s, lens never moved),
  although it had worked on a far scene (0.6353 repeatable). Reeflex Pro
  gave 0.05 for the same 3 cm scene.
- Final implementation: `.continuousAutoFocus` with `focusPointOfInterest`
  (0.5, 0.5) for a bounded time (max 4 s), settled = `isAdjustingFocus`
  false and lens unchanged for 0.5 s, then re-lock at that value. Results at
  ~3 cm: 0.0627 / 0.0627 / 0.0627 from 0.5, 0.0627 from 0.0, 0.0588 from
  0.95 (Reeflex 0.05) — repeatable within ±0.004, i.e. < 1/3 of the stack
  step 0.0128. ~1.5 s per call.
- Plan: `focus_mode: auto`, `frames_per_stop: 21`, `focus_stack_step:
  0.0128` -> stack = AF ± 0.128 (at 3 cm: ~0.0 .. 0.19); the AF frame is the
  middle = merge reference. Frames below lens 0 are clipped to 0, so with the
  AF near 0.06 the stack is asymmetric — raise `frames_per_stop` or lower
  `focus_stack_step` if the near side matters.

## Stack grid (decided 2026-09-09)
- 50 frames per stop, uniform 0.0..0.5 (step 0.5/49 = 0.0102), shifted by
  < 1 step so the init focus (from the old images) is exactly one frame;
  that frame is the merge reference. Note the lens is quantised to 1/255
  (all applied values are k/255: 0.0627 = 16/255, 0.2588 = 66/255 ...), so
  the requested grid values round to the nearest 1/255 on the phone; the
  init focus values from the table are k/255 already and stay exact.

## Eco demo (2026-09-09, bench, beetle ~10 cm)
- Eco (10 fps, preview off, screen 0.02) + ProRAW at quality priority: 21
  frames in 42 s = 2.12 s/frame (1.90–2.40), thermal state nominal before
  and after (the cap-and-no-preview settings alone do not make ProRAW
  cheaper per frame; they remove the continuous load).
- The first demo stack (0.19–0.45 around the aug31 pose-1 value) MISSED the
  focus: beetle sharpness peaked at the first frame (lens 0.19) and rose
  towards lower values; the merge then fused defocus halos (enlarged,
  smeared beetle). Not a merge fault — the stack centre must fit the
  scene, which is what `pose_confirm` (Enter at every pose, or a typed
  lens value) is for. The chain alignment itself was verified correct
  (composed links = direct measurement; beetle and background scale
  equally within 0.2% -> the phone did not move).
- `alignment.csv` `shift_px_*` now = displacement of the frame centre in
  the reference (earlier it was measured at the principal point and looked
  like a 40-60 px error that was not there).

## Screen layout (2026-09-08, from phone_mount_conf3.stl)
- Cradle clips cover ~11 mm in from each long edge, 42–80 mm below the camera
  end (≈ y 240–466 pt, 50 pt deep each side, portrait). Top 40 mm and the
  whole lower half are free. App layout: preview at the camera end (only its
  side edges can be clipped), all text/buttons in the lower half. "Clips"
  button draws the zones for checking against the real cradle.
- Final layout (confirmed on the rig 2026-09-08): info panel in the top
  240 pt free zone (rows, stack fields, buttons, last log line — all fit in
  both modes, verified by screenshot), preview below it. Horizontal = UI
  rotated 270° (camera end to the right in the cradle); 90° was upside down.
  Red clip overlay confirmed correct by the user.
- Info.plist MUST contain `UILaunchScreen` — without it iOS runs the app
  letterboxed at 320×480 pt (first builds looked wrong because of this).
- Screen mirroring on the Mac: QuickTime Player → File → New Movie Recording →
  camera source = the iPhone (USB), shows the phone screen live. Screenshots
  from the terminal: `pymobiledevice3 developer dvt screenshot out.png`.
- Zoom vs focus (2026-09-08): `videoZoomFactor` stays 1.0 (= "0.5x" ultra-wide)
  at every lens position — we never change zoom. The visible "zoom" while
  focusing is focus breathing: fx goes 1779 px (lens 0.2) -> 1611 px (0.8),
  ~10% field-of-view change, inherent to the ultra-wide at macro distances
  and recorded per frame in the intrinsics. Preview sharpness confirms focus
  moves (76 at lens 0.0 vs 509 at 0.5 on the same scene).
- Hidden band: the whole 42–80 mm band (incl. the middle) is unusable on the
  real cradle; layout = panel (top 240 pt) / black band (226 pt) / preview.
- Thermal throttle in the app: 3 s pause between frames at "serious", capture
  paused (max 5 min) at "critical"; coordinator stack timeout allows for it.
- Files per stack: 40 frames = 80 files (DNG + JSON sidecar each).
