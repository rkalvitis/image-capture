# CLAUDE.md — image-capture

Automated insect photo sessions: an iPhone app that shoots ProRAW on
command, a Mac coordinator that runs the session against the OptiTrack
stream, and a socket-confirm mode for the Franka robot script. Everything
is driven from the Mac; the phone is never touched during a session.
Owner: Roberts Kalvitis (r.kalvitis@gmail.com), thesis project (Gaussian
splatting of pinned insects, popillia beetle).

Read next: `README.md` (how to run), `NOTES.md` (every measured fact),
`docs/DATASET_RUNBOOK.md` (the full dataset session of 2026-09-16),
`docs/DEVELOPMENT_PLAN.md`, `docs/PROTOCOL.md`, `docs/APP_BUILD_PROMPT.md`
§8 (all decisions taken with the user — do not re-ask them).

## Repo layout

```
app/CaptureApp/     Swift/SwiftUI/AVFoundation app (5 files). app/project.yml = xcodegen spec
coordinator/        python package capture_coordinator (state machine, mocap, phone/robot clients,
                    session writer, house-arrest pull, fake phone, poses, stack_merge (fusions A pyramid /
                    B wavelet), merge_worker, colmap_export, synth, exposure (start check), launcher
                    (tunnel/app/mocap bag/dump-bodies)), plan.example.yaml, plan.maintest.yaml,
                    plan.sep15.yaml (THE full dataset plan), requirements-merge.txt
robot/              COPY of the robot side (panda_manual_poses.py patched, .patch, confirm_socket.py,
                    fake_robot.py, run_cycles.sh, manual_poses.csv = 9 aug31 poses for the main test). SOURCE OF TRUTH = ~/Documents/repos/robo_arm (applied
                    there 2026-09-09) — the arm runs on a different PC; edit there, keep this copy in sync
tools/              mac_client.py (bench client), usb_tunnel.sh, pull_frames.py, run_coordinator.sh,
                    check_intrinsics.py (Stage 3: Apple vs ChArUco per lens position),
                    ubuntu_info.sh (run on the robot PC, paste the output: arch, usbmuxd, pixi, sudo, ROS 1, tools)
optitrack/          THE FORMER ~/Documents/repos/optitrack REPO, merged 2026-09-28 with git subtree (history kept,
                    38 commits, its own CLAUDE.md/README/RECORD_DATASET.md). pixi.toml = the ONE python env
                    (RoboStack ROS 2 Humble + rawpy/opencv/dtcwt; platforms osx-arm64, linux-64, linux-aarch64),
                    src/ = NatNet driver + listener + launch files, scripts/ = calibration, matching, plots,
                    masking and GS scripts (kept as they were; masking/3DGS are out of scope), submodules
                    src/optitrack_msgs (https) and vendor/hand_eye_calibration. datasets/, calibration photos
                    and bags, docs/, thesis/, 3dprinter/ are git-ignored and STILL LIVE in the old folder
                    ~/Documents/repos/optitrack/ (user's decision) -> plan paths point there.
hardware/turntable/ turntable_a4988.ino (Arduino-class board: NEMA 17 17HS4401 + A4988, USB serial line protocol)
                    + README (wiring, current limit, plan keys). Coordinator side: capture_coordinator/turntable.py
                    (SerialTurntable / FakeTurntable / FakeBoardTransport). Hardware not built yet (2026-09-28).
                    summary.py = session summary (summary.json + SUMMARY.md) from the session files.
tests/              test_dry_run.py, test_stack_merge.py, test_colmap_export.py, test_turntable.py — must stay green
                    (merge/export tests need opencv: run with the pixi env or a venv from requirements-merge.txt)
docs/               plan, protocol, decisions
```

## Hardware / accounts (as of 2026-09-08)

- Phone: iPhone 15 Pro (iPhone16,1), iOS 18.7.8, udid `00008130-000C49D92281001C`,
  CoreDevice id `F69E5036-B4BE-5579-B8CD-DDC135DF373F`, Developer Mode on, trusted.
- Mac: this MacBook (macOS 15.7), Xcode 26.3 with the iOS platform installed,
  xcodegen (brew), pymobiledevice3 11.x via **pipx** (its python:
  `~/.local/pipx/venvs/pymobiledevice3/bin/python`; the coordinator's pull
  worker finds it from the `pymobiledevice3` shebang automatically).
- Signing: free Apple ID, **personal team `4QBP63RP9N`** (the certificate
  string shows a different id — use 4QBP63RP9N), bundle id `ch.idsia.capture`.
- Robot PC (= the future one-computer host, probed 2026-09-28 with `tools/ubuntu_info.sh`):
  leon-OptiPlex-7070, `ssh leon@192.168.201.158` (password; the Mac must be on the lab
  switch via USB-Ethernet en6 192.168.201.245, the VPN does not route 192.168.201.0/24),
  x86_64, Ubuntu 20.04.6, RT kernel 5.15.96-rt61, i9-9900 8 cores, 31 GB RAM, no NVIDIA,
  51 GB free, system python 3.8 (pymobiledevice3 therefore lives in the pixi env on Linux),
  no pixi yet, usbmuxd installed, ROS 1 noetic sourced in .bashrc, spd-say/espeak/paplay/
  notify-send present, Motive PC reachable. The user runs every command there (Claude has no access).
- Mocap: OptiTrack driver runs via the pixi env in `optitrack/` (this repo since
  2026-09-28; `cd optitrack && pixi run record-poses`), on the Mac or on the Ubuntu PC.

## Build / install / run (all from the terminal, phone on USB-C)

```bash
cd app && xcodegen generate                                   # only after editing project.yml
xcodebuild -project CaptureApp.xcodeproj -scheme CaptureApp \
  -destination 'id=00008130-000C49D92281001C' -configuration Debug \
  -allowProvisioningUpdates -derivedDataPath /tmp/dd build
xcrun devicectl device install app --device F69E5036-B4BE-5579-B8CD-DDC135DF373F /tmp/dd/Build/Products/Debug-iphoneos/CaptureApp.app
xcrun devicectl device process launch --device F69E5036-B4BE-5579-B8CD-DDC135DF373F --terminate-existing ch.idsia.capture
bash tools/usb_tunnel.sh &                                    # Mac:5601 -> phone:5601
python3 tools/mac_client.py hello --session x --lens 0.3      # then focus / stack / status / ui / bye
python3 tools/pull_frames.py x /tmp/x                         # DNG + sidecars over USB
pymobiledevice3 developer dvt screenshot /tmp/s.png           # see the phone screen
python3 tests/test_dry_run.py                                 # coordinator dry run, ~15 s
```
A quick compile check without the phone: add `CODE_SIGNING_ALLOWED=NO` and
`-destination 'generic/platform=iOS'`.

## Non-obvious facts (each cost time once)

- `Info.plist` MUST have `UILaunchScreen: {}` (in project.yml) — without it
  iOS runs the app letterboxed at 320×480 pt.
- Photo-level `cameraCalibrationData` is NOT delivered for the single
  physical ultra-wide; per-frame intrinsics come from the video data output
  connection (1572×1178 frame) and are scaled to the 4032×3024 photo in the
  sidecar. Scale assumes equal field of view — not yet verified with ChArUco.
- `videoZoomFactor` is always 1.0 (= "0.5x"). The visible zoom while
  focusing is focus breathing: fx 1779 px @ lens 0.2 → 1611 px @ 0.8.
- EXIF `DateTimeOriginal` on ProRAW is ~0.93 s LATE; the sidecar's
  `capture_time` (from `AVCapturePhoto.timestamp`) is the true one.
- ProRAW: ~1.95 s per frame; the phone reaches thermal "serious" after ~30
  rapid frames. The app throttles (3 s gap at serious, pause at critical).
- Cradle (phone_mount_conf3.stl + confirmed on the rig): the band 42–80 mm
  below the camera end is covered (≈ y 240–466 pt, whole width). UI: panel in
  the top 240 pt, black band, preview below. Horizontal = UI rotated 270°.
- pymobiledevice3 ≥ 5 API is async (`await create_using_usbmux()`,
  `HouseArrestService.create(...)`); `coordinator/.../pull_worker.py` handles both.
- Shell: never put `~` inside a double-quoted string (Singularity/rsync paths).
- The robot script listens (server) only while a run is active; the
  coordinator reconnects per turntable step and tolerates the old socket's
  reset (`robot.py`).
- frames.csv: `phone_*`/`pop_*` = mocap bodies in the world frame (Z-up, the
  driver converts Motive's Y-up); `cam_*`/`quat_xyzw` = T_insect<-cam after
  the hand-eye transform (`rig_transforms` in the plan, or later via
  `colmap_export.py --rig`) — same meaning as in the optitrack repo's
  `camera_poses.csv`.
- Merge: rawpy decodes ProRAW as linear demosaiced RGB; `user_flip=0` keeps
  sensor-native orientation. ECC between frames far apart in focus
  diverges; align neighbour-to-neighbour, seeded by the intrinsics ratio.
  The 2026-09-08 bench stack was handheld — never tune against it.
- The phone never deletes (full raw dataset is kept there as the backup, 328 GB
  free on 2026-09-15); `merge_delete_raw` removes only the Mac copies after the
  merged file was written and read back.
- rawpy ignores `half_size` for ProRAW (already demosaiced "Stack" raw): it
  returns the full 4032x3024. `decode_dng` shrinks itself; before the fix
  (2026-09-15) every alignment shift was doubled and the merges were smeared.
- Focus breathing measured on the rig (stop 1 of 9 Sep): scale 0.972 (lens 0.0)
  .. 1.032 (lens 0.5) relative to lens 0.255; Apple's fx ratio overpredicts
  it by ~25 % -> keep the ECC alignment, intrinsics only as the seed.
- Merge anchor (2026-10-06, user): the reference frame = the frame where the BEETLE is sharpest, measured on
  the unwarped frames inside a beetle mask (box per frame + dark silhouette vs the box border, pin cut,
  OR over frames, dilated; `beetle_sharpness`, `<image>_beetle_mask.png`); warps use Lanczos. Before, the
  never-warped middle frame k25 won the fusion because bilinear warps kept only 0.47-0.64 of the fine
  detail -> soft masters (92/250 dataset stops "sharpest" at k25). Test: 5 stops 1.4x sharper on the beetle,
  `dataset_popillia_2026-09-30/derived/anchor_test_2026-10-06/compare/`. Merge ~100-150 s (3 jobs).
  SAM 3 masks (user, 2026-10-06): `--mask sam3` / plan `merge_mask: sam3` (plan.dataset.yaml) = SAM 3 text prompt
  "insect" (`capture_coordinator/sam3_mask.py`, optitrack `masking` env, facebook/sam3 in the HF cache, MPS) on the
  provisional sharpest frame, then all frames re-scored inside it; one SAM 3 at a time (`<session>/sam3.lock`,
  19 GB RAM), ~15-25 s per stop; failure -> threshold mask, `mask_method` column. Writes `<image>_sam3_mask.png`.
- Merge quality (2026-10-06, tests on 5 stops, `dataset_popillia_2026-09-30/derived/blur_causes_2026-10-06/` + README):
  (1) alignment is fitted on the beetle (`--align-on beetle`, default; ECC on a crop around the mask, ~110-150 s per
  stop): fitted on the whole frame the beetle drifted up to 4 px and its near end came out black and smeared;
  (2) `--fusion-space srgb` (opt-in): fusion on gamma-encoded values; in linear the foam edge (80 %) outweighs the
  beetle (1 % linear), the merge crushed the beetle's blacks to 0 and darkened it 20-40 % against the single photo;
  (3) `--depth-guard 4` (opt-in, pyramid only): a pixel takes detail only from photos within 4 focus steps of its own
  sharpest photo (`focus_depth_map`, `<image>_depth.png`): removes the grey band along the outline and the
  star-shaped dust (lens astigmatism: vertical dash before focus, horizontal after). Best tested = sam3 + srgb +
  guard 4; NOT yet the default and no plan keys (user has not chosen). Wavelet fusion and dropping blurry photos
  (`--min-rel-sharpness`) were not better. The Laplacian variance cannot rank MERGED images (artefacts count as
  detail: merged = 2.4x the best single photo per tile); it is only used to pick the anchor. The single photos are
  the limit: beetle ~770 px wide, at ~1 % linear, no photo shows fine texture on the dark body.
- Full re-merge on a Windows PC (user, 2026-10-06: gamma-space fusion chosen): `tools/remerge_windows.py --raw
  <OneDrive>/SUPSI/dataset_popillia_2026-09-30/raw --out <the user's folder>` = every stop of the flat cloud raw/
  (468 x 50, 351 GB; rows built from the sidecars, `raw_dir` key in frame_path) -> `<out>/stacked|previews|stack_meta|
  masks`, srgb + guard 4 + beetle align, built-in threshold mask by default (SAM 3 optional via --sam3-python),
  resume by re-running, cloud-only files made cloud-only again after each stop, never deletes. Smoke-tested on the
  Mac with 2 local stops (107 s per stop); NOT yet run on Windows (`_CsvLock` got an msvcrt branch, untested).
- Exposure: `locked` meters the white foam to middle grey -> the beetle sits at
  2 % linear. The session-start exposure check (`exposure_check`) applies an
  EV as a CUSTOM exposure (shutter first). What Preview shows for a ProRAW is
  Apple's tone-mapped rendering (BaselineExposure -0.41, ProfileToneCurve,
  ProfileGainTableMap), not the linear data; the master TIFF is sRGB-encoded so
  it opens correctly in viewers.
- The optitrack pixi env got `dtcwt` (`pixi add --pypi dtcwt`, 2026-09-15) for
  the wavelet fusion; `record.launch.py` there honours `RECORD_DIR`.

## Decisions (see docs/APP_BUILD_PROMPT.md §8 for the full list)

Turntable = outer loop, manual, indexed 10°, confirmed by popillia mocap yaw
±0.5°; robot poses = inner loop, one robot run per turntable step; focus
stack per stop: `focus_mode: table` (decided 2026-09-09) = centre from the
old dataset's autofocus for the same robot pose and nearest popillia yaw
(`coordinator/focus_aug31.yaml`, `tools/focus_from_dataset.py`, Apple
MakerNote FocusPosition; robot poses 1..9 = aug31 poses 0-7,9); `pose` /
`auto` (bounded continuous AF; one-shot AF does not scan at macro) /
`range` remain; 50 frames per stop on the uniform grid 0.0..0.5 shifted so
the init focus is exactly one frame = the merge reference (`focus_stack_step: 0`), then 21 frames = AF ± 10 × `focus_stack_step`
(`fixed` = linspace start→end still available); merged masters are 16-bit
linear TIFF, export renders lossless PNG;
exposure/WB modes auto | locked (dataset default) | custom; phone never
deletes files; thermal warn+throttle; failed stop after 2 retries → ask the
operator; USB only; stillness thresholds tuned by experiment (printed).

## Decisions 2026-09-28 (user)

- **No calibration at capture time.** The aug31 hand-eye / intrinsics / rigid-body YAMLs are
  applied afterwards on the Mac when the dataset is processed. Every plan has
  `rig_transforms: ""` and no dataset paths in `calibration_files` (only the repo's focus
  table is copied); the coordinator warns and continues if a rig file is missing.
- **Two collection modes** (`loop_order`): `poses-per-rotation` (turntable outer, as in the
  2026-09-16 dataset) and `rotations-per-pose` (arm holds a pose for all 36 rotations, then the
  platter REVERSES to step 0 so the insect does not creep on its thread, one robot run).
  `plan.loop_order_text()` is the sentence that goes into the summary.
- **Session summary**: `summary.json` + `SUMMARY.md` in every session folder
  (`capture_coordinator/summary.py`, rewritten after each stop; `python -m
  capture_coordinator.summary <dir>` regenerates). Plan keys `subject`, `notes` feed it.
- **Motorised turntable**: NEMA 17 17HS4401 + A4988, connected to the computer via USB
  through an Arduino-class board (the PC cannot pulse STEP/DIR itself). `turntable_drive:
  manual | serial | fake`; stops per revolution, microstepping, gear ratio and direction are
  plan keys because the user has not decided them yet. Enter stays only for placing the insect.
  With the motor `turntable_confirm: enter` = no gate, `mocap`/`both` = yaw gate.

## Full dataset (prepared 2026-09-15 for 2026-09-16)

`coordinator/plan.sep15.yaml` + `docs/DATASET_RUNBOOK.md`: 36 rotations x 9
poses x 50 frames into `optitrack/datasets/sep15_10deg_9vertical_FS/`
(layout in that README: `images/<s00_p0>/<s00_p0>.tif` + `stack/` with the 50
sidecars, stack.json, mocap.csv; frames.csv, stacks.csv, alignment.csv,
calibrations/, mocap/ bag, robot/). Decisions 2026-09-15 (user): phone keeps
all raw, Mac deletes after the merge; highest-quality master (16-bit TIFF);
fusion B (Forster 2004 complex wavelets) implemented, A kept, choice after
the demo (`phone_20260909_144852/merged_A|B`); 50 frames; Enter only for
manual actions (insect placed, EV, each rotation, the final stop); the
coordinator starts/stops tunnel, app, mocap bag, dump-bodies itself; image
names `s<step>_p<pose>`; the beetle was too dark (locked metering) ->
exposure check; 250-frame Mac buffer (`merge_max_pending`); 10 s settle
after every movement (`settle_after_move_s`); stillness gate OFF (popillia noise 0.1-0.2 mm at rest); "go" right after
the capture, pull + rows in a background thread (`_finalize_async`);
pause/resume at checkpoints
(`p`, Ctrl-C once, `<session>/PAUSE`; `console.py`); macOS sound+speech
notifications for every prompt (`notify`); robot script has a debug walk
(`_confirm_via:=none`, `_obstacles:=false`) — see NOTES "Robot reflex".

## Main test (done 2026-09-09)

`coordinator/plan.maintest.yaml` + README "MAIN TEST": 9 poses per run,
`turntable_confirm: enter`, `pose_confirm: true` (Enter / lens value at every
pose), `--mocap none` possible. `robot_host` still to be set.

## STATE 2026-09-30 (read this first)

- **Back to the Mac as coordinator + merger + uploader (user, 2026-09-30)**: the Ubuntu PC merges a
  50-frame stack in ~10 min CPU (Mac 90 s) — too slow. The PC runs only the robot script
  (`run_cycles.sh N _confirm_port:=5600`, `robot_host: 192.168.201.158`), as in the 2026-09-16
  dataset; phone on the Mac's USB-C, mocap bag from the Mac, rclone remote `onedrive` on the Mac
  (13.7 MB/s). Open questions asked, not yet answered: OneDrive client on the Mac (Files On-Demand
  vs full download of SUPSI/), Wi-Fi internet in the lab, Mac session folder (~/sessions/<name>?)
  + delete at the end, robot run logs (manual scp vs rsync over the user's ssh key), next run name.
  **Answers 2026-09-30**: OneDrive app on the Mac = Files On-Demand (cloud-only, fine); the lab
  Ethernet gives the Mac internet (same IPs as before); sessions in `~/Documents/repos/optitrack/datasets/<name>`,
  kept as a folder but `upload_delete_local: true` (user changed it 2026-09-30 afternoon: masters must not stay on the Mac either; images/ emptied after the verified upload, ~60 MB of session files remain; mactest images/ verified against the cloud and emptied by hand); robot run logs copied by hand;
  exposure ISO 80 1/100 s fixed + exposure_check; first run = `plan.mactest.yaml` (mactest_2026-09-30,
  2 rotations WITH the Portenta motor, p0..p8, 50 frames) + `docs/MAC_RUNBOOK.md`.
- **ProRAW is a requirement** (user). Bench 2026-09-30 (phone on the Mac, ISO 80 1/100 s, quality
  speed, preview off, brightness 0.02): ProRAW 10 fps 1.15 s/frame (57 s per 50), ProRAW 3 fps
  1.67 s/frame (slower! keep 10 fps), Bayer 0.70 s/frame (not allowed). ProRAW DNG 29 MB on the
  bench scene vs 9 MB on the white-foam dataset. September dataset: 19 of 324 stacks at "serious"
  took 200 s each (3 s per frame), 0.7 h of 5.4 h lost; the user sees heat start after ~6 stacks. Bench 2026-09-30, six stacks with no gap, charging at
  65 %: 57, 55, 62, 64, 145, 205 s (fair from stack 3, serious inside stack 5); idle cool-down
  serious->nominal measured in scratchpad heat_cool.log (see README "Phone heat").
- **Heat work done**: sidecar `thermal` is now per frame (+ `thermal_at_start`; app rebuilt on
  the Mac 2026-09-30, install pending), plan `thermal_wait_s` = before every stack poll the phone's
  status and idle (5 s polls, max N s) while it is serious/critical. Suggestions to the user:
  iPhone charge limit 80 % (Settings > Battery > Charging; charging at 60 % was on during the bench),
  a small fan across the top of the phone's back, airplane mode (all USB anyway).
- **Blackout mode (user, 2026-09-30: "the screen can be black, i don't care about anything there")**:
  hello/ui `blackout: true` -> the app draws only Color.black, preview off, brightness 0; `bye` or a
  2 s long-press on the phone restores the UI. Plan `phone_blackout: true` in example/sep15/ubuntu_test.
  App rebuilt + installed 2026-09-30 11:00 with blackout + per-frame thermal (verified: sidecars carry
  `thermal` per frame + `thermal_at_start`). Idle cool-down measured: serious -> fair in < 4 min
  after the last frame, then "fair" persists (> 8 min, still charging 75-80 %); `thermal_wait_s: 240`. **Second bench with blackout + airplane mode + battery 90 % + apps closed:
  55, 55, 60, 58, 55, 55 s, nominal x4 then fair, never serious** (vs 145/205 s in the morning run
  at 65 % charging with the UI on). 10-stack stress test from "fair": 55, 56, 56, 55, 60, 187, 205,
  87, 69, 119 s, battery 90 -> 80 % on the cable (iOS suspends charging when hot). Each stop now
  logs a `phone_status` event (thermal, battery, charging) via `_thermal_wait`.
- **Mac test run 2026-09-30 14:36 (mactest_2026-09-30, running)**: the first attempt was killed before the
  frames.csv header was flushed, the resume appended rows to the empty file -> every inline merge died
  with KeyError 'stop' (raw kept locally + in the cloud, phone emptied, nothing lost). Fixed live:
  `session.read_frames_csv` tolerates a header-less file (merge, poses_collect, summary use it), the
  writer flushes the header and writes it into an empty file; stops 0-3 merged by hand; new recovery
  command `python -m capture_coordinator.upload_worker <session> --stops ...` re-runs a stop's final
  upload phase (masters -> cloud, local raw deleted per plan). Upload 21 MB/s from the lab.
- **mactest_2026-09-30 RESULT** (18/18 stops, 900 frames, all in OneDrive: raw 1800, stacked/previews/
  stack_meta 18, poses + 3 bags; phone emptied 1800 files; 0 dng left on the Mac after the recovery upload
  of the 5 hand-merged stops; 15 GB, 25 min): capture 54 s per stack, pull 23 s, raw upload 36-45 s at
  19 MB/s, merge 84-188 s (3 jobs), stack-to-stack 67-69 s, one 273 s gap (motor turn + robot run 2),
  no buffer wait, thermal nominal->fair only. Mocap: every frame has both body poses; popillia yaw 27.1
  (rot 0) / -114 (rot 1), jitter 0.5-1.4 mm; motor +180 deg -> yaw -140.7 deg (direction inverted, ratio
  ~1.3: mechanical, user's side); after the reverse the insect yaw was 17.2 vs 27.1 at the start.
  Images DARK (ISO 80 1/100 s copied from aug31; foam p99 0.40 sRGB) -> `exposure_check_ev: auto`
  (foam p99 -> 0.85 sRGB, +2.4 EV today). p3's sharpest frame = last of the stack (focus range end).
- **Rehearsal plan** `coordinator/plan.rehearsal.yaml` (the run before the dataset; user's inputs 2026-09-30):
  4 rotations x 90 deg, poses-per-rotation, p0 + the robot's 12 NEW poses = 13 stops per rotation, 52 stacks
  (user 2026-09-30 evening); the robot runs WITHOUT the obstacle scene: `run_cycles.sh 4 _confirm_port:=5600
  _obstacles:=false` on the PC (robot-side parameter, nothing in the coordinator), 50 frames on the FIXED range 0.0-0.5 (`focus_mode: range`, no AF: the popillia
  is not guaranteed centred), auto EV check, blackout, upload all / delete raw+phone, dataset kept.
  `{date}` in name / session_dir / upload_remote = the day the plan is loaded (Plan.load). Before it:
  coupling glued -> `turntable_direction` from the measured sign; new manual_poses.csv on the PC (copy into
  robot/); beetle at 55 cm (`_object_height:=0.55`). Then the dataset plan = the same with 36 rotations.
- **Rehearsal 2026-09-30 evening (killed by the user at 46/52 stops, local folder deleted; cloud copy
  rehearsal_2026-09-30 has 42 stops)**: glue fixed the ratio (+90 commanded -> yaw -89.9/-89.6/-86.0: direction
  inverted, turn 3 four deg short), 55 s per stack, all frames "fair", 69 s stop-to-stop, 130 s per rotation
  change, auto EV applied -0.3, upload 17 MB/s, 4100 files deleted on the phone; the last ~4 stops' raw stayed on
  the phone (session 20260930_17xx) and exists nowhere else.
- **THE DATASET PLAN `coordinator/plan.dataset.yaml`** (2026-09-30): rehearsal settings + 36 x 10 deg,
  `turntable_direction: -1`, name/folder/remote dataset_popillia_{date}, 468 stops, 23 400 frames, ~195 GB raw
  + 28 GB masters to OneDrive (836 GB free), ~10.5 h; nothing stays on the Mac or the phone. PC: `run_cycles.sh 36
  _confirm_port:=5600 _obstacles:=false`.
- **DATASET RUN dataset_popillia_2026-09-30 (started 19:45, running 2026-10-01 morning)**: capture fine
  (55 s/stack, 69 s/stop, all "fair", motor dir -1 OK), but the pipeline fell behind: the uploader slows as
  cloud raw/ grows (every rclone call lists all 21k files, 4x per stop -> 287 s/stop at stop 214 vs 69 s
  capture), capture ran ~157 stops ahead, those wait on the phone (session 20260930_194518). CONSEQUENCE:
  mocap history is 30 s (`Ros2MocapSource(history_s=30)`), so every stop finalized late has EMPTY body-pose
  columns in frames.csv (184 of 218 by stop 217). Recoverable: the rosbag mocap/bag_20260930_194523 records
  both bodies all session; sidecars/captured events hold t0/t1. User's decision: stop with Ctrl-C x2 once the
  capture is complete (NOT kill -9: the bag must get its SIGINT via launcher.stop), finish offline later.
  **MOCAP LOSS, my fault**: at 06:53:18 the bag recorder died ("SQLite error (5): database is locked") because
  I ran a sqlite3 count on the live .db3; the bag covers 19:45:26-06:53:18 (7.78 M msgs, stops 0..~377);
  rotations 29-35 (stops ~378-467, 90 stops) have NO mocap anywhere. No metadata.yaml (crash) -> `ros2 bag
  reindex` recreates it. Options: derive poses as in September (camera from the repeated robot joints, insect
  from the EXACT motor angles) or re-shoot rotations 29-35 with mocap (~2 h). Capture finished 09:32:05
  (468 stops); user stops the drain with Ctrl-C x2 and finishes offline.
  PENDING, NOT BUILT (user: "dont build them yet", 2026-10-01): (1) offline drainer: pull the un-pulled
  stops (captured events minus stop_ok), rebuild their frames.csv rows from sidecars + robot_poses.csv, merge
  (stack_merge --stops), upload (upload_worker --stops), delete on the phone; (2) fill body poses from the bag
  for ALL stops -> derived/frames_filled.csv + poses/mocap/<image>.csv (reuse optitrack/scripts/match_photos.py
  load_pose_track, header stamps = Mac time, same as the live source); (3) uploader per-stop cost: rclone
  --files-from + --no-traverse, no raw re-check in the final phase; (4) pull waits for a disconnected phone
  instead of failing the stop; (5) snapshot the mocap window at capture time (in _capture), not at pull time.
- **Turntable = the user's Portenta stepper repo** (`~/Documents/repos/phd_roberts_thesis_stepper_motor_control`,
  unfinished): `turntable_drive: portenta` -> `PortentaTurntable` speaks its protocol directly (PING/INFO/
  ZERO/MOVETO+DONE/STATUS/SPEED/ACCEL/ENABLE, microsteps, absolute only), `FakePortentaTransport` for the
  tests, dry-run test `test_portenta_turntable_full_loop`, end-to-end OK against the repo's firmware
  simulator (PlatformIO via `pipx install platformio`, `~/.local/bin/platformio`; the `pio` on PATH is a
  broken unrelated package). docs/TURNTABLE.md. Gear ratio / direction / microstepping / zero still open.
  **Real board tested 2026-09-30** (branch motor-and-wiring-update, /dev/cu.usbmodem2101, only wiring +
  motor model changed, protocol same): coordinator loop with the real motor OK, 36 x 10 deg = exactly
  3200 microsteps, reverse 360 deg 18 s; user confirmed it moved. Decisions in docs/TURNTABLE.md
  ("How it rotates and moves"): absolute MOVETO, yaw-increasing = positive, 20 deg/s, 40 deg/s2, hold on,
  zero at Enter, gear 1.
  **Early turn** (user, 2026-09-30, "motor rotations can start as soon as images finish"): poses-per-rotation
  starts the next step's motor move in a thread right after the last stack of a rotation (`_start_pre_turn`,
  event `turntable_early`), the next `_wait_turntable` joins it. Slip in the first run = coupling, hot glue.
  **Every revolution is undone backwards** (user, 2026-09-30): rotations-per-pose already returned to 0
  per pose; poses-per-rotation now sends `goto(0)` after the last rotation of the session (manual platter:
  one Enter "turn it BACK"), event `turntable_return` pose="session". The angle per step = 360/N from the
  plan / start prompt, absolute targets `s*360/N`.
- Exposure reference: aug31 dataset (Reeflex) = 1/100 s, ISO 50-100 (median ~80); 9 Sep phone
  auto = ISO 125 1/100 s; Sep dataset fixed ISO 38 1/15 s (user did not like it). Proposal ISO 80
  1/100 s fixed + exposure_check.

## STATE 2026-09-29 (superseded where the 2026-09-30 block says so)

- **OneDrive upload implemented** (`capture_coordinator/upload_worker.py`, `docs/ONEDRIVE.md`):
  per stop after the merge `rclone copy` + `rclone check --one-way`, then (plan
  `upload_delete_local` / `upload_delete_phone`) the dng/tif on the PC and the DNG+JSON on the
  phone are deleted — only after the verified copy; failed merge → upload, no deletion; failed
  upload ×3 → stop stays local + on the phone. `upload_what all|masters|raw`; `raw|all` needs
  `merge_delete_raw: false`. Session files, `poses/`, mocap bag (→ `poses/mocap_bag/`) synced.
  Cloud layout (user, 2026-09-29, = OneDrive-USI/SUPSI/dataset_popillia_15_09_2026_FS): `raw/` flat
  dng+json, `stacked/` tif, `previews/`, `stack_meta/`, `poses/` ...; locally images/<image>/stack/ stays.
  Phone delete = `pull_worker.py delete` (AFC rm, verified with exists). Two phases since the
  first PC run (2026-09-29 midday, user: "upload every time images get transferred"): raw phase right
  after the pull (parallel to the merge, phone deleted then), final phase after the merge (master,
  local deletion). The PC ran the first test with the old one-phase order -> re-sync needed. rclone installed on the
  Mac (brew, 1.75); **token created 2026-09-29** (tenant consent OK), remote `onedrive` in
  `~/.config/rclone/rclone.conf` on the Mac (business drive, 897 GB free), forced refresh OK,
  hourly refresh check running until ~2026-09-30 16:00 (`~/.config/rclone/refresh_check.log`),
  13.7 MB/s from the Mac. PC: copy rclone.conf with scp (docs/ONEDRIVE.md 2b).
- **`<session>/poses/`** (`poses_collect.py`): stops.csv / turntable.csv / robot_poses.csv /
  mocap/<image>.csv / robot_runs/ (robot scripts' `manual_poses_<ts>.csv`, `joint_states_<ts>.db`
  via plan `poses_collect` globs). User's wish: all mocap + robot pose outputs in one subfolder.
- **Start questions** in `coordinator.main()`: rotations per revolution (`Plan.set_turntable_steps`
  → step exactly 360/N, `turntable_precision()` prints microstep resolution / worst rounding;
  firmware rounds absolute targets), session folder (size + free space), OneDrive remote
  (pre-flight). `--no-ask` / `--auto-operator` / `--rotations --session-dir --remote` skip them.
- **Unattended end (user, 2026-09-29 midday)**: after the verified final upload of a stop with
  `upload_what all` the WHOLE local `images/<image>/` folder is removed (sidecars too); the test plan
  has `stop_confirm: false`; the final sync (mocap bag -> poses/mocap_bag) runs in `main()` AFTER
  `launcher.stop()` closed the bag; `upload_delete_session: true` then deletes the whole session
  folder on the PC once `rclone check` verified the final sync, every captured stop was uploaded and
  images/ is empty (`delete_session_folder`, prints KEPT + reason otherwise). Test plan: p0..p8 =
  `shoot_initial_pose: true` + `poses: 8` (the robot has 8 rows), `merge_jobs 6`, buffer 20.
- **First PC run 2026-09-29 (old one-phase code)**: capture 55 s + pull 36 s per stop fine; merge
  ~10 min CPU per 50-frame stop on the i9 (Mac 90 s), <1 GB RAM each, 3 jobs at nice 19 -> merges
  lag, buffer stalls the capture. Exact times from the `[merge] ok in NNNs` lines still to be pasted.
- **Motor NOT part of the Ubuntu test** (user, 2026-09-29: the NEMA 17 board has not arrived; the
  motor stays planned). The test plan runs `turntable_drive: manual`; the 360/N even split and the
  precision report are prepared for the motor and only printed with `serial|fake`.
- **Ubuntu test**: `plan.ubuntu_test.yaml` (1 rotation × 9 vertical poses × 50 frames, upload all,
  delete both) + `docs/UBUNTU_TEST_RUNBOOK.md`; `tools/sync_to_pc.sh` → PC path
  `~/shared_ws/roberts_ws/image-capture/` (user, 2026-09-29). NOT yet run on the PC.
- App rebuilt/reinstalled 2026-09-29 (profile expires ~2026-10-06); the developer had to be
  re-trusted on the phone after the reinstall.
- Tests: 51 green in the pixi env (dry run 22, upload 11, turntable 7, merge 5, export 6).
  Nothing committed yet (all of 2026-09-28/29 is uncommitted on `image-capture-pipeline`).

## STATE 2026-09-28 (superseded by 2026-09-29 for upload/poses; still valid)

- **Branch `image-capture-pipeline`** (master = the old two-commit state): first commit = the
  whole working tree of 2026-09-28 (+ .gitignore, pyc/xcuserstate untracked), then the
  optitrack repo merged into `optitrack/` by `git subtree add` (history kept), submodules
  re-registered, then the Linux changes below. The user's robot-arm code stays in its own
  repo (robo_arm); `robot/` here is the copy needed by the dry run and tests.
- **One computer (Ubuntu robot PC) is the goal.** Everything session-time now runs on Linux
  too: `launcher.py` (Notifier: spd-say|espeak-ng + paplay|aplay + notify-send; app launch:
  `pymobiledevice3 developer dvt launch --kill-existing`, needs `sudo pymobiledevice3 remote
  tunneld` on iOS 17+, else it tells the operator to tap the icon; `app_launch_cmd` override),
  `plan.py` (`optitrack_repo` "" or relative = `<repo>/optitrack`; `~` and `$VARS` in all
  plan paths), `tools/run_coordinator.sh` (uses `<repo>/optitrack`, strips a sourced ROS 1
  env), `optitrack/pixi.toml` + lock with `linux-64`. README "One-time setup on the Ubuntu
  robot PC" is the checklist. Verified on the Mac only: 26 tests + dry run in the merged
  env, colcon build and `record-mock` bag from the new location. NOT yet run on the robot PC
  (no access from here): first step there is `bash tools/ubuntu_info.sh`.
- **Mac stays the app build machine**: free Apple ID -> reinstall from Xcode every 7 days.
- **Datasets stay in `~/Documents/repos/optitrack/datasets/` on the Mac, NEVER moved or
  rsynced** (user's rule, repeated 2026-09-28; also 60 GB > 51 GB free on the PC). The plan
  files keep those absolute paths. On Ubuntu nothing from that folder is needed: the
  calibration YAMLs are optional at capture time (missing -> warning, `cam_*` columns
  computed later on the Mac with `colmap_export.py --rig`); new sessions go into a new
  folder on the PC. All processing (poses, export) stays on the Mac (user, 2026-09-28).

## STATE 2026-09-18 (superseded by 2026-09-28 for the repo layout; data facts still valid)

- **The dataset was captured 2026-09-16/17**: `optitrack/datasets/sep15_10deg_9vertical_FS`,
  324 stops (36 rotations x 9 poses), all merged (pyramid, 16-bit sRGB TIFF), TIFFs also
  copied to `~/Desktop/dataset_17_09_2026/`. Full account of what went wrong and what was
  recovered: NOTES.md "Dataset session 2026-09-16/17".
- **Poses**: only 79 stops have real mocap (Motive PC unreachable 17:39-00:28). All 324
  have a pose in `derived/stacks_filled.csv` (`pose_source` column): camera from the
  repeated robot poses, insect of rotations 3-29 by ONE INDEX STEP PER ROTATION
  (user's decision; -9.948 deg/step deduced from the data). Tool: `tools/fill_poses.py`.
- **File policy (user)**: recorded files are never edited; every corrected/derived file is
  a duplicate under `derived/` with provenance in its README.
- **Export**: `colmap_export.py --stacks-csv derived/stacks_filled.csv --out derived/gs_upload`
  (not run yet). Then masks (aug31 `masking/` pipeline) and 3DGS.
- **Open**: (1) decide whether rotations 3-29 get re-shot with mocap (about 7 h) or the
  indexed poses are used, possibly refined by SfM against the 79 mocap stops;
  (2) before any new session: the arm turns the platter on its move from pose 5 to 6 in some
  runs -> bigger keep-out (`run_cycles.sh N _confirm_port:=5600 _object_radius:=0.04 _rod_radius:=0.01`)
  and the launcher must kill leftover `record-poses` drivers and stop when mocap samples stop;
  (3) a redo tool (remove chosen stops from the CSVs, move folders aside, `--resume --start-step`)
  is not written yet; (4) stops with motion inside the stack: s03_p4, s11_p4, s18_p5, s01_p0.
- **Coordinator features since 2026-09-15** (all tested, dry-run + 26 tests): stacks layout,
  inline merge with 250-frame buffer, exposure fixed ISO 38 1/15 s, 10 s settle after every
  move, no stillness gate, no yaw check (Enter only), pause/resume (`p`, Ctrl-C, PAUSE file),
  sound/speech notifications, USB auto-reconnect, background pull, per-topic mocap check at
  start, `--rebuild-csv`. Robot script: `_confirm_via:=none` debug walk, `_obstacles:=false`.
- **Nothing is committed** (one "init" commit in image-capture; optitrack repo has dtcwt in
  pixi.toml and `RECORD_DIR` in record.launch.py uncommitted; robo_arm has the script changes).

## Status / next (2026-09-15, superseded by the STATE above)

Verified today: the 9 Sep rig stacks merge cleanly after the half-size fix
(alignment dev from intrinsics <= 10 px on 6 of 9 stops, cc >= 0.994), both
fusions run, 24 tests green in the pixi env (`python tests/test_*.py`; pytest
does not work there because of a ROS plugin), full dry run of plan.sep15
(2 rotations x 9 stops, exposure check, inline wavelet merge, raw deletion,
resume test). Not yet: the real session (2026-09-16), `merge_fusion` choice,
stillness thresholds, ChArUco board session, masks + 3DGS from `gs_upload/`.

## Status / next (2026-09-09, superseded)

Done & verified on the bench: app builds/installs from CLI, ProRAW on the
ultra-wide, USB tunnel (19 ms RTT), 40-frame stacks, pull, intrinsics sweep,
UI in both orientations, coordinator dry runs (12 tests), focus-stack merge
+ COLMAP export (synthetic tests, one handheld bench stack), inline merge
with Mac-side raw deletion, robot patch applied in robo_arm, rawpy in the
pixi env. Not yet done: first live run with the rig (needs `robot_host`,
robot files copied to the robot PC, `focus_stack_step`/`frames_per_stop`
chosen, stillness thresholds), first rig stack through the merge
(alignment.csv decides intrinsics-only vs ECC), ChArUco board session for
`tools/check_intrinsics.py`, 3DGS training from `gs_upload/`.
