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
tests/              test_dry_run.py, test_stack_merge.py, test_colmap_export.py — must stay green
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
- Robot PC: separate machine on the lab LAN, ROS 1, NO access from here.
  Its IP/port still to be filled into `coordinator/plan.yaml` (`robot_host`).
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

## STATE 2026-09-28 (read this first)

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
- **Datasets** still in `~/Documents/repos/optitrack/datasets/` (git-ignored, 60 GB); the plan
  files keep those absolute paths; on Ubuntu rsync them and edit `session_dir` etc.

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
