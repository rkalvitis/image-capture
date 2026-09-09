# CLAUDE.md — image-capture

Automated insect photo sessions: an iPhone app that shoots ProRAW on
command, a Mac coordinator that runs the session against the OptiTrack
stream, and a socket-confirm mode for the Franka robot script. Everything
is driven from the Mac; the phone is never touched during a session.
Owner: Roberts Kalvitis (r.kalvitis@gmail.com), thesis project (Gaussian
splatting of pinned insects, popillia beetle).

Read next: `README.md` (how to run), `NOTES.md` (every measured fact),
`docs/DEVELOPMENT_PLAN.md`, `docs/PROTOCOL.md`, `docs/APP_BUILD_PROMPT.md`
§8 (all decisions taken with the user — do not re-ask them).

## Repo layout

```
app/CaptureApp/     Swift/SwiftUI/AVFoundation app (5 files). app/project.yml = xcodegen spec
coordinator/        python package capture_coordinator (state machine, mocap, phone/robot clients,
                    session writer, house-arrest pull, fake phone), plan.example.yaml
robot/              panda_manual_poses.py patched (+ .patch), confirm_socket.py, fake_robot.py, run_cycles.sh
tools/              mac_client.py (bench client), usb_tunnel.sh, pull_frames.py, run_coordinator.sh
tests/              test_dry_run.py — fake phone + fake robot + synthetic mocap, must stay green
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
- Mocap: OptiTrack driver runs on this MacBook via the optitrack repo's pixi env
  (`~/Documents/repos/optitrack`, `pixi run record-poses`).

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

## Decisions (see docs/APP_BUILD_PROMPT.md §8 for the full list)

Turntable = outer loop, manual, indexed 10°, confirmed by popillia mocap yaw
±0.5°; robot poses = inner loop, one robot run per turntable step; focus
stack per stop (default 40 frames, lens 0.0→0.5, uniform), no autofocus;
exposure/WB modes auto | locked (dataset default) | custom; phone never
deletes files; thermal warn+throttle; failed stop after 2 retries → ask the
operator; USB only; stillness thresholds tuned by experiment (printed).

## Status / next

Done & verified on the bench: app builds/installs from CLI, ProRAW on the
ultra-wide, USB tunnel (19 ms RTT), 40-frame stacks, pull, intrinsics sweep,
UI in both orientations, coordinator dry runs. Not yet done: first live run
with the rig (needs `robot_host`, robot-side files copied, lens interval
checked with the beetle at working distance, stillness thresholds), ChArUco
check of the scaled intrinsics, Mac-side focus-stack merge (Stage 5).
