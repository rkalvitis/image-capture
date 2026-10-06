# image-capture — automated insect photo sessions

Phone app + Mac coordinator + robot handshake that run a capture session
without touching the phone: the arm goes to a pose, the mocap stream says
the rig is still, the phone shoots a focus stack on command, files and
metadata land in the dataset folder, the arm moves on. The turntable is
turned by hand between runs (Enter) or, from 2026-09-28, by a NEMA 17 stepper
on a USB-serial board (`hardware/turntable/`, `turntable_drive: serial`). The
stacks are merged on the Mac (during the session if wanted) and exported
straight to a COLMAP model with one camera per image. No calibration is
applied at capture time: the hand-eye / intrinsics YAMLs are used afterwards,
on the Mac, when the dataset is processed (user, 2026-09-28).

Plan, protocol and decisions: `docs/` (start with `docs/README.md`).

```
image-capture/
  app/            iOS app (Swift/SwiftUI, AVFoundation) — project.yml for xcodegen, sources in CaptureApp/
  coordinator/    Mac coordinator (python): capture_coordinator/ package, plan.example.yaml,
                  stack_merge.py (focus-stack merge), colmap_export.py, merge_worker.py (inline merge)
  robot/          panda_manual_poses.py with the socket-confirm mode, the .patch, confirm_socket.py, fake_robot.py, run_cycles.sh
                  (the SOURCE OF TRUTH for the robot side is ~/Documents/repos/robo_arm — keep both in sync)
  tools/          mac_client.py (bench), usb_tunnel.sh, pull_frames.py, run_coordinator.sh, check_intrinsics.py (ChArUco vs Apple)
  tests/          test_dry_run.py (fake phone + fake robot + synthetic mocap, incl. inline merge),
                  test_stack_merge.py (synthetic stack with known breathing), test_colmap_export.py (poses + COLMAP model)
  docs/           DEVELOPMENT_PLAN.md, PROTOCOL.md, APP_BUILD_PROMPT.md (decisions), README.md
  NOTES.md        measured facts from the real phone/rig
```

## OneDrive upload, `poses/`, start questions (2026-09-29)

- **Upload during the session** (`docs/ONEDRIVE.md`): plan `upload_remote: onedrive:SUPSI/<dataset>`,
  `upload_what all|masters|raw`, `upload_delete_local`, `upload_delete_phone`. After each stop's
  merge the upload worker runs `rclone copy` + `rclone check --one-way`; only a verified copy
  lets it delete the dng/tif locally and the DNG + sidecar on the phone (AFC, `pull_worker.py
  delete`). Session files, `poses/` and the mocap bag (→ `poses/mocap_bag/`) are synced too.
  Cloud layout = the existing datasets: `raw/` (dng + json, flat), `stacked/` (tif), `previews/`,
  `stack_meta/`, `poses/`, `robot/`, `calibrations/` + the session files.
  rclone token: created once on the Mac (`rclone authorize "onedrive"`), pasted into the PC.
  `tools/upload_speedtest.sh remote:path` measures MB/s. With `upload_what raw|all` the plan must
  have `merge_delete_raw: false` (the upload deletes).
- **`<session>/poses/`** (`poses_collect.py`, after every stop): `stops.csv` (per stop: joints,
  phone/popillia body poses, platter deg, yaw), `turntable.csv`, `robot_poses.csv`,
  `mocap/<image>.csv`, `robot_runs/` = the robot scripts' own `manual_poses_<ts>.csv` /
  `joint_states_<ts>.db` matched by the plan's `poses_collect` globs.
- **The run command asks at start** (Enter = plan value; `--no-ask`, `--auto-operator`, or
  `--rotations N --session-dir D --remote R` skip it): number of insect rotations per revolution
  (`turntable_step_deg` becomes exactly 360/N; with the motor the microstep resolution and the
  worst per-stop rounding are printed, the board rounds absolute targets so nothing accumulates),
  the session folder (with size estimate + free space), the OneDrive destination (`-` = none,
  then a pre-flight `rclone mkdir`/`lsd`).
- **Ubuntu test run**: `coordinator/plan.ubuntu_test.yaml` (1 rotation × 9 vertical poses × 50 frames,
  upload all, delete local + phone) + `docs/UBUNTU_TEST_RUNBOOK.md`; repo goes to the PC with
  `tools/sync_to_pc.sh` → `~/shared_ws/roberts_ws/image-capture/`.
- Tests: `tests/test_upload.py` (11) + two dry-run upload tests against `tests/fake_rclone.py`; 51 green.

## Status (2026-09-28)

The OptiTrack repo is merged into `optitrack/` (git subtree, history kept) on
the branch `image-capture-pipeline`, and every session-time part also runs on
Ubuntu (see "One-time setup on the Ubuntu robot PC"). Only the iPhone app build
stays on the Mac. Nothing has been run on the robot PC yet.

## Two collection modes and the session summary (2026-09-28)

`loop_order` in the plan:
- `poses-per-rotation` (default, the 2026-09-16 dataset): all robot poses at one insect
  rotation, then the insect is turned, 36 robot runs (`run_cycles.sh 36`).
- `rotations-per-pose`: the arm goes to a pose and holds it while the insect does all 36
  rotations, then the platter is turned BACK to step 0 (reverse, never a full turn forward:
  the insect sits on a thread and would creep in height), then the next pose. ONE robot run
  (`run_cycles.sh 1`, the coordinator passes `{cycles}` = 1). Meant for the motorised
  turntable; with `turntable_drive: manual` the operator gets an Enter per rotation and a
  "turn BACK" prompt per pose. Resume: finished stops are skipped, `--start-step` is ignored.

Every session writes `summary.json` + `SUMMARY.md` (rewritten after every stop, final at
the end): how the data was collected (loop order in words, turntable drive, poses,
rotations, frames per stop, focus mode/stack, exposure incl. the EV applied, white
balance, phone, mocap, merge settings, plan `subject` / `notes`), timing (start, end, wall
and capture time, pauses, resumes, seconds per stop and per frame, settle waits, platter
moves), results (stops planned/ok/skipped/failed/retried, per robot pose, complete
rotations, frames, merged masters, raw DNGs left, GB on disk), quality (jitter, thermal
states per frame, turntable yaw error, operator prompts, focus sources), host and
coordinator commit. Regenerate: `python -m capture_coordinator.summary <session_dir>`.

## Motorised turntable: the Portenta controller (2026-09-30)

The motor will be driven by the user's controller repo
`~/Documents/repos/phd_roberts_thesis_stepper_motor_control` (Portenta H7 + A4988 + NEMA 17,
text serial protocol, `MOVETO`/`DONE`/`ZERO`). `turntable_drive: portenta` in the plan uses
`PortentaTurntable` (`capture_coordinator/turntable.py`): absolute targets only, verified
against that repo's firmware simulator. Everything is in `docs/TURNTABLE.md`; the section
below describes the older Arduino sketch, kept as `turntable_drive: serial`.

## Motorised turntable, older sketch (2026-09-28, hardware not built)

Parts: NEMA 17 17HS4401 + A4988. The PC cannot pulse STEP/DIR over USB, so an
Arduino-class board runs `hardware/turntable/turntable_a4988.ino` and the
coordinator sends absolute platter angles over USB serial
(`coordinator/capture_coordinator/turntable.py`; wiring, current limit, plan keys
and the bench commands in `hardware/turntable/README.md`). Everything still
unknown is a plan key: `turntable_steps_per_rev`, `turntable_microsteps`,
`turntable_gear_ratio`, `turntable_direction`, `turntable_steps` (stops per
revolution), speed and acceleration. `turntable_drive: manual` keeps the old
Enter flow; `serial` = the motor (Enter only once, to place the insect);
`fake` = dry run. With the motor, `turntable_confirm: enter` means no gate (yaw
logged), `mocap`/`both` = popillia yaw gate. Tested: 3 dry-run tests with the
fake drive + 7 protocol tests against an in-process board emulation
(`tests/test_turntable.py`); the real board is untested until it exists.

## Status (2026-09-15)

**Ready for the full dataset on 2026-09-16: `docs/DATASET_RUNBOOK.md`,
plan `coordinator/plan.sep15.yaml`, dataset folder
`~/Documents/repos/optitrack/datasets/sep15_10deg_9vertical_FS/` (its
README documents the layout). One command runs everything
(`bash tools/run_coordinator.sh --plan coordinator/plan.sep15.yaml`).**

| part | state |
|---|---|
| Part A app | built, installed, bench-tested (ProRAW on the ultra-wide, 40/50-frame stacks, USB pull, `focus auto`, eco buttons: Eco / 10fps / Prev / Dim) |
| Part B coordinator | dry runs pass (16 tests: state machine, retries, stacks, focus table, Enter confirmations, no-mocap, inline merge, export) |
| Part C robot | socket-confirm mode applied in `~/Documents/repos/robo_arm`; tested against `fake_robot.py` only — the real test is the MAIN TEST below |
| Network | done: Mac (USB-Ethernet en6, 192.168.201.245) and robot PC (**192.168.201.158**) on one switch; `robot_host` set in both plans |
| Stage 5 merge + export | verified on the 9 Sep rig stacks (2026-09-15): ECC alignment exact to a few px, two fusions (A Laplacian pyramid, B complex wavelets after Forster 2004), 16-bit sRGB TIFF masters + JPEG previews, `stacks.csv`; demo merges in `datasets/phone_20260909_144852/merged_A` / `merged_B` |
| Dataset layout | `images/<s00_p0>/<s00_p0>.tif` + `stack/` (sidecars, stack.json, mocap.csv), raw deleted on the Mac after a verified merge; resume (`--resume --start-step`), session-start exposure check, launcher for tunnel/app/mocap bag/dump-bodies |
| Stage 3 intrinsics | `tools/check_intrinsics.py` ready; needs a board session at the stack lens positions |

Where the photos go: the app writes every DNG + sidecar into its own
Documents folder (Files app > On My iPhone > CaptureApp > `<session>`),
never into the Photos library, and never deletes anything.

## One-time setup on the Mac

1. **Xcode** (App Store) with the iOS platform; `sudo xcode-select -s /Applications/Xcode.app`.
2. `brew install xcodegen` — generates the Xcode project from `app/project.yml`.
3. `pipx install pymobiledevice3` — USB tunnel + file pull.
4. Coordinator python = the pixi env of `optitrack/` (rclpy for the mocap
   topics; since 2026-09-28 the OptiTrack repo lives inside this one).
   `cd optitrack && pixi install` (RoboStack ROS 2 Humble, ~10 min once).
   It has numpy/opencv/pyyaml and, since 2026-09-09, `rawpy` (ProRAW decode
   for the merge). Any other interpreter for the merge tools:
   `pip install -r coordinator/requirements-merge.txt`.
5. `brew install colmap` (optional; validates the exported model in the tests).

## One-time setup on the Ubuntu robot PC (one computer for everything)

Everything of a session runs on Linux too (verified on the Mac with the same
code paths, 26 tests + dry run; not yet run on the robot PC — start with
`bash tools/ubuntu_info.sh` there and paste the output). Only building and
installing the iPhone app needs the Mac, and with the free Apple ID the app
must be reinstalled from Xcode every 7 days (the profile expires).

The robot PC as probed on 2026-09-28 (`tools/ubuntu_info.sh`): leon-OptiPlex-7070,
x86_64, Ubuntu 20.04.6, RT kernel 5.15.96-rt61, i9-9900 (8 cores), 31 GB RAM, no
NVIDIA GPU, **51 GB free** in /home, system python 3.8 (too old for
pymobiledevice3), no pixi, usbmuxd installed (socket-activated), user in plugdev,
spd-say/espeak/paplay/aplay/notify-send present, ROS 1 noetic sourced in
`.bashrc`, lab NIC = USB-Ethernet `enx9cebe85dcb1d` 192.168.201.158, Motive PC
reachable, docker0 present (harmless: ROS_LOCALHOST_ONLY=1 in the pixi env).

1. Repo: from the Mac, `bash tools/sync_to_pc.sh` = rsync (excluding .pixi, build, install,
   log, datasets, caches) to `leon@192.168.201.158:~/shared_ws/roberts_ws/image-capture/`
   (user's path, 2026-09-29; the submodules travel with it; `.git` too, so the branch is there). **The datasets folder `~/Documents/repos/optitrack/datasets/` on
   the Mac is never moved or rsynced** (user's rule; it is also 60 GB > 51 GB free).
   Nothing from it is needed at capture time: the calibration YAMLs named in the
   plan (`rig_transforms`, `calibration_files`) are only copied into the session for
   provenance and used for the optional `cam_*` columns of frames.csv; when they are
   missing the coordinator warns and carries on, and the camera poses are computed
   later on the Mac with `colmap_export.py --rig` (user's decision 2026-09-28: all
   processing stays on the Mac). In the plan set `session_dir` / `output_root` to a
   new folder on the PC such as `~/sessions/<name>` (paths accept `~` and `$VARS`);
   the calibration paths can stay as they are. New sessions recorded on the PC stay
   there; whether they are copied to the Mac afterwards is the user's decision.
   Budget: a full 324-stop session leaves ~24 GB of 16-bit TIFF masters plus the
   250-frame raw buffer (~6 GB) on the PC; the phone keeps the raw.
2. `curl -fsSL https://pixi.sh/install.sh | bash` (then open a new shell), then
   `cd ~/image-capture/optitrack && pixi install` (linux-64 is in pixi.lock; RoboStack
   ROS 2 Humble, colcon, cmake, compilers all come from the env, ~10 min, ~5 GB).
   `pixi run build` once (colcon), then `pixi run record-mock` for 10 s = the bag
   pipeline works without hardware.
3. iPhone over USB: `pymobiledevice3` is a Linux pypi dependency of the pixi env
   (system python 3.8 cannot run it). usbmuxd is already installed; it starts by
   udev when the phone is plugged in. Check: plug the phone, trust the computer,
   `pixi run pymobiledevice3 usbmux list` shows it. Tunnel and frame pull need no
   root; `tools/usb_tunnel.sh` finds the env's binary by itself.
4. Launching the app without touching the phone (`launch_app: true`) needs the
   developer tunnel on iOS 17+, as root, in its own terminal for the whole session:
   `sudo ~/image-capture/optitrack/.pixi/envs/default/bin/pymobiledevice3 remote tunneld`.
   The launcher then runs `pymobiledevice3 developer dvt launch --kill-existing
   ch.idsia.capture`. Without tunneld it prints one line and you tap the icon once
   before the session; nothing else changes. `app_launch_cmd` in the plan overrides
   the command.
5. Notifications: everything needed is installed (spd-say, espeak, paplay,
   notify-send). Run the coordinator from a terminal on the PC's desktop session,
   not over plain ssh: `DISPLAY` and the session bus are unset over ssh, so the
   banner and speech would be skipped (the sound still plays if PulseAudio is up).
6. ROS 1 noetic is sourced in `.bashrc` (lines 122/140 and a realsense workspace).
   `tools/run_coordinator.sh` strips `/opt/ros/*`, `*/devel/*` and `ROS_*` before
   starting pixi, so no `.bashrc` change is needed; keep the robot script
   (ROS 1, `run_cycles.sh`) in its own terminal. `robot_host: 127.0.0.1` in the plan.
7. Run exactly as on the Mac: `bash tools/run_coordinator.sh --plan coordinator/plan.sep15.yaml`
   (it starts tunnel, app, dump-bodies and the mocap bag itself). First a dry run:
   `python3 tests/test_dry_run.py` inside `pixi run` (see "Dry run" below) and one
   real stop with `plan.test1loop.yaml`. Merge speed on the i9 (no GPU is needed)
   is unmeasured: watch the `merge` queue in the status line during the first stops.

## Building and installing the app (phone connected via USB-C)

```bash
cd app && xcodegen generate                                   # only after editing project.yml
xcodebuild -project CaptureApp.xcodeproj -scheme CaptureApp \
  -destination 'id=00008130-000C49D92281001C' -configuration Debug \
  -allowProvisioningUpdates -derivedDataPath /tmp/dd build
xcrun devicectl device install app --device F69E5036-B4BE-5579-B8CD-DDC135DF373F /tmp/dd/Build/Products/Debug-iphoneos/CaptureApp.app
xcrun devicectl device process launch --device F69E5036-B4BE-5579-B8CD-DDC135DF373F --terminate-existing ch.idsia.capture
```
Free Apple ID signing: the install expires after 7 days, rebuild to renew.

## Bench test (phone on the cable, app in the foreground)

```bash
bash tools/usb_tunnel.sh                       # terminal 1, keep running (Mac:5601 -> phone:5601)
python3 tools/mac_client.py ping               # round trip + clock offset
python3 tools/mac_client.py hello --session bench1 --lens 0.45
python3 tools/mac_client.py focus 0.5
python3 tools/mac_client.py focus auto                              # AF on the frame centre (~1.5 s), prints the lens position
python3 tools/mac_client.py stack --stop 1 --lens 0.0 0.1 0.2 0.3   # one frame per lens position
python3 tools/pull_frames.py bench1 /tmp/bench1     # DNG + sidecar over USB (--list to only list)
pymobiledevice3 developer dvt screenshot /tmp/s.png # see the phone screen
```

## Dry run of the whole loop (no hardware)

```bash
python3 tests/test_dry_run.py                  # ~20 s; the merge/export tests need opencv (skipped otherwise)
python3 tests/test_stack_merge.py; python3 tests/test_colmap_export.py
# or interactively:
python3 -m coordinator.capture_coordinator.fake_phone --docs-dir /tmp/fakephone --port 5601 --real-frames &
python3 robot/fake_robot.py --port 5600 --poses 9 --cycles 3 &
PYTHONPATH=coordinator python3 -m capture_coordinator.coordinator --plan coordinator/plan.example.yaml \
    --mocap synthetic --puller local --fake-phone-dir /tmp/fakephone --output-root /tmp/sessions \
    --auto-operator skip
```
(`plan.example.yaml` says 36 turntable steps and robot_host on the LAN; for the
interactive dry run copy it and set `turntable_steps: 3`, `robot_host: 127.0.0.1`.)

## Real session

Mac or Ubuntu PC (on the lab LAN, phone on USB-C, app open). By hand, or let
the plan's `start_tunnel` / `launch_app` / `record_mocap` / `dump_bodies` do it:
```bash
cd optitrack && RECORD_BODIES="phone popillia" pixi run record-poses   # OptiTrack driver + bag (terminal 1)
cd ~/Documents/repos/image-capture
cp coordinator/plan.example.yaml coordinator/plan.yaml        # edit: robot_host, poses, focus_mode/frames_per_stop, turntable_drive, merge_*
bash tools/run_coordinator.sh --plan coordinator/plan.yaml --tunnel      # terminal 2
```
(`RECORD_BODIES` matters only for the bag: the record launch's default body
list is the old `phone phone_mount popillia_mount`; the topics themselves are
published for every Motive body.)

Robot PC (ROS 1 env, move_group up, joint-state recorder running):
```bash
# once: copy panda_manual_poses.py + confirm_socket.py + run_cycles.sh from ~/Documents/repos/robo_arm
#       into src/panda_moveit_ctrl/scripts/ (they are the patched versions; robo_arm/README.md has the details)
bash run_cycles.sh 36 _confirm_port:=5600
```
The coordinator connects to the robot for every run, turns the platter
(motor) or waits for Enter / the popillia yaw (manual), gates every shot on
stillness, with `focus_mode: auto` runs autofocus on the centre of the
frame at every stop and shoots `frames_per_stop` frames centred on
the result (21 frames = AF ± 10 steps; the AF frame is the merge
reference), pulls the files after each stop and writes
`datasets/phone_<ts>/{images,sidecars,frames.csv,session.jsonl,plan.json}`.
With `merge_inline: true` every verified stop is also merged in the
background while the next one is being shot (`merged/`), and with
`merge_delete_raw: true` the stop's DNGs are then removed from the Mac —
the phone keeps every frame. Nothing is ever deleted on the phone.

## MAIN TEST: arm motion + automatic capture (both computers)

Full runbook with every command and manual step: `docs/MAIN_TEST_RUNBOOK.md`.

Plan `coordinator/plan.maintest.yaml`: **ONE run = 9 stacks × 50 frames =
450 frames → 9 merged images**: robot pose 0 = the initial pose
(`joint_start.csv`, shot too: `shoot_initial_pose`) + the 8 rows of
`manual_poses.csv`; no insect rotation, no OptiTrack (`--mocap none`).
Enter on the Mac to start, Enter at every pose after the lens is at the
init focus (type a value 0..1 to re-centre, `skip`, `abort`). Init focus
per pose = the EXACT lens position of the aug31 image of the same pose at
yaw 176.48° (`focus_table_yaw`; pose 0 IMG_6823 0.3333, then IMG_6859
0.2510, IMG_6895 0.3412, IMG_6931 0.1961, IMG_6968 0.1020, IMG_7005 0.1569,
IMG_7040 0.3098, IMG_7077 0.1608, IMG_7150 0.2392 — no medians). Each stack
= 50 frames on the uniform grid 0.0..0.5 shifted so the init focus is exactly
one frame (the merge reference). Phone in eco mode (10 fps, preview off,
screen dim, ProRAW speed).

Robot PC — once: copy the three files (from the Mac, one line):
```bash
cd ~/Documents/repos/robo_arm && scp panda_manual_poses.py confirm_socket.py run_cycles.sh leon@192.168.201.158:/home/leon/shared_ws/SynthVLA/src/panda_moveit_ctrl/scripts/
```
The robot PC's own `manual_poses.csv` and `joint_start.csv` are NOT
replaced (decided 2026-09-09): the patched script reads both CSV formats
(plain 7 values, or the quoted joints + pose columns) and the coordinator
adapts to the pose count the robot announces; the reference copies in
`robot/` are for comparison only. Every session (each terminal:
`cd /home/leon/shared_ws/SynthVLA && source devel/setup.bash`):
```bash
roslaunch panda_moveit_config franka_control.launch robot_ip:=172.16.0.2 load_gripper:=false   # T1
rosrun panda_moveit_ctrl panda_moveit_ctrl_server_node.py                                      # T2
bash src/panda_moveit_ctrl/scripts/run_cycles.sh 1 _confirm_port:=5600   # T3: ONE run
```
Mac (phone on USB-C, unlocked — Auto-Lock Never —, app in the foreground):
```bash
cd ~/Documents/repos/image-capture
ping -c 2 192.168.201.158                                       # USB-Ethernet adapter plugged in?
xcrun devicectl device process launch --device F69E5036-B4BE-5579-B8CD-DDC135DF373F --terminate-existing ch.idsia.capture
bash tools/run_coordinator.sh --plan coordinator/plan.maintest.yaml --tunnel --mocap none
```
Sequence: the robot goes to the initial pose, `reached 0` -> the Mac asks
"place/check the insect, Enter" -> stack at the initial pose (pose 0) ->
`go 0` -> for each of the 8 poses:
`reached k` -> lens to the init focus -> "Enter = shoot" -> 50 frames
(~100 s) -> pull -> `go k` -> after pose 8 the arm returns, `done`, the
session ends (`stops_ok: 9`). ~15 min + 10 Enter presses. Output:
`datasets/phone_<ts>/{images,sidecars,frames.csv,session.jsonl,plan.json}`.
Merge afterwards (below), not during the test.

## After the session: merge and export

```bash
# merge (skip if merge_inline ran; re-runs only stops that are not merged yet)
PYTHONPATH=coordinator pixi run --manifest-path optitrack/pixi.toml \
    python -m capture_coordinator.stack_merge datasets/phone_<ts> --jobs 4 [--ref-index 10] [--align intrinsics]   # 16-bit TIFF masters
# COLMAP model + camera_poses.csv (one PINHOLE camera per image, true cx/cy)
PYTHONPATH=coordinator pixi run --manifest-path optitrack/pixi.toml \
    python -m capture_coordinator.colmap_export datasets/phone_<ts> [--rig .../rig_transforms.yaml] [--scale 1.0]
```
`merged/alignment.csv` compares, per frame, the breathing predicted by
Apple's intrinsics with what the image alignment measured — the first rig
stack decides whether `--align intrinsics` is already good enough.
`colmap_export.py --source frames` does the same for single-frame sessions
(`frames_per_stop: 1`), rendering the DNGs to sensor-native JPEG.

## Focus per pose from a previous dataset

Every iPhone DNG stores the lens position (Apple MakerNote `FocusPosition`,
lens = 1 − FocusPosition/255, verified against the app's sidecars). For a
dataset shot with the same robot poses:
```bash
brew install exiftool
python3 tools/focus_from_dataset.py ~/Documents/repos/optitrack/datasets/aug31_20deg_9vertical
```
writes `coordinator/focus_aug31.yaml`: the focus of every old image with its
robot pose and popillia yaw (`--pose-order` maps robot poses 1..N to the
dataset's pose ids; poses left out are dropped). With `focus_mode: table`
the coordinator picks, at every stop, the entry of the same pose with the
nearest yaw (the focus depends on the pose AND the insect's rotation; the
order the old session was shot in does not matter, re-shots at the same yaw
resolve to the latest one) and shoots `frames_per_stop` frames centred on
it — no autofocus during the session. `focus_mode: pose` uses the per-pose
medians (`lens_position[]`), `auto` the app's AF, `range` a fixed sweep.
Stack per stop (decided 2026-09-09): 50 frames on the uniform grid 0.0..0.5
(step 0.0102), shifted by less than one step so that the init focus is
EXACTLY one of the frames; that frame is the merge reference of the stop
(`focus_centre` in frames.csv, `ref_focus_index` in merged_frames.csv).

## Phone heat (measured 2026-09-30 on the September dataset)

Of 324 stacks, 19 were shot at thermal "serious" and took 200 s each (the app's 3 s
pause before every frame), the other 305 took 52 s: 0.7 h lost of 5.4 h shooting. The
pace is set by the phone, not the Mac (one `stack` command, the frame loop runs in the
app, the throttle is checked before every frame). Since 2026-09-30 every sidecar carries
the thermal state of its own frame (`thermal`, plus `thermal_at_start`), and the plan key
`thermal_wait_s` lets the coordinator idle after a hot stack until the phone reports
"fair" (poll `status`), which cools faster than shooting at 3 s per frame.
Heat sources, largest first: ProRAW multi-frame processing per capture (`phone_raw:
bayer` removes it, `phone_quality: speed` reduces it), battery charging from the cable
(set the iPhone's charge limit to 80 % in Settings > Battery > Charging: the battery
stops charging and the phone runs from the cable), the continuous 1572x1178 video
stream for the intrinsics (`phone_frame_rate`, 10 fps now; 3 fps to be measured),
the screen (`phone_screen_brightness 0.02`, preview off), radios (airplane mode is
fine, everything goes over USB), and the cradle insulating the back (a small USB fan
across the top of the back, where the SoC sits, is the one physical fix).
Bench 2026-09-30 (phone on the Mac's USB-C, charging at 65 %, ISO 80 1/100 s, quality speed,
10 fps, preview off, brightness 0.02): ProRAW 1.15 s/frame (57 s per 50), ProRAW at 3 fps 1.67
s/frame (slower: keep 10 fps), Bayer 0.70 s/frame (not allowed: ProRAW is a requirement).
Six ProRAW stacks back to back with NO gap: 57, 55, 62, 64, 145, 205 s — nominal for 2 stacks,
fair from stack 3, serious inside stack 5 (the per-frame throttle kicks in mid-stack). In a
session the ~30 s of arm move + settle between stacks is what keeps it at "fair" most of
the time (19 of 324 stacks hot in September). Same six stacks 40 min later with **blackout
on, airplane mode, all apps closed, battery 90 %** (still charging): 55, 55, 60, 58, 55, 55 s,
nominal for 4 stacks, "fair" for 5-6, never serious — no throttling at all. Those are the
session settings now (`phone_blackout: true`). Stress test right after (start at "fair", 10
stacks, no gap): 55, 56, 56, 55, 60, 187, 205, 87, 69, 119 s — serious from stack 6, recovering
mid-stack (the per-frame check) and relapsing; the battery fell 90 -> 80 % on the cable because
iOS suspends charging when hot. So: ~10 continuous stacks from a cool start before throttling,
the 30 s session gaps stretch that, `thermal_wait_s` catches the rest, and the run must start
at 100 %. A fan on the back is the remaining lever.

## Phone heat (older notes)

ProRAW processing on the phone is the main heat source (thermal "serious"
after ~30 rapid frames). Knobs, all sent by the coordinator from the plan
(`phone_*`) or live with `mac_client.py ui --raw bayer --quality speed --fps 10
--brightness 0.05 --preview off`): `raw bayer` (sensor data only, the Mac
demosaics — rawpy handles both), `quality speed|balanced|quality`, a lower
preview/intrinsics frame rate, screen brightness, preview off. Compare
per-frame time and `status` thermal state between settings on the bench.

## Stage 3: Apple intrinsics vs ChArUco per lens position

```bash
python3 tools/mac_client.py hello --session board --lens 0.3
python3 tools/mac_client.py stack --stop 1 --lens 0.0 0.1 0.2 0.3 0.4 0.5   # ~20-40 stops, board moved/tilted each time
python3 tools/pull_frames.py board /tmp/board
python3 tools/check_intrinsics.py /tmp/board --square-mm <caliper value>     # table + intrinsics_by_lens_position.yaml
```

## FULL DATASET (2026-09-16): `docs/DATASET_RUNBOOK.md`

`coordinator/plan.sep15.yaml`: 36 rotations x 9 poses x 50 frames, focus
table by live yaw, `turntable_confirm: both` (Enter after turning + yaw
check), no Enter at the poses, exposure check at the start (+1.5 EV
suggested), inline merge (`merge_fusion` wavelet|pyramid, 3 jobs) with raw
deletion on the Mac, everything into
`datasets/sep15_10deg_9vertical_FS/`. The coordinator starts the tunnel,
the app, the mocap driver + rosbag (`RECORD_DIR` → `<dataset>/mocap/`) and
dumps the rigid bodies; the robot's `run_cycles.sh 36` is started on the
robot PC (or via `robot_start_cmd`). `--resume --start-step N` continues
after a crash.

## Open items
- Turntable motor (2026-09-28): build it (board choice, coupling, microstepping, stops per revolution),
  flash `hardware/turntable/turntable_a4988.ino`, bench-test with `python -m capture_coordinator.turntable cycle 36 10`,
  then a dry session with `turntable_drive: serial` and the arm.
- 2026-09-16 morning: pick `merge_fusion` (A vs B) from `merged_A`/`merged_B` of the 9 Sep session; `run_cycles.sh 36` on the robot PC.
- Later: stillness thresholds from the printed jitter, the board session for Stage 3 (`tools/check_intrinsics.py`), masks + 3DGS training from `gs_upload/`.
