# image-capture — automated insect photo sessions

Phone app + Mac coordinator + robot handshake that run a capture session
without touching the phone: the arm goes to a pose, the mocap stream says
the rig is still, the phone shoots a focus stack on command, files and
metadata land in the dataset folder, the arm moves on. The turntable is
still turned by hand between runs (confirmed by mocap, no keypress). The
stacks are merged on the Mac (during the session if wanted) and exported
straight to a COLMAP model with one camera per image.

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
4. Coordinator python = the optitrack pixi env (rclpy for the mocap topics).
   It has numpy/opencv/pyyaml and, since 2026-09-09, `rawpy` (ProRAW decode
   for the merge). Any other interpreter for the merge tools:
   `pip install -r coordinator/requirements-merge.txt`.
5. `brew install colmap` (optional; validates the exported model in the tests).

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

Mac (this MacBook, on the lab LAN, phone on USB-C, app open):
```bash
cd ~/Documents/repos/optitrack && RECORD_BODIES="phone popillia" pixi run record-poses   # OptiTrack driver + bag (terminal 1)
cd ~/Documents/repos/image-capture
cp coordinator/plan.example.yaml coordinator/plan.yaml        # edit: robot_host, poses, focus_mode/frames_per_stop, rig_transforms, merge_*
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
The coordinator connects to the robot for every run, confirms each
turntable step from the popillia body's mocap yaw, gates every shot on
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
PYTHONPATH=coordinator pixi run --manifest-path ~/Documents/repos/optitrack/pixi.toml \
    python -m capture_coordinator.stack_merge datasets/phone_<ts> --jobs 4 [--ref-index 10] [--align intrinsics]   # 16-bit TIFF masters
# COLMAP model + camera_poses.csv (one PINHOLE camera per image, true cx/cy)
PYTHONPATH=coordinator pixi run --manifest-path ~/Documents/repos/optitrack/pixi.toml \
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

## Phone heat

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
- 2026-09-16 morning: pick `merge_fusion` (A vs B) from `merged_A`/`merged_B` of the 9 Sep session; `run_cycles.sh 36` on the robot PC.
- Later: stillness thresholds from the printed jitter, the board session for Stage 3 (`tools/check_intrinsics.py`), masks + 3DGS training from `gs_upload/`.
