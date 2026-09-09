# image-capture — automated insect photo sessions

Phone app + Mac coordinator + robot handshake that run a capture session
without touching the phone: the arm goes to a pose, the mocap stream says
the rig is still, the phone shoots on command, files and metadata land in
the dataset folder, the arm moves on. The turntable is still turned by
hand between runs (confirmed by mocap, no keypress). Focus stacking is
designed in but not enabled (v1 = one frame per stop).

Plan, protocol and decisions: `docs/` (start with `docs/README.md`).

```
image-capture/
  app/            iOS app (Swift/SwiftUI, AVFoundation) — project.yml for xcodegen, sources in CaptureApp/
  coordinator/    Mac coordinator (python): capture_coordinator/ package, plan.example.yaml
  robot/          panda_manual_poses.py with the socket-confirm mode, the .patch, confirm_socket.py, fake_robot.py, run_cycles.sh
  tools/          mac_client.py (bench), usb_tunnel.sh, pull_frames.py, run_coordinator.sh
  tests/          end-to-end dry run (fake phone + fake robot + synthetic mocap) — `python3 tests/test_dry_run.py`
  docs/           DEVELOPMENT_PLAN.md, PROTOCOL.md, APP_BUILD_PROMPT.md (decisions), README.md
  NOTES.md        measured facts from the real phone/rig — fill in as tests are run
```

## Status (2026-09-08)

| part | state |
|---|---|
| Part A app | written, **not yet compiled** (no Xcode on this Mac at the time of writing) |
| Part B coordinator | written, dry-run tests pass (2 steps × 3 poses; retry path) |
| Part C robot | patch written against `robo_arm/panda_manual_poses.py` (2026-08-31 version), tested only against `fake_robot.py` |

## One-time setup on the Mac

1. **Xcode** (App Store, ~15 GB) — needed to build and install the app. After
   installing: `sudo xcode-select -s /Applications/Xcode.app`.
2. `brew install xcodegen` — generates the Xcode project from `app/project.yml`.
3. `pipx install pymobiledevice3` (or `pip install pymobiledevice3`) — USB tunnel + file pull.
4. Coordinator python: it runs inside the optitrack pixi env (rclpy);
   `pip`/`pixi add` **numpy pyyaml** there if missing (`tools/run_coordinator.sh` handles the env).

## Building and installing the app (phone connected via USB-C)

```bash
cd app && xcodegen generate && open CaptureApp.xcodeproj
```
In Xcode: Signing & Capabilities → Team = your Apple ID (free account; the
install expires after 7 days, re-run from Xcode to renew). Select the iPhone
as run destination, press Run. First launch: allow camera access; on the
phone enable Settings → Privacy → Developer Mode if Xcode asks.

## Bench test (phone on the cable, app in the foreground)

```bash
bash tools/usb_tunnel.sh                       # terminal 1, keep running
python3 tools/mac_client.py ping               # round trip + clock offset
python3 tools/mac_client.py hello --session bench1 --lens 0.45
python3 tools/mac_client.py focus 0.5
python3 tools/mac_client.py stack --stop 1 --lens 0.45
python3 tools/pull_frames.py bench1 /tmp/bench1     # DNG + sidecar over USB
```
Record what the app reports (format ProRAW/BayerRAW, calib_available,
intrinsics) in `NOTES.md`.

## Dry run of the whole loop (no hardware)

```bash
python3 tests/test_dry_run.py
# or interactively:
python3 -m coordinator.capture_coordinator.fake_phone --docs-dir /tmp/fakephone --port 5601 &
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
cd ~/Documents/repos/optitrack && pixi run record-poses      # OptiTrack driver + bag (terminal 1)
cd ~/Documents/repos/image-capture
cp coordinator/plan.example.yaml coordinator/plan.yaml        # edit: robot_host, poses, lens_position[]
bash tools/run_coordinator.sh --plan coordinator/plan.yaml --tunnel      # terminal 2
```
Robot PC (ROS 1 env, move_group up, joint-state recorder running):
```bash
# once: copy robot/panda_manual_poses.py + robot/confirm_socket.py + robot/run_cycles.sh
#       next to the existing script (or apply robot/panda_manual_poses.socket.patch)
bash run_cycles.sh 36 _confirm_port:=5600
```
The coordinator connects to the robot for every run, confirms each
turntable step from the popillia body's mocap yaw, gates every shot on
stillness, pulls the files after each stop and writes
`datasets/phone_<ts>/{images,sidecars,frames.csv,session.jsonl,plan.json}`.
Nothing is ever deleted on the phone.

## Open items before the first lab run
- Compile the app in Xcode and run the bench test; fill `NOTES.md`.
- Set `robot_host`/`robot_port` in the plan; the robot PC must allow the port.
- Decide the per-pose `lens_position[]` (one sweep with `mac_client.py focus`).
- Stillness thresholds: start with 0.1 mm / 1 s and read the printed jitter.
