# Runbook — full dataset `sep15_10deg_9vertical_FS` (2026-09-16)

36 insect rotations × 9 robot poses = 324 stops × 50 ProRAW frames
(16 200 frames), merged on the Mac during the session, raw frames deleted
from the Mac after every verified merge, everything written into
`~/Documents/repos/optitrack/datasets/sep15_10deg_9vertical_FS/` (layout:
that folder's `README.md`). Plan: `coordinator/plan.sep15.yaml`.

Budget (measured 2026-09-09 / 09-15): ~85 s per stop (100 s capture + pull)
+ 10 s settle after every arm move (`settle_after_move_s`, the move swings
the popillia) → ~8.6 h of capture + 36 turns; merge (pyramid, 3 jobs) 105-260 s per stop,
~50 s effective, so it keeps up. Raw on the Mac is capped at 5 unmerged
stacks = 250 frames ≈ 3.5 GB (`merge_max_pending`; the next pull waits);
merged masters 324 × 63 MB ≈ 20 GB. The phone keeps all 16 200 DNGs
(≈ 230 GB; 328 GB free on 2026-09-15).

## 0. Before the lab (Mac, 5 min)

```bash
cd ~/Documents/repos/image-capture
PYTHONPATH=coordinator python3 -c "from capture_coordinator.plan import Plan; Plan.load('coordinator/plan.sep15.yaml')"
ls ~/Documents/repos/optitrack/datasets/sep15_10deg_9vertical_FS      # README.md only (the coordinator refuses to overwrite a session)
```
`merge_fusion` is `pyramid` (A): on the 9 Sep stops A and B were visually
identical (demo merges `datasets/phone_20260909_144852/merged_A` /
`merged_B`, stop 8 excluded — the rig shook there), and B is ~2.5x slower:
with 3 jobs it merges ~120 s per stop effective vs 85 s of capture, so the
250-frame buffer (`merge_max_pending: 5`) would stall the capture and the
day would take ~11 h. Switch to `wavelet` only with that in mind (or re-run
B afterwards from the phone's raw copies).

## 1. Robot PC (terminals T1-T3, as for the main test)

```bash
cd /home/leon/shared_ws/SynthVLA && source devel/setup.bash
roslaunch panda_moveit_config franka_control.launch robot_ip:=172.16.0.2 load_gripper:=false   # T1
rosrun panda_moveit_ctrl panda_moveit_ctrl_server_node.py                                      # T2
bash src/panda_moveit_ctrl/scripts/run_cycles.sh 36 _confirm_port:=5600                        # T3: 36 runs = 36 rotations
```
`run_cycles.sh` repeats one run per rotation; each run waits at the initial
pose for the Mac. If a run fails right after the initial pose with
"joint_motion_generator_velocity_limits_violation / velocity_discontinuity /
acceleration_discontinuity" in `last_motion_errors` (robot_mode 4 = reflex,
seen 2026-09-15), restart T1 and T2 together, then
`rostopic pub -1 /franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal "{}"`
(details in NOTES.md). Debug walk without the Mac:
`rosrun panda_moveit_ctrl panda_manual_poses.py _execute:=true _confirm_via:=none`
(no prompts; `_confirm_each_pose:=true` instead = Enter at every pose; `_obstacles:=false` drops the keep-out scene). (Optional: put the T3 line into `robot_start_cmd` of the
plan as an ssh command — needs `ssh-copy-id leon@192.168.201.158` once.)

## 2. Motive PC

Stream the `phone` and `popillia` bodies (both must be tracked before the
start — the bag only records topics that exist). Check the marker rules of
`aug31_20deg_9vertical/README.md` (m3 on the phone case moves if the phone is
reseated → re-dump happens automatically at the start).

## 3. Mac — ONE command

Phone on USB-C, unlocked (Auto-Lock Never), USB-Ethernet adapter in
(`ping -c 2 192.168.201.158`), Motive streaming, robot at T3 waiting.
```bash
cd ~/Documents/repos/image-capture
bash tools/run_coordinator.sh --plan coordinator/plan.sep15.yaml
```
The coordinator itself: starts the USB tunnel, launches the app, dumps the
rigid bodies into `calibrations/`, starts the OptiTrack driver + rosbag into
`mocap/`, copies the calibration and robot files, connects to the phone and
the robot, then runs the 36 rotations and merges in the background. Log
line "[mocap] receiving phone, popillia" must appear; if not, Motive is not
streaming those bodies.

### Manual input (everything else is automatic)

Whenever one of these prompts appears the Mac plays a sound, speaks the
request and shows a banner, repeated every 45 s until you answer
(`notify`, `notify_repeat_s` in the plan; turn the Mac's volume up).

| when | prompt | what to do |
|---|---|---|
| start | "Robot at the initial pose. Place/check the insect (step 0), then press Enter" | insect on the pin, turntable at its start mark, Enter |
| once, right after | "EV to apply on top of this exposure (Enter = +1.5, 0 = keep, or a value)" | read the table (foam / beetle brightness, clipped %), Enter for +1.5 EV or type another value; the next round shows the result, Enter (= 0) accepts |
| every rotation | "TURN the insect to step N (+10 deg, yaw now Y), then press Enter" | turn the platter one index, Enter. No yaw check (removed 2026-09-16); the yaw is logged in the event and per frame. With `turntable_drive: serial` (motor, 2026-09-28) there is no prompt: the platter turns itself, the first prompt says "the motor turns it from now on" |
| failure | "stop N (step s, pose k) failed 3 times: ..." | retry / skip / abort |
| per pose (`loop_order: rotations-per-pose`, manual platter only) | "Pose k done. Turn the insect BACK to step 0 (reverse), then press Enter" | turn the platter back the same way, Enter (a motor does it alone) |
| end | "Session finished (...). Press Enter to stop the mocap recording / tunnel" | Enter — closes the bag cleanly |

### Pause / resume at any time

Three ways, all equivalent: type `p` + Enter in the coordinator terminal,
press Ctrl-C once, or `touch <dataset>/PAUSE` from any other terminal (also
over ssh). The coordinator finishes what it is doing (a running stack takes
up to ~2 min), then pauses at the next checkpoint — the arm holds its pose,
the phone idles, the merges keep running, the bag keeps recording — and
says so (sound + speech). Enter resumes (the 10 s settle runs again),
`abort` at that prompt ends the session cleanly. `s` + Enter prints a
status line at any time. Ctrl-C a second time = abort.

Ctrl-C twice (or `abort` at a prompt) ends the session cleanly: the robot
run is aborted, the merges are drained, the bag closed. The stop numbers
continue where they were: after a crash or abort restart with
```bash
bash tools/run_coordinator.sh --plan coordinator/plan.sep15.yaml --resume --start-step <rotation to redo from>
```
(and restart `run_cycles.sh <remaining>` on the robot PC). Stops already in
`frames.csv` are skipped, the phone session folder, the reference yaw and
the applied exposure are restored.

### Timing per stop (2026-09-16)

"go" is sent to the robot the moment the last frame of a stack is shot; the
pull of the 50 files over USB, the CSV rows, the mocap window and the merge
hand-off run in a background thread while the arm moves and settles
(`captured` event = the arm may leave, `stop_ok` = pulled and recorded). The
merges run in their own subprocesses. Nothing but the shooting itself holds
the arm. Tests 1-3 on 2026-09-16 exercised: 3 frames/pose, 50 frames with a
fixed exposure (ISO 38, 1/15 s, `plan.test2.yaml`), and the parallel pull
(`plan.test3.yaml`).

### Cooling the phone: unplugging the USB cable

With the coordinator started after 2026-09-16 18:00 (auto-reconnect in
`phone.py`): at a "TURN the insect" prompt wait for `pulls pending 0` (`s`
+ Enter), unplug the cable, let the phone cool, plug it back in, press
Enter. The coordinator restarts the USB forwarder, reconnects, re-sends
the session settings (exposure, WB, focus, power) and continues; a pull
that hits a pulled cable retries 3 times and the frames stay on the phone.
With an OLDER running coordinator: answer `abort` at the prompt, unplug,
cool, replug, then `--resume --start-step <next rotation>` (+ `run_cycles.sh
<remaining>` on the robot PC).

## 4. During the session — what to watch

- `[stop N] settling 10.0s after the last movement`, then the jitter line (`gate off`: logged only),
  then the progress line: `stop N sXX_pK: [####] 37/50 frames lens 0.3843 ... | stacks 123/324`.
- `[merge] stop N: ok in 150s — ...; sharpest on the beetle k=31 lens 0.32; raw deleted` — one per stop, 3 in flight.
  `sharpest on the beetle` = the measured best-focus frame inside the detected beetle box
  (`stacks.csv` best_focus_index / best_focus_lens vs ref_focus_index = the aug31 anchor).
- `[merge] buffer full: 5 stacks on the Mac still merging — the next pull waits`: normal if it
  clears within a minute; if it stays, the merges are slower than the capture (fewer frames? more jobs?).
  `FAILED` → the raw frames of that stop stay in `images/<image>/stack/`;
  merge later with `python -m capture_coordinator.stack_merge <dataset> --stops N --fusion pyramid --delete-raw`.
- Thermal state in the phone's `frame`/`done` messages (`thermal` in every
  sidecar): 9 Sep stayed at fair/nominal over 450 frames.
- Disk: `df -h ~` — the Mac needs ~25 GB over the day.

## 5. After the session

```bash
D=~/Documents/repos/optitrack/datasets/sep15_10deg_9vertical_FS
ls $D/images | wc -l                     # 324
python3 -c "import csv; r=list(csv.DictReader(open('$D/stacks.csv'))); print(len(r), sum(1 for x in r if x['raw_deleted_on_mac']=='1'))"
PYTHONPATH=coordinator pixi run --manifest-path ~/Documents/repos/optitrack/pixi.toml \
    python -m capture_coordinator.stack_merge $D --fusion pyramid --delete-raw --jobs 4      # merges anything left over (same fusion as the plan)
PYTHONPATH=coordinator pixi run --manifest-path ~/Documents/repos/optitrack/pixi.toml \
    python -m capture_coordinator.colmap_export $D --out gs_upload
```
Then the masks (aug31 `masking/` pipeline) and the upload as in
`aug31_20deg_9vertical/gs_upload/README_server.md`.

## 6. Dry run of exactly this plan on the Mac alone (done 2026-09-15, 2 rotations × 9 stops OK)

```bash
PYTHONPATH=coordinator pixi run --manifest-path ~/Documents/repos/optitrack/pixi.toml \
    python -m capture_coordinator.fake_phone --docs-dir /tmp/fakephone --port 5611 --real-frames &
python3 robot/fake_robot.py --port 5610 --poses 8 --cycles 2 --move-time 0.2 &
sed 's/^robot_host:.*/robot_host: 127.0.0.1/; s/^robot_port:.*/robot_port: 5610/; s/^phone_port:.*/phone_port: 5611/; s/^turntable_steps:.*/turntable_steps: 2/; s/^frames_per_stop:.*/frames_per_stop: 3/; s/^start_tunnel:.*/start_tunnel: false/; s/^launch_app:.*/launch_app: false/; s/^record_mocap:.*/record_mocap: false/; s/^dump_bodies:.*/dump_bodies: false/; s#^session_dir:.*#session_dir: /tmp/sessions/dataset#' \
    coordinator/plan.sep15.yaml > /tmp/plan.dry.yaml
PYTHONPATH=coordinator pixi run --manifest-path ~/Documents/repos/optitrack/pixi.toml \
    python -m capture_coordinator.coordinator --plan /tmp/plan.dry.yaml --mocap synthetic --puller local --fake-phone-dir /tmp/fakephone
```
