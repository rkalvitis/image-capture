# Main test runbook — arm motion + automatic capture (2026-09-09)

Goal: ONE run = 9 robot poses with a 50-frame focus stack at each
(9 × 50 = 450 frames, 9 merged images afterwards), Enter on the Mac before
the run and at every pose (focus check). No insect rotation, no OptiTrack. Init focus per pose = the exact focus of the aug31
image at yaw 176.48°. Merge and analysis afterwards, not during the test.

## 0. Network (done 2026-09-09)

Both machines are on the same switch, subnet 192.168.201.0/24: the robot PC
is **192.168.201.158** (`robot_host` in the plans), the Mac's USB-Ethernet
adapter (en6) gets 192.168.201.245 by DHCP. The Mac's Wi-Fi (10.21.x, the
campus network) cannot reach the robot PC's Franka link 172.16.0.1 — that
address is only the cable to the arm. ufw on the robot PC is inactive.
The Mac connects TO the robot PC, TCP port 5600 (the robot script is the
server and listens only while a run is active); nothing connects to the Mac.

Check each session (the adapter must be plugged in and active):
```bash
ping -c 2 192.168.201.158
```
(A connection test end-to-end: start `run_cycles.sh 1` on the robot PC,
then on the Mac `nc -vz 192.168.201.158 5600` while the script waits at the
initial pose — it prints "succeeded". Ctrl-C the robot script afterwards.)

## 1. Robot PC (once): the patched script + the 9-pose file

Copy the three files from the Mac (`~/Documents/repos/robo_arm`) next to the
existing script on the robot PC (`/home/leon/shared_ws/SynthVLA/src/panda_moveit_ctrl/scripts/`):
```bash
# on the Mac
cd ~/Documents/repos/robo_arm && scp panda_manual_poses.py confirm_socket.py run_cycles.sh \
    leon@192.168.201.158:/home/leon/shared_ws/SynthVLA/src/panda_moveit_ctrl/scripts/
```
`panda_manual_poses.py` replaces the 2026-08-31 version (same file + the
socket mode; keyboard mode unchanged). `confirm_socket.py` has no ROS
dependency. `chmod +x run_cycles.sh`.

**Pose files on the robot PC are NOT modified** (decided 2026-09-09). The
patched `panda_manual_poses.py` reads `manual_poses.csv` / `joint_start.csv`
in any layout: comma / semicolon / tab / space separated, quoted
`"j1, ..., j7",x,y,z,...` rows, bracketed lists, or a header line (then
`joint1..joint7` are taken by column name). Tested 2026-09-09 on all of
these and on the robot PC's own files (copies) — an earlier version of the
loader rejected tab/space-separated rows, that was the parse error. What the robot PC's files contain decides the plan: `poses:` =
rows of its `manual_poses.csv`, the robot announces the count at connect
and the coordinator warns on a mismatch. The initial pose (`joint_start.csv`)
is robot pose 0 and gets the first stack (`shoot_initial_pose: true`).
Reference copies built from the aug31 dataset (`robot/manual_poses.csv` =
aug31 recorded poses 1–7 + vertical; `robot/joint_start_aug31.csv` = the
aug31 initial pose) are for comparison only: copy the robot PC's files to
the Mac (read-only) and diff them, then set `poses:` and the focus-table
pose order accordingly.

## 2. Robot PC terminals (every session)

```bash
cd /home/leon/shared_ws/SynthVLA && source devel/setup.bash      # in EVERY terminal
# T1
roslaunch panda_moveit_config franka_control.launch robot_ip:=172.16.0.2 load_gripper:=false
# T2
rosrun panda_moveit_ctrl panda_moveit_ctrl_server_node.py
# T3 (optional, joint log for later registration)
rosrun panda_moveit_ctrl record_joint_states.py
# T4 — ONE run (the full dataset later: run_cycles.sh 36)
bash src/panda_moveit_ctrl/scripts/run_cycles.sh 1 _confirm_port:=5600
```
Robot unlocked in Franka Desk, FCI enabled, e-stop at hand, workspace clear.
Nothing has to be typed on the robot PC during the runs: every Enter of the
old procedure is now a message from the Mac. Ctrl-C in T4 between runs
stops the test.

## 3. Mac (every session)

Phone: on USB-C, **unlocked** (Settings → Display & Brightness → Auto-Lock →
Never for the session), app in the foreground:
```bash
cd ~/Documents/repos/image-capture
xcrun devicectl device process launch --device F69E5036-B4BE-5579-B8CD-DDC135DF373F --terminate-existing ch.idsia.capture
```
Coordinator (starts the USB tunnel itself; no OptiTrack):
```bash
bash tools/run_coordinator.sh --plan coordinator/plan.maintest.yaml --tunnel --mocap none
```
It prints the session folder (`datasets/phone_<YYYYMMDD_HHMMSS>/`), sends
`hello` to the phone (eco mode: 10 fps, preview off, screen dim, ProRAW
speed), then waits for the robot's first `reached 0`.

## 4. What happens and what you press (the one run)

| moment | robot PC | Mac |
|---|---|---|
| run starts | arm to the initial pose, `reached 0` | prompt: *place/check the insect (step 0), Enter* → then, at the initial pose (robot pose 0): lens to 0.3333 (IMG_6823) → prompt *Enter = shoot* → 50 frames → pull → `go 0` |
| pose k reached (1–8) | `reached k` | lens set to the init focus of pose k (e.g. pose 1: 0.2510 = IMG_6859) → prompt: *lens at 0.2510 (table:IMG_6859). Enter = shoot, or type 0..1 to re-centre, `skip`, `abort`* |
| Enter | holds | 50 frames 0.0..0.5 (the init focus is exactly one of them), ~100 s; then pull (50 DNG + 50 JSON) → `go k` |
| after pose 8 | returns to the initial pose, `done` | session ends: `session <ts> finished: {'stops_ok': 9, ...}` |

Typing a value at the focus prompt moves the lens there immediately (the
preview is off in eco mode — turn it on with the **Prev** button on the
phone or `python3 tools/mac_client.py ui --preview on` if you want to look),
and the stack is rebuilt around the new value. `skip` skips that pose,
`abort` ends the session (the arm returns via its normal path).

A failed stop (phone error, missing files) is retried twice, then you are
asked *retry / skip / abort*.

## 5. Output

`datasets/phone_<ts>/`: `images/` (DNG), `sidecars/` (JSON per frame),
`frames.csv` (stop, step, pose, focus_index, lens_position, focus_centre,
focus_source, fx fy cx cy, capture_time — no mocap columns), `session.jsonl`
(every message both ways, prompts, timings), `plan.json`. The phone keeps
its own copy of every frame (Files app > CaptureApp > `<session>`).

Afterwards (not during the test):
```bash
PYTHONPATH=coordinator pixi run --manifest-path ~/Documents/repos/optitrack/pixi.toml \
    python -m capture_coordinator.stack_merge datasets/phone_<ts> --jobs 4
```

## 6. Before the lab: dry run of exactly this plan on the Mac alone

```bash
# ports 5611/5610 so a running USB tunnel (5601) is not hit by mistake
python3 -m coordinator.capture_coordinator.fake_phone --docs-dir /tmp/fakephone --port 5611 &   # add --real-frames in the pixi env for mergeable PNGs
python3 robot/fake_robot.py --port 5610 --poses 8 --cycles 2 --move-time 0.5 &
sed 's/^robot_host:.*/robot_host: 127.0.0.1/; s/^robot_port:.*/robot_port: 5610/; s/^phone_port:.*/phone_port: 5611/; s/^turntable_steps:.*/turntable_steps: 2/; s/^frames_per_stop:.*/frames_per_stop: 3/' \
    coordinator/plan.maintest.yaml > /tmp/plan.dry.yaml
PYTHONPATH=coordinator python3 -m capture_coordinator.coordinator --plan /tmp/plan.dry.yaml \
    --mocap none --puller local --fake-phone-dir /tmp/fakephone --output-root /tmp/sessions
```
You get the same prompts as in the lab (Enter at every pose and turn).

## 7. Open items (need you)

- The robot PC's `joint_start.csv` must be the aug31 initial pose (section 1).
- The popillia/phone marker bodies are irrelevant for this test (no mocap).
