# Recording a dataset (camera pose + RGB-D)

Step-by-step guide to record a synchronized dataset: a marker-tracked Orbbec
Femto Mega films a scene while OptiTrack streams the camera's pose, all captured
to a single rosbag and exported to per-frame `image + pose` rows.

> **Goal format:** one color image ↔ one camera pose, correctly time-aligned.
> The pipeline stamps each frame with its *capture* time and interpolates the
> OptiTrack pose to that instant, so USB/network lag does not mistime the link.

---

## 0. Status / read before recording a *final* dataset

- **USB link:** the Femto Mega must be on a **USB 3.0** link for full quality.
  Today it enumerates at **USB 2.0 (480 Mb/s)** — bandwidth-limited (forces
  compressed color, risks frame drops). See [Pre-flight](#2-pre-flight-checks).
- **Timestamp fix:** the camera capture timestamp is being corrected to use the
  device hardware clock (the SDK's global clock intermittently duplicates
  stamps). Until that fix is committed and re-validated, treat exported poses as
  **provisional** — verify with the health check in [step 6](#6-verify-the-dataset).
- **For Gaussian Splatting specifically:** you also need a **hand-eye
  calibration** (marker rigid body → RGB optical center). OptiTrack tracks the
  marker cluster, not the lens; without that transform the poses are offset.
  Not yet implemented.

---

## 1. One-time setup (per machine)

```bash
# Install the environment (~10 min first run; pulls ROS 2 Humble)
cd ~/optitrack        # must be a local-disk path, no spaces, NOT iCloud Drive
pixi install

# Grant passwordless sudo for the camera (macOS needs root to claim the USB
# device). Run the script DIRECTLY — not via `sudo pixi run`.
sudo bash scripts/setup_usb_permissions.sh
# Expect: "Done. Installed: /etc/sudoers.d/orbbec_camera"
```

### Configure Motive (once, on the Windows PC)
In Motive → **View → Data Streaming Pane → OptiTrack Streaming Engine**:
- **Streaming Engine: ON** (all other engines OFF)
- **Transmission Type: Unicast**
- **Local Interface:** the PC's lab-LAN Ethernet (`192.168.201.X`)
- **Rigid Bodies: ON**, command port `1510`, data port `1511`
- Give the camera rigid body a clear **name** (e.g. `camera`) — you pass this to
  the exporter. Save the streaming profile.

---

## 2. Pre-flight checks

Run these every session before recording.

```bash
# a) Camera present + LINK SPEED. Want "Up to 5 Gb/s". "Up to 480 Mb/s" = USB 2.0
#    (bandwidth-limited): swap to a certified SuperSpeed USB-C cable / USB 3 port.
system_profiler SPUSBDataType | grep -A3 -i femto

# b) Camera timestamp health (device clock steady, ~33 ms, no duplicates).
#    Streams ~40 s and prints per-clock stats.
sudo pixi run python scripts/diag_timestamps.py --frames 1200

# c) OptiTrack reachable and streaming poses (Ctrl-C after you see poses):
pixi run launch
#    Expect: "Connected to server 192.168.201.5:1510" and
#            "[camera] pos=(...)  quat=(...)" lines.
```

**If the camera errors on start** (`recursive_mutex` / `unknown exception`):
fully **power-cycle** it — unplug the USB-C completely, wait ~15 s, replug. A
half-open device does not recover on its own.

---

## 3. Record

```bash
# Bag lands in datasets/optitrack_<timestamp>/ . Ctrl-C to stop.
pixi run record
```

- Needs root; the sudoers rule from step 1 makes it password-free. (First ever
  run, before that rule: `sudo pixi run record`.)
- Records: `/optitrack/*` poses, `/tf`, color (compressed), depth, camera_info,
  diagnostics.
- **Capture tips for reconstruction/GS:** move **slowly and smoothly** (motion
  blur and fast motion hurt both image quality and pose interpolation); keep the
  subject **0.5–3.86 m** away (NFOV depth range); ensure the camera's markers
  stay visible to OptiTrack the whole time (watch for tracking dropouts).

Override lab defaults per run if needed:
```bash
OPTITRACK_PC_IP=192.168.201.6 pixi run record        # different Motive PC
CAMERA_COLOR_WIDTH=1280 CAMERA_COLOR_HEIGHT=720 pixi run record   # lighter load
```

### Dry run without hardware
```bash
pixi run record-mock     # synthetic camera + pose, no camera, no sudo
```

---

## 4. Stop

Press **Ctrl-C** in the recording terminal. Wait for it to flush and for the
`chown` to hand the bag back to your user (the task does this automatically).

---

## 5. Export to frames + per-frame pose

```bash
pixi run export -- datasets/optitrack_<timestamp> --rigid-body camera
```
- `--rigid-body` = the Motive name of the camera body (auto-detected if the bag
  has exactly one `/optitrack/*` topic).
- Useful flags: `--time-offset <sec>` (cancel a residual constant lag),
  `--max-pose-gap <sec>` (drop frames near tracking dropouts, default 0.1).

Output in `datasets/optitrack_<timestamp>_export/`:
```
color/000000.jpg ...     camera-native JPEGs
depth/000000.png ...     16-bit depth, millimetres
camera_info.yaml         color + depth intrinsics
frames.csv               one row per color frame:
                         index, color_file, depth_file, t_capture_ns,
                         x,y,z, qx,qy,qz,qw, pose_gap_ms, depth_dt_ms
```

---

## 6. Verify the dataset

```bash
# Frame count + how many were dropped (out of pose coverage / tracking gaps)
tail -n +1 datasets/optitrack_<timestamp>_export/frames.csv | wc -l

# HEALTH CHECK: capture timestamps must be unique & steadily increasing.
# Duplicate t_capture_ns across rows = the timestamp bug (see Status). Quick look:
cut -d, -f4 datasets/optitrack_<timestamp>_export/frames.csv | tail -n +2 | uniq -d | head
#   ^ prints nothing if all timestamps are unique (good).
```
Also sanity-check a depth frame opens and has real structure (values in mm, some
zero/invalid regions) and that `pose_gap_ms` stays small (large values = pose
interpolated across a tracking dropout).

---

## 7. Visualize (optional)

```bash
pixi run rviz     # loads rviz/optitrack.rviz
```

---

## Troubleshooting quick reference

| Symptom | Fix |
|---|---|
| Camera link shows `480 Mb/s` | USB 2.0 — use a SuperSpeed USB-C cable + USB 3 port |
| `recursive_mutex` / `unknown exception` on camera start | Power-cycle the camera (unplug 15 s, replug) |
| `setup_usb_permissions.sh`: "pixi not found" | Run it directly (`sudo bash …`), not via `sudo pixi run` |
| Empty bag / `ros2 topic list` hangs | Off the lab LAN? `ROS_LOCALHOST_ONLY=1` is set; check Motive streaming + `OPTITRACK_PC_IP` |
| Export: "no pose messages on /optitrack/…" | Wrong `--rigid-body` name; run without it to see available topics |
| Many frames dropped at export | Tracking dropouts (raise `--max-pose-gap`) or capture-time outside pose span |
| Duplicate `t_capture_ns` in frames.csv | Camera timestamp bug — see Status; needs the device-clock fix |
