# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Native-macOS (Apple Silicon) setup that receives OptiTrack/Motive motion-capture
data as ROS 2 topics and records it alongside an Orbbec Femto Mega RGB-D camera,
so every camera frame can later be tagged with the camera's mocap pose at its
capture instant. Built on [pixi](https://pixi.sh) + RoboStack ROS 2 Humble — there
is no system ROS install; everything lives in the pixi conda env.

## Commands

All commands run through pixi tasks (defined in `pixi.toml`), never bare `ros2`:

- `pixi install` — resolve + install the env (~10 min first time; downloads ROS 2 Humble).
- `pixi run build` — `colcon build` the workspace (runs `patch-natnet` first). `launch`/`record`/`camera` depend on this, so it runs automatically.
- `pixi run launch` — OptiTrack driver + example listener. No camera, no bag. The everyday command.
- `pixi run record` — driver + camera + rosbag → `datasets/optitrack_<timestamp>/`. **Needs root** (see USB/root note). First time: `sudo pixi run record`; after `setup-usb-perms`: plain `pixi run record`.
- `pixi run record-mock` — same pipeline with synthetic camera + pose, no hardware, no sudo. Use this to test recording/export logic.
- `pixi run camera` — only the Orbbec camera node. Also needs root.
- `pixi run export -- datasets/<bag> [--rigid-body NAME] [--time-offset SEC]` — bag → `color/*.jpg`, `depth/*.png`, `camera_info.yaml`, `frames.csv`.
- `pixi run setup-usb-perms` — one-time passwordless-sudo grant for `record`.
- `pixi run rviz` — RViz with the saved `rviz/optitrack.rviz` layout.

There is no test suite for the project code. The `test/` dirs under `src/natnet_py`
and `src/optitrack_ros_py` are upstream lint stubs (flake8/pep257/copyright), not
behavioral tests.

To run a single node manually for debugging, source the overlay first:
`source install/setup.bash && ros2 run <package> <executable>`.

## Architecture

ROS 2 dataflow (one bag captures all of it):

```
Motive PC ──NatNet/UDP──▶ optitrack_ros ──▶ /optitrack/<body> (PoseStamped) + /tf
Femto Mega ──USB──▶ orbbec_camera ──▶ /camera/color/image_raw/compressed + /camera/depth/image_raw + camera_info
                                          │
                                    ros2 bag record (-e regex)
                                          ▼
                          datasets/optitrack_<ts>/  ──export_dataset.py──▶ frames.csv (per-frame interpolated pose)
```

### Packages (`src/`)
- **`optitrack_ros_py`** — the driver. Vendored copy of Jerome Guzzi's `optitrack_ros_py`. `optitrack_ros.py` is an asyncio node that connects to Motive, and for every rigid body Motive streams publishes a `PoseStamped` on `/optitrack/<name>` and a TF. The `rigid_bodies.default.*` params are a template where the literal string `NAME` is replaced per body at runtime. Coordinate conversion (Motive Y-up → ROS) happens in `pose_msg()`.
- **`natnet_py`** — vendored pure-Python NatNet client used by the driver. Installed **editable** so `scripts/patch_natnet.py` stays applied (see below).
- **`orbbec_camera_ros`** — `camera_node.py`, the Femto Mega driver via `pyorbbecsdk`. Has a `mock` param for hardware-free synthetic frames.
- **`optitrack_listener`** — this repo's own package. `listener.py` (example subscriber that auto-discovers `/optitrack/*` topics), `mock_pose_publisher.py` (synthetic poses for `record-mock`), and **all the launch files** (`optitrack.launch.py`, `record.launch.py`, `camera.launch.py`). Edit `listener.py` to build your own subscriber.
- **`optitrack_msgs_stub`** — empty stub satisfying `optitrack_ros.py`'s unconditional `import optitrack_msgs.msg`. The real messages are only used when `publish_data`/`publish_description` are on (both default off).

### Non-obvious constraints
- **Editable vs colcon.** `natnet_py` and `optitrack_msgs_stub` are pip-editable (in `pixi.toml`); the two ament_python packages (`optitrack_ros_py`, `optitrack_listener`, `orbbec_camera_ros`) are **not** — they're colcon-built so their `data_files` register in the ament index. Don't move them to pypi-dependencies; `ros2 launch` won't find them.
- **natnet_py patch.** Upstream `AsyncClient.connect()` forgets `self.cmd_protocol._server = server` in the direct-connect branch, causing `[Errno 65] No route to host`. `scripts/patch_natnet.py` inserts it idempotently and runs before every build. If natnet_py source changes upstream, this patch's marker check will fail loudly — review it then.
- **`ROS_LOCALHOST_ONLY=1`** is forced in `pixi.toml` `[activation.env]`. NatNet↔Motive is plain UDP, not DDS; off the lab LAN, CycloneDDS otherwise binds the wrong interface and local nodes never discover each other (empty bags, hanging `ros2 topic list`). Keep it on.
- **`client_address` matters.** It's the local socket bind address. On a multi-interface Mac, binding `0.0.0.0` fails to route to Motive; the launch files pick the outbound IP via a UDP `connect()` trick (`_default_client_ip`). Override with `MY_IP` if needed.
- **Root for the camera.** macOS attaches its kernel UVC driver to the Femto Mega; libusb must detach it, which needs root. The `record`/`camera` tasks `chown` build/install/log/datasets back to the real user afterward so the next non-sudo build doesn't hit EPERM on root-owned files.
- **Color is passed through as MJPEG**, never decoded in the camera node (decoding 1080p in Python managed ~2 fps). The bitstream is recorded as-is on `.../image_raw/compressed`; `export_dataset.py` writes those JPEG bytes straight to `.jpg`. Old raw-`rgb8` bags on `.../image_raw` are still handled.
- **Timestamps carry capture time, not arrival time.** Camera frames use the SDK global (hardware-clock-fitted) timestamp; poses use the NatNet clock-synced mid-exposure time (`sync=True, restamp=False`). This removes USB/network lag. A residual constant offset is corrected at export with `--time-offset`, no re-recording.
- **rosbag record uses a single `-e` regex**, never a topic list alongside `-e` — on Humble that combo silently records nothing (empty bags). See `record.launch.py`.

### Export pipeline (`scripts/export_dataset.py`)
Two passes over the bag: pass 1 builds a time-sorted `PoseTrack` and depth index; pass 2 walks color frames, interpolates the camera pose (position lerp + quaternion slerp) at each frame's capture time, pairs the nearest depth frame, and writes one `frames.csv` row. Frames whose capture time falls outside pose coverage, or whose bracketing poses are >`--max-pose-gap` (default 0.1 s, i.e. tracking dropout) apart, are dropped. Rigid body auto-detected when the bag has exactly one `/optitrack/*` topic.

## Lab environment defaults
Driver reads three env vars (lab defaults): `OPTITRACK_PC_IP=192.168.201.5`,
`MY_IP=192.168.201.89`, `OPTITRACK_FRAME_ID=world`. Override per run, e.g.
`OPTITRACK_PC_IP=192.168.201.6 pixi run launch`. Motive must have the OptiTrack
Streaming Engine ON, Unicast, rigid bodies ON, command port 1510 / data port 1511.

## Repo hygiene
Clone must live on a **local-disk path with no spaces** (not iCloud Drive) — RoboStack/colcon break otherwise. `build/`, `install/`, `log/`, `datasets/` are generated.
