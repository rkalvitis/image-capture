# CLAUDE.md

> **Since 2026-09-28 this folder is a subtree of the `image-capture` repo** (branch
> `image-capture-pipeline`): the session coordinator lives in `../coordinator`, the plan
> files in `../coordinator/plan.*.yaml`, and `../tools/run_coordinator.sh` runs the
> coordinator inside this pixi env. `pixi.toml` targets macOS and Linux (`linux-64`,
> `linux-aarch64`) since the goal is one Ubuntu computer; "this Mac" below reads as
> "the machine running the session". `datasets/`, `calibration/*/photos|bag`, `docs/`,
> `thesis/`, `3dprinter/` are git-ignored and still live in `~/Documents/repos/optitrack/`.
> The submodule `src/optitrack_msgs` now uses the https URL. Read `../CLAUDE.md` first.

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Native-macOS (Apple Silicon) setup that receives OptiTrack/Motive motion-capture
data as ROS 2 topics and records rigid-body poses during **iPhone photo
sessions**: the phone is a tracked rigid body (`phone`), photos are taken
stop-and-shoot (hold still ~3 s per shot), and `scripts/match_photos.py` later
pairs each photo with the phone's pose via EXIF timestamps. Built on
[pixi](https://pixi.sh) + RoboStack ROS 2 Humble — there is no system ROS
install; everything lives in the pixi conda env.

Plan documents: `docs/IPHONE_IMPLEMENTATION_PLAN.md` (build list, staged) and
`docs/IPHONE_CAPTURE_PLAN.md` (rationale, risks, decisions).

## Commands

All commands run through pixi tasks (defined in `pixi.toml`), never bare `ros2`:

- `pixi install` — resolve + install the env (~10 min first time; downloads ROS 2 Humble).
- `pixi run build` — `colcon build` the workspace (runs `patch-natnet` first). `launch`/`record-poses` depend on this, so it runs automatically.
- `pixi run launch` — OptiTrack driver + example listener. No bag. The everyday command.
- `pixi run record-poses` — driver + rosbag of every `/optitrack/*` body → `datasets/phone_<timestamp>/`. No sudo. Run for the duration of a photo session.
- `pixi run record-mock` — same pipeline with a synthetic pose publisher; no hardware, no OptiTrack server. Use this to test recording/matching logic.
- `pixi run ms-clock` — millisecond clock display; photograph it at session start/end (clock photo, measures the iPhone↔Mac clock offset).
- `pixi run match -- datasets/phone_<ts> --clock-shows HH:MM:SS.mmm [--bodies phone phone_mount popillia_mount]` — match `photos/*.dng` to poses → `frames.csv` (+ `rejected.csv`).
- `pixi run rviz-optitrack` — RViz with the saved `rviz/optitrack.rviz` layout.

There is no test suite for the project code, except `scripts/match_photos.py
--self-test` (synthetic poses + fabricated EXIF times, no lab needed). The
`test/` dirs under `src/natnet_py` and `src/optitrack_ros_py` are upstream lint
stubs, not behavioral tests.

To run a single node manually for debugging, source the overlay first:
`source install/setup.bash && ros2 run <package> <executable>`.

## Architecture

```
Motive PC ──NatNet/UDP──▶ optitrack_ros ──▶ /optitrack/<body> (PoseStamped) + /tf
                                          │
                                    ros2 bag record (-e regex)
                                          ▼
iPhone (ProRAW stills, ──AirDrop──▶ datasets/phone_<ts>/photos/*.dng
 stop-and-shoot)                          │
                                   match_photos.py (EXIF time ⇄ pose window)
                                          ▼
                                   frames.csv (one photo ↔ one median pose)
```

### Packages (`src/`)
- **`optitrack_ros_py`** — the driver. Vendored copy of Jerome Guzzi's `optitrack_ros_py`. `optitrack_ros.py` is an asyncio node that connects to Motive, and for every rigid body Motive streams publishes a `PoseStamped` on `/optitrack/<name>` and a TF. The `rigid_bodies.default.*` params are a template where the literal string `NAME` is replaced per body at runtime. Coordinate conversion (Motive Y-up → ROS) happens in `pose_msg()`.
- **`natnet_py`** — vendored pure-Python NatNet client used by the driver. Installed **editable** so `scripts/patch_natnet.py` stays applied (see below).
- **`optitrack_listener`** — this repo's own package. `listener.py` (example subscriber that auto-discovers `/optitrack/*` topics), `mock_pose_publisher.py` (synthetic poses for `record-mock`), and the launch files (`optitrack.launch.py`, `record.launch.py`). Edit `listener.py` to build your own subscriber.
- **`optitrack_msgs_stub`** — empty stub satisfying `optitrack_ros.py`'s unconditional `import optitrack_msgs.msg`. The real messages are only used when `publish_data`/`publish_description` are on (both default off).

### Photo sessions (`scripts/`)
Session folder contract (everything reads/writes this layout):

```
datasets/phone_<YYYYMMDD_HHMMSS>/
  <rosbag files>          # poses-only bag from record-poses
  photos/*.dng            # the dataset shots (ProRAW)
  clock/*.dng             # clock photos (session start + end)
  frames.csv              # output of match_photos.py
```

`match_photos.py` reads each DNG's EXIF capture time (`DateTimeOriginal` +
`SubSecTimeOriginal` + `OffsetTimeOriginal`, parsed with a stdlib TIFF walk —
no exif library), applies the clock offset measured from the clock photo
(`--clock-shows`), and takes the **median pose over a still window** around
each shot; photos without a still window (phone was moving) are rejected.
Until the Stage 2 hand-eye calibration exists, poses are of the marker body,
not the lens optical center.

### Non-obvious constraints
- **Editable vs colcon.** `natnet_py` and `optitrack_msgs_stub` are pip-editable (in `pixi.toml`); the two ament_python packages (`optitrack_ros_py`, `optitrack_listener`) are **not** — they're colcon-built so their `data_files` register in the ament index. Don't move them to pypi-dependencies; `ros2 launch` won't find them.
- **natnet_py patch.** Upstream `AsyncClient.connect()` forgets `self.cmd_protocol._server = server` in the direct-connect branch, causing `[Errno 65] No route to host`. `scripts/patch_natnet.py` inserts it idempotently and runs before every build. If natnet_py source changes upstream, this patch's marker check will fail loudly — review it then.
- **`ROS_LOCALHOST_ONLY=1`** is forced in `pixi.toml` `[activation.env]`. NatNet↔Motive is plain UDP, not DDS; off the lab LAN, CycloneDDS otherwise binds the wrong interface and local nodes never discover each other (empty bags, hanging `ros2 topic list`). Keep it on.
- **`client_address` matters.** It's the local socket bind address. On a multi-interface Mac, binding `0.0.0.0` fails to route to Motive; the launch files pick the outbound IP via a UDP `connect()` trick (`_default_client_ip`). Override with `MY_IP` if needed.
- **Pose timestamps carry capture time, not arrival time.** The driver runs with `sync=True, restamp=False`: poses are stamped with the mocap mid-exposure time converted through the NatNet clock sync, removing network lag.
- **rosbag record uses a single `-e` regex**, never a topic list alongside `-e` — on Humble that combo silently records nothing (empty bags). See `record.launch.py`.
- **Photo timestamps:** EXIF stores capture time to 1 ms resolution, but true accuracy is ~tens of ms (NTP clock offset + multi-frame merge). The stop-and-shoot protocol (~3 s still per shot) makes this irrelevant — any timestamp inside the still window selects the same pose. The clock photo measures the residual offset per session.

## Lab environment defaults
Driver reads three env vars (lab defaults): `OPTITRACK_PC_IP=192.168.201.5`
(the Motive PC, hostname dti-22398; verified alive 2026-07-21 — NOT `.245`,
which is the lab Mac's own address), `MY_IP` (auto-detected from the route to
the Motive PC; fallback `192.168.201.245` = the lab Mac),
`OPTITRACK_FRAME_ID=world`. `MY_IP` must NOT be set in `pixi.toml` activation
env — that would override both auto-detection and per-run shell overrides.
Override per run, e.g. `OPTITRACK_PC_IP=192.168.201.6 pixi run launch`. Motive
must have the OptiTrack Streaming Engine ON, Unicast, rigid bodies ON, command
port 1510 / data port 1511. The tracked rigid bodies must be named exactly
**`phone`**, **`phone_mount`**, **`popillia_mount`** in Motive (topic names
are case-sensitive).

## Repo hygiene
Clone must live on a **local-disk path with no spaces** (not iCloud Drive) —
RoboStack/colcon break otherwise. `build/`, `install/`, `log/`, `datasets/`
are generated.
