# OptiTrack → ROS 2 (Humble) on Mac — pixi edition

Native macOS setup: receives NatNet from Motive on the lab LAN, exposes
each rigid body as a ROS 2 `geometry_msgs/PoseStamped` topic plus TF.
**No Docker** (Docker on Mac can't handle NatNet's asymmetric UDP ports).
**Conda env managed by [pixi](https://pixi.sh)** — `pixi run launch` is
the only command you need.

## Why this lives at `~/optitrack` (not in iCloud Drive)

The lab project tree lives in iCloud Drive at
`~/Library/Mobile Documents/com~apple~CloudDocs/Luca/PhD/Optitrack/`, but
this workspace cannot. Two reasons, both incompatible with `Library/Mobile
Documents/`:

1. **Space in the path** breaks colcon's generated `install/setup.bash`
   (line 106 expands `$_colcon_python_executable` unquoted; bash splits at
   the first space).
2. **iCloud dematerialization** evicts binary files (including the
   environment's `python3.11`) from local disk when iCloud decides space
   is tight, leaving 5 MB placeholders that exec as `Undefined error: 0`.

So the pixi env (`.pixi/`), colcon build outputs (`build/`, `install/`,
`log/`), and the rest of this workspace live at `~/optitrack/` on local
disk. Back up via git, not iCloud.

---

## Setup (first time on this Mac)

```bash
# install pixi if you don't have it
curl -fsSL https://pixi.sh/install.sh | bash
# (or: brew install pixi)

cd ~/optitrack
pixi install        # downloads ROS 2 Humble via RoboStack + pip deps, ~5–10 min
```

That's it. No conda activate, no `setup.sh`. Pixi handles env state.

If you ever want to wipe & start over: `rm -rf .pixi build install log && pixi install`.

---

## Daily usage

In one terminal, with Motive running on the Windows PC:

```bash
cd ~/optitrack
pixi run launch
```

That's the only command. Under the hood pixi:

1. Activates the conda env (sources RoboStack ROS 2).
2. Runs `patch-natnet` (idempotent — one-line fix to upstream natnet_py;
   see "The natnet_py patch" below).
3. Runs `colcon build` if the workspace needs it (incremental, ~1 s steady
   state). Builds the two ament packages in `src/`:
   `optitrack_ros_py` and `optitrack_listener`.
4. Sources `install/setup.bash`.
5. Runs `ros2 launch optitrack_listener optitrack.launch.py`.

Expected output once steady-state (rigid body called `GATE` in this lab):

```
[optitrack_ros]: Connecting to 192.168.201.5 ...
[optitrack_ros]: Connected to server 192.168.201.5:1510
[optitrack_ros]: Use RigidBodyConfig(name='GATE', tf=True, topic='/optitrack/GATE', ...) for GATE
[optitrack_listener]: Subscribed to /optitrack/GATE
[optitrack_listener]: [GATE] pos=(+0.206, -0.213, +1.420) quat=(...)
```

Stop with `Ctrl-C`.

### Visualizing in RViz

In a second terminal:

```bash
cd ~/optitrack
pixi run rviz
```

The first time, configure RViz (the launched window starts with
`Frame [map] does not exist` — that's expected):

1. **Displays panel → Global Options → Fixed Frame** — change `map` → **`world`**.
2. **Add** (bottom-left) → **By topic** tab → `/tf` → **TF**. Triads appear.
3. **Add** → **By topic** → `/optitrack/GATE` → **PoseStamped**. Arrow appears.
4. **File → Save Config As… → `rviz/optitrack.rviz`**.

Next time, `pixi run rviz` loads the saved config and skips manual setup.

---

## ROS topics

| Topic                          | Type                          | Description                            |
|--------------------------------|-------------------------------|----------------------------------------|
| `/optitrack/<rigid_body>`      | `geometry_msgs/PoseStamped`   | One topic per rigid body Motive streams |
| `/tf`                          | `tf2_msgs/TFMessage`          | `world → <rigid_body>` for each body    |

To inspect from a third terminal:

```bash
cd ~/optitrack
pixi run ros2 topic list                      # any pixi-run-prefixed command works
pixi run ros2 topic hz /optitrack/GATE
pixi run ros2 topic echo /optitrack/GATE
```

---

## Editing your subscriber

Your code: [src/optitrack_listener/optitrack_listener/listener.py](src/optitrack_listener/optitrack_listener/listener.py).

After editing, just `pixi run launch` again — colcon will pick up the
change (incremental rebuild) and the listener re-launches.

If you change `setup.py` / `package.xml` (uncommon), do a clean build:

```bash
cd ~/optitrack
rm -rf build install log && pixi run launch
```

---

## Tuning the driver

Driver parameters are set in
[src/optitrack_listener/launch/optitrack.launch.py](src/optitrack_listener/launch/optitrack.launch.py).
The launch file reads three env vars with lab-default fallbacks:

| Env var              | Default            | Used for                                  |
|----------------------|--------------------|-------------------------------------------|
| `OPTITRACK_PC_IP`    | `192.168.201.5`    | Motive PC's IP                            |
| `MY_IP`              | `192.168.201.89`   | This Mac's lab-LAN IP (NIC bind address)  |
| `OPTITRACK_FRAME_ID` | `world`            | Top-level TF frame                        |

Override per-invocation:

```bash
OPTITRACK_PC_IP=192.168.201.6 pixi run launch
```

Full driver param surface is in
[src/optitrack_ros_py/optitrack_ros_py/optitrack_ros.py](src/optitrack_ros_py/optitrack_ros_py/optitrack_ros.py).

---

## The `natnet_py` patch

Upstream `jeguzzi/natnet_py` (commit `e5deb0e7`) has a real bug:
`AsyncClient.connect()`'s direct-connect branch never updates
`self.cmd_protocol._server` from its constructor default of `('', 1510)`.
The discovery branch does it; the direct-connect branch doesn't. So the
first `sendto(ConnectRequest, ('', 1510))` goes to an empty-string
destination and macOS rejects it with **`[Errno 65] No route to host`**.
The lab's Linux+Docker setup never hit this because it goes through
discovery; our direct-connect path on Mac does.

The patch ([scripts/patch_natnet.py](scripts/patch_natnet.py)) inserts
one missing line right after the `info("Connecting to ...")` log:

```python
self.cmd_protocol._server = server
```

It's idempotent and applied automatically before every build. Worth
filing upstream — when Jerome merges a fix, delete `patch-natnet` from
[pixi.toml](pixi.toml) and the script.

---

## Why is there an `optitrack_msgs` stub?

`optitrack_ros_py` unconditionally `import optitrack_msgs.msg` at the
top of its source file, but Jerome hasn't published `optitrack_msgs`
publicly. The message classes are only *instantiated* when
`publish_description` or `publish_data` (both default `false`) are on, so
the actual functionality doesn't depend on the real messages — only the
*import* has to succeed. `src/optitrack_msgs_stub/` is a 6-line PEP 562
shim that returns a placeholder class for any attribute access; pixi
installs it editable.

If Jerome publishes the real `optitrack_msgs`, swap the path entry in
[pixi.toml](pixi.toml)'s `pypi-dependencies` to install the real one.

---

## Troubleshooting

### `[Errno 65] No route to host` immediately on connect

The `natnet_py` patch isn't applied. Verify:

```bash
grep -A2 'Connecting to {server_address}' src/natnet_py/natnet_py/async_client.py
```

You should see `self.cmd_protocol._server = server` on the line right
after the `info(...)` call. If not, re-run `pixi run launch` (which calls
`patch-natnet` automatically), or manually `pixi run patch-natnet`.

### `Command request ConnectRequest timed out after 5.0 s`

Motive isn't reachable from this Mac. Check:

```bash
ping -c2 192.168.201.5
pixi run python -c "import socket, struct; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(2); s.sendto(struct.pack('<HH',0,0), ('192.168.201.5',1510)); print('reply', s.recvfrom(2048)[0][:20])"
```

If ping fails: wrong subnet / Mac unplugged from lab LAN.
If ping works but the probe times out: Motive's Streaming Engine is off,
or it's in Multicast mode (switch to Unicast in the Streaming pane).

### `[Errno 48] Address already in use` on port 1511

A previous run is still holding it. Find and kill:

```bash
lsof -nP -iUDP:1511 -t | xargs -r kill -9
```

### Workspace state got weird (mysterious build errors)

Clean rebuild:

```bash
cd ~/optitrack
rm -rf build install log
pixi run launch
```

If that doesn't help, nuke the env too:

```bash
rm -rf .pixi build install log
pixi install && pixi run launch
```

---

## File layout

```
~/optitrack/
├── README.md                 ← this file
├── pixi.toml                 ← deps + tasks
├── pixi.lock                 ← reproducible lock file (commit to git)
├── scripts/
│   └── patch_natnet.py       ← idempotent upstream bug fix
├── rviz/                     ← (your saved RViz configs go here)
└── src/
    ├── natnet_py/                  pure-Python NatNet client (patched)
    ├── optitrack_ros_py/           NatNet → ROS 2 driver node
    ├── optitrack_listener/         your example subscriber — edit listener.py
    └── optitrack_msgs_stub/        placeholder for the unpublished optitrack_msgs
```

---

## Credits

- `natnet_py` and `optitrack_ros_py`: **Jerome Guzzi** (IDSIA).
- RoboStack: the conda-forge community.
- pixi: prefix.dev.

This workspace just glues them together for native macOS use.
