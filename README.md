# OptiTrack → ROS 2 on Mac

Native macOS setup for receiving OptiTrack/Motive NatNet data as ROS 2
topics, using [pixi](https://pixi.sh) + RoboStack ROS 2 Humble.

## Install

```bash
# 1. Install pixi (once per Mac)
curl -fsSL https://pixi.sh/install.sh | bash

# 2. Clone this repo (NOT into iCloud Drive — must be a local-disk path with no spaces)
git clone <repo-url> ~/optitrack

# 3. Resolve and install the env (~10 min; downloads ROS 2 Humble)
cd ~/optitrack
pixi install
```

## Configure Motive (once, on the Windows PC)

In Motive → **View → Data Streaming Pane → OptiTrack Streaming Engine**:

- **Streaming Engine: ON** (all other engines OFF)
- **Transmission Type: Unicast**
- **Local Interface:** the PC's lab-LAN Ethernet (`192.168.201.X`)
- **Rigid Bodies: ON**
- Command port `1510`, Data port `1511`

Save the streaming profile.

## Run

```bash
cd ~/optitrack
pixi run launch
```

That's the only command needed every session. Output once connected:

```
[optitrack_ros]: Connected to server 192.168.201.5:1510
[optitrack_listener]: Subscribed to /optitrack/GATE
[optitrack_listener]: [GATE] pos=(+0.206, -0.213, +1.420) quat=(...)
```

## Visualize (optional)

In a second terminal:

```bash
cd ~/optitrack
pixi run rviz2
```

Inside RViz: set **Global Options → Fixed Frame** to `world`, then
**Add → By topic → /tf → TF** and **Add → By topic → /optitrack/<name> → PoseStamped**.
Save the layout via **File → Save Config As… → `rviz/optitrack.rviz`**
and from then on use `pixi run rviz` to reload it.

## Override defaults

The driver reads three env vars (lab defaults shown):

| Env var              | Default           |
|----------------------|-------------------|
| `OPTITRACK_PC_IP`    | `192.168.201.5`   |
| `MY_IP`              | `192.168.201.89`  |
| `OPTITRACK_FRAME_ID` | `world`           |

Override per-run:

```bash
OPTITRACK_PC_IP=192.168.201.6 pixi run launch
```

## Edit your code

Your subscriber:
[`src/optitrack_listener/optitrack_listener/listener.py`](src/optitrack_listener/optitrack_listener/listener.py).
Edit it, then `pixi run launch` again.
