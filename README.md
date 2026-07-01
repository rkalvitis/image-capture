# OptiTrack → ROS 2

Setup for receiving OptiTrack/Motive NatNet data as ROS 2 topics on macOS
(Apple Silicon) and Linux (aarch64), using [pixi](https://pixi.sh) +
RoboStack ROS 2 Humble.

## Install

```bash
# 1. Install pixi (once per machine)
curl -fsSL https://pixi.sh/install.sh | bash   # macOS / Linux

# 2. Clone this repo (NOT into iCloud Drive — must be a local-disk path with no spaces)
git clone --recursive <repo-url> ~/optitrack

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
pixi run rviz-optitrack
```

Inside RViz: set **Global Options → Fixed Frame** to `world`, then
**Add → By topic → /tf → TF** and **Add → By topic → /optitrack/<name> → PoseStamped**.
Save the layout via **File → Save Config As… → `rviz/optitrack.rviz`**
and from then on use `pixi run rviz-optitrack` to reload it.

## Override defaults

The driver reads three env vars (lab defaults shown):

| Env var              | Default           |
|----------------------|-------------------|
| `OPTITRACK_PC_IP`    | `192.168.201.5`   |
| `MY_IP`              | `192.168.201.89`  |
| `OPTITRACK_FRAME_ID` | `world`           |

### Find the OptiTrack PC IP (on the Windows Motive machine)

In a Windows `cmd` / PowerShell:

```cmd
ipconfig
```

Look for the **Ethernet adapter** on the lab LAN (subnet `192.168.201.X`)
and read its `IPv4 Address`. That's `OPTITRACK_PC_IP`. The same value is
shown in Motive under **Data Streaming Pane → Local Interface**.

### Find the OptiTrack PC IP from a client (no Windows access needed)

Motive answers NatNet discovery broadcasts. From your Mac or Linux box
on the same LAN, with the env installed:

```bash
pixi run python -m natnet_py.natnet_discover
```

Output looks like:

```
------------------------------------------------------------
192.168.201.5:1510
    Motive using NatNet version 4.1.0.0
    unicast data stream on :1511
------------------------------------------------------------
```

The IP on the first line is `OPTITRACK_PC_IP`. If discovery finds nothing,
fall back to scanning the lab subnet:

```bash
# macOS / Linux — show all live hosts on 192.168.201.0/24
nmap -sn 192.168.201.0/24            # if nmap is installed
# or, after pinging the broadcast once:
arp -a | grep 192.168.201            # macOS / Linux
ip neigh show | grep 192.168.201     # Linux only
```

### Find your Mac IP (on the lab LAN interface)

Plug into the lab Ethernet, then in a terminal:

```bash
# All interfaces with an IPv4 address on the lab subnet
ifconfig | grep "inet 192.168.201"
```

Or, if you know the interface name (usually `en0` for Wi-Fi, `en6`/`en7`
for a USB-Ethernet adapter — check `networksetup -listallhardwareports`):

```bash
ipconfig getifaddr en7
```

That address is `MY_IP`.

### Find your Linux PC IP (on the lab LAN interface)

```bash
# All interfaces with an IPv4 address on the lab subnet
ip -4 addr show | grep "inet 192.168.201"
```

Or, if you know the interface name (list them with `ip -br link`, usually
`eth0`/`enp3s0` for built-in Ethernet, `enx...` for a USB adapter):

```bash
ip -4 addr show dev enp3s0 | awk '/inet /{print $2}' | cut -d/ -f1
```

That address is `MY_IP`.

### Override per-run

```bash
OPTITRACK_PC_IP=192.168.201.6 MY_IP=192.168.201.42 pixi run launch
```

## Edit your code

Your subscriber:
[`src/optitrack_listener/optitrack_listener/listener.py`](src/optitrack_listener/optitrack_listener/listener.py).
Edit it, then re-run:

```bash
pixi install   # rebuilds changed packages
pixi run launch
```
