#!/bin/bash
# Forward localhost:5601 -> phone:5601 over the USB cable, Mac or Linux (needs pymobiledevice3,
# on Linux also the usbmuxd daemon: sudo apt install usbmuxd;
# phone plugged in, unlocked, "Trust this computer" accepted, app in the foreground).
#   bash tools/usb_tunnel.sh            (Ctrl-C to stop)
# On Linux pymobiledevice3 is inside the pixi env (optitrack/.pixi/envs/default/bin); it is
# used when it exists, otherwise the one on PATH (pipx on the Mac).
PORT=${1:-5601}
HERE=$(cd "$(dirname "$0")/.." && pwd)
PMD3=$HERE/optitrack/.pixi/envs/default/bin/pymobiledevice3
[ -x "$PMD3" ] || PMD3=pymobiledevice3
exec "$PMD3" usbmux forward "$PORT" "$PORT"
