#!/bin/bash
# Forward localhost:5601 -> phone:5601 over the USB cable, Mac or Linux (needs pymobiledevice3,
# on Linux also the usbmuxd daemon: sudo apt install usbmuxd;
# phone plugged in, unlocked, "Trust this computer" accepted, app in the foreground).
#   bash tools/usb_tunnel.sh            (Ctrl-C to stop)
PORT=${1:-5601}
exec pymobiledevice3 usbmux forward "$PORT" "$PORT"
