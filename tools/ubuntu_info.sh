#!/bin/bash
# Print everything about the Ubuntu robot PC that the capture pipeline depends on.
# Run it there and paste the output back:   bash tools/ubuntu_info.sh
# (read-only: it changes nothing, needs no sudo; sudo is only *tested* for)
have() { command -v "$1" >/dev/null 2>&1 && echo "yes ($(command -v "$1"))" || echo "no"; }
echo "=== machine ==="
echo "date       : $(date -Is)"
echo "hostname   : $(hostname)"
echo "arch       : $(uname -m)"
echo "kernel     : $(uname -r)"
echo "os         : $(. /etc/os-release 2>/dev/null && echo "$PRETTY_NAME")"
echo "cpu        : $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2 | sed 's/^ //') x$(nproc)"
echo "ram        : $(free -g | awk '/Mem:/ {print $2 " GB"}')"
echo "gpu        : $(lspci 2>/dev/null | grep -i -e vga -e 3d | cut -d: -f3 | paste -sd, -)"
echo "user/sudo  : $(id -un), sudo=$(sudo -n true 2>/dev/null && echo 'yes (no password)' || (groups | grep -q sudo && echo 'yes (password)' || echo 'no'))"
echo "home free  : $(df -h "$HOME" | awk 'NR==2 {print $4 " of " $2}')"
echo
echo "=== python / pixi ==="
echo "python3    : $(have python3) $(python3 --version 2>&1)"
echo "pipx       : $(have pipx)"
echo "pixi       : $(have pixi) $(pixi --version 2>/dev/null)"
echo "git        : $(have git) $(git --version 2>/dev/null)"
echo "rsync      : $(have rsync)"
echo
echo "=== iPhone over USB ==="
echo "usbmuxd    : $(have usbmuxd) service=$(systemctl is-active usbmuxd 2>/dev/null || echo unknown)"
echo "iproxy     : $(have iproxy)"
echo "idevice_id : $(have idevice_id)"
echo "pymobiledevice3: $(have pymobiledevice3) $(pymobiledevice3 version 2>/dev/null)"
echo "usb devices (Apple): $(lsusb 2>/dev/null | grep -i apple || echo none)"
echo "plugdev    : $(groups | grep -q plugdev && echo 'user in plugdev' || echo 'user NOT in plugdev')"
echo "udev rules : $(ls /lib/udev/rules.d/*usbmuxd* /etc/udev/rules.d/*usbmux* 2>/dev/null | paste -sd, - || echo none)"
echo
echo "=== notifications ==="
for t in spd-say espeak-ng espeak paplay aplay ffplay notify-send; do echo "$t: $(have $t)"; done
echo "sounds     : $(ls /usr/share/sounds/freedesktop/stereo/complete.oga /usr/share/sounds/freedesktop/stereo/bell.oga 2>/dev/null | paste -sd, - || echo none)"
echo "display    : DISPLAY=${DISPLAY:-unset} WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-unset}"
echo
echo "=== ROS ==="
echo "ROS 1      : ROS_VERSION=${ROS_VERSION:-unset} ROS_DISTRO=${ROS_DISTRO:-unset} /opt/ros: $(ls /opt/ros 2>/dev/null | paste -sd, - || echo none)"
echo "in .bashrc : $(grep -n 'setup.bash' ~/.bashrc 2>/dev/null | paste -sd'|' - || echo none)"
echo "colcon     : $(have colcon)   cmake: $(have cmake) $(cmake --version 2>/dev/null | head -1)"
echo "compilers  : gcc=$(have gcc) g++=$(have g++)"
echo
echo "=== network (Motive PC 192.168.201.5, robot script socket) ==="
ip -4 -o addr show 2>/dev/null | awk '{print "iface      : " $2 " " $4}'
echo "motive ping: $(ping -c1 -W1 192.168.201.5 >/dev/null 2>&1 && echo reachable || echo 'no answer')"
echo "multicast  : $(ip -o link show 2>/dev/null | grep -c MULTICAST) interfaces with MULTICAST"
echo
echo "=== robot side ==="
echo "robo_arm   : $(ls -d ~/shared_ws 2>/dev/null || echo 'shared_ws not in HOME') ; panda script: $(find / -name panda_manual_poses.py -not -path '*/image-capture/*' 2>/dev/null | head -3 | paste -sd, - || echo none)"
echo "python3 pkgs: $(python3 -c 'import yaml; print("pyyaml ok")' 2>&1) / $(python3 -c 'import numpy; print("numpy", numpy.__version__)' 2>&1)"
