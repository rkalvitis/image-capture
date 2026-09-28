#!/bin/bash
# Run the coordinator with the OptiTrack pixi env (rclpy, rawpy, opencv) from anywhere,
# on the Mac or on the Ubuntu robot PC.
#   bash tools/run_coordinator.sh --plan coordinator/plan.yaml --tunnel
#   bash tools/run_coordinator.sh --plan coordinator/plan.yaml --mocap synthetic --puller local --fake-phone-dir /tmp/fakephone
# The pixi project is <repo>/optitrack (merged 2026-09-28); OPTITRACK_REPO overrides it.
HERE=$(cd "$(dirname "$0")/.." && pwd)
OPTITRACK=${OPTITRACK_REPO:-$HERE/optitrack}
if [ ! -f "$OPTITRACK/pixi.toml" ]; then
    echo "run_coordinator.sh: no pixi.toml in $OPTITRACK (set OPTITRACK_REPO)" >&2; exit 1
fi
if [ "${ROS_VERSION:-}" = "1" ] || [ -n "${ROS_PACKAGE_PATH:-}" ]; then
    # A sourced ROS 1 (noetic) setup.bash on the robot PC would leak /opt/ros/* into the
    # pixi ROS 2 env (PYTHONPATH shadows packages, CMAKE_PREFIX_PATH confuses colcon).
    echo "run_coordinator.sh: ROS 1 environment detected -> stripping /opt/ros/* and ROS_* for the pixi env" >&2
    strip() { echo "$1" | tr ':' '\n' | grep -v -e '^$' -e '/opt/ros/' -e 'catkin_ws' -e 'devel/' | paste -sd: -; }
    export PATH=$(strip "$PATH")
    export PYTHONPATH=$(strip "${PYTHONPATH:-}")
    export LD_LIBRARY_PATH=$(strip "${LD_LIBRARY_PATH:-}")
    export CMAKE_PREFIX_PATH=$(strip "${CMAKE_PREFIX_PATH:-}")
    export PKG_CONFIG_PATH=$(strip "${PKG_CONFIG_PATH:-}")
    unset ROS_VERSION ROS_DISTRO ROS_ROOT ROS_PACKAGE_PATH ROS_MASTER_URI ROS_ETC_DIR ROSLISP_PACKAGE_DIRECTORIES ROS_PYTHON_VERSION
fi
export PYTHONPATH="$HERE/coordinator${PYTHONPATH:+:$PYTHONPATH}"
cd "$HERE" || exit 1            # relative plan paths (coordinator/..., robot/...) resolve against the repo root
exec pixi run --manifest-path "$OPTITRACK/pixi.toml" python -m capture_coordinator.coordinator "$@"
