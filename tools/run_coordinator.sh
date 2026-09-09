#!/bin/bash
# Run the coordinator with the optitrack pixi env (rclpy) from anywhere.
#   bash tools/run_coordinator.sh --plan coordinator/plan.yaml --tunnel
#   bash tools/run_coordinator.sh --plan coordinator/plan.yaml --mocap synthetic --puller local --fake-phone-dir /tmp/fakephone
HERE=$(cd "$(dirname "$0")/.." && pwd)
OPTITRACK=${OPTITRACK_REPO:-$HOME/Documents/repos/optitrack}
export PYTHONPATH="$HERE/coordinator:${PYTHONPATH:-}"
exec pixi run --manifest-path "$OPTITRACK/pixi.toml" python -m capture_coordinator.coordinator "$@"
