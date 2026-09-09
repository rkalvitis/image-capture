#!/bin/bash
# One robot run per turntable step: repeat panda_manual_poses.py N times
# in socket-confirm mode. Run on the ROBOT PC (ROS 1 env sourced,
# move_group up, record_joint_states.py running in another terminal).
#
#   bash run_cycles.sh <cycles> [extra rosrun params...]
#   e.g. bash run_cycles.sh 36 _confirm_port:=5600
#
# Each run starts and ends at the initial pose; the Mac coordinator
# reconnects for every run. Ctrl-C stops after the current run.
set -u
N=${1:?number of cycles}; shift || true
for c in $(seq 1 "$N"); do
    echo "=== cycle $c / $N ==="
    rosrun panda_moveit_ctrl panda_manual_poses.py _execute:=true _confirm_via:=socket "$@" \
        || { echo "cycle $c ended with an error/abort — stopping"; exit 1; }
done
echo "all $N cycles done"
