#!/bin/bash
# Copy this repo to the Ubuntu robot PC (user's path, 2026-09-29): ~/shared_ws/roberts_ws/image-capture/
#   bash tools/sync_to_pc.sh                 # leon@192.168.201.158 (the Mac must be on the lab switch)
#   bash tools/sync_to_pc.sh user@host       # another target
# Excludes the pixi env, colcon build/install/log, datasets, caches. The submodules and .git travel.
# The datasets folder ~/Documents/repos/optitrack/datasets/ is NEVER synced (user's rule).
set -eu
HERE=$(cd "$(dirname "$0")/.." && pwd)
TARGET=${1:-leon@192.168.201.158}
DEST=${2:-shared_ws/roberts_ws/}
ssh "$TARGET" "mkdir -p $DEST"
rsync -a --progress \
    --exclude .pixi --exclude build --exclude install --exclude log --exclude datasets \
    --exclude '__pycache__' --exclude '*.pyc' --exclude 'xcuserdata' --exclude '.DS_Store' \
    "$HERE" "$TARGET:$DEST"
echo "synced -> $TARGET:$DEST$(basename "$HERE")/"
