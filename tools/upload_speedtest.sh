#!/bin/bash
# Measure the upload speed to the OneDrive remote before a session (decides raw-during vs raw-after):
#   bash tools/upload_speedtest.sh onedrive:SUPSI/speedtest [MB]      (default 300 MB)
# Copies a random file, verifies it with `rclone check`, prints MB/s, deletes it again.
set -eu
REMOTE=${1:?rclone remote:path}
MB=${2:-300}
TMP=$(mktemp -d)
dd if=/dev/urandom of="$TMP/speedtest_${MB}MB.bin" bs=1M count="$MB" status=none
START=$(date +%s.%N)
rclone copy "$TMP" "$REMOTE" --transfers 4 -q
rclone check "$TMP" "$REMOTE" --one-way -q
END=$(date +%s.%N)
SEC=$(echo "$END - $START" | bc)
echo "$MB MB in $SEC s = $(echo "scale=1; $MB / $SEC" | bc) MB/s (verified)"
echo "  a 50-frame ProRAW stop (~750 MB) + master (~60 MB): ~$(echo "scale=0; 810 / ($MB / $SEC)" | bc) s"
rclone delete "$REMOTE/speedtest_${MB}MB.bin" -q
rm -rf "$TMP"
