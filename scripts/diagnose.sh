#!/bin/bash
# Snapshot of the running stack: topic rates, TF chain, stamp sanity, CPU/GPU load.
# Run inside the container (or any shell with the same ROS_DOMAIN_ID) while the stack runs.
#   scripts/diagnose.sh [seconds]   # default 5 s per topic
set -uo pipefail
SECS="${1:-5}"

echo "== topic rates (target: lidar ~10 Hz, zed odom ~15-30 Hz, elevation map 5 Hz)"
for t in /livox/lidar /livox/imu /zed/zed_node/odom /elevation_map/cloud; do
  printf '%-28s ' "$t"
  timeout "$SECS" ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate' || echo 'no messages'
done

echo
echo "== lidar stamp vs wall clock (large offset => use_msg_stamp must stay false)"
STAMP=$(timeout 5 ros2 topic echo /livox/lidar --field header.stamp --once 2>/dev/null | tr -d '\n ')
if [ -n "$STAMP" ]; then
  echo "lidar header.stamp: $STAMP sec: $(echo "$STAMP" | sed -n 's/^sec: *\([0-9]*\).*/\1/p')"
  echo "wall clock:         $(date +%s)"
else
  echo 'no lidar message'
fi

echo
echo "== TF chain (each frame needs exactly one parent)"
for pair in "odom zed_camera_link" "zed_camera_link base_link" "base_link livox_frame"; do
  set -- $pair
  printf '%s -> %s : ' "$1" "$2"
  timeout 5 ros2 run tf2_ros tf2_echo "$1" "$2" 2>/dev/null | grep -m1 'Translation' || echo 'MISSING'
done

echo
echo "== system load"
if command -v tegrastats >/dev/null; then
  timeout 3 tegrastats --interval 1000 2>/dev/null | head -n 1
elif command -v nvidia-smi >/dev/null; then
  nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader
fi
uptime
