#!/bin/bash
set -euo pipefail

if [ $# -lt 1 ]; then
    echo "Użycie:"
    echo "  $0 RUN_NAME"
    echo
    echo "Przykład:"
    echo "  $0 rosbag2_real_slam_04"
    exit 1
fi

RUN_NAME="$1"

VIDEO="/home/jaokoz/ros_scripts/ground_truth_test/input/nagranie.mp4"
HOMOGRAPHY="/home/jaokoz/ros_scripts/ground_truth_test/input/homography.json"
SCRIPT="/home/jaokoz/ros_scripts/ground_truth_test/align_online_to_slam.py"

OUT_DIR="/home/jaokoz/ros_scripts/wyniki/ground_truth/${RUN_NAME}"

mkdir -p "$OUT_DIR"

echo "========================================"
echo "GROUND TRUTH"
echo "RUN_NAME   = $RUN_NAME"
echo "VIDEO      = $VIDEO"
echo "HOMOGRAPHY = $HOMOGRAPHY"
echo "OUT_DIR    = $OUT_DIR"
echo "DICT       = 5x5_1000"
echo "MARKER_ID  = 5"
echo "========================================"

python3 "$SCRIPT" \
  --video "$VIDEO" \
  --homography "$HOMOGRAPHY" \
  --dict 5x5_1000 \
  --marker-id 5 \
  --out-dir "$OUT_DIR" \
  --preview \
  --save-debug-frames \
  --debug-every-n 1 \
  --max-debug-frames 150 \
  --frame-step 20 \
  --progress-every 10 \
  --max-jump-m 0.50 \
  --full-scale 1.0 \
  --local-scale 1.8 \
  --relocalize-every 20 \
  --min-detections 3

echo
echo "Gotowe."
echo "Wyniki zapisane do: $OUT_DIR"
