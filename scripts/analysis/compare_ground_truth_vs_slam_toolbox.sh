#!/bin/bash

set -e

if [ $# -lt 1 ]; then
    echo "Użycie:"
    echo "./compare_ground_truth_vs_slam_toolbox.sh rosbag2_real_slam_04"
    exit 1
fi

RUN_NAME="$1"

SCRIPT="/home/jaokoz/ros_scripts/compare_ground_truth_vs_slam.py"

GT_CSV="/home/jaokoz/ros_scripts/wyniki/ground_truth/${RUN_NAME}/ground_truth_raw.csv"
SLAM_DIR="/home/jaokoz/ros_scripts/wyniki/slam_toolbox_real/${RUN_NAME}"
OUT_DIR="/home/jaokoz/ros_scripts/wyniki/comparison_slam_toolbox/${RUN_NAME}"

mkdir -p "$OUT_DIR"

SLAM_CSV=""
for f in \
    "$SLAM_DIR/slam_toolbox.csv" \
    "$SLAM_DIR/slam.csv" \
    "$SLAM_DIR/trajectory.csv"
do
    if [ -f "$f" ]; then
        SLAM_CSV="$f"
        break
    fi
done

ODOM_CSV="$SLAM_DIR/odom.csv"

if [ -z "$SLAM_CSV" ]; then
    echo "Nie znaleziono CSV slam_toolbox w: $SLAM_DIR"
    exit 1
fi

if [ ! -f "$ODOM_CSV" ]; then
    echo "Nie znaleziono odom.csv w: $SLAM_DIR"
    exit 1
fi

echo "========================================"
echo "COMPARISON SLAM TOOLBOX"
echo "RUN_NAME  = $RUN_NAME"
echo "GT_CSV    = $GT_CSV"
echo "SLAM_CSV  = $SLAM_CSV"
echo "ODOM_CSV  = $ODOM_CSV"
echo "OUT_DIR   = $OUT_DIR"
echo "========================================"

python3 "$SCRIPT" \
  --gt-csv "$GT_CSV" \
  --slam-csv "$SLAM_CSV" \
  --odom-csv "$ODOM_CSV" \
  --out-dir "$OUT_DIR" \
  --method-name "slam_toolbox" \
  --prefix "compare_ground_truth_vs_slam_toolbox"

echo
echo "Gotowe."
echo "Wyniki zapisane do: $OUT_DIR"
