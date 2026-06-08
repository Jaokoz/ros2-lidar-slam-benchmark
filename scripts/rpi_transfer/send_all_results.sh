#!/bin/bash

set -e

if [ $# -lt 2 ]; then
    echo "Użycie:"
    echo "./send_all_results.sh run_001 192.168.0.57"
    exit 1
fi

RUN_NAME="$1"
RPI_IP="$2"

SEND_SCRIPT="/home/jaokoz/ros_scripts/send_results_to_rpi.py"

CARTO_DIR="/home/jaokoz/ros_scripts/wyniki/cartographer_real/${RUN_NAME}"
SLAM_DIR="/home/jaokoz/ros_scripts/wyniki/slam_toolbox_real/${RUN_NAME}"
GT_DIR="/home/jaokoz/ros_scripts/wyniki/ground_truth/${RUN_NAME}"
CMP_CARTO_DIR="/home/jaokoz/ros_scripts/wyniki/comparison_cartographer/${RUN_NAME}"
CMP_SLAM_DIR="/home/jaokoz/ros_scripts/wyniki/comparison_slam_toolbox/${RUN_NAME}"

echo "========================================"
echo "WYSYŁKA NA RPI"
echo "RUN_NAME = $RUN_NAME"
echo "RPI_IP   = $RPI_IP"
echo "========================================"

if [ -d "$CARTO_DIR" ]; then
    python3 "$SEND_SCRIPT" cartographer "$CARTO_DIR" "$RPI_IP"
else
    echo "Brak folderu: $CARTO_DIR"
fi

if [ -d "$SLAM_DIR" ]; then
    python3 "$SEND_SCRIPT" slam_toolbox "$SLAM_DIR" "$RPI_IP"
else
    echo "Brak folderu: $SLAM_DIR"
fi

if [ -d "$GT_DIR" ]; then
    python3 "$SEND_SCRIPT" ground_truth "$GT_DIR" "$RPI_IP"
else
    echo "Brak folderu: $GT_DIR"
fi

if [ -d "$CMP_CARTO_DIR" ]; then
    python3 "$SEND_SCRIPT" comparison_cartographer "$CMP_CARTO_DIR" "$RPI_IP"
else
    echo "Brak folderu: $CMP_CARTO_DIR"
fi

if [ -d "$CMP_SLAM_DIR" ]; then
    python3 "$SEND_SCRIPT" comparison_slam_toolbox "$CMP_SLAM_DIR" "$RPI_IP"
else
    echo "Brak folderu: $CMP_SLAM_DIR"
fi

echo
echo "Gotowe."
