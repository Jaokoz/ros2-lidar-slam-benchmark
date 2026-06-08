#!/bin/bash

set -e

if [ -z "$1" ]; then
    echo "Użycie: ~/ros_scripts/process_real_cartographer.sh nazwa_baga"
    echo "Przykład: ~/ros_scripts/process_real_cartographer.sh rosbag2_real_cartographer_01"
    exit 1
fi

BAG_NAME="$1"
BAG_PATH="$HOME/$BAG_NAME"
RESULT_DIR="$HOME/ros_scripts/wyniki/cartographer_real/$BAG_NAME"
RPI_IP="192.168.0.57"

if [ ! -d "$BAG_PATH" ]; then
    echo "Nie znaleziono baga: $BAG_PATH"
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

mkdir -p "$RESULT_DIR"

echo "=== Rozpakowanie CSV i wykresów ==="
python3 ~/ros_scripts/extract_one_bag_cartographer_real.py "$BAG_PATH"

echo "=== Zapis mapy ==="
timeout 25s ros2 run nav2_map_server map_saver_cli -f "$RESULT_DIR/map" &
MAP_SAVER_PID=$!

sleep 2

ros2 bag play "$BAG_PATH" >/dev/null 2>&1 &
BAG_PLAY_PID=$!

wait $MAP_SAVER_PID || true
kill $BAG_PLAY_PID >/dev/null 2>&1 || true

if [ -f "$RESULT_DIR/map.pgm" ]; then
    if command -v convert >/dev/null 2>&1; then
        convert "$RESULT_DIR/map.pgm" "$RESULT_DIR/map.png"
    fi
fi

echo "=== Przełączenie sieci na RPi ==="
sudo ip addr flush dev eno1 || true
nmcli radio wifi on || true
sleep 15

echo "=== Wysyłka na stronę ==="
python3 ~/ros_scripts/send_results_to_rpi.py \
cartographer \
"$RESULT_DIR" \
"$RPI_IP"

echo "=== Gotowe ==="
echo "Wyniki: $RESULT_DIR"
