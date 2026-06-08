#!/bin/bash

set -e

if [ -z "$1" ]; then
    echo "Użycie: ~/ros_scripts/record_real_cartographer.sh nazwa_baga"
    echo "Przykład: ~/ros_scripts/record_real_cartographer.sh rosbag2_real_cartographer_01"
    exit 1
fi

BAG_NAME="$1"
BAG_PATH="$HOME/$BAG_NAME"

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo "=== Nagrywam do: $BAG_PATH ==="
echo "=== Sprawdzam dostępne topiki Cartographera ==="
ros2 topic list | grep -E "scan|odom|tf|map|trajectory|submap|constraint|scan_matched" || true

echo "=== Start nagrywania ==="
ros2 bag record \
  /scan_fullframe \
  /odom \
  /tf \
  /tf_static \
  /map \
  /trajectory_node_list \
  /submap_list \
  /constraint_list \
  /scan_matched_points2 \
  -o "$BAG_PATH"
