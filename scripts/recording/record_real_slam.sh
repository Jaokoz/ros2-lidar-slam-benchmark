#!/bin/bash

set -e

if [ -z "$1" ]; then
    echo "Użycie: ~/ros_scripts/record_real_slam.sh nazwa_baga"
    echo "Przykład: ~/ros_scripts/record_real_slam.sh rosbag2_real_slam_01"
    exit 1
fi

BAG_NAME="$1"
BAG_PATH="$HOME/$BAG_NAME"

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo "=== Nagrywam do: $BAG_PATH ==="
ros2 bag record /scan_fullframe /odom /tf /tf_static /map -o "$BAG_PATH"
