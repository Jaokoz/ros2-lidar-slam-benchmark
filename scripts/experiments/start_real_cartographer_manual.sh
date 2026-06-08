#!/bin/bash

set -e

WS="$HOME/ros2_ws"
IFACE="eno1"
LIDAR_IP="192.168.0.1"
HOST_IP="192.168.0.2"

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

echo "=== Konfiguracja sieci LiDAR ==="
nmcli radio wifi off || true
sudo ip addr flush dev "$IFACE"
sudo ip addr add ${HOST_IP}/24 dev "$IFACE"
sudo ip link set "$IFACE" up

echo "=== Test połączenia z LiDARem ==="
ping -c 2 "$LIDAR_IP"

echo "=== Start LiDAR ==="
gnome-terminal --title="LiDAR" -- bash -lc "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
ros2 launch sick_scan_xd sick_picoscan.launch.py hostname:=$LIDAR_IP udp_receiver_ip:=$HOST_IP
exec bash
"

echo "=== Czekam na /scan_fullframe ==="
timeout 20 bash -lc "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
until ros2 topic echo /scan_fullframe --once >/dev/null 2>&1; do
  sleep 1
done
"

echo "=== Start fake odom ==="
gnome-terminal --title="Fake Odom" -- bash -lc "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
python3 ~/ros_scripts/fake_odom_from_cmd_vel.py
exec bash
"

echo "=== Czekam na /odom ==="
timeout 10 bash -lc "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
until ros2 topic list | grep -q '^/odom$'; do
  sleep 1
done
"

echo "=== Start Cartographer ==="
gnome-terminal --title="Cartographer" -- bash -lc "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
ros2 run cartographer_ros cartographer_node \
  -configuration_directory $HOME/ros2_ws/config/cartographer \
  -configuration_basename my_robot_2d_real.lua \
  --ros-args -r scan:=/scan_fullframe
exec bash
"

sleep 2

echo "=== Start Occupancy Grid ==="
gnome-terminal --title="Cartographer Grid" -- bash -lc "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
ros2 run cartographer_ros cartographer_occupancy_grid_node \
  --ros-args -p resolution:=0.05 -p publish_period_sec:=1.0
exec bash
"

sleep 2

echo "=== Start RViz ==="
gnome-terminal --title="RViz" -- bash -lc "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
rviz2
exec bash
"

sleep 2

echo "=== Terminal sterowania ==="
gnome-terminal --title="Sterowanie ręczne" -- bash -lc "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
echo 'Przykłady:'
echo 'python3 ~/ros_scripts/cmd_vel_for_time.py 0.20 0.00 5'
echo 'python3 ~/ros_scripts/cmd_vel_for_time.py 0.20 0.10 5'
echo 'python3 ~/ros_scripts/cmd_vel_for_time.py 0.00 0.30 3'
exec bash
"

echo "=== Wszystko uruchomione ==="
echo "Najpierw w RViz sprawdź sam skan:"
echo "  Fixed Frame = world_1"
echo "  Add -> LaserScan -> /scan_fullframe"
echo ""
echo "Dopiero potem przejdź na mapę:"
echo "  Fixed Frame = map"
echo "  Add -> Map -> /map"
echo "  Add -> TF"
