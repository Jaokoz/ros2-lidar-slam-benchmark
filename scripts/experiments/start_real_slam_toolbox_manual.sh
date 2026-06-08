#!/bin/bash

set -e

WS="$HOME/ros2_ws"
IFACE="eno1"
LIDAR_IP="192.168.0.1"
HOST_IP="192.168.0.2"
SLAM_PARAMS="$HOME/ros2_ws/config/slam_toolbox_real_cmdvel.yaml"

echo "=== Konfiguracja sieci LiDAR ==="
nmcli radio wifi off || true
sudo ip addr flush dev "$IFACE"
sudo ip addr add ${HOST_IP}/24 dev "$IFACE"
sudo ip link set "$IFACE" up

echo "=== Test połączenia z LiDARem ==="
ping -c 2 "$LIDAR_IP"

echo "=== Otwieram terminale ==="

gnome-terminal --title="LiDAR" -- bash -c "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
ros2 launch sick_scan_xd sick_picoscan.launch.py hostname:=$LIDAR_IP udp_receiver_ip:=$HOST_IP
exec bash
"

sleep 2

gnome-terminal --title="Fake Odom" -- bash -c "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
python3 ~/ros_scripts/fake_odom_from_cmd_vel.py
exec bash
"

sleep 2

gnome-terminal --title="SLAM Toolbox" -- bash -c "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
ros2 launch slam_toolbox online_sync_launch.py slam_params_file:=$SLAM_PARAMS use_sim_time:=false
exec bash
"

sleep 2

gnome-terminal --title="RViz" -- bash -c "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
rviz2
exec bash
"

sleep 2

gnome-terminal --title="Sterowanie ręczne" -- bash -c "
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
echo 'Przyklady:'
echo 'python3 ~/ros_scripts/cmd_vel_for_time.py 0.20 0.00 5'
echo 'python3 ~/ros_scripts/cmd_vel_for_time.py 0.20 0.10 5'
echo 'python3 ~/ros_scripts/cmd_vel_for_time.py 0.00 0.30 3'
exec bash
"

echo "=== Wszystko uruchomione ==="
echo "RViz:"
echo "  Fixed Frame = map"
echo "  Add -> Map -> /map"
echo "  Add -> LaserScan -> /scan_fullframe"
