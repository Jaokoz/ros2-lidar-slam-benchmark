#!/bin/bash

if [ $# -ne 3 ]; then
    echo "Użycie: ./run_cartographer_experiment.sh V OMEGA CZAS"
    echo "Przykład: ./run_cartographer_experiment.sh 0.20 0.10 6"
    exit 1
fi

V="$1"
OMEGA="$2"
MOVE_TIME="$3"

RECORD_TIME=$(python3 -c "print(int(float('$MOVE_TIME') + 8))")

WS=~/ros2_ws

RUN_ID=$(date +%Y%m%d_%H%M%S)
BAG_PATH="${WS}/rosbag2_cartografer_sim_proba_${RUN_ID}"

echo "=========================================="
echo "Eksperyment Cartographer"
echo "v     = ${V} m/s"
echo "omega = ${OMEGA} rad/s"
echo "czas ruchu = ${MOVE_TIME} s"
echo "czas nagrywania = ${RECORD_TIME} s"
echo "bag = ${BAG_PATH}"
echo "=========================================="

# ============================================================
# 1. Gazebo + robot
# ============================================================

gnome-terminal -- bash -c "
cd \$WS
source /opt/ros/jazzy/setup.bash
colcon build
source ~/ros2_ws/install/setup.bash
ros2 launch four_wheel_lidar_bot sim.launch.py
exec bash
"

sleep 4

# ============================================================
# 2. Static TF LiDAR-a
# ============================================================

gnome-terminal -- bash -c "
cd \$WS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run tf2_ros static_transform_publisher 0.15 0 0.10 0 0 0 base_link four_wheel_bot/base_link/lidar_sensor
exec bash
"

# ============================================================
# 3. Bridge ground truth z Gazebo
# ============================================================

gnome-terminal -- bash -c "
cd \$WS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge /model/four_wheel_bot/pose@geometry_msgs/msg/Pose[gz.msgs.Pose
exec bash
"

# ============================================================
# 4. Cartographer node
# ============================================================

gnome-terminal -- bash -c "
cd \$WS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run cartographer_ros cartographer_node \
  -configuration_directory /home/jaokoz/ros2_ws/config/cartographer \
  -configuration_basename my_robot_2d.lua \
  --ros-args -r scan:=/scan
exec bash
"

# ============================================================
# 5. Occupancy grid node
# ============================================================

gnome-terminal -- bash -c "
cd \$WS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run cartographer_ros cartographer_occupancy_grid_node \
  --ros-args -p resolution:=0.05 -p publish_period_sec:=1.0
exec bash
"

# ============================================================
# 6. RViz
# ============================================================

gnome-terminal -- bash -c "
cd \$WS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2
exec bash
"

sleep 12

# ============================================================
# 7. Nagrywanie rosbaga
# ============================================================

gnome-terminal -- bash -c "
cd \$WS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo '=========================================='
echo 'Topiki przed nagrywaniem:'
ros2 topic list -t | grep -E 'map|trajectory|submap|constraint|scan_matched|scan|odom|tf|model'
echo '=========================================='

echo 'Czekam na /trajectory_node_list...'
for i in {1..20}; do
  if ros2 topic list | grep -q '^/trajectory_node_list$'; then
    echo '/trajectory_node_list jest dostępny.'
    break
  fi
  echo 'Brak /trajectory_node_list, czekam...'
  sleep 1
done

echo 'Czekam na /submap_list...'
for i in {1..20}; do
  if ros2 topic list | grep -q '^/submap_list$'; then
    echo '/submap_list jest dostępny.'
    break
  fi
  echo 'Brak /submap_list, czekam...'
  sleep 1
done

echo '=========================================='
echo 'Topiki bezpośrednio przed ros2 bag record:'
ros2 topic list -t | grep -E 'map|trajectory|submap|constraint|scan_matched|scan|odom|tf|model'
echo '=========================================='

echo 'Nagrywanie rosbaga przez ${RECORD_TIME}s...'
echo 'Bag path: ${BAG_PATH}'

timeout ${RECORD_TIME}s ros2 bag record \
  -o ${BAG_PATH} \
  /scan \
  /odom \
  /tf \
  /tf_static \
  /map \
  /trajectory_node_list \
  /submap_list \
  /constraint_list \
  /scan_matched_points2 \
  /model/four_wheel_bot/pose

echo 'Nagrywanie zakończone.'
exec bash
"

# ============================================================
# 8. Ruch robota
# ============================================================

gnome-terminal -- bash -c "
sleep 3
cd \$WS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros_scripts/cmd_vel_for_time.py ${V} ${OMEGA} ${MOVE_TIME}
exec bash
"
