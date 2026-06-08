#!/bin/bash

if [ $# -ne 3 ]; then
    echo "Użycie: ./run_slam_experiment.sh V OMEGA CZAS"
    echo "Przykład: ./run_slam_experiment.sh 0.20 0.10 6"
    exit 1
fi

V="$1"
OMEGA="$2"
MOVE_TIME="$3"

# ile sekund ma trwać nagrywanie rosbaga
RECORD_TIME=$(python3 -c "print(int(float('$MOVE_TIME') + 8))")

WS=~/ros2_ws

gnome-terminal -- bash -c "
cd \$WS
source /opt/ros/jazzy/setup.bash
colcon build
source ~/ros2_ws/install/setup.bash
ros2 launch four_wheel_lidar_bot sim.launch.py
exec bash
"

sleep 4

gnome-terminal -- bash -c "
cd \$WS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run tf2_ros static_transform_publisher 0.15 0 0.10 0 0 0 base_link four_wheel_bot/base_link/lidar_sensor
exec bash
"

gnome-terminal -- bash -c "
cd \$WS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge /model/four_wheel_bot/pose@geometry_msgs/msg/Pose[gz.msgs.Pose
exec bash
"

gnome-terminal -- bash -c "
cd \$WS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch slam_toolbox online_sync_launch.py slam_params_file:=/home/jaokoz/ros2_ws/config/slam_toolbox_sim.yaml use_sim_time:=false
exec bash
"

gnome-terminal -- bash -c "
cd \$WS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2
exec bash
"

sleep 5

gnome-terminal -- bash -c "
cd \$WS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
echo 'Nagrywanie rosbaga przez ${RECORD_TIME}s...'
timeout ${RECORD_TIME}s ros2 bag record /scan /odom /tf /tf_static /map /model/four_wheel_bot/pose
echo 'Nagrywanie zakończone.'
exec bash
"

gnome-terminal -- bash -c "
sleep 3
cd \$WS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros_scripts/cmd_vel_for_time.py ${V} ${OMEGA} ${MOVE_TIME}
exec bash
"
