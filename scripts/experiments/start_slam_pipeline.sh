#!/bin/bash

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

gnome-terminal -- bash -c "
cd \$WS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
echo 'Odpal ruch np.: python3 ~/ros_scripts/cmd_vel_for_time.py 0.20 0.0 5'
exec bash
"

gnome-terminal -- bash -c "
cd \$WS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
exec bash
"
