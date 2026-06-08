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
ros2 run cartographer_ros cartographer_node \
  -configuration_directory /home/jaokoz/ros2_ws/config/cartographer \
  -configuration_basename my_robot_2d.lua \
  --ros-args -r scan:=/scan
exec bash
"

gnome-terminal -- bash -c "
cd \$WS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run cartographer_ros cartographer_occupancy_grid_node \
  --ros-args -p resolution:=0.05 -p publish_period_sec:=1.0
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
ros2 run teleop_twist_keyboard teleop_twist_keyboard
exec bash
"
