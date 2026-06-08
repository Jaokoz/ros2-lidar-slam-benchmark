# ROS 2 LiDAR SLAM Benchmark

Research platform for evaluating 2D LiDAR SLAM systems in ROS 2.

The project was developed as part of an engineering research workflow for comparing `slam_toolbox` and Cartographer in simulation and real-world experiments using a four-wheel mobile platform equipped with a 2D LiDAR sensor.

## Main features

- ROS 2 mobile robot package
- Gazebo simulation environment
- `slam_toolbox` configuration
- Cartographer configuration
- real robot experiment scripts
- rosbag recording and processing pipeline
- Raspberry Pi Flask server
- Marlin-based rover motion driver
- ground truth alignment using video and ArUco markers
- MATLAB analysis script
- original experiment notes and launch commands

## Repository structure

```text
ros2_ws/src/four_wheel_lidar_bot/   ROS 2 robot package
config/                             SLAM configuration files
scripts/                            experiment, recording, processing and analysis scripts
rpi_server/                         Raspberry Pi server and rover motion driver
ground_truth/                       video/ArUco ground truth alignment tools
matlab/                             MATLAB analysis script
docs/                               original experiment notes and commands
data_processing/                    reserved for additional processing tools
results_examples/                   small example outputs only
