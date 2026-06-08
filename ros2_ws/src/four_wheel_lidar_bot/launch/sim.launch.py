from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('four_wheel_lidar_bot')
    urdf = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    world = os.path.join(pkg, 'worlds', 'test_world.sdf')

    robot_description = Command(['xacro ', urdf])

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py', f'gz_args:={world}'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=['-topic', 'robot_description', '-name', 'four_wheel_bot', '-z', '0.2'],
                    output='screen'
                )
            ]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            ],
            output='screen'
        ),
    ])