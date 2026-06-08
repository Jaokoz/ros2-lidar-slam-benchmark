#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class FakeOdomNode(Node):
    def __init__(self):
        super().__init__("fake_odom_from_cmd_vel")

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "world_1")
        self.declare_parameter("cmd_topic", "/cmd_vel_executed")
        self.declare_parameter("publish_rate", 20.0)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v = 0.0
        self.omega = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.last_time = self.get_clock().now()

        self.cmd_timeout = 0.7

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_topic,
            self.cmd_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.update)

        self.get_logger().info(
            f"Publikuję /odom oraz TF: {self.odom_frame} -> {self.base_frame}"
        )
        self.get_logger().info(
            f"Słucham komend wykonanych z: {self.cmd_topic}"
        )

    def cmd_callback(self, msg):
        self.v = float(msg.linear.x)
        self.omega = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        cmd_age = (now - self.last_cmd_time).nanoseconds * 1e-9

        if cmd_age > self.cmd_timeout:
            self.v = 0.0
            self.omega = 0.0

        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.omega

        self.odom_pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0

        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = FakeOdomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
