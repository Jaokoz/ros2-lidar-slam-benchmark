#!/usr/bin/env python3

import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

V_SIGN = 1.0
OMEGA_SIGN = 1.0


class CmdVelForTimeNode(Node):
    def __init__(self, v, omega, duration):
        super().__init__("cmd_vel_for_time")

        self.v_user = float(v)
        self.omega_user = float(omega)
        self.duration = float(duration)

        self.v = V_SIGN * self.v_user
        self.omega = OMEGA_SIGN * self.omega_user

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.start_time = time.time()
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            f"Start: v={self.v:.3f}, omega={self.omega:.3f}, czas={self.duration:.3f}s"
        )

    def timer_callback(self):
        elapsed = time.time() - self.start_time

        msg = Twist()

        if elapsed < self.duration:
            msg.linear.x = self.v
            msg.angular.z = self.omega
            self.publisher.publish(msg)
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

            for _ in range(5):
                self.publisher.publish(msg)
                time.sleep(0.05)

            self.get_logger().info("STOP.")
            rclpy.shutdown()


def main():
    if len(sys.argv) != 4:
        print("Użycie:")
        print("python3 cmd_vel_for_time.py V OMEGA CZAS")
        print("Przykład:")
        print("python3 cmd_vel_for_time.py 0.2 0.0 5")
        sys.exit(1)

    rclpy.init()
    node = CmdVelForTimeNode(sys.argv[1], sys.argv[2], sys.argv[3])

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop = Twist()
        node.publisher.publish(stop)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
