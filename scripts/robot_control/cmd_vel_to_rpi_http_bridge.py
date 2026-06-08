#!/usr/bin/env python3

import argparse
import time
import threading
import requests

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelToHttpBridge(Node):
    def __init__(self, url, duration, timeout, period_factor):
        super().__init__("cmd_vel_to_rpi_http_bridge")

        self.url = url
        self.duration = float(duration)
        self.timeout = float(timeout)
        self.period_factor = float(period_factor)

        self.min_period = self.duration * self.period_factor
        self.last_send_time = 0.0

        self.session = requests.Session()

        self.sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_callback,
            10
        )

        self.exec_pub = self.create_publisher(
            Twist,
            "/cmd_vel_executed",
            10
        )

        self.get_logger().info(f"HTTP bridge aktywny: {self.url}")
        self.get_logger().info(
            f"duration={self.duration:.3f}, "
            f"timeout={self.timeout:.3f}, "
            f"period_factor={self.period_factor:.3f}, "
            f"min_period={self.min_period:.3f}"
        )
        self.get_logger().info("Publikuję wykonane komendy na: /cmd_vel_executed")
        self.get_logger().info("UWAGA: omega dla /cmd_vel_executed ma odwrócony znak")

    def publish_executed(self, v, omega):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self.exec_pub.publish(msg)

    def publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.exec_pub.publish(msg)

    def cmd_callback(self, msg):
        now = time.time()

        v = float(msg.linear.x)
        omega = float(msg.angular.z)

        if abs(v) < 1e-9 and abs(omega) < 1e-9:
            self.publish_stop()
            return

        if now - self.last_send_time < self.min_period:
            return

        self.last_send_time = now

        payload = {
            "v": v,
            "omega": omega,
            "duration": self.duration
        }

        try:
            response = self.session.post(
                self.url,
                json=payload,
                timeout=self.timeout
            )

            if response.status_code == 200:
                # RPi dostaje omega zgodne z komendą użytkownika,
                # ale fake odom musi mieć przeciwny znak skrętu.
                self.publish_executed(v, -omega)

                threading.Timer(self.duration, self.publish_stop).start()

            elif response.status_code == 429:
                self.get_logger().warn(
                    "RPi busy: HTTP 429 — nie liczę tego segmentu w odometrii"
                )

            else:
                self.get_logger().warn(
                    f"HTTP {response.status_code}: {response.text}"
                )

        except Exception as e:
            self.get_logger().warn(f"Nie wysłano do RPi: {e}")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--url",
        required=True,
        help="Adres endpointu RPi, np. http://192.168.0.57:5001/cmd_vel"
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=0.5,
        help="Czas jednego segmentu ruchu na RPi [s]"
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=1.5,
        help="Timeout HTTP [s]"
    )

    parser.add_argument(
        "--period-factor",
        type=float,
        default=1.05,
        help="min_period = duration * period_factor"
    )

    args = parser.parse_args()

    rclpy.init()

    node = CmdVelToHttpBridge(
        url=args.url,
        duration=args.duration,
        timeout=args.timeout,
        period_factor=args.period_factor
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
