#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class OdometryPublsiher(Node):

    def __init__(self):
        super().__init__("odometry_publsiher")
        self.get_logger().info("Hello world from the Python node odometry_publsiher")


def main(args=None):
    rclpy.init(args=args)

    odometry_publsiher = OdometryPublsiher()

    try:
        rclpy.spin(odometry_publsiher)
    except KeyboardInterrupt:
        pass

    odometry_publsiher.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
