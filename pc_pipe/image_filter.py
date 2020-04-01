#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class MockImageFilter(Node):

    def __init__(self):
        super().__init__('image_filter')
        self.sub = self.create_subscription(Image, 'image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Image, 'image_filtered', 10)

    def image_callback(self, msg):
        now = self.get_clock().now()
        diff = (now.nanoseconds - rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds) / 1e9
        self.get_logger().info(f'Message received [{msg.header.stamp}]. Delay: {diff}')

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockImageFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
