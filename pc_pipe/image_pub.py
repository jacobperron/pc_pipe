#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty


class MockImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.sub = self.create_subscription(
            Empty, 'image_trigger', self.trigger_callback, 1)
        self.left_pub = self.create_publisher(Image, 'left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, 'right/image_raw', 10)
        width = self.declare_parameter('width', 640).value
        height = self.declare_parameter('height', 480).value
        print(f'width x height: {width} x {height}')
        self.__image = Image()
        self.__image.width = width
        self.__image.height = height
        self.__image.encoding = 'rgb8'
        self.__image.step = width
        self.__image.data = (width * height) * [0]

    def trigger_callback(self, msg):
        self.get_logger().info(f'Trigger message received [{self.get_clock().now().to_msg()}]')
        self.__image.header.stamp = self.get_clock().now().to_msg()
        self.left_pub.publish(self.__image)
        self.right_pub.publish(self.__image)


def main(args=None):
    rclpy.init(args=args)
    node = MockImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
