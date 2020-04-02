#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Empty


class MockImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.sub = self.create_subscription(
            Empty, 'image_trigger', self.trigger_callback, 1)
        self.left_image_pub = self.create_publisher(Image, 'left/image_raw', 10)
        self.right_image_pub = self.create_publisher(Image, 'right/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, 'left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, 'right/camera_info', 10)
        width = self.declare_parameter('width', 640).value
        height = self.declare_parameter('height', 480).value
        self._image = Image()
        self._image.width = width
        self._image.height = height
        self._image.encoding = 'rgb8'
        self._image.step = width
        self._image.data = (width * height) * [0]
        self._info = CameraInfo()
        self.get_logger().info(f'Ready to publish images of size {width} x {height}')

    def trigger_callback(self, msg):
        now = self.get_clock().now().to_msg()
        self.get_logger().info(f'Trigger message received [{now}]')
        self._image.header.stamp = now
        self._info.header.stamp = now
        self.left_image_pub.publish(self._image)
        self.right_image_pub.publish(self._image)
        self.left_info_pub.publish(self._info)
        self.right_info_pub.publish(self._info)


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
