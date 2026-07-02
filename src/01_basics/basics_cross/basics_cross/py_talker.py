#!/usr/bin/env python3
# CROSS-LANGUAGE — Topics
# Python publisher that the C++ listener subscribes to.
#
# Terminal 1: ros2 run basics_cross py_talker.py
# Terminal 2: ros2 run basics_cross cpp_listener

import rclpy
from rclpy.node import Node
from basics_cross.msg import Greeting  # our own message — no std_msgs


class PyTalker(Node):
    def __init__(self):
        super().__init__('py_talker')
        self.pub = self.create_publisher(Greeting, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.count = 0
        self.get_logger().info('Python talker started — publishing on /chatter')

    def publish_message(self):
        msg = Greeting()
        msg.data = f'[Python] Hello from Python! count={self.count}'
        self.pub.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = PyTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
