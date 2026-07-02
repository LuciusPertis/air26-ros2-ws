#!/usr/bin/env python3
# CROSS-LANGUAGE — Services
# Python client that calls the C++ service server.
#
# Terminal 1: ros2 run basics_cross cpp_service_server
# Terminal 2: ros2 run basics_cross py_service_client.py

import rclpy
from rclpy.node import Node
from basics_cross.srv import AddTwoInts  # our own service — no example_interfaces


class PyServiceClient(Node):
    def __init__(self):
        super().__init__('py_service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

    def send_request(self, a: int, b: int):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for C++ service server...')
        req = AddTwoInts.Request()
        req.a = a
        req.b = b
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = PyServiceClient()
    result = node.send_request(3, 5)
    node.get_logger().info(f'[Python client] Result from C++ server: 3 + 5 = {result.sum}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
