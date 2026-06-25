"""
TUTORIAL 2b — Services: Client

Calls /add_two_ints once and prints the result.

    ros2 run basics service_server   (separate terminal first)
    ros2 run basics service_client
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddClient(Node):
    def __init__(self):
        super().__init__('add_client')

        # === CHECKPOINT: services ===
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        # === END CHECKPOINT: services ===

    # === CHECKPOINT: services ===
    def send_request(self, a: int, b: int):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /add_two_ints service...')

        req = AddTwoInts.Request()
        req.a = a
        req.b = b
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    # === END CHECKPOINT: services ===


def main(args=None):
    rclpy.init(args=args)
    node = AddClient()

    # === CHECKPOINT: services ===
    a, b = 3, 5
    result = node.send_request(a, b)
    node.get_logger().info(f'Result: {a} + {b} = {result.sum}')
    # === END CHECKPOINT: services ===

    node.destroy_node()
    rclpy.shutdown()
