"""
TUTORIAL 2a — Services: Server

Provides an /add_two_ints service that adds two integers.

    ros2 run basics service_server
    ros2 run basics service_client   (separate terminal)
    ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 5}"
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddServer(Node):
    def __init__(self):
        super().__init__('add_server')

        # === CHECKPOINT: services ===
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.handle_request)
        self.get_logger().info('Service /add_two_ints ready')
        # === END CHECKPOINT: services ===

    # === CHECKPOINT: services ===
    def handle_request(self, request: AddTwoInts.Request, response: AddTwoInts.Response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response
    # === END CHECKPOINT: services ===


def main(args=None):
    rclpy.init(args=args)
    node = AddServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
