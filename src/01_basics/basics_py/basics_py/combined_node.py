"""
TUTORIAL 4 — Combined Node

One node that demonstrates topics, services, AND actions simultaneously.
Students can comment out any CHECKPOINT block to isolate a concept.

    ros2 run basics combined_node

Interact with it:
    ros2 topic echo /chatter
    ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 4, b: 6}"
    ros2 run basics action_client
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
from example_interfaces.action import Fibonacci


class CombinedNode(Node):
    def __init__(self):
        super().__init__('combined_node')
        self.count = 0

        # === CHECKPOINT: topics ===
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.get_logger().info('Topic publisher on /chatter — every 1 s')
        # === END CHECKPOINT: topics ===

        # === CHECKPOINT: services ===
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.handle_add)
        self.get_logger().info('Service /add_two_ints ready')
        # === END CHECKPOINT: services ===

        # === CHECKPOINT: actions ===
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'count_up',
            self.execute_count,
        )
        self.get_logger().info('Action server /count_up ready')
        # === END CHECKPOINT: actions ===

    # === CHECKPOINT: topics ===
    def publish_message(self):
        msg = String()
        msg.data = f'Combined node alive — tick {self.count}'
        self.pub.publish(msg)
        self.count += 1
    # === END CHECKPOINT: topics ===

    # === CHECKPOINT: services ===
    def handle_add(self, request: AddTwoInts.Request, response: AddTwoInts.Response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Service: {request.a} + {request.b} = {response.sum}')
        return response
    # === END CHECKPOINT: services ===

    # === CHECKPOINT: actions ===
    def execute_count(self, goal_handle):
        target = goal_handle.request.order
        self.get_logger().info(f'Action goal: count to {target}')
        feedback = Fibonacci.Feedback()
        for i in range(target + 1):
            feedback.sequence = [i]
            goal_handle.publish_feedback(feedback)
            time.sleep(1.0)
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = [target]
        return result
    # === END CHECKPOINT: actions ===


def main(args=None):
    rclpy.init(args=args)
    node = CombinedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
