"""
TUTORIAL 3a — Actions: Server

Counts from 0 to a target number, publishing feedback at each step.
Use this to see goal / feedback / result lifecycle.

    ros2 run basics action_server
    ros2 run basics action_client    (separate terminal)
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci   # reuse builtin: goal=order, feedback=sequence, result=sequence


# We repurpose the Fibonacci action type for simplicity:
#   goal.order      → count target
#   feedback.sequence  → [current_count]
#   result.sequence    → [final_count]

class CountServer(Node):
    def __init__(self):
        super().__init__('count_server')

        # === CHECKPOINT: actions ===
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'count_up',
            self.execute_callback,
        )
        self.get_logger().info('Action server /count_up ready')
        # === END CHECKPOINT: actions ===

    # === CHECKPOINT: actions ===
    def execute_callback(self, goal_handle):
        target = goal_handle.request.order
        self.get_logger().info(f'Goal received: count to {target}')

        feedback = Fibonacci.Feedback()
        for i in range(target + 1):
            feedback.sequence = [i]
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'Feedback: {i}/{target}')
            time.sleep(1.0)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = [target]
        self.get_logger().info(f'Goal succeeded — reached {target}')
        return result
    # === END CHECKPOINT: actions ===


def main(args=None):
    rclpy.init(args=args)
    node = CountServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
