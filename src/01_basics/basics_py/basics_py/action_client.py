"""
TUTORIAL 3b — Actions: Client

Sends a goal to /count_up and prints feedback + result.

    ros2 run basics action_server    (separate terminal first)
    ros2 run basics action_client
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci


class CountClient(Node):
    def __init__(self):
        super().__init__('count_client')

        # === CHECKPOINT: actions ===
        self._client = ActionClient(self, Fibonacci, 'count_up')
        # === END CHECKPOINT: actions ===

    # === CHECKPOINT: actions ===
    def send_goal(self, target: int):
        self._client.wait_for_server()
        self.get_logger().info(f'Sending goal: count to {target}')

        goal = Fibonacci.Goal()
        goal.order = target

        future = self._client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_accepted)

    def on_feedback(self, feedback_msg):
        count = feedback_msg.feedback.sequence[0]
        self.get_logger().info(f'Feedback: {count}')

    def on_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return
        self.get_logger().info('Goal accepted — waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: reached {result.sequence[0]}')
        rclpy.shutdown()
    # === END CHECKPOINT: actions ===


def main(args=None):
    rclpy.init(args=args)
    node = CountClient()

    # === CHECKPOINT: actions ===
    node.send_goal(5)
    # === END CHECKPOINT: actions ===

    rclpy.spin(node)
    node.destroy_node()
