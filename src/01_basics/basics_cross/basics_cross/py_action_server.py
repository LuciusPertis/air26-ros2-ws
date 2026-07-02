#!/usr/bin/env python3
# CROSS-LANGUAGE — Actions
# Python action server that the C++ client sends goals to.
#
# Terminal 1: ros2 run basics_cross py_action_server.py
# Terminal 2: ros2 run basics_cross cpp_action_client

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from basics_cross.action import CountUp  # our own action — no example_interfaces


class PyActionServer(Node):
    def __init__(self):
        super().__init__('py_action_server')
        self._action_server = ActionServer(
            self,
            CountUp,
            'count_up',
            self.execute_callback,
        )
        self.get_logger().info('Python action server /count_up ready')

    def execute_callback(self, goal_handle):
        target = goal_handle.request.order
        self.get_logger().info(f'[Python server] Goal from C++ client: count to {target}')
        feedback = CountUp.Feedback()
        for i in range(target + 1):
            feedback.sequence = [i]
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'[Python server] Feedback: {i}/{target}')
            time.sleep(1.0)
        goal_handle.succeed()
        result = CountUp.Result()
        result.sequence = [target]
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PyActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
