"""obstacle_services — the SERVICE + ACTION side of behaviour 3.

Behaviour 1 and 2 are pure pub-sub (in behavior_manager). Behaviour 3 is deliberately
"bigger": it needs an instant question and a long, watchable, cancelable task — which is
exactly what services and actions are for. This node provides both:

  service  /check_openings  (microbot_interfaces/CheckOpenings)
      instant request/response: are left/right open right now? (one question, one answer)
  action   /escape_obstacle  (microbot_interfaces/EscapeObstacle)
      a multi-second maneuver with live feedback and cancel: turn toward the open side,
      or back up + spin 180 if both sides are blocked.

The action publishes /cmd_vel while it runs; the manager stays quiet during that time.
"""

import random
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

from microbot_interfaces.srv import CheckOpenings
from microbot_interfaces.action import EscapeObstacle

TURN_W = 0.9        # rad/s while turning
BACK_V = -0.15      # m/s while backing up
TURN_90_T = 1.8     # s to turn ~90 deg
TURN_180_T = 3.6    # s to turn ~180 deg
BACK_T = 1.5        # s to back up


class ObstacleServices(Node):

    def __init__(self):
        super().__init__('obstacle_services')
        self.declare_parameter('side_threshold', 0.35)
        cb = ReentrantCallbackGroup()

        self.left = self.right = 99.0
        self.create_subscription(Range, '/ultrasonic/left',
                                 lambda m: setattr(self, 'left', m.range), 10)
        self.create_subscription(Range, '/ultrasonic/right',
                                 lambda m: setattr(self, 'right', m.range), 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # === CHECKPOINT: service ===
        self.create_service(CheckOpenings, '/check_openings', self.check_openings,
                            callback_group=cb)
        # === END CHECKPOINT: service ===

        # === CHECKPOINT: action ===
        self._action = ActionServer(
            self, EscapeObstacle, '/escape_obstacle',
            execute_callback=self.execute_escape,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=cb)
        # === END CHECKPOINT: action ===

        self.get_logger().info('obstacle_services up: /check_openings (srv) + /escape_obstacle (action).')

    # --- the instant query: a SERVICE is the right tool ---
    def check_openings(self, request, response):
        thr = request.threshold if request.threshold > 0 else \
            self.get_parameter('side_threshold').value
        response.left_range = float(self.left)
        response.right_range = float(self.right)
        response.left_open = self.left > thr
        response.right_open = self.right > thr
        self.get_logger().info(
            f'/check_openings -> left_open={response.left_open} right_open={response.right_open}')
        return response

    # --- the long maneuver: an ACTION is the right tool (feedback + cancel) ---
    def execute_escape(self, goal_handle):
        g = goal_handle.request
        if g.left_open or g.right_open:
            if g.left_open and g.right_open:
                go_left = random.random() < 0.5
            else:
                go_left = g.left_open
            steps = [('turning', 0.0, TURN_W if go_left else -TURN_W, TURN_90_T)]
            outcome = 'turned_left' if go_left else 'turned_right'
        else:
            steps = [('backing_up', BACK_V, 0.0, BACK_T),
                     ('turning_180', 0.0, TURN_W, TURN_180_T)]
            outcome = 'backed_and_turned'

        total = sum(s[3] for s in steps)
        elapsed = 0.0
        for label, lin, ang, dur in steps:
            t0 = time.time()
            while time.time() - t0 < dur:
                if goal_handle.is_cancel_requested:
                    self.cmd_pub.publish(Twist())          # stop
                    goal_handle.canceled()
                    self.get_logger().info('escape CANCELLED')
                    return EscapeObstacle.Result(succeeded=False, outcome='cancelled')
                tw = Twist()
                tw.linear.x = lin
                tw.angular.z = ang
                self.cmd_pub.publish(tw)
                fb = EscapeObstacle.Feedback()
                fb.step = label
                fb.progress = min((elapsed + (time.time() - t0)) / total, 1.0)
                goal_handle.publish_feedback(fb)
                time.sleep(0.1)
            elapsed += dur

        self.cmd_pub.publish(Twist())                       # stop at the end
        goal_handle.succeed()
        self.get_logger().info(f'escape DONE: {outcome}')
        return EscapeObstacle.Result(succeeded=True, outcome=outcome)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleServices()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
