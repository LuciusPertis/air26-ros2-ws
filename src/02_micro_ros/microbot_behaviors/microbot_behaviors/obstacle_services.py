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
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

from microbot_interfaces.srv import CheckOpenings
from microbot_interfaces.action import EscapeObstacle

_factor = 1.8 # scale factor to tweak timing; this is a trial and error thing, since we don't have odometry feedback to know how far we have turned or backed up. The robot is not very precise, so we need to overshoot a bit to make sure it actually turns enough to escape the obstacle.
TURN_W = 0.9*_factor        # rad/s while turning
BACK_V = -0.15*_factor      # m/s while backing up
TURN_90_T = 1.8*_factor     # s to turn ~90 deg
TURN_180_T = 3.6*_factor    # s to turn ~180 deg
BACK_T = 2.5*_factor        # s to back up


class ObstacleServices(Node):

    def __init__(self):
        super().__init__('obstacle_services')
        self.declare_parameter('side_threshold', 50)  # cm (was 0.50 m)
        cb = ReentrantCallbackGroup()

        self.left = self.right = 255   # cm; 255 = "far / no reading yet"
        # best-effort to match the sim driver / real ESP32 sensor QoS (latency branch).
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(UInt8, '/ultrasonic/left',
                                 lambda m: setattr(self, 'left', m.data), sensor_qos)
        self.create_subscription(UInt8, '/ultrasonic/right',
                                 lambda m: setattr(self, 'right', m.data), sensor_qos)
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

        total_time_req = sum(s[3] for s in steps)
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
                fb.progress = min((elapsed + (time.time() - t0)) / total_time_req, 1.0)
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
