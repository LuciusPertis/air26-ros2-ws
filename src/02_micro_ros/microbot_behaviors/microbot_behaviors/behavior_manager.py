"""behavior_manager — random walk + the three switchable obstacle behaviours.

This is what the kids write/read. It does a random walk and, when the front ultrasonic
sees something close, reacts according to the ACTIVE behaviour:

  B1 (pub-sub):  stop, wait on a timer, resume random walk.
  B2 (pub-sub):  stop, turn a random direction briefly, resume.
  B3 (service+action): ask /check_openings (service), then run /escape_obstacle (action)
                       -- a long, watchable, cancelable maneuver. Switching away from B3
                       mid-escape CANCELS the action (the "why we need actions" moment).

  subscribes:  /ultrasonic/front|left|right  (sensor_msgs/Range)
  publishes:   /cmd_vel                       (geometry_msgs/Twist)
  service:     /set_behavior                  (switch 1|2|3 at runtime)
  clients:     /check_openings (srv) + /escape_obstacle (action)   [B3 only]

The deliberately escalating cost of B1 -> B2 -> B3 motivates topic -> service -> action.
"""

import random

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

from microbot_interfaces.srv import CheckOpenings
from microbot_interfaces.action import EscapeObstacle
from microbot_interfaces.srv import SetBehavior

# control states
WALK, B1_WAIT, B2_TURN, B3_ESCAPE = 'walk', 'b1_wait', 'b2_turn', 'b3_escape'


class BehaviorManager(Node):

    def __init__(self):
        super().__init__('behavior_manager')
        self.declare_parameter('front_threshold', 0.35)
        self.declare_parameter('walk_period', 2.0)     # s between random-walk samples
        self.declare_parameter('b1_wait', 2.0)         # s to pause in B1
        self.declare_parameter('b2_turn', 1.5)         # s to turn in B2
        self.front_thr = self.get_parameter('front_threshold').value

        self.behavior = 1
        self.state = WALK
        self.front = 99.0
        self.walk_cmd = Twist()
        self.t_next_sample = 0.0
        self.t_state_end = 0.0
        self.turn_dir = 1.0
        self.escape_handle = None

        self.create_subscription(Range, '/ultrasonic/front',
                                 lambda m: setattr(self, 'front', m.range), 2) # shortening the queue size to 2 (old 10); avoids old readings when robot is stopped; keeps latest reading
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_service(SetBehavior, '/set_behavior', self.on_set_behavior)

        # B3 clients
        self.check_cli = self.create_client(CheckOpenings, '/check_openings')
        self.escape_cli = ActionClient(self, EscapeObstacle, '/escape_obstacle')

        self.create_timer(0.02, self.control_loop)   # earlier 0.1=>10 Hz ; now 0.02=>50 Hz
        self.get_logger().info('behavior_manager up: behaviour=1, random walking. '
                               'Switch with /set_behavior.')

    # ---------- runtime behaviour switch (a SERVICE) ----------
    def on_set_behavior(self, request, response):
        if request.behavior not in (1, 2, 3):
            response.success = False
            response.message = 'behavior must be 1, 2 or 3'
            return response
        # cancel an in-flight escape if we're leaving B3
        if self.state == B3_ESCAPE and self.escape_handle is not None and request.behavior != 3: # repeated B3 requests are allowed, we don't want to cancel the action if we're staying in B3
            self.escape_handle.cancel_goal_async()
        self.behavior = request.behavior
        self.state = WALK
        response.success = True
        response.message = f'behavior set to {request.behavior}'
        self.get_logger().info(response.message)
        return response

    # ---------- random walk ----------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sample_walk(self):
        self.walk_cmd = Twist()
        self.walk_cmd.linear.x = random.uniform(0.08, 0.20)        # rand linear vel
        self.walk_cmd.angular.z = random.uniform(-0.4, 0.4)        # rand small angular vel
        self.t_next_sample = self.now() + self.get_parameter('walk_period').value

    # ---------- the 50 Hz control loop ----------
    def control_loop(self):
        t = self.now()

        if self.state == B3_ESCAPE:
            return                       # the escape action owns /cmd_vel right now

        if self.state == B1_WAIT:
            self.cmd_pub.publish(Twist())                # hold still
            if t >= self.t_state_end:
                self.state = WALK
            return

        if self.state == B2_TURN:
            tw = Twist()
            tw.angular.z = 0.9 * self.turn_dir # if we really want to turn it proper 90 degrees, we need feedback from the odometry !
            self.cmd_pub.publish(tw)
            if t >= self.t_state_end:
                self.state = WALK
            return

        # --- WALK ---
        if self.front < self.front_thr:
            self.trigger_obstacle_response()
            return
        if t >= self.t_next_sample:
            self.sample_walk()
        self.cmd_pub.publish(self.walk_cmd)

    # ---------- obstacle response depends on the active behaviour ----------
    def trigger_obstacle_response(self):
        self.cmd_pub.publish(Twist())                    # always stop first
        if self.behavior == 1:
            # === CHECKPOINT: behavior_1 ===  (pub-sub: stop + timer)
            self.state = B1_WAIT
            self.t_state_end = self.now() + self.get_parameter('b1_wait').value
            # === END CHECKPOINT: behavior_1 ===
        elif self.behavior == 2:
            # === CHECKPOINT: behavior_2 ===  (pub-sub: stop + random turn)
            self.turn_dir = random.choice((-1.0, 1.0))
            self.state = B2_TURN
            self.t_state_end = self.now() + self.get_parameter('b2_turn').value
            # === END CHECKPOINT: behavior_2 ===
        else:
            # === CHECKPOINT: behavior_3 ===  (service + action)
            self.state = B3_ESCAPE
            self.start_escape()
            # === END CHECKPOINT: behavior_3 ===

    # ---------- B3: service query, then action maneuver ----------
    def start_escape(self):
        self.check_cli.wait_for_service(timeout_sec=2.0) # we have already stoped; avoids looping back to control_loop and triggering another obstacle response while waiting for the service to be ready
        
        # if not self.check_cli.service_is_ready():
        #     self.get_logger().warn('/check_openings not ready; resuming walk')
        #     self.state = WALK   # control_loop(:WALK) -> trigger_obstacle_response(:B3) -> back to start_escape() | last published /cmd_vel is stop
        #     return

        # new loop
        while not self.check_cli.service_is_ready():
            self.get_logger().warn('/check_openings not ready; waiting 1s and retrying')
            self.check_cli.wait_for_service(timeout_sec=2.0)
            # we can either sleep or spin once to allow the service to be ready; if we sleep, we need to make sure we don't block the control loop for too long
            # self.sleep(1.0)  # wait a bit before retrying 

        req = CheckOpenings.Request()
        req.threshold = 0.0                               # use server default
        self.check_cli.call_async(req).add_done_callback(self.on_openings)

    def on_openings(self, fut):
        resp = fut.result()
        goal = EscapeObstacle.Goal(left_open=resp.left_open, right_open=resp.right_open)
        self.escape_cli.wait_for_server(timeout_sec=2.0)
        self.escape_cli.send_goal_async(
            goal, feedback_callback=self.on_escape_feedback
        ).add_done_callback(self.on_escape_accepted)

    def on_escape_accepted(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().warn('/escape_obstacle goal REJECTED; resuming walk')
            self.state = WALK
            return
        self.escape_handle = handle
        handle.get_result_async().add_done_callback(self.on_escape_done)

    def on_escape_feedback(self, fb):
        self.get_logger().info(f'  escape step: {fb.feedback.step} '
                               f'({fb.feedback.progress*100:.0f}%)')

    def on_escape_done(self, _fut):
        self.escape_handle = None
        if self.state == B3_ESCAPE:        # only resume if we weren't switched away
            self.state = WALK


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
