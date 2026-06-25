"""behavior_manager — six switchable behaviours for the perception rover.

Project 05 keeps project-02's three OBSTACLE behaviours (driven by the ultrasonics) and
adds three VISION behaviours (driven by the camera topics). One /set_behavior service
switches between all of them at runtime — the whole "modular additive" idea: same robot,
same /cmd_vel, progressively richer senses.

  1  obstacle: stop + timer            (pub-sub on /ultrasonic/front)      [from 02]
  2  obstacle: stop + random turn      (pub-sub)                           [from 02]
  3  obstacle: service + escape action (/check_openings + /escape_obstacle)[from 02]
  4  light-seek : go toward brightness (/camera/mean_intensity, scalar)
  5  colour-seek: chase a target hue   (/camera/mean_color, scalar)
  6  ArUco      : search + approach     (/aruco/detections + /approach_marker action)

  subscribes: /ultrasonic/front|left|right, /camera/mean_intensity,
              /camera/mean_color, /aruco/detections
  publishes:  /cmd_vel
  service:    /set_behavior
  clients:    /check_openings, /escape_obstacle [B3];  /approach_marker [B6]

B4/B5 use a single scalar (exactly what the cheap ESP32-CAM topics give): "see it -> drive
at it, else spin to search". B6 uses real per-marker positions + an action — the contrast
motivates why localisation/actions matter.
"""

import math
import random

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, ColorRGBA
from vision_msgs.msg import Detection2DArray

from perceptbot_interfaces.srv import CheckOpenings, SetBehavior
from perceptbot_interfaces.action import EscapeObstacle, ApproachMarker

# obstacle-behaviour control states
WALK, B1_WAIT, B2_TURN, B3_ESCAPE = 'walk', 'b1_wait', 'b2_turn', 'b3_escape'


class BehaviorManager(Node):

    def __init__(self):
        super().__init__('behavior_manager')
        # obstacle params (B1-B3)
        self.declare_parameter('front_threshold', 0.35)
        self.declare_parameter('walk_period', 2.0)
        self.declare_parameter('b1_wait', 2.0)
        self.declare_parameter('b2_turn', 1.5)
        # vision params (B4-B6)
        self.declare_parameter('intensity_threshold', 0.55)   # B4: "bright enough" to advance
        self.declare_parameter('target_color', [0.1, 0.7, 0.2])  # B5: r,g,b 0..1 (greenish)
        self.declare_parameter('color_match_threshold', 0.65)  # B5: 1-dist must exceed this
        self.declare_parameter('target_marker', -1)           # B6: which id (-1 = any)
        self.declare_parameter('search_turn', 0.5)            # rad/s spin while searching
        self.declare_parameter('seek_lin', 0.15)              # m/s when homing in
        self.declare_parameter('vision_front_stop', 0.28)     # safety stop for B4-B6
        self.front_thr = self.get_parameter('front_threshold').value

        self.behavior = 1
        self.state = WALK
        self.front = self.left = self.right = 99.0
        self.mean_intensity = 0.0
        self.mean_color = (0.0, 0.0, 0.0)
        self.detections = []
        self.walk_cmd = Twist()
        self.t_next_sample = 0.0
        self.t_state_end = 0.0
        self.turn_dir = 1.0
        self.escape_handle = None
        self.approach_handle = None
        self.approaching = False

        # senses
        self.create_subscription(Range, '/ultrasonic/front',
                                 lambda m: setattr(self, 'front', m.range), 10)
        self.create_subscription(Range, '/ultrasonic/left',
                                 lambda m: setattr(self, 'left', m.range), 10)
        self.create_subscription(Range, '/ultrasonic/right',
                                 lambda m: setattr(self, 'right', m.range), 10)
        self.create_subscription(Float32, '/camera/mean_intensity',
                                 lambda m: setattr(self, 'mean_intensity', m.data), 10)
        self.create_subscription(ColorRGBA, '/camera/mean_color',
                                 lambda m: setattr(self, 'mean_color', (m.r, m.g, m.b)), 10)
        self.create_subscription(Detection2DArray, '/aruco/detections',
                                 lambda m: setattr(self, 'detections', m.detections), 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_service(SetBehavior, '/set_behavior', self.on_set_behavior)

        # B3 clients
        self.check_cli = self.create_client(CheckOpenings, '/check_openings')
        self.escape_cli = ActionClient(self, EscapeObstacle, '/escape_obstacle')
        # B6 client
        self.approach_cli = ActionClient(self, ApproachMarker, '/approach_marker')

        self.create_timer(0.1, self.control_loop)   # 10 Hz
        self.get_logger().info('behavior_manager up: behaviour=1. Switch with /set_behavior (1-6).')

    # ---------- runtime behaviour switch ----------
    def on_set_behavior(self, request, response):
        if request.behavior not in range(1, 7):
            response.success = False
            response.message = 'behavior must be 1..6'
            return response
        # cancel any in-flight action when leaving its behaviour
        if self.state == B3_ESCAPE and self.escape_handle is not None:
            self.escape_handle.cancel_goal_async()
        if self.approaching and self.approach_handle is not None:
            self.approach_handle.cancel_goal_async()
        self.behavior = request.behavior
        self.state = WALK
        self.approaching = False
        self.cmd_pub.publish(Twist())
        response.success = True
        response.message = f'behavior set to {request.behavior}'
        self.get_logger().info(response.message)
        return response

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- the 10 Hz dispatcher ----------
    def control_loop(self):
        if self.behavior in (1, 2, 3):
            self.obstacle_loop()
        elif self.behavior == 4:
            self.light_loop()
        elif self.behavior == 5:
            self.color_loop()
        elif self.behavior == 6:
            self.aruco_loop()

    # safety used by the vision behaviours: returns True if it took over for an obstacle
    def avoid_guard(self):
        if self.front < self.get_parameter('vision_front_stop').value:
            tw = Twist()
            tw.angular.z = self.get_parameter('search_turn').value   # turn away
            self.cmd_pub.publish(tw)
            return True
        return False

    # ===================== B1-B3 (obstacle, from 02) =====================
    def sample_walk(self):
        self.walk_cmd = Twist()
        self.walk_cmd.linear.x = random.uniform(0.08, 0.20)
        self.walk_cmd.angular.z = random.uniform(-0.4, 0.4)
        self.t_next_sample = self.now() + self.get_parameter('walk_period').value

    def obstacle_loop(self):
        t = self.now()
        if self.state == B3_ESCAPE:
            return
        if self.state == B1_WAIT:
            self.cmd_pub.publish(Twist())
            if t >= self.t_state_end:
                self.state = WALK
            return
        if self.state == B2_TURN:
            tw = Twist()
            tw.angular.z = 0.9 * self.turn_dir
            self.cmd_pub.publish(tw)
            if t >= self.t_state_end:
                self.state = WALK
            return
        # WALK
        if self.front < self.front_thr:
            self.trigger_obstacle_response()
            return
        if t >= self.t_next_sample:
            self.sample_walk()
        self.cmd_pub.publish(self.walk_cmd)

    def trigger_obstacle_response(self):
        self.cmd_pub.publish(Twist())
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

    def start_escape(self):
        if not self.check_cli.service_is_ready():
            self.get_logger().warn('/check_openings not ready; resuming walk')
            self.state = WALK
            return
        req = CheckOpenings.Request()
        req.threshold = 0.0
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
            self.state = WALK
            return
        self.escape_handle = handle
        handle.get_result_async().add_done_callback(self.on_escape_done)

    def on_escape_feedback(self, fb):
        self.get_logger().info(f'  escape step: {fb.feedback.step} '
                               f'({fb.feedback.progress*100:.0f}%)')

    def on_escape_done(self, _fut):
        self.escape_handle = None
        if self.state == B3_ESCAPE:
            self.state = WALK

    # ===================== B4 light-seek =====================
    def light_loop(self):
        if self.avoid_guard():
            return
        tw = Twist()
        # === CHECKPOINT: behavior_4 ===  (phototaxis on a single scalar)
        if self.mean_intensity >= self.get_parameter('intensity_threshold').value:
            tw.linear.x = self.get_parameter('seek_lin').value      # bright -> head in
        else:
            tw.angular.z = self.get_parameter('search_turn').value  # dark -> spin to search
        # === END CHECKPOINT: behavior_4 ===
        self.cmd_pub.publish(tw)

    # ===================== B5 colour-seek =====================
    def color_loop(self):
        if self.avoid_guard():
            return
        tw = Twist()
        # === CHECKPOINT: behavior_5 ===  (chromotaxis: how close is the mean colour to target?)
        tgt = self.get_parameter('target_color').value
        dist = math.sqrt(sum((c - t) ** 2 for c, t in zip(self.mean_color, tgt))) / math.sqrt(3)
        match = 1.0 - dist
        if match >= self.get_parameter('color_match_threshold').value:
            tw.linear.x = self.get_parameter('seek_lin').value      # colour present -> approach
        else:
            tw.angular.z = self.get_parameter('search_turn').value  # not seen -> spin to search
        # === END CHECKPOINT: behavior_5 ===
        self.cmd_pub.publish(tw)

    # ===================== B6 ArUco search + approach =====================
    def aruco_loop(self):
        if self.approaching:
            return                       # the approach action owns /cmd_vel
        if self.avoid_guard():
            return
        # === CHECKPOINT: behavior_6 ===  (search by spinning; on sight, launch the approach action)
        target = self.get_parameter('target_marker').value
        seen = any(d.results and (target < 0 or int(d.results[0].hypothesis.class_id) == target)
                   for d in self.detections)
        if seen and self.approach_cli.server_is_ready():
            self.start_approach(target)
        else:
            tw = Twist()
            tw.angular.z = self.get_parameter('search_turn').value
            self.cmd_pub.publish(tw)
        # === END CHECKPOINT: behavior_6 ===

    def start_approach(self, target):
        self.approaching = True
        goal = ApproachMarker.Goal(marker_id=int(target), stop_area_frac=0.0)
        self.approach_cli.send_goal_async(
            goal, feedback_callback=self.on_approach_feedback
        ).add_done_callback(self.on_approach_accepted)

    def on_approach_accepted(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self.approaching = False
            return
        self.approach_handle = handle
        handle.get_result_async().add_done_callback(self.on_approach_done)

    def on_approach_feedback(self, fb):
        f = fb.feedback
        self.get_logger().info(f'  approach: {f.step} visible={f.visible} '
                               f'bearing={f.bearing:+.2f} area={f.area_frac:.3f}')

    def on_approach_done(self, fut):
        outcome = fut.result().result.outcome
        self.get_logger().info(f'approach finished: {outcome}')
        self.approach_handle = None
        self.approaching = False     # resume searching (or reached & idle until re-seen)


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
