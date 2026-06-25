"""marker_approach — the ApproachMarker action server (behaviour 6's "go to the marker").

Why an action: driving to a marker takes several seconds, we want live feedback
(searching -> aligning -> approaching) and the ability to cancel (switch behaviour). It
steers from pixels only — no camera intrinsics:

  bearing  = how far left/right the marker is in the frame  -> turn to centre it
  area_frac= how big the marker looks                        -> drive until close enough

  subscribes:  /aruco/detections  (vision_msgs/Detection2DArray)
               /camera/camera_info (sensor_msgs/CameraInfo)   -> image width/height
  publishes:   /cmd_vel           (geometry_msgs/Twist)
  action:      /approach_marker   (perceptbot_interfaces/ApproachMarker)
"""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray

from perceptbot_interfaces.action import ApproachMarker


class MarkerApproach(Node):

    def __init__(self):
        super().__init__('marker_approach')
        self.declare_parameter('search_turn', 0.5)        # rad/s while searching
        self.declare_parameter('approach_lin', 0.15)      # m/s when aligned
        self.declare_parameter('align_gain', 1.2)         # bearing -> angular.z
        self.declare_parameter('align_tol', 0.15)         # |bearing| below this = "centred"
        self.declare_parameter('default_stop_area', 0.10) # stop when marker fills this frac
        self.declare_parameter('lost_timeout', 3.0)       # s without a seen marker -> lost
        self.declare_parameter('overall_timeout', 40.0)   # s total before giving up
        cb = ReentrantCallbackGroup()

        self.w = self.h = 0
        self.detections = []
        self.det_stamp = 0.0
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, 10,
                                 callback_group=cb)
        self.create_subscription(Detection2DArray, '/aruco/detections', self.on_dets, 10,
                                 callback_group=cb)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._action = ActionServer(
            self, ApproachMarker, '/approach_marker',
            execute_callback=self.execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=cb)
        self.get_logger().info('marker_approach up: /approach_marker (action).')

    def on_info(self, msg):
        self.w, self.h = msg.width, msg.height

    def on_dets(self, msg):
        self.detections = msg.detections
        self.det_stamp = time.time()

    def _find(self, marker_id):
        """Return (bearing, area_frac) for the wanted marker, or None if not visible."""
        if not self.w or not self.h:
            return None
        for d in self.detections:
            if not d.results:
                continue
            mid = int(d.results[0].hypothesis.class_id)
            if marker_id >= 0 and mid != marker_id:
                continue
            bearing = (d.bbox.center.position.x - self.w / 2.0) / (self.w / 2.0)
            area_frac = (d.bbox.size_x * d.bbox.size_y) / float(self.w * self.h)
            return bearing, area_frac
        return None

    def execute(self, goal_handle):
        g = goal_handle.request
        stop_area = g.stop_area_frac if g.stop_area_frac > 0 else \
            self.get_parameter('default_stop_area').value
        search_turn = self.get_parameter('search_turn').value
        approach_lin = self.get_parameter('approach_lin').value
        gain = self.get_parameter('align_gain').value
        tol = self.get_parameter('align_tol').value
        lost_timeout = self.get_parameter('lost_timeout').value
        overall_timeout = self.get_parameter('overall_timeout').value

        t_start = time.time()
        t_last_seen = None

        def finish(outcome, ok):
            self.cmd_pub.publish(Twist())
            return ApproachMarker.Result(succeeded=ok, outcome=outcome)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('approach CANCELLED')
                return finish('canceled', False)

            if time.time() - t_start > overall_timeout:
                goal_handle.abort()
                return finish('timeout', False)

            # marker counts as visible only if detections are fresh
            found = self._find(g.marker_id) if (time.time() - self.det_stamp) < 0.5 else None
            tw = Twist()
            fb = ApproachMarker.Feedback()

            if found is None:
                # lost after having seen it -> give up; else keep searching (spin)
                if t_last_seen and (time.time() - t_last_seen) > lost_timeout:
                    goal_handle.abort()
                    return finish('lost', False)
                tw.angular.z = search_turn
                fb.step, fb.visible, fb.bearing, fb.area_frac = 'searching', False, 0.0, 0.0
            else:
                bearing, area_frac = found
                t_last_seen = time.time()
                if area_frac >= stop_area:
                    goal_handle.succeed()
                    self.get_logger().info('approach DONE: reached')
                    return finish('reached', True)
                tw.angular.z = -gain * bearing          # turn to centre the marker
                if abs(bearing) < tol:
                    tw.linear.x = approach_lin          # centred enough -> drive in
                    fb.step = 'approaching'
                else:
                    fb.step = 'aligning'
                fb.visible, fb.bearing, fb.area_frac = True, float(bearing), float(area_frac)

            self.cmd_pub.publish(tw)
            goal_handle.publish_feedback(fb)
            time.sleep(0.1)

        return finish('aborted', False)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerApproach()
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
