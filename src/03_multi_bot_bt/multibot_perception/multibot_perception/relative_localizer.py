"""relative_localizer — turn detected markers (name+pose) into named peer positions + TF.

Tier-1 localization: a marker id = a *named entity* (robot*10+face; 99 = world anchor). For
each marker this unit sees, we rotate the marker's camera-frame position into our base frame
(robust, intrinsics-free beyond the detector) and publish:
  - TF  <ns>/base_link -> <ns>/m_<id>      (so RViz shows where each named thing is)
  - geometry_msgs/PoseArray on 'peers'      (peer-marker positions in our base frame)
  - the world-anchor (id 99) as <ns>/anchor frame in base
Range = ||p||, bearing = atan2(y, x) in base (x fwd, y left). The BT uses these directly.

Tier-2 (deferred, needs on-display tuning): fuse the marker *orientation* + an EKF to get a
globally-consistent world->odom and full 6-DOF peer poses. Hook left in the comments below.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]])


# base_link -> camera_optical_frame (from the xacro): camera at (0.15,0,0.07),
# optical rotation rpy(-pi/2, 0, -pi/2).
BASE_R_OPT = rpy_to_R(-math.pi/2, 0.0, -math.pi/2)
BASE_t_OPT = np.array([0.15, 0.0, 0.07])
ANCHOR_ID = 99


class RelativeLocalizer(Node):

    def __init__(self):
        super().__init__('relative_localizer')
        ns = self.get_namespace().strip('/')
        self.prefix = (ns + '/') if ns else ''
        self.me = ns                       # e.g. 'r1'

        self.create_subscription(Detection2DArray, 'aruco/detections', self.on_dets, 10)
        self.peers_pub = self.create_publisher(PoseArray, 'peers', 10)
        # robust scalar for the leader/lost role decision: how far am I from the world anchor
        # (= position along the patrol axis). No orientation needed. -1 = anchor not in view.
        self.anchor_pub = self.create_publisher(Float32, 'anchor_range', 10)
        self.tf = TransformBroadcaster(self)
        self.get_logger().info(f'relative_localizer up (ns="{self.prefix}")')

    def on_dets(self, msg):
        pa = PoseArray()
        pa.header.stamp = msg.header.stamp
        pa.header.frame_id = f'{self.prefix}base_link'
        anchor_rng = -1.0
        for d in msg.detections:
            if not d.results:
                continue
            mid = int(d.results[0].hypothesis.class_id)
            p = d.results[0].pose.pose.position
            # marker position in OUR base frame
            p_base = BASE_R_OPT @ np.array([p.x, p.y, p.z]) + BASE_t_OPT
            rng = float(np.linalg.norm(p_base[:2]))
            bearing = math.atan2(p_base[1], p_base[0])

            # name the frame: peer markers -> m_<id>; anchor -> anchor
            child = f'{self.prefix}anchor' if mid == ANCHOR_ID else f'{self.prefix}m_{mid}'
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = f'{self.prefix}base_link'
            t.child_frame_id = child
            t.transform.translation.x = float(p_base[0])
            t.transform.translation.y = float(p_base[1])
            t.transform.translation.z = float(p_base[2])
            t.transform.rotation.w = 1.0
            self.tf.sendTransform(t)
            # === Tier-2 hook: use d.results[0].pose.pose.orientation here to recover the
            #     entity's full pose / a world->odom correction (deferred). ===

            if mid == ANCHOR_ID:
                anchor_rng = rng
            if mid != ANCHOR_ID:
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = \
                    float(p_base[0]), float(p_base[1]), float(p_base[2])
                pose.orientation.w = 1.0
                pa.poses.append(pose)
                self.get_logger().debug(
                    f'peer marker {mid} (unit r{mid//10} face {mid%10}): '
                    f'range={rng:.2f} bearing={math.degrees(bearing):+.0f}deg')
        self.peers_pub.publish(pa)
        self.anchor_pub.publish(Float32(data=anchor_rng))


def main(args=None):
    rclpy.init(args=args)
    node = RelativeLocalizer()
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
