"""aruco_pose_detector — each visible marker -> its 3D pose (Tier-1 localization).

Runs per unit (in that unit's namespace). For every ArUco marker in view it solves the
marker's pose from the 4 corners + known camera intrinsics + known marker size (solvePnP),
and publishes it as a vision_msgs/Detection2DArray (id in class_id, 3D pose in results.pose)
plus a TF camera_optical_frame -> <ns>/obs_marker_<id>. A marker id encodes name+face of an
entity (robot*10+face; 99 = world anchor), so this is "where is each named thing I can see".

  subscribes:  camera/image_raw, camera/camera_info   (relative -> namespaced)
  publishes:   aruco/detections (vision_msgs/Detection2DArray)  + TF per marker
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import cv2


def rvec_to_quat(rvec):
    R, _ = cv2.Rodrigues(rvec)
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    return x, y, z, w


class ArucoPoseDetector(Node):

    def __init__(self):
        super().__init__('aruco_pose_detector')
        self.declare_parameter('marker_size', 0.096)   # black-square side (m)
        self.declare_parameter('dictionary', '4x4_250')
        self.s = self.get_parameter('marker_size').value
        ns = self.get_namespace().strip('/')
        self.prefix = (ns + '/') if ns else ''

        self.bridge = CvBridge()
        self.K = None
        self.dist = np.zeros(5)
        d = {'4x4_50': cv2.aruco.DICT_4X4_50, '4x4_250': cv2.aruco.DICT_4X4_250}[
            self.get_parameter('dictionary').value]
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(d), cv2.aruco.DetectorParameters())
        # marker corner object points (TL, TR, BR, BL) in the marker plane
        s = self.s
        self.objp = np.array([[-s/2, s/2, 0], [s/2, s/2, 0],
                              [s/2, -s/2, 0], [-s/2, -s/2, 0]], np.float32)

        self.create_subscription(CameraInfo, 'camera/camera_info', self.on_info, 10)
        self.create_subscription(Image, 'camera/image_raw', self.on_image, 10)
        self.det_pub = self.create_publisher(Detection2DArray, 'aruco/detections', 10)
        self.tf = TransformBroadcaster(self)
        self.get_logger().info(f'aruco_pose_detector up (ns="{self.prefix}", '
                               f'marker_size={self.s} m)')

    def on_info(self, msg):
        if self.K is None and any(msg.k):
            self.K = np.array(msg.k, np.float32).reshape(3, 3)
            if any(msg.d):
                self.dist = np.array(msg.d, np.float32)

    def on_image(self, msg):
        if self.K is None:
            return
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        out = Detection2DArray()
        out.header = msg.header
        if ids is not None:
            for mc, mid in zip(corners, ids.flatten()):
                pts = mc.reshape(4, 2).astype(np.float32)
                ok, rvec, tvec = cv2.solvePnP(self.objp, pts, self.K, self.dist,
                                              flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue
                qx, qy, qz, qw = rvec_to_quat(rvec)
                cx, cy = pts.mean(axis=0)

                det = Detection2D()
                det.header = msg.header
                det.bbox.center.position.x = float(cx)
                det.bbox.center.position.y = float(cy)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(int(mid))
                hyp.hypothesis.score = 1.0
                hyp.pose.pose.position.x = float(tvec[0])
                hyp.pose.pose.position.y = float(tvec[1])
                hyp.pose.pose.position.z = float(tvec[2])
                hyp.pose.pose.orientation.x = qx
                hyp.pose.pose.orientation.y = qy
                hyp.pose.pose.orientation.z = qz
                hyp.pose.pose.orientation.w = qw
                det.results.append(hyp)
                out.detections.append(det)

                t = TransformStamped()
                t.header = msg.header        # parent = <ns>/camera_optical_frame
                t.child_frame_id = f'{self.prefix}obs_marker_{int(mid)}'
                t.transform.translation.x = float(tvec[0])
                t.transform.translation.y = float(tvec[1])
                t.transform.translation.z = float(tvec[2])
                t.transform.rotation.x = qx
                t.transform.rotation.y = qy
                t.transform.rotation.z = qz
                t.transform.rotation.w = qw
                self.tf.sendTransform(t)
        self.det_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseDetector()
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
