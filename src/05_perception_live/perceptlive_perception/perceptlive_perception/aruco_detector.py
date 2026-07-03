"""aruco_detector — find ArUco markers in the camera image.

Behaviour 6 (search + approach) needs to know WHERE a marker is. This node does the
vision; the behaviour does the driving. We publish the marker as a standard
vision_msgs/Detection2DArray (bbox in pixels, marker id as the class label) so it plugs
into the wider ROS perception ecosystem — and an annotated image for RViz.

  subscribes:  /camera/image_raw   (sensor_msgs/Image)
  publishes:   /aruco/detections   (vision_msgs/Detection2DArray)  bbox center+size, id
               /aruco/image        (sensor_msgs/Image)             overlay for RViz

Steering is done from pixels only (bbox center -> bearing, bbox area -> range proxy), so
no camera calibration / intrinsics are needed — deliberately simple for the workshop.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np

# map a friendly name -> cv2 predefined dictionary
ARUCO_DICTS = {
    '4x4_50': cv2.aruco.DICT_4X4_50,
    '5x5_50': cv2.aruco.DICT_5X5_50,
    '6x6_250': cv2.aruco.DICT_6X6_250,
}


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('dictionary', '4x4_50')
        self.declare_parameter('publish_overlay', True)
        topic = self.get_parameter('image_topic').value
        dict_name = self.get_parameter('dictionary').value
        self.publish_overlay = self.get_parameter('publish_overlay').value

        self.bridge = CvBridge()
        aruco_dict = cv2.aruco.getPredefinedDictionary(
            ARUCO_DICTS.get(dict_name, cv2.aruco.DICT_4X4_50))
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

        self.create_subscription(Image, topic, self.on_image, 10)
        self.det_pub = self.create_publisher(Detection2DArray, '/aruco/detections', 10)
        if self.publish_overlay:
            self.img_pub = self.create_publisher(Image, '/aruco/image', 10)

        self.get_logger().info(f'aruco_detector up ({dict_name}), reading {topic} -> '
                               '/aruco/detections (+ /aruco/image)')

    def on_image(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        # === CHECKPOINT: aruco_detect ===
        corners, ids, _ = self.detector.detectMarkers(gray)
        # === END CHECKPOINT: aruco_detect ===

        out = Detection2DArray()
        out.header = msg.header
        if ids is not None:
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                pts = marker_corners.reshape(4, 2)
                cx, cy = pts.mean(axis=0)
                w = float(np.linalg.norm(pts[0] - pts[1]) + np.linalg.norm(pts[2] - pts[3])) / 2
                h = float(np.linalg.norm(pts[1] - pts[2]) + np.linalg.norm(pts[3] - pts[0])) / 2

                det = Detection2D()
                det.header = msg.header
                det.bbox.center.position.x = float(cx)
                det.bbox.center.position.y = float(cy)
                det.bbox.size_x = w
                det.bbox.size_y = h
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(int(marker_id))
                hyp.hypothesis.score = 1.0
                det.results.append(hyp)
                out.detections.append(det)

        self.det_pub.publish(out)

        # === CHECKPOINT: aruco_overlay ===
        if self.publish_overlay:
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
            overlay = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            overlay.header = msg.header
            self.img_pub.publish(overlay)
        # === END CHECKPOINT: aruco_overlay ===


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
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
