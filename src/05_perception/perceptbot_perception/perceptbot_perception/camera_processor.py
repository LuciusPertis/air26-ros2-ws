"""camera_processor — turn a camera image into two cheap, behaviour-friendly numbers.

This is the software twin of what the ESP32-CAM firmware computes on-board: reduce a
whole frame to (a) one brightness number and (b) one average colour. Behaviours 4 and 5
condition on these instead of the ultrasonics.

  subscribes:  /camera/image_raw     (sensor_msgs/Image)
  publishes:   /camera/mean_intensity (std_msgs/Float32)   0.0 (dark) .. 1.0 (bright)
               /camera/mean_color     (std_msgs/ColorRGBA)  mean R,G,B in 0..1, a=1

Both are normalised to 0..1 so the SAME numbers come out of the sim and the real
ESP32-CAM (the firmware publishes the identical two topics). That is the whole point of
project 05's "same interface, swappable embodiment" design.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, ColorRGBA
from cv_bridge import CvBridge
import cv2


class CameraProcessor(Node):

    def __init__(self):
        super().__init__('camera_processor')
        self.declare_parameter('image_topic', '/camera/image_raw')
        topic = self.get_parameter('image_topic').value
        self.bridge = CvBridge()

        self.create_subscription(Image, topic, self.on_image, 10)
        # === CHECKPOINT: mean_intensity ===
        self.intensity_pub = self.create_publisher(Float32, '/camera/mean_intensity', 10)
        # === END CHECKPOINT: mean_intensity ===
        # === CHECKPOINT: mean_color ===
        self.color_pub = self.create_publisher(ColorRGBA, '/camera/mean_color', 10)
        # === END CHECKPOINT: mean_color ===

        self.get_logger().info(f'camera_processor up, reading {topic} -> '
                               '/camera/mean_intensity + /camera/mean_color')

    def on_image(self, msg):
        # bgr8: handles Webots bgra8 and a real cam's rgb/jpeg alike via cv_bridge
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # === CHECKPOINT: mean_intensity ===
        # one grayscale mean over the frame, normalised 0..1 (same maths the firmware does)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        self.intensity_pub.publish(Float32(data=float(gray.mean()) / 255.0))
        # === END CHECKPOINT: mean_intensity ===

        # === CHECKPOINT: mean_color ===
        # average B,G,R over the frame -> ColorRGBA (note cv2 is BGR, ROS wants R,G,B)
        b, g, r = bgr.reshape(-1, 3).mean(axis=0) / 255.0
        self.color_pub.publish(ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0))
        # === END CHECKPOINT: mean_color ===


def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessor()
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
