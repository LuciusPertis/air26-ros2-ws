"""mjpeg_bridge — pull the real ESP32-CAM's WiFi MJPEG stream into ROS.

The ESP32-CAM serves the full image as multipart MJPEG over HTTP (too big for micro-ROS).
This node reads http://<board-ip>/stream and republishes each frame as a normal ROS image,
so camera_processor and aruco_detector (and behaviour 6) run on the real camera exactly as on
the Webots one.

  param:       stream_url   (e.g. http://10.65.205.246/stream)
  publishes:   /camera/image_raw   (sensor_msgs/Image, bgr8)
               /camera/camera_info (sensor_msgs/CameraInfo, width/height from the frame)

NOTE: cv2.VideoCapture hangs on the ESP32-CAM's multipart stream, so we parse it ourselves —
read chunks in a background thread and cut out each JPEG by its SOI (FFD8) / EOI (FFD9)
markers. Reconnects if the board drops.
"""

import threading
import time
import urllib.request

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

SOI = b'\xff\xd8'   # JPEG start-of-image
EOI = b'\xff\xd9'   # JPEG end-of-image
MAX_BUF = 300000    # drop the buffer if it grows past this without a full frame


class MjpegBridge(Node):

    def __init__(self):
        super().__init__('mjpeg_bridge')
        self.declare_parameter('stream_url', 'http://192.168.4.1/stream')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('rate', 15.0)
        self.url = self.get_parameter('stream_url').value
        self.frame_id = self.get_parameter('frame_id').value

        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        self.latest = None
        self.lock = threading.Lock()
        self.alive = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)
        self.get_logger().info(f'mjpeg_bridge up, streaming {self.url} -> /camera/image_raw')

    def _reader(self):
        while self.alive and rclpy.ok():
            try:
                stream = urllib.request.urlopen(self.url, timeout=5)
                buf = b''
                while self.alive and rclpy.ok():
                    chunk = stream.read(4096)
                    if not chunk:
                        break
                    buf += chunk
                    a = buf.find(SOI)
                    b = buf.find(EOI, a + 2) if a != -1 else -1
                    if a != -1 and b != -1:
                        jpg = buf[a:b + 2]
                        buf = buf[b + 2:]
                        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                        if img is not None:
                            with self.lock:
                                self.latest = img
                    elif len(buf) > MAX_BUF:
                        buf = buf[-2:]            # keep tail in case a marker straddles
            except Exception as e:
                self.get_logger().warn(f'stream error: {e}; reconnecting in 1s')
                time.sleep(1.0)

    def tick(self):
        with self.lock:
            img = None if self.latest is None else self.latest
            self.latest = None
        if img is None:
            return
        stamp = self.get_clock().now().to_msg()
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.img_pub.publish(msg)
        info = CameraInfo(width=img.shape[1], height=img.shape[0])
        info.header = msg.header
        self.info_pub.publish(info)

    def destroy_node(self):
        self.alive = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MjpegBridge()
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
