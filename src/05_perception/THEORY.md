# Project 05 — Perception: theory

The background for the perception day. The demo is **Webots only**; Gazebo and MuJoCo appear
here for comparison so students understand *why* each simulator exists.

---

## 1. Simulators: architecture & features

All three are physics + sensor simulators that can talk to ROS 2, but they were built with
different priorities.

| | **Webots** (our target) | **Gazebo** (Sim / Ignition) | **MuJoCo** |
|---|---|---|---|
| Origin | Cyberbotics, education/robotics | OSRF, the ROS ecosystem | DeepMind, RL / contact dynamics |
| World format | `.wbt` (VRML-like tree) | SDF (XML) | MJCF (XML) |
| ROS 2 link | `webots_ros2` driver (device plugins) | `ros_gz` bridge | none official (you write a node) |
| Sensors built-in | camera, lidar, range, IMU, GPS, … rich | camera, lidar, IMU, … rich | minimal; you compute from state |
| Strengths | batteries-included sensors, easy setup, stable | deepest ROS integration, big model DB | fastest, best contacts, RL/MPC |
| Weakness | smaller community than Gazebo | heavier setup, version churn | no native sensors/ROS |

**Mental model:**
- **Webots** = a self-contained robot lab. The world tree *contains* the robot and its
  devices; `webots_ros2` attaches ROS publishers to those devices via small `<device>` tags.
  Great when you want working sensors immediately (this project).
- **Gazebo** = the ROS-native simulator. Everything is a plugin; you wire topics through the
  `ros_gz` bridge. Most ROS tutorials/robots ship Gazebo models (project 02/04 reference it).
- **MuJoCo** = a physics engine first. Blazing fast and superb at contact, but you publish
  ROS messages yourself from the sim state (that's exactly what project 02's `mujoco_driver`
  and project 07 do). Pick it for RL and contact-rich manipulation.

**How our Webots demo is wired** (`perceptbot_sim`): a `.wbt` world holds the robot
(wheels + 3 distance sensors + camera); a tiny URDF declares `<device>` tags that map those
to `/ultrasonic/*` and `/camera/*`; one Python plugin turns `/cmd_vel` into wheel speeds.
The behaviours never know it's Webots — same topics as the MuJoCo bot and the real ESP32.

---

## 2. Camera messages in ROS 2

A camera produces more than pixels. The standard `sensor_msgs` types:

- **`sensor_msgs/Image`** — raw pixels + `encoding` (e.g. `rgb8`, `bgra8`, `mono8`) +
  `width`/`height`/`step`. Big (a 320×240 rgb8 frame is ~230 KB).
- **`sensor_msgs/CompressedImage`** — JPEG/PNG bytes; what you send over a network/WiFi.
- **`sensor_msgs/CameraInfo`** — the intrinsics (focal length, principal point, distortion)
  needed to relate pixels to angles/metres. Published alongside the image.
- **`image_transport`** — a helper that offers `raw` and `compressed` flavours of a topic
  transparently, so subscribers pick what they can afford.

**Project 05's twist — derived topics.** A whole image is expensive and a behaviour rarely
needs every pixel. So we also publish *reductions*:
- `/camera/mean_intensity` (`std_msgs/Float32`) — one brightness number.
- `/camera/mean_color` (`std_msgs/ColorRGBA`) — one average colour.
These are cheap enough to compute **on the ESP32 itself** and send over micro-ROS, while the
full `Image` stays on WiFi/HTTP. That split (cheap scalars on the MCU, heavy pixels off-board)
is the core lesson: *match the message to the channel and the consumer.*

---

## 3. ArUco markers

An **ArUco marker** is a black-and-white square barcode (a fiducial). A dictionary (e.g.
`4x4_50` = 50 markers of a 4×4 bit grid) defines the valid patterns. OpenCV's `cv2.aruco`:
1. thresholds the image and finds square contours,
2. reads the bit grid, error-checks it against the dictionary → a marker **id**,
3. returns the four **corner pixels** (and, with `CameraInfo`, a full 6-DOF **pose**).

Why fiducials matter: they give a robot a **cheap, unambiguous, uniquely-identified landmark**
— perfect for "go to marker 3", docking, or ground-truth. They're the bridge from raw vision
to a *goal* a behaviour can chase.

In this project `aruco_detector` publishes detections as **`vision_msgs/Detection2DArray`**
(bbox + id), the standard ROS perception type, so it plugs into the wider ecosystem. B6 then
steers from pixels only (bbox centre → bearing, bbox area → range proxy) — no calibration
needed. With a real `CameraInfo` you'd instead recover metric pose (`estimatePoseSingleMarkers`).

---

## 4. PointCloud2 vs LaserScan

Two ways ROS represents range/depth data:

- **`sensor_msgs/LaserScan`** — a **2-D** slice: ranges at evenly-spaced angles in one plane
  (`angle_min`, `angle_increment`, `ranges[]`). Compact, ideal for a spinning 2-D lidar and
  for 2-D SLAM/Nav2 (project 04). Implicit geometry: index → angle.
- **`sensor_msgs/PointCloud2`** — a generic, **3-D** bag of points (x,y,z [+ rgb, intensity…])
  described by a `fields` layout. What 3-D lidars / depth cameras / stereo produce. Flexible
  and dense, but heavier and unordered.

Rule of thumb: **LaserScan** when your data is one horizontal ring (2-D nav); **PointCloud2**
when it's genuinely 3-D (depth camera, 3-D lidar, or fused multi-sensor). A depth camera can
publish both — a full cloud, plus a flattened scan for a 2-D navigation stack. Our rover uses
neither directly (it has discrete ultrasonics → `Range`), but the contrast frames *why* sensor
choice drives message choice — the same theme as the camera section.
