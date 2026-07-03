# 05_perception_live — real-hardware, many-viewers live demo

A **self-contained** spin-off of `05_perception`, focused on one scenario:

> A room full of attendees join the local WiFi and, on their own laptops, watch the **real
> rover's camera feed and its motion in RViz** — live.

It is deliberately isolated so the base `05_perception` workshop stays untouched. This project
**copies** the packages it needs and **carries its own copies of both firmwares**; it does not
build, modify, or depend on the `02_micro_ros` or `05_perception` trees.

## How it differs from `05_perception`

| | `05_perception` | `05_perception_live` (this) |
|---|---|---|
| Goal | teach 6 perception behaviours in sim | show the **real** robot to many viewers in RViz |
| Sims | Webots / MuJoCo / Gazebo | **none** — real hardware only |
| Odometry | provided by the sim driver | **`cmd_vel_odometry`** dead-reckons `/cmd_vel` (real firmware has no encoders) |
| Viewers | one operator | **N attendees**, each running only `rviz2` over the LAN |
| Firmware | referenced in-place | **vendored copies** under `firmware/` (see below) |

The key gap this project fills: the rover's ESP32 firmware only *drives* on `/cmd_vel` and has
**no wheel encoders**, so nothing publishes a pose — without `odom → base_link` the robot can't
move in RViz. `cmd_vel_odometry` integrates the *commanded* velocity (open-loop dead reckoning;
it drifts, it is not localization) so the robot visibly moves. No firmware change needed.

## Layout

```
05_perception_live/
  perceptlive_description/     # rover URDF (02 base + front ESP32-CAM) for RViz/robot_state_publisher
  perceptlive_perception/      # nodes + launch + rviz
    perceptlive_perception/
      mjpeg_bridge.py          # ESP32-CAM HTTP MJPEG  → /camera/image_raw
      camera_processor.py      # image → /camera/mean_intensity, /camera/mean_color
      aruco_detector.py        # image → /aruco/detections (+ /aruco/image overlay)
      cmd_vel_odometry.py      # NEW: /cmd_vel → /odom + TF odom→base_link + /joint_states
    launch/live.launch.py      # the whole PC-side graph (+ optional RViz, optional agent)
    launch/real_camera.launch.py  # (inherited) camera-only bring-up
    rviz/live.rviz             # RobotModel + TF + Odometry + Camera image
  firmware/                    # COLCON_IGNORE'd; vendored, self-contained copies
    esp32cam_perception/       # the front camera (micro-ROS mean_* + HTTP MJPEG stream)
    esp32_microbot/            # the "esp-dev" rover base (micro-ROS: /cmd_vel → L298N, /ultrasonic/*)
  README.md  SETUP.md  NETWORKING.md
```

> The two firmwares are **unmodified copies** of the ones flashed for `05_perception` (camera)
> and `02_micro_ros` (rover base). We keep them here so this project is standalone; we do not
> touch or rebuild the originals. Their own `README.md` files hold the flashing instructions.

## ROS graph (on hardware)

```
ESP32-CAM  ──HTTP──►  mjpeg_bridge ──► /camera/image_raw ──► camera_processor ─► /camera/mean_*
                                                        └──► aruco_detector   ─► /aruco/detections, /aruco/image
ESP32 rover ─micro-ROS─► /ultrasonic/front|left|right
teleop ─► /cmd_vel ─► (micro-ROS agent → rover motors)
                   └─► cmd_vel_odometry ─► /odom, TF odom→base_link, /joint_states
robot_state_publisher ─► /robot_description, TF base_link→{wheels,camera}
                        every attendee's rviz2 subscribes to all of the above
```

## Quick start (host laptop)

```bash
cd ~/air26-ros2-ws
colcon build --packages-select perceptlive_description perceptlive_perception
source install/setup.bash

# 1) micro-ROS agent for the boards (or pass agent:=true to the launch below)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# 2) the live stack + RViz  (point stream_url at YOUR ESP32-CAM)
ros2 launch perceptlive_perception live.launch.py \
    stream_url:=http://10.42.0.51/stream rviz:=true

# 3) drive it (movement shows up in RViz via cmd_vel_odometry)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Each **attendee** (same LAN, same `ROS_DOMAIN_ID`) then just runs RViz — see **`NETWORKING.md`**,
which is the important document here: it covers the WiFi/router requirements, DDS discovery,
the "one HTTP client" limit, unicast/Discovery-Server fallbacks when classroom multicast is
flaky, and the image-bandwidth (compressed transport) problem.

## Docs
- **`NETWORKING.md`** — WiFi routing / DDS / multi-viewer (read this).
- **`SETUP.md`** — build, dependencies, firmware flashing pointers, hardware notes.
