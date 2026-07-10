# Project 05 — setup

## Provisioning (done 2026-06-24)
```bash
# Webots R2025a + the ROS-side perception deps
sudo apt-get install -y ros-humble-webots-ros2 ros-humble-cv-bridge \
                        ros-humble-vision-msgs ros-humble-image-transport \
                        ros-humble-compressed-image-transport
wget https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb
sudo apt-get install -y ./webots_2025a_amd64.deb     # ~2 GB, installs to /usr/local/webots
```
- `webots_ros2` is **2025.0.0**, which pairs with **Webots R2025a** (versions must match).
- `webots` binary: `/usr/local/webots/webots`; set `WEBOTS_HOME=/usr/local/webots` if a launch
  can't find it.
- `cv2.aruco` comes from the system `python3-opencv` (4.10) — no extra install.

## Build & run (Webots demo)
```bash
cd ~/air26-ros2-ws
source /opt/ros/humble/setup.bash
colcon build --packages-select perceptbot_interfaces perceptbot_description \
  perceptbot_perception perceptbot_behaviors perceptbot_sim
source install/setup.bash

# terminal 1 — pick ONE embodiment (all expose the same interface):
ros2 launch perceptbot_sim webots.launch.py            # Webots (primary) — add use_rviz:=true
ros2 launch perceptbot_sim mujoco.launch.py            # MuJoCo  (reference; B1-5)
ros2 launch perceptbot_sim gazebo.launch.py            # Gazebo Fortress (reference; B1-5)
# terminal 2 — the behaviours (same for every embodiment)
ros2 launch perceptbot_behaviors behaviors.launch.py
# switch behaviour live (1-3 obstacle, 4 light, 5 colour, 6 ArUco [Webots/real-cam])
ros2 service call /set_behavior perceptbot_interfaces/srv/SetBehavior "{behavior: 6}"
```

### Real ESP32-CAM (no simulator)
```bash
# Agent + behaviours as for project 02; then bridge the board's MJPEG stream into ROS:
ros2 launch perceptbot_perception real_camera.launch.py \
     stream_url:=http://<board-ip>/stream
```
The board publishes `/camera/light_level` + `/camera/mean_color` itself over micro-ROS;
`mjpeg_bridge` adds `/camera/image_raw` (+ `camera_info`) so `aruco_detector` and B6 work too.

### ArUco scope
ArUco (B6) works in **Webots** (ImageTexture renders the marker crisply) and on the **real
cam**. **MuJoCo and Gazebo cover B1-5** — MuJoCo's 2d/cube texture projection can't map a flat
marker decal onto a box face cleanly, so the marker is omitted there for consistency.

## Headless verification (no display)
Webots needs a GL context. On a headless box, `xvfb` works (camera renders under software GL):
```bash
sudo apt-get install -y xvfb
xvfb-run -a -s "-screen 0 1280x1024x24" ros2 launch perceptbot_sim webots.launch.py
```
Verified headless 2026-06-25:
- **Webots** (`xvfb-run … webots.launch.py`): all `/ultrasonic/*`, `/camera/*`,
  `/aruco/detections` publish; `/cmd_vel` drives (front range 1.85→0.65 m at the wall).
- **MuJoCo** (`mujoco.launch.py mujoco_gl:=egl` — set `MUJOCO_GL=egl` for offscreen camera):
  ranges + camera @14 Hz + mean topics.
- **Gazebo** (`xvfb-run … gazebo.launch.py gz_args:='-r -s …'`): SDF valid; lidars→`/ultrasonic/*`,
  camera @17 Hz + mean topics.

## Gotchas hit (and fixed)
- **`robot_description` launch error** ("Unable to parse … as yaml") — wrap the xacro
  `Command(...)` in `ParameterValue(..., value_type=str)` for `robot_state_publisher`
  (done in `webots.launch.py`).
- **Webots `<extern>` controller** — the robot's `controller "<extern>"` lets `WebotsController`
  attach; `WebotsLauncher(ros2_supervisor=True)` provides `/clock`. Add `webots._supervisor`
  to the launch description.
- **Firmware micro-ROS build needs the official PlatformIO `penv`.** `pip install platformio`
  does NOT create `~/.platformio/penv`, so `micro_ros_platformio` fails with
  `cannot open .platformio/penv/bin/activate`. Install via the official `get-platformio.py`
  (per-user). `esp32cam_health` builds without it (no micro-ROS); `esp32cam_perception` needs it.
- Same WiFi/flashing caveats as project 02 firmware (2.4 GHz only; brltty grabs CH340; 5 V).

## Reference simulators (Gazebo / MuJoCo)
The demo is **Webots only**. Gazebo (Ignition, installed from project 07) and MuJoCo (3.2.6)
are kept for the architecture comparison in `THEORY.md`; no Gazebo/MuJoCo world is built for
this rover.
