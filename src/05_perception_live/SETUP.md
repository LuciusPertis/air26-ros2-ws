# 05_perception_live — Setup (contributor guide)

Build, dependencies, and hardware/firmware flashing for the live demo. For the WiFi/routing
side — the part that actually decides whether the room works — see **`NETWORKING.md`**.

## 1. Dependencies

ROS 2 Humble + colcon (workspace already assumes this). Extra apt packages:

```bash
sudo apt install \
  ros-humble-robot-state-publisher ros-humble-xacro ros-humble-rviz2 \
  ros-humble-cv-bridge ros-humble-vision-msgs ros-humble-image-transport \
  ros-humble-tf2-ros ros-humble-teleop-twist-keyboard python3-opencv
```

For talking to the ESP32 boards you need the micro-ROS agent:

```bash
sudo apt install ros-humble-micro-ros-agent    # if unavailable, build micro-ROS agent from source
```

(Optional, for the robust classroom network path in NETWORKING.md §5–6:
`ros-humble-rmw-cyclonedds-cpp`, and the `fastdds` CLI from `ros-humble-fastrtps` for the
Discovery Server.)

## 2. Build

Only the two ROS packages build; `firmware/` is `COLCON_IGNORE`d.

```bash
cd ~/air26-ros2-ws
source /opt/ros/humble/setup.bash
colcon build --packages-select perceptlive_description perceptlive_perception
source install/setup.bash
```

Sanity check (no hardware needed) — `cmd_vel_odometry` should move a fake robot in RViz:
```bash
ros2 launch perceptlive_perception live.launch.py stream_url:=http://127.0.0.1/none rviz:=true
# in another terminal:
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.3}}'
# RViz: RobotModel drives in a circle, /odom trail appears. (mjpeg_bridge just retries the bad URL.)
```

Attendee laptops do **not** need to build this workspace — every topic uses standard messages,
so `source /opt/ros/humble/setup.bash` + `rviz2` is enough (details in NETWORKING.md §2).

## 3. Node / topic reference

| Node | Subscribes | Publishes |
|---|---|---|
| `mjpeg_bridge` | — (HTTP `stream_url`) | `/camera/image_raw`, `/camera/camera_info` |
| `camera_processor` | `/camera/image_raw` | `/camera/mean_intensity`, `/camera/mean_color` |
| `aruco_detector` | `/camera/image_raw` | `/aruco/detections`, `/aruco/image` |
| `cmd_vel_odometry` | `/cmd_vel` | `/odom`, TF `odom→base_link`, `/joint_states` |
| `robot_state_publisher` | `/joint_states` | `/robot_description`, TF `base_link→{wheels,camera}` |

`cmd_vel_odometry` params: `rate` (30 Hz), `cmd_timeout` (0.5 s), `wheel_radius` (0.05),
`wheel_base` (0.32 = 2×`wheel_dy`), `odom_frame`, `base_frame`. Restart the node to reset pose.

`live.launch.py` args: `stream_url`, `rviz` (default true), `agent` (default false → run the
micro-ROS agent yourself), `agent_port` (8888).

## 4. Firmware (vendored, self-contained)

Two **unmodified copies** live under `firmware/` so this project stands alone. We do **not**
touch or rebuild the originals in `02_micro_ros` / `05_perception`. Full flashing steps and
troubleshooting are in each firmware's own `README.md`; summary:

### `firmware/esp32_microbot` — the rover base ("esp-dev")
- ESP32 dev board, micro-ROS over WiFi/UDP. Subscribes `/cmd_vel` → L298N (4-wheel skid steer),
  publishes `/ultrasonic/{front,left,right}` (HC-SR04). **No encoders** → that's why
  `cmd_vel_odometry` exists.
- Edit `src/main.cpp` `AGENT_IP` (host laptop LAN IP) + WiFi SSID/pass before flashing.
- Build/flash with the official PlatformIO `penv` (micro-ROS needs it):
  `~/.platformio/penv/bin/pio run -t upload` as user `lsp`.

### `firmware/esp32cam_perception` — the front camera
- AI-Thinker ESP32-CAM. micro-ROS publishes `/camera/mean_intensity` + `/camera/mean_color`;
  serves the full frame as **multipart MJPEG at `http://<cam-ip>/stream`** (too big for
  micro-ROS) which `mjpeg_bridge` consumes.
- Also needs `AGENT_IP` + WiFi set. Flash via FTDI/USB-TTL or an ESP32-CAM-MB dock (no USB on
  the bare board). `upload_speed` pinned to 115200.

> Flashing gotchas we hit before (all in the firmware READMEs): remove `brltty` (hijacks CH340
> adapters on Ubuntu 22.04); ESP32-CAM brownouts mid-write → power its **5V** pin from the
> adapter's 5V; don't flash with `sudo` (re-roots `.pio`); `lsp` must be in `dialout`.

## 5. Environment note

Claude runs as `root`; the human is `lsp`. Newly created files under this project should be
`chown`ed to `lsp` and any PlatformIO builds run as `lsp` (toolchains live under `/home/lsp`).

```bash
sudo chown -R lsp:lsp ~/air26-ros2-ws/src/05_perception_live
```
