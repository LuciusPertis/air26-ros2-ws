# Project 05 — Perception: student walkthrough

You already built a rover that dodges walls with ultrasonics (project 02). Today it grows an
**eye** (a front camera) and learns to chase light, colour, and a marker. Same robot, same
`/cmd_vel` — just richer senses. Read `THEORY.md` first for the simulator/camera/ArUco
background.

## 0. Build & launch
```bash
cd ~/air26-ros2-ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select perceptbot_interfaces perceptbot_description \
  perceptbot_perception perceptbot_behaviors perceptbot_sim
source install/setup.bash

ros2 launch perceptbot_sim webots.launch.py        # Webots opens: arena + rover
ros2 launch perceptbot_behaviors behaviors.launch.py   # (second terminal) the brain
```
Webots shows the arena: a **white light panel**, a **green box**, an **ArUco marker** on the
far wall, and two pillars.

> **Three robots, one brain.** The same demo runs in **MuJoCo** (`mujoco.launch.py`) and
> **Gazebo** (`gazebo.launch.py`) — they publish the identical `/cmd_vel` + `/ultrasonic/*` +
> `/camera/*` topics, so the behaviours don't change. (ArUco/B6 is a Webots + real-cam feature;
> MuJoCo/Gazebo do behaviours 1-5.) That swap-the-body, keep-the-brain idea is the whole point.

## 1. See the new senses
```bash
ros2 topic list | grep camera
ros2 topic echo /camera/mean_intensity      # one brightness number, 0..1
ros2 topic echo /camera/mean_color          # average colour (ColorRGBA)
ros2 topic echo /aruco/detections           # ArUco bbox + id when a marker is in view
```
In RViz (`use_rviz:=true`) add an **Image** display on `/camera/image_raw` (the view) and
`/aruco/image` (with marker outlines drawn).

## 2. The six behaviours
Switch any time with one service call:
```bash
ros2 service call /set_behavior perceptbot_interfaces/srv/SetBehavior "{behavior: N}"
```
- **1-3 (from project 02): obstacle avoidance** — random walk; on a close front wall it
  (1) waits, (2) turns randomly, or (3) asks `/check_openings` then runs the cancelable
  `/escape_obstacle` action. Watch the action feedback in the behaviours terminal.
- **4 light-seek** — spins to search; when the view gets bright enough (`mean_intensity`
  high) it drives forward → it homes toward the white light panel.
- **5 colour-seek** — spins to search; when the average colour matches the target
  (`target_color`, green by default) it drives in → it homes toward the green box.
- **6 ArUco search + approach** — spins until it *sees marker 0*, then launches the
  `/approach_marker` **action**: it centres the marker (bearing → turn) and drives until the
  marker looks big enough (area → range), then stops. Like a one-shot waypoint.

## 3. The "why an action?" moment (same lesson as 02)
While in behaviour 6 and the rover is approaching the marker, switch away:
```bash
ros2 service call /set_behavior perceptbot_interfaces/srv/SetBehavior "{behavior: 1}"
```
The approach is **cancelled** mid-motion — a topic or service couldn't do that. Actions are
for long, watchable, cancelable tasks (approach, escape); services for instant questions
(`/check_openings`); topics for streams (`/cmd_vel`, the camera scalars).

## 4. Make it yours (checkpoints)
Every feature is wrapped in `# === CHECKPOINT: <name> ===`. Comment a block, rebuild, relaunch
— the feature vanishes; restore it and it's back.
- `camera` in the xacro → remove the camera link.
- `mean_color` in `camera_processor.py` → `/camera/mean_color` stops; behaviour 5 goes blind.
- `behavior_4`/`_5`/`_6` in `behavior_manager.py` → drop a vision behaviour.
- Tune params: `intensity_threshold`, `target_color`, `target_marker`, `search_turn`, … (see
  `behavior_manager.py`). Try `target_color:=[0.8,0.1,0.1]` and add a red box.

## 5. On real hardware
The Webots camera and the **ESP32-CAM** publish the *same* `/camera/mean_intensity` and
`/camera/mean_color`, so behaviours 4 and 5 run on the real board with no code change — the
sim just swaps for `micro_ros_agent` + the board. For the full image (and ArUco/B6), bridge
the board's WiFi MJPEG stream into ROS:
```bash
ros2 launch perceptbot_perception real_camera.launch.py stream_url:=http://<board-ip>/stream
```
That gives `/camera/image_raw`, so `aruco_detector` and behaviour 6 work on the real camera
too. See `firmware/esp32cam_perception/README.md`.
