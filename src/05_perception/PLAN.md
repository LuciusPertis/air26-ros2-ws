# Project 05 — Perception: plan

## Goal
Take the project-02 obstacle-avoider rover, **add a front camera**, and grow the behaviour
set from "react to ultrasonics" to "react to vision". Same robot, same `/cmd_vel`,
progressively richer senses — the workshop's "modular additive embodiments" idea. Target
simulator is **Webots**; Gazebo/MuJoCo are reference only (see `THEORY.md`).

## The one new sensor, three new topics
The rover gains an ESP32-CAM at the front. From its image we publish:
- `/camera/image_raw` (`sensor_msgs/Image`) — the full picture (sim camera / WiFi MJPEG on HW).
- `/camera/mean_intensity` (`std_msgs/Float32`) — one brightness scalar.
- `/camera/mean_color` (`std_msgs/ColorRGBA`) — one average colour.
The two scalars are cheap enough for the real ESP32 to compute on-board and send over
micro-ROS; the image stays on WiFi/HTTP. Plus `/aruco/detections` (`vision_msgs/Detection2DArray`).

## Six switchable behaviours (one `/set_behavior` service)
| # | Name | Senses | Mechanism |
|---|------|--------|-----------|
| 1 | obstacle: stop + timer | `/ultrasonic/front` | pub-sub | (from 02) |
| 2 | obstacle: stop + random turn | ultrasonics | pub-sub | (from 02) |
| 3 | obstacle: service + escape action | ultrasonics | `/check_openings` + `/escape_obstacle` | (from 02) |
| 4 | light-seek | `/camera/mean_intensity` | scalar phototaxis |
| 5 | colour-seek | `/camera/mean_color` | scalar chromotaxis |
| 6 | ArUco search + approach | `/aruco/detections` | `/approach_marker` action |

Escalation mirrors 02: scalar reactions (B4/B5, like the cheap ESP32 topics) → full detection
+ a waypoint-like **action** (B6) that motivates localisation. B4/B5/B6 keep a front-stop
safety overlay.

## Packages (`src/05_perception/`)
- `perceptbot_interfaces` — `SetBehavior` (1-6), `CheckOpenings`, `EscapeObstacle` (from 02) +
  new `ApproachMarker` action.
- `perceptbot_description` — the 02 skid-steer xacro **+ a front camera link** (RViz/TF).
- `perceptbot_perception` — `camera_processor` (image → mean_intensity/mean_color),
  `aruco_detector` (image → detections + overlay), `mjpeg_bridge` (real ESP32-CAM WiFi MJPEG
  → `/camera/image_raw`). Sim-agnostic; same nodes run on sim or the real cam's stream.
- `perceptbot_behaviors` — `behavior_manager` (B1-B6 dispatcher), `obstacle_services`
  (B3, from 02), `marker_approach` (B6 ApproachMarker server).
- `perceptbot_sim` — **three embodiments**, all exposing the same interface:
  - **Webots** (primary): `.wbt` world (arena + light panel + green box + ArUco marker +
    pillars), device URDF, `/cmd_vel` skid-steer driver plugin, `webots.launch.py`.
  - **MuJoCo** (reference): `mujoco_driver` (02 base + offscreen camera render), `mujoco.launch.py`.
  - **Gazebo Fortress** (reference): `perceptbot.sdf` (DiffDrive + 3 gpu_lidars + camera),
    `gz_bridge.yaml`, `scan_to_range` (LaserScan→Range), `gazebo.launch.py`.
  - **ArUco (B6) is Webots-sim + real-cam only**; MuJoCo/Gazebo cover B1-5 (MuJoCo can't map a
    flat marker decal onto a box face; kept simple/consistent across both reference sims).
- `firmware/esp32cam_perception` — real ESP32-CAM: mean_intensity + mean_color over micro-ROS,
  full image over WiFi MJPEG. Same interface as the sim camera → behaviours unchanged.

## Checkpoint blocks (comment out → feature disappears)
`camera` (description), `mean_intensity` / `mean_color` (camera_processor),
`aruco_detect` / `aruco_overlay` (aruco_detector), `behavior_1..6` (behavior_manager),
`service` / `action` (obstacle_services).

## Status (2026-06-25)
- **Built + verified headless:** all 5 ROS packages build; vision pipeline proven on a
  synthetic frame (intensity/colour correct, ArUco id detected); behaviours up and `/set_behavior`
  switches 1-6.
- **All three sims run headless (xvfb / MUJOCO_GL=egl):**
  - Webots — sensors + camera + ArUco publish; `/cmd_vel` drives (front range 1.85→0.65).
  - MuJoCo — ranges + offscreen camera @14 Hz + mean topics; B1-5.
  - Gazebo — SDF valid; lidars→`scan_to_range`→`/ultrasonic/*`, camera @17 Hz + mean topics; B1-5.
- **Firmware:** `esp32cam_perception` compiles (micro-ROS + camera + MJPEG, Flash 29.7%).
  `mjpeg_bridge` turns its WiFi stream into `/camera/image_raw` for ArUco on real HW.
- **For the user's display:** GUI runs of each sim, RViz, and the real-cam end-to-end.
