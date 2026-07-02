# AIR26 ROS2 Student Workshop

## Workspace Overview

ROS2 Humble workspace for a student workshop. All projects are modular and self-contained so students can enable/disable/remove individual features and observe how the system changes.

**Working directory:** `/home/lsp/air26-ros2-ws`
**ROS2 distro:** Humble (`source /opt/ros/humble/setup.bash`)
**Build system:** colcon (`colcon build --packages-select <pkg>`)

## Workshop Structure

Each project lives under `src/` as one or more ROS2 packages. Projects are numbered; each compiles and runs independently.

```
src/
  01_basics/
    basics_py/      # ament_python  — 7 nodes in Python
    basics_cpp/     # ament_cmake   — same 7 nodes in C++
    basics_cross/   # ament_cmake   — cross-language (Py↔C++) interop
    TUTORIAL.md
  02_turtlesim/     # (planned)
  ...
```

## Design Rules

1. **No launch files** in the basics projects — students run nodes with `ros2 run`.
2. **Modular feature blocks** — each node has clearly marked sections students can comment out to disable a feature.
3. **Checkpoint markers** wrap every major feature block:
   - Python: `# === CHECKPOINT: <name> ===` / `# === END CHECKPOINT: <name> ===`
   - C++:    `// === CHECKPOINT: <name> ===` / `// === END CHECKPOINT: <name> ===`
4. **Minimal dependencies** — only `rclpy`/`rclcpp` and standard ROS2 message packages.
5. **One concept per file** where possible; combined node is a separate file.

## Checkpoint Convention

Comment out a block, rebuild, run — feature disappears. Restore and rebuild — it returns.

```python
# === CHECKPOINT: topics ===
self.pub = self.create_publisher(...)
# === END CHECKPOINT: topics ===
```

## Build & Run

```bash
source /opt/ros/humble/setup.bash
cd ~/air26-ros2-ws
colcon build --packages-select basics_py basics_cpp basics_cross
source install/setup.bash
ros2 run <package_name> <node_name>
```

## Package Map

| Project | Package | Lang | Nodes |
|---------|---------|------|-------|
| 01_basics | `basics_py` | Python | topic_talker/listener, service_server/client, action_server/client, combined_node |
| 01_basics | `basics_cpp` | C++ | same 7 nodes |
| 01_basics | `basics_cross` | Py+C++ | py_talker→cpp_listener, cpp_service_server→py_client, py_action_server→cpp_client. **Self-contained interfaces:** defines its own `msg/Greeting`, `srv/AddTwoInts`, `action/CountUp` in-package (rosidl) — depends on nothing but rcl (no example_interfaces/std_msgs). Python nodes installed as scripts (no `ament_python_install_package`, so rosidl owns the `basics_cross` py module) |
| 02_micro_ros | `microbot_interfaces` | msgs | `SetBehavior.srv`, `CheckOpenings.srv`, `EscapeObstacle.action` |
| 02_micro_ros | `microbot_description` | Python | skid-steer rover URDF (chassis, 4 wheels, 3 ultrasonics, motor/MCU/battery) + MJCF + RViz; `use_gazebo` arg (Gazebo target deferred) |
| 02_micro_ros | `microbot_sim` | Python | `mujoco_driver` (drive + 3 rangefinder ultrasonics + odom/TF), `scan_to_range` (Gazebo helper), `mujoco.launch.py` |
| 02_micro_ros | `microbot_behaviors` | Python | `behavior_manager` (random walk + B1/B2 pub-sub + B3 client + `/set_behavior`), `obstacle_services` (`/check_openings` srv + `/escape_obstacle` action) |
| 04_stretch-hr-se3 | `upstream/stretch_ros2`, `upstream/stretch_mujoco` | vendored | official Hello Robot SE3 stack (driver, nav2, description) + MuJoCo sim |
| 04_stretch-hr-se3 | `stretch_se3_bringup` | Python | `sim.launch.py` (clean MuJoCo bringup, plain scene) + SE3 URDF/meshes + RViz |
| 04_stretch-hr-se3 | `stretch_se3_control` | Python | Part B control demos: square_drive, lift_arm, head_scan, wrist_gripper, wake_up (checkpoint-marked) |
| 04_stretch-hr-se3 | `stretch_se3_nav` | Python | Part C Nav2: `mapping.launch.py` (slam_toolbox) + `navigation.launch.py` (AMCL+Nav2), sim-tuned params, cmd_vel_relay, waypoint_demo |
| 04_stretch-hr-se3 | `stretch_se3_moveit2` | C++/Py | Part D MoveIt2: hand-generated config (SRDF via `scripts/generate_srdf.py`), `trajectory_bridge` (l0-l3→wrist_extension), `move_group.launch.py`, C++ `reach_demo` |
| 06_llm-integration | `llm_integration` | Python | **local Ollama LLM (qwen3:1.7b default, qwen3:4b opt-in) + native tool-calling** → robot motion (MuJoCo, sim-only). Robot-agnostic core (`ollama_agent`, `prompt`, `robot_base`) + adapters `robots/micro.py` (02 rover: `/cmd_vel` raw drive + discrete move/turn/stop) and `robots/stretch.py` (04 Stretch: base drive + `set_lift/arm/head/gripper` via FollowJointTrajectory + mode-switch). One-shot `/llm/command`; warm-ups `chat_terminal`+`ollama_api_demo`; `backend:=mock` offline path; `micro.launch.py`/`stretch.launch.py` include the 02/04 sims |
| 07_vla_demo | `vla_so101_description` | Python | SO-ARM100/101 (vendored MuJoCo Menagerie `trs_so_arm100`) tabletop scene: arm + table + 3 graspable cubes + front/top cameras |
| 07_vla_demo | `vla_so101_demo` | Python | **real SmolVLA-450M** demo: `mujoco_driver` (SO-101 tabletop, sys python), `smolvla_node` (lerobot SmolVLA, **venv** python), `instruction_pub`, RViz, `vla.launch.py` |
| 07_vla_demo | _archived_ `_archive_mini_vla/` | Python | original no-GPU mini-VLA toy (`vla_demo` + `vla_arm_description`); `COLCON_IGNORE`'d, kept for reference |
| 05_perception | `perceptbot_interfaces` | msgs | `SetBehavior.srv` (1-6), `CheckOpenings.srv`, `EscapeObstacle.action` (from 02) + `ApproachMarker.action` |
| 05_perception | `perceptbot_description` | Python | project-02 skid-steer base + front camera link (`perceptbot.urdf.xacro`) for RViz/TF |
| 05_perception | `perceptbot_perception` | Python | `camera_processor` (image→`/camera/mean_intensity`+`mean_color`), `aruco_detector` (image→`vision_msgs/Detection2DArray`+overlay), `mjpeg_bridge` (ESP32-CAM WiFi MJPEG→`/camera/image_raw`) |
| 05_perception | `perceptbot_behaviors` | Python | `behavior_manager` (B1-3 obstacle + B4 light + B5 colour + B6 ArUco), `obstacle_services` (B3), `marker_approach` (B6 action) |
| 05_perception | `perceptbot_sim` | Python | 3 embodiments: **Webots** (`.wbt`+device URDF+driver plugin, `webots.launch.py`), **MuJoCo** (`mujoco_driver` w/ offscreen camera, `mujoco.launch.py`), **Gazebo** (`perceptbot.sdf`+`gz_bridge.yaml`+`scan_to_range`, `gazebo.launch.py`) |
| 03_multi_bot_bt | `multibot_interfaces` | msgs | `SetFormation.srv` (convoy\|parallel) |
| 03_multi_bot_bt | `multibot_description` | Python | perceptbot variant + back/right ArUco marker links; Webots `patrol.wbt` (3 namespaced units r1/r2/r3 + world-anchor marker, `DICT_4X4_250`) |
| 03_multi_bot_bt | `multibot_perception` | Python | `aruco_pose_detector` (solvePnP marker pose + TF), `relative_localizer` (named peer positions + world anchor; Tier-1, EKF deferred) |
| 03_multi_bot_bt | `multibot_bt` | Python | `patrol_bt` (**py_trees**: Selector safety/follower/leader; convoy=US+ArUco fused; parallel=vel-match+side-US handoff), `formation_anchor` (leader velocity broadcaster) |
| 03_multi_bot_bt | `multibot_sim` | Python | self-namespacing Webots driver plugin (`multibot_driver`, ns from robot name) + `patrol.launch.py` (world + 3× per-unit stack, `formation:=convoy\|parallel`) |

> **03 notes:** the project-05 rover ×3, namespaced (`/r1 /r2 /r3`), patrolling under a per-unit
> **py_trees Behaviour Tree**; **self-contained** (does NOT modify 05). Each unit wears ArUco
> markers on **back (id `N0`) + right (id `N1`)** + a fixed **world anchor (id 99)**. Two styles
> via `SetFormation`/launch arg: **convoy** (column; emergent leader; back-marker follow with
> **fused US+ArUco** distance) and **parallel** (abreast; **velocity-match + side-US lateral
> hold**, ArUco bootstraps). BT (py_trees_ros, live-viewable via `py-trees-tree-watcher`):
> `Selector[safety→Avoid, patrol(InPatrolMode→[follow,recover,Lead]), RandomWalk]`. Modes:
> **random** (wander+avoid baseline), convoy, parallel. Lost unit runs **SearchAndRecover**
> (vision-only 360 spin; `enable_recovery` toggles it); **leader-vs-lost anchor-based**
> (`anchor_range`; nearest=leader; auto-promote after turnaround). Tutorial phases
> random→patrol(no recovery)→patrol(+recovery). **Tier-1** localization (EKF deferred).
> Theory: FSM vs BT, Nav2 BT touch, DDS discovery/namespacing. Docs in
> `src/03_multi_bot_bt/{THEORY,PLAN,TUTORIAL,SETUP}.md`. Run: `ros2 launch multibot_sim
> patrol.launch.py formation:=convoy|parallel`.

> **04 notes:** uses **launch files** (Nav2/MoveIt2 require them) — checkpoint blocks
> live in launch files/nodes. Provisioning, pins, and the one vendored patch are in
> `src/04_stretch-hr-se3/SETUP.md`; build/checkpoint plan in `PLAN.md`; student
> walkthrough in `TUTORIAL.md`. Run: `ros2 launch stretch_se3_bringup sim.launch.py`.

> **02 notes:** ROS-side only (ESP32 micro-ROS flashing is a SEPARATE hardware plan). One
> rover, MuJoCo sim (Gazebo target deferred) + RViz. Behaviours escalate topic→service→action
> (B1/B2 pub-sub; B3 = `/check_openings` service + `/escape_obstacle` action, switchable via
> `/set_behavior`). Docs in `src/02_micro_ros/{PLAN,TUTORIAL,SETUP}.md`. Run: `ros2 launch
> microbot_sim mujoco.launch.py` + `ros2 launch microbot_behaviors behaviors.launch.py`.

> **05 notes:** the project-02 rover **+ a front ESP32-CAM**; "same base, richer senses".
> **Three embodiments** (like 07): **Webots** (R2025a, primary), **MuJoCo** and **Gazebo**
> (Fortress) — all expose the same `/cmd_vel` + `/ultrasonic/*` + `/camera/*` interface, so
> behaviours don't change. 3 camera topics: `/camera/{image_raw,mean_intensity,mean_color}`
> (+ `/aruco/detections`). Six behaviours via `/set_behavior`: 1-3 obstacle (from 02), 4
> light-seek, 5 colour-seek, 6 ArUco search+approach (`/approach_marker` action). **ArUco (B6)
> is Webots-sim + real-cam only** — MuJoCo/Gazebo do B1-5 (flat marker decals don't render in
> MuJoCo). Real `firmware/esp32cam_perception` mirrors the cheap topics over micro-ROS + image
> over WiFi MJPEG; `mjpeg_bridge` pulls that stream into ROS for ArUco. Docs in
> `src/05_perception/{THEORY,PLAN,TUTORIAL,SETUP}.md`. Run: `ros2 launch perceptbot_sim
> {webots,mujoco,gazebo}.launch.py` + `ros2 launch perceptbot_behaviors behaviors.launch.py`.

> **06 notes:** **sim-only (MuJoCo)** LLM→motion. A local **Ollama** instruct model
> (default **`qwen3:1.7b`**, fast on CPU ~2-3 s; `qwen3:4b` via `model:=` = more
> accurate but slow) with **native tool-calling** turns ONE natural-language command on
> `/llm/command` into movement tool calls; the model also returns a natural-language
> **reply** published on `/llm/response`, and may call a **`respond` (no-action)**
> tool to answer questions about state without moving. Adds **no new robot** —
> reuses 02 (rover, base-only `/cmd_vel`) and 04 (Stretch, base + lift/arm/head/
> gripper via FollowJointTrajectory + per-tool mode-switch). One package
> `llm_integration`: robot-agnostic `ollama_agent`/`prompt`/`robot_base` + thin
> `robots/{micro,stretch}.py` adapters; runs under a MultiThreadedExecutor +
> ReentrantCallbackGroup (blocking agent + action futures coexist). Launches
> **include** the 02/04 sims (`micro.launch.py`/`stretch.launch.py`). Offline
> `backend:=mock` (keyword matcher) exercises the full ROS path with no model/GPU;
> warm-ups `chat_terminal` + `ollama_api_demo`. CPU tuning: `think=False` +
> `temperature=0`; agent recovers tool-calls a small model prints as JSON text and
> de-duplicates repeated calls. **Verified:** mock path (rover `/cmd_vel`+US/odom
> state; Stretch lift 0.59→0.94 m) AND **real Ollama** (`qwen3:1.7b`) on the rover —
> movement → `turn`+`move_forward` w/ NL reply on `/llm/response`; question →
> `respond` clean answer, no motion (~2-3 s warm). Ollama installed here (v0.30.11
> systemd; qwen3:1.7b+4b pulled). Harmless upstream `left_wheel_vel` driver error
> per joint goal (pre-existing in 04). Docs in
> `src/06_llm-integration/{PLAN,SETUP,TUTORIAL}.md` + `llm_integration/README.md`.

> **07 notes:** the **real VLA** demo — `vla_so101_demo` runs **SmolVLA-450M** (lerobot) on a
> **SO-101 (SO-ARM100)** MuJoCo tabletop with graspable cubes + **RViz** (camera feed):
> text+camera+state → 6 joint targets. SmolVLA lives in an isolated venv (`/home/lsp/vla_venv`;
> torch/lerobot) to protect the numpy-1.24 pin; **CPU inference ~25 s/chunk** (bursts). Base
> `smolvla_base` (not fine-tuned → moves but won't reliably grasp; honest plumbing demo). **One
> launch** = MuJoCo+SmolVLA+RViz (no separate mujoco launch): `ros2 launch vla_so101_demo
> vla.launch.py instruction:='pick up the red cube'`. Docs:
> `src/07_vla_demo/{TUTORIAL,SETUP,PLAN}.md` + `vla_so101_demo/README.md`.
> The original no-GPU mini-VLA toy (Δθ pipeline, 3 sims) is **archived** under
> `src/07_vla_demo/_archive_mini_vla/` (`COLCON_IGNORE`).

## Adding New Projects

- Create under `src/<NN>_<topic>/`
- Python: `ros2 pkg create --build-type ament_python`
- C++: `ros2 pkg create --build-type ament_cmake`
- Follow checkpoint convention and update the package map above
