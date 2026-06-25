# Project 04 — Stretch (Hello Robot SE3) on ROS2 + MuJoCo

> Build & teaching plan. This project shows the **ROS2 architecture of the Stretch
> SE3**, basic control, then **Nav2** and **MoveIt2** tasks — all virtually in
> **MuJoCo**. Unlike `01_basics`, this project **uses launch files** (Nav2/MoveIt2
> require them); the checkpoint convention still applies via commentable blocks.

---

## 0 · Ground truth (verified 2026-06)

Official Hello Robot stack is MuJoCo-native on Humble:

- `hello-robot/stretch_ros2` (branch **humble**) — metapackage:
  - `stretch_core` — ROS2 drivers; motion modes, joint trajectory
  - `stretch_description` — URDF/meshes (source for the MoveIt2 config)
  - `stretch_nav2` — Nav2 + slam_toolbox + AMCL + Simple Commander
  - `stretch_simulation` — **MuJoCo** ROS2 sim, `stretch_mujoco_driver.launch.py`,
    modes `position | navigation | trajectory | gamepad`, D405+D435i cameras, 2D lidar
  - `hello_helpers`, `stretch_funmap`, `stretch_octomap`, `stretch_rtabmap`,
    `stretch_deep_perception`, `stretch_demos`
- `hello-robot/stretch_mujoco` — Python MuJoCo simulation library (dependency of sim)
- **MoveIt2 is NOT in the official humble stack** → we generate `stretch_moveit2`
  via MoveIt Setup Assistant from `stretch_description`.
  - **Reference to port from:** the `galactic` branch *does* have a `stretch_moveit2`
    package (Galactic, marked "unstable" — not installable on Humble as-is). Use it as
    a template: lift SRDF planning-group names, controller config, the
    `movegroup_moveit2.launch.py` structure, and the C++ `movegroup_test.cpp`
    MoveGroup-API demo (good basis for CP-D3); reconcile against a fresh Setup-Assistant
    pass on the humble `stretch_description`. (0.2 docs tutorial:
    docs.hello-robot.com/0.2/stretch-tutorials/ros2/moveit_basics/ — Galactic-era.)

### Decisions (confirmed with user)
- **Vendor** `stretch_ros2` + `stretch_mujoco` into `src/04_.../upstream/` so students
  can read/modify the real driver, URDF, and nav configs.
- **Generate a full MoveIt2 config** and demo arm/gripper planning + a pick task.

### Environment / feasibility (this machine)
- ROS2 Humble base only — none of the Stretch stack, nav2, moveit2, or `mujoco`
  python is installed yet → **Phase 0 provisions everything**.
- **No GPU** (no `nvidia-smi`/`glxinfo`), 12 cores, 14 GiB RAM, `DISPLAY=:0`.
  - Lidar `/scan` is computed, not rendered ⇒ **Nav2 runs fine on CPU**.
  - Camera RGB-D rendering + MuJoCo/RViz GUIs run on **software GL (llvmpipe)** → slow.
  - Defaults: `use_cameras:=false` where possible; `MUJOCO_GL=egl` for offscreen.
  - **Caveat documented for students:** smooth cameras/perception want an Nvidia GPU.

---

## 1 · Directory layout

```
src/04_stretch-hr-se3/
  PLAN.md                     # this file
  TUTORIAL.md                 # student walkthrough (checkpoint-driven)
  upstream/                   # VENDORED, not authored (git clones, built by colcon)
    stretch_ros2/             # -b humble
    stretch_mujoco/
  stretch_se3_bringup/        # ament_python — launch wrappers + RViz configs
  stretch_se3_control/        # ament_python — basic control demo nodes (checkpointed)
  stretch_se3_nav/            # nav2 params/maps + Simple Commander waypoint demo
  stretch_se3_moveit2/        # generated MoveIt2 config + programmatic pick demo
```

Authored packages stay thin: they wrap/teach the upstream stack, they don't fork it.

---

## 2 · Student-facing checkpoint flow

Four parts, each a sequence of **checkpoints** (`CP-x#`). Every checkpoint is a launch
block or node section students can comment out → rebuild → observe the change.

### Part A — Architecture (understand before driving)
- **CP-A1 Body & DOF map** — base (diff-drive), lift, telescoping arm (4 segs),
  dex wrist (yaw/pitch/roll = the "SE3" 3-DOF wrist), gripper, head pan/tilt,
  D435i head cam, D405 gripper cam, RPLidar. (doc + diagram, no code)
- **CP-A2 The ROS2 graph** — launch MuJoCo driver `mode:=position`; explore
  `ros2 node list`, `ros2 topic list`, `rqt_graph`.
- **CP-A3 URDF & TF** — `robot_state_publisher`, `tf2_tools view_frames`, RViz.
- **CP-A4 Sensors** — `/joint_states`, `/scan`, camera topics (cameras optional/slow).

### Part B — Basic control
- **CP-B1 Base** — `teleop_twist_keyboard` on `/cmd_vel`; then a scripted square-drive node.
- **CP-B2 Lift & arm** — joint position / trajectory commands (per-joint enable blocks).
- **CP-B3 Head** — pan/tilt to aim the camera.
- **CP-B4 Wrist + gripper** — dex wrist yaw/pitch/roll + grasp open/close.
- **CP-B5 Combined pose** — a single "stow → wake-up" demo node tying B1–B4 together.

### Part C — Nav2 (mapping + navigation) — strongest in MuJoCo on CPU
- **CP-C1 SLAM mapping** — `mode:=navigation` + slam_toolbox; drive, then save map.
- **CP-C2 Localization** — AMCL on the saved map; set initial pose in RViz.
- **CP-C3 Goals** — RViz Nav2 goal; then a Simple Commander Python waypoint script.
- **CP-C4 Costmaps & recovery** — inspect costmaps, tune one param, observe behaviour.

### Part D — MoveIt2 (manipulation planning)
- **CP-D1 Bring up move_group** — launch generated `stretch_moveit2` + RViz MotionPlanning.
- **CP-D2 Interactive planning** — drag the arm goal, Plan & Execute in RViz.
- **CP-D3 Programmatic** — Python node (move_group action / MoveItPy) for a reach.
- **CP-D4 (stretch) Nav + MoveIt** — drive to a surface, then reach/pick.

---

## 3 · Build phases (for the authors)

- **Phase 0 — Provision & smoke test**
  `pip install mujoco`; clone `stretch_ros2 -b humble` + `stretch_mujoco` into
  `upstream/`; `rosdep install`; `colcon build`; verify MuJoCo launches headless
  (`MUJOCO_GL=egl`) and `/scan` + `/joint_states` publish. Pin upstream commits.
- **Phase 1 — Part A** — `stretch_se3_bringup` (launch wrappers, RViz cfg) + TUTORIAL skeleton.
- **Phase 2 — Part B** — `stretch_se3_control` demo nodes with checkpoint markers.
- **Phase 3 — Part C** — `stretch_se3_nav` params/map + Simple Commander demo.
- **Phase 4 — Part D** — generate `stretch_moveit2` (Setup Assistant), porting SRDF
  groups / controllers / launch + C++ demo from the `galactic` reference; then pick demo.
- **Phase 5 — Polish** — TUTORIAL.md, troubleshooting (GPU/headless/llvmpipe),
  update CLAUDE.md package map, full checkpoint verification pass in sim.

## 4 · Open risks
- MuJoCo perf without a GPU (cameras/GUI). Mitigate: lidar-only Nav2, `use_cameras:=false`,
  `MUJOCO_GL=egl`, reduced viewer rate.
- MoveIt2 config quality from auto-generation (collision matrix, planning group naming)
  — mitigated by porting from the galactic `stretch_moveit2` reference.
- Upstream `humble` branch drift — pin commits in Phase 0.
