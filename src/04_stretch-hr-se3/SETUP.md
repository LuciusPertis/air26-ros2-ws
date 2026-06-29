# Project 04 — Setup & Provisioning (Phase 0 — COMPLETE)

A complete, reproducible record of how the **Stretch (Hello Robot SE3) + MuJoCo +
Nav2** stack was provisioned for this workshop. If you are rebuilding this machine
from scratch, follow sections **1 → 5 in order** (order matters — pip before colcon,
the patch before the smoke test). Section 6 is verification, 7 is troubleshooting,
8 is performance/GPU notes, 9 is what was deliberately *not* installed.

---

## 0. Target system (verified facts)

| Item | Value |
|------|-------|
| OS | Ubuntu 24.04 LTS (Noble) |
| ROS 2 | Jazzy (`source /opt/ros/jazzy/setup.bash`) |
| Python | 3.12 |
| Compiler | gcc 13 |
| GPU | **none** (`nvidia-smi`/`glxinfo` absent) → software GL / EGL offscreen |
| CPU / RAM | 12 cores / 14 GiB |
| Display | `DISPLAY=:0` present (GUIs work, but software-rendered = slow) |
| Workspace | `/home/lsp/air26-ros2-ws` |

> The "no GPU" fact drives several decisions below: we render offscreen with
> `MUJOCO_GL=egl`, default cameras **off**, and skip the Robocasa kitchen scenes.

---

## 1. Vendored upstream (pinned commits)

Per the workshop's "students read/modify real code" decision, the official Hello
Robot repos are vendored **into the workspace** (not installed as opaque deps):

```
src/04_stretch-hr-se3/upstream/
  stretch_ros2/    # branch humble @ 73decc6adc45986744e36df2bac19fda7eb6aec8
  stretch_mujoco/  #               @ d107e094cc295d92f7461bd233daef96e9e22fab
```

```bash
cd /home/lsp/air26-ros2-ws/src/04_stretch-hr-se3
mkdir -p upstream && cd upstream
git clone --depth 1 -b jazzy https://github.com/hello-robot/stretch_ros2.git
git clone --depth 1            https://github.com/hello-robot/stretch_mujoco.git
```

`colcon` discovers every `package.xml` under `src/`, so these build automatically.
We only build the subset needed for the workshop (see §3).

---

## 2. Python dependencies (the fiddly part)

### 2a. Bootstrap pip + uv
`pip` was **not installed**, and `python3 -m ensurepip` is stripped on Ubuntu:
```bash
sudo apt-get install -y python3-pip          # gives pip 22.0.2
python3 -m pip install uv                     # uv is stretch_mujoco's build backend
```
`stretch_mujoco`'s `pyproject.toml` uses `build-backend = "uv_build"`, which plain
`pip install` **cannot** drive inside its build isolation (`RuntimeError: uv-build was
not properly installed`). Install it with `uv` directly instead.

### 2b. Editable install of the sim library
```bash
python3 -m uv pip install --system -e \
  /home/lsp/air26-ros2-ws/src/04_stretch-hr-se3/upstream/stretch_mujoco
```
Editable (`-e`) so students' edits to the vendored sim take effect without reinstall.

### 2c. Known-good pins  ⚠️ ROS 2 Jazzy expects numpy 1.x
`uv` and `opencv-python` will happily pull **numpy 2**, which breaks Jazzy's
`rclpy`/`cv_bridge`/`ros2_numpy`. Hold these versions:

```
numpy==1.26.4             # PIN — Jazzy/Ubuntu 24.04 ships & builds against numpy 1.26
opencv-python==4.10.0.84  # 4.13 hard-requires numpy>=2; 4.10 works with numpy 1.x
mujoco==3.2.6             # upstream pin (Robocasa compat); runs the 3.3.0 model fine
ros2-numpy==0.0.5         # from PyPI — there is NO apt pkg ros-jazzy-ros2-numpy
pyquaternion==0.9.9       # required by hello_helpers, not auto-pulled
termcolor==3.3.0          # required by robocasa_gen, not auto-pulled
hello-robot-stretch-mujoco==0.5.0   # editable, from upstream/ (§2b)
hello-robot-stretch-urdf==0.1.2     # generic URDF + meshes for sim
urchin==0.0.30  scipy==1.15.3  trimesh==4.12.2   # transitive, recorded for reference
```

One-shot install of the non-editable pins:
```bash
python3 -m pip install \
  "numpy==1.26.4" "opencv-python==4.10.0.84" "mujoco==3.2.6" \
  "ros2-numpy==0.0.5" "pyquaternion==0.9.9" "termcolor==3.3.0"
```
> `pip` will warn that opencv-python *wants* numpy≥2 — that warning is expected and
> safe; we intentionally keep numpy 1.24.2. If cameras misbehave later, this
> numpy/opencv pairing is the first suspect.

---

## 3. ROS dependencies + build

```bash
cd /home/lsp/air26-ros2-ws
source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null; rosdep update

U=src/04_stretch-hr-se3/upstream/stretch_ros2
rosdep install -y --ignore-src --from-paths \
  $U/hello_helpers $U/stretch_description $U/stretch_core $U/stretch_nav2 $U/stretch_simulation
```

apt packages rosdep pulled (verified versions on this machine):

| Package | Version |
|---------|---------|
| ros-jazzy-nav2-bringup | 1.1.20 |
| ros-jazzy-slam-toolbox | 2.6.10 |
| ros-jazzy-nav2-amcl | 1.1.20 |
| ros-jazzy-nav2-map-server | 1.1.20 |
| ros-jazzy-rplidar-ros | 2.1.4 |
| ros-jazzy-laser-filters | 2.0.9 |
| ros-jazzy-tf-transformations | 1.1.1 |
| ros-jazzy-realsense2-camera (+ description) | 4.57.7 |
| ros-jazzy-diagnostic-updater (+ aggregator) | 4.0.6 |
| ros-jazzy-joint-state-publisher (+ gui) | 2.4.0 |
| ros-jazzy-control-msgs, nav2-* deps, xterm, launch-pytest | (deps) |

Build the workshop subset:
```bash
colcon build --packages-up-to stretch_simulation stretch_nav2
colcon build --packages-select stretch_description   # NOT pulled above; needed for URDF/TF/MoveIt2
source install/setup.bash
```
> Deliberately **not** built (hardware-only / heavy, not needed for the workshop yet):
> `stretch_calibration`, `stretch_deep_perception`, `stretch_funmap`, `stretch_rtabmap`,
> `stretch_demos`. Add later if a module needs them.

> `stretch_description` ships only **xacro fragments** + the base-IMU xacro; the full
> calibrated `stretch.urdf` is normally generated on the real robot by
> `stretch_calibration`. For sim, the URDF comes from the `hello-robot-stretch-urdf`
> pip package / the driver's own `robot_state_publisher` (a Phase-1/Phase-4 detail).

---

## 3b. MoveIt2 (Phase 4)

No official Humble MoveIt2 config for Stretch, so we install MoveIt2 and hand-generate
`stretch_se3_moveit2` (no `moveit_py` package exists on Humble → the programmatic demo is
C++ `MoveGroupInterface`).

```bash
sudo apt-get install -y \
  ros-jazzy-moveit ros-jazzy-moveit-setup-assistant \
  ros-jazzy-moveit-simple-controller-manager ros-jazzy-moveit-planners-ompl \
  ros-jazzy-moveit-ros-move-group ros-jazzy-moveit-configs-utils \
  ros-jazzy-moveit-kinematics ros-jazzy-moveit-ros-visualization
# (ros-jazzy-moveit-py does NOT exist for Humble — do not include it)
colcon build --packages-select stretch_se3_moveit2
```

Config provenance + the gotchas actually hit (all encoded in `stretch_se3_moveit2/`):
- SRDF is generated by `scripts/generate_srdf.py` from the saved **galactic** dex SRDF
  (`scripts/galactic_dex_reference.srdf`), reconciled to our URDF: drop `joint_arm_l4`
  (fixed here), rename gripper body `link_straight_gripper`→`link_gripper_s3_body`, drop
  the custom-IK `mobile_base_arm`/`position` groups, fully-disable sensor/cosmetic links.
- **Robot name** in the SRDF must equal the URDF's (`<robot name="stretch">`), else
  "Semantic description is not specified for the same robot as the URDF".
- **Gripper-finger URDF limit** shipped degenerate `[0,0]`; widened to `[-0.4,0.4]`
  (AIR26 PATCH in `config/stretch_se3.urdf`) so the live start state is in-bounds.
- **Opposing gripper fingers** self-collide at the home pose → must be disabled
  (`EXTRA_DISABLE` in the generator), else every plan fails "start state in collision".
- **OMPL not CHOMP:** `config/ompl_planning.yaml` must be the flat moveit default (has
  `planning_plugin: ompl_interface/OMPLPlanner`); the galactic file's `move_group:`
  wrapper hides it and move_group silently falls back to CHOMP.
- **Execution:** MoveIt → `trajectory_bridge` (`/stretch_arm_controller/...`) → driver
  (`/stretch_controller/...`); the bridge sums `joint_arm_l0..l3`→`wrist_extension`. It
  forwards only the goal waypoint (sim is obstacle-free) and `allowed_start_tolerance:
  0.0` disables the start-deviation check (the sim arm settles near, not exactly on, each
  target). Verified: OMPL plans + arm executes named targets and a joint goal on the sim.

apt pulled (versions on this machine): `ros-jazzy-moveit*` **2.5.9**, plus `ompl 1.5.2`,
`ros-jazzy-moveit-resources-*`, `warehouse-ros`/`-mongo` deps.

---

## 4. Vendored source patch (intentional, marked)

**File:** `upstream/stretch_mujoco/stretch_mujoco/robocasa_gen.py`

The driver node (`stretch_mujoco_driver.py`) does
`from stretch_mujoco.robocasa_gen import (...)` at module top, and `robocasa_gen`
in turn did `import robosuite` / `from robocasa...` at module top. Result: the driver
**cannot even be imported** unless the (large, GPU-hungry, asset-downloading) Robocasa
stack is installed — even when you only want the plain scene.

Fix: wrapped those imports in `try/except` → `ROBOCASA_AVAILABLE` flag, bracketed by
`# === AIR26 WORKSHOP PATCH ===`. Full Robocasa behaviour is preserved *if* the
packages are present; otherwise the kitchen-only functions raise clearly when called,
and the plain-scene path (`use_robocasa:=false`) works. This is the one upstream edit
the plan anticipated ("some src changes will be needed").

---

## 5. Smoke test (PASSED)

The upstream `stretch_mujoco_driver.launch.py` calls Robocasa helpers at **parse
time** (to populate layout/style launch-arg choices), so it cannot run without
Robocasa. Until Phase 1 ships our own launch file, run the **node directly**:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export MUJOCO_GL=egl        # offscreen GL — no GPU/X needed
ros2 run stretch_simulation stretch_mujoco_driver --ros-args \
  -p use_robocasa:=false -p use_mujoco_viewer:=false -p use_cameras:=false
```
Startup ends with `The Mujoco Simulatior is connected.` (sic).

---

## 6. Verification checklist

In a second terminal (`source` both setups first):
```bash
ros2 node list                       # → /stretch_mujoco_driver (+ helpers)
ros2 topic list                      # see expected topics below
ros2 topic echo /joint_states --once # full DOF (below)
ros2 topic hz /scan_filtered         # ~ lidar rate; frame_id: laser, 0.2–20 m
ros2 topic echo /odom --once         # base odometry
ros2 run tf2_tools view_frames       # writes frames.pdf of the TF tree
```

Expected topics (plain scene, cameras off):
```
/battery /clock /cmd_vel /gamepad_joy /imu_mobile_base /imu_wrist
/is_homed /is_runstopped /is_streaming_position /joint_limits
/joint_pose_cmd /joint_states /magnetometer_mobile_base /mode
/odom /scan_filtered /stretch_gamepad_state /tf /tf_static /tool
```
> ⚠️ The lidar topic is **`/scan_filtered`**, not `/scan`. Nav2 configs must match.

### Stretch SE3 DOF (from `/joint_states`)
| Group | Joints / interface |
|-------|--------------------|
| Mobile base | diff-drive via `/cmd_vel`, odometry on `/odom` |
| Lift | `joint_lift` |
| Telescoping arm (4 segments) | `joint_arm_l0`, `l1`, `l2`, `l3` + aggregate `wrist_extension` |
| Head | `joint_head_pan`, `joint_head_tilt` |
| Dex wrist (the SE3 3-DOF wrist) | `joint_wrist_yaw`, `joint_wrist_pitch`, `joint_wrist_roll` |
| Gripper | `joint_gripper_finger_left`, `joint_gripper_finger_right` |

---

## 7. Troubleshooting (errors actually hit, in order)

| Symptom | Cause | Fix |
|---------|-------|-----|
| `pip: command not found` | pip not installed | `sudo apt-get install -y python3-pip` |
| `No module named ensurepip` | Ubuntu strips ensurepip | use apt (above), not ensurepip |
| `uv-build was not properly installed` (pip build) | `uv_build` backend needs the `uv` binary, absent in pip's build isolation | install with `uv pip install --system -e .` instead of `pip install` |
| `No module named 'robosuite'` at launch parse | upstream launch imports Robocasa at parse time | run the **node** directly (§5); Phase 1 adds a clean launch |
| `No module named 'robosuite'` on driver import | `robocasa_gen` imported it at module top | the §4 patch (try/except) |
| `No module named 'termcolor'` | not auto-pulled | `pip install termcolor` |
| `No module named 'ros2_numpy'` | no apt pkg `ros-jazzy-ros2-numpy` | `pip install ros2-numpy` (PyPI) |
| `No module named 'pyquaternion'` | not auto-pulled by hello_helpers | `pip install pyquaternion` |
| opencv wants `numpy>=2` / numpy 2 breaks rclpy | opencv 4.13 forces numpy 2 | pin `opencv-python==4.10.0.84` + `numpy==1.26.4` |
| MuJoCo GL/context errors, headless | no GPU/X for the GL backend | `export MUJOCO_GL=egl` (offscreen); for on-screen viewer use `MUJOCO_GL=glx` with a display |

---

## 8. Performance / GPU caveats (important for the workshop)

- **No GPU here.** MuJoCo physics is CPU-fine; *rendering* (cameras, viewer, RViz) uses
  software GL (llvmpipe) or EGL offscreen and is **slow**.
- **Lidar is computed, not rendered** → Nav2 (mapping + navigation) runs fine on CPU.
  This is why the workshop leans on Nav2 as the heavy demo.
- **Cameras default OFF** (`use_cameras:=false`). Enabling RGB-D rendering on CPU is
  laggy; do it only for short perception demos, ideally on a GPU machine.
- Use `MUJOCO_GL=egl` for headless/offscreen; `MUJOCO_GL=glx` only when showing the
  interactive MuJoCo viewer on a real display.
- Keep the MuJoCo viewer **off** (`use_mujoco_viewer:=false`) for Nav2/MoveIt2 runs to
  save cycles; RViz is the primary visualization.

---

## 9. Deliberately NOT installed (yet)

| Thing | Why skipped | When we'll revisit |
|-------|-------------|--------------------|
| Robocasa / robosuite kitchens | heavy, asset downloads, GPU-hungry; plain scene is enough | optional, GPU machines only |
| ~~MoveIt2 for Stretch~~ | ~~no official Humble config~~ | **DONE (Phase 4)** — installed + hand-generated `stretch_se3_moveit2`; see §3b |
| `stretch_calibration/_funmap/_rtabmap/_deep_perception/_demos` | hardware-only / heavy, not needed for current modules | as later modules require |

---

## 10. Teardown / restart

```bash
# stop a running sim: Ctrl-C the driver, or
pkill -f stretch_mujoco_driver

# clean rebuild
cd /home/lsp/air26-ros2-ws
rm -rf build install log
colcon build --packages-up-to stretch_simulation stretch_nav2
colcon build --packages-select stretch_description
```
Python pins live in the system site (installed via pip/uv); a workspace clean does
not touch them. Re-run §2c if the Python env is rebuilt.
