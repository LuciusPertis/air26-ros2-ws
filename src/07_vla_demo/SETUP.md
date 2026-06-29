# Project 07 — provisioning & reproducible setup

One-time machine setup for the VLA demo. Build/run is plain colcon after this.

## 0. Target system
- ROS2 Jazzy, no GPU. MuJoCo python (3.2.6) + xacro/rviz already present (from project 04).
- **Gazebo + ros2_control were NOT installed** → §1 provisions them.
- No `torch`/`transformers` → the "VLA" is the lightweight `ScriptedPolicy`, not a real
  model (a real one like SmolVLA-450M is GB-scale, slow on CPU, and trained for a specific
  arm). The `Policy` interface lets a real model drop in later — see
  `vla_demo/policies/smolvla_adapter.py`.

## 1. Install Gazebo Harmonic + ros2_control
```bash
sudo apt-get install -y \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```
Versions pulled on this machine (verify against your Jazzy/Harmonic install):
| Package | Version |
|---------|---------|
| ros-jazzy-ros-gz (ros_gz_sim/bridge) | (Jazzy) |
| Gazebo Harmonic (gz-sim8 / `gz sim`) | 8.x (**Harmonic**, binary `gz sim`) |
| ros-jazzy-gz-ros2-control | (Jazzy) |
| ros-jazzy-ros2-control | (Jazzy) |
| ros-jazzy-ros2-controllers | (Jazzy) |

> Jazzy pairs with **Gazebo Harmonic** (the modern `gz sim`, not the old `ign gazebo`).
> The binary is `gz sim`; `ros_gz_sim`'s `gz_sim.launch.py` wraps it (arg `gz_args`).

## 2. Build
```bash
cd ~/air26-ros2-ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select vla_arm_description vla_demo
source install/setup.bash
```

## 3. Smoke test (all verified headless)
```bash
# RViz pipeline (no physics)
ros2 launch vla_demo rviz.launch.py use_rviz:=false
ros2 topic pub --once /instruction std_msgs/String "{data: up}"
ros2 topic echo /delta_theta            # the star topic streams
# MuJoCo
MUJOCO_GL=egl ros2 launch vla_demo mujoco.launch.py use_viewer:=false
# Gazebo (headless server)
ros2 launch vla_demo gazebo.launch.py gz_args:='-r -s -v1 empty.sdf'
ros2 control list_controllers           # joint_state_broadcaster + position_controller active
```

## 4. Gotchas actually hit (and the fixes, baked into the repo)
| Symptom | Cause | Fix |
|---------|-------|-----|
| `xacro: not well-formed (invalid token)` | `--` inside an XML comment (ASCII-art arrows) | XML comments can't contain `--`; reworded |
| gz plugin not found / wrong system | Harmonic uses the **GazeboSim** names (not the old Ignition ones) | use `gz_ros2_control/GazeboSimSystem` + `gz_ros2_control::GazeboSimROS2ControlPlugin`, filename `gz_ros2_control-system` (ships in `libgz_ros2_control-system.so`) |
| `UnboundLocalError: 'mujoco'` | `import mujoco.viewer` inside a method shadows the module name | `from mujoco import viewer as mj_viewer` |
| `home` overshoots in Gazebo (joint races past 0) | brain read **measured** `/joint_states` (laggy under physics) → unstable closed loop | brain reads **commanded** `/joint_command` instead → stable |
| `A message was lost!!!` on `ros2 topic echo /joint_states` | echo QoS vs publisher QoS mismatch | cosmetic (echo-side only); pipeline unaffected |

## 5. Performance notes (no GPU)
- Physics (MuJoCo, Gazebo) is fine on CPU — the scene is one tiny arm.
- Gazebo **GUI** uses software GL (llvmpipe) and is slow to open; the control path is
  verified with the headless server (`-s`). The GUI is for the kids' display.
- MuJoCo: `MUJOCO_GL=egl` for offscreen, or `use_viewer:=true` on a real display.
