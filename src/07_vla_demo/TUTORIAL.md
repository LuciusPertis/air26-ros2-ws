# Project 07 — VLA demo: student walkthrough

Run a **real Vision-Language-Action model** (SmolVLA-450M) on a **SO-101 (SO-ARM100)** arm at
a tabletop in **MuJoCo**, with **RViz** showing what the model sees. You type an instruction;
the model turns camera + instruction + joint state into robot joint commands.

> The earlier no-GPU "mini-VLA" toy (scripted brain → 3-DOF arm) is archived under
> `_archive_mini_vla/`. This tutorial is the real thing.

## 0. One-time setup
SmolVLA needs `torch` + `lerobot` (heavy, and they clash with the workspace's pinned numpy),
so they live in an **isolated venv**. See `SETUP.md` for the full commands — in short:
```bash
python3 -m venv --system-site-packages ~/vla_venv
source ~/vla_venv/bin/activate
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install "lerobot[smolvla]"
pip install "numpy<2"
deactivate
```

## 1. Build & run (this is the whole demo — one launch)
```bash
cd ~/air26-ros2-ws && source /opt/ros/humble/setup.bash
colcon build --packages-select vla_so101_description vla_so101_demo
source install/setup.bash

ros2 launch vla_so101_demo vla.launch.py instruction:='pick up the red cube'
```
That single launch starts **everything**: MuJoCo (the SO-101 tabletop, via `mujoco_driver`),
the **SmolVLA** policy (in the venv), the instruction publisher, and **RViz** (showing the
front camera = the model's eye). There is no separate "mujoco launch" — MuJoCo runs *inside*
`mujoco_driver`. Headless? add `use_rviz:=false`.

> **First run downloads ~2 GB** (`smolvla_base` + its SmolVLM2 backbone) to `~/.cache/huggingface`.

## 2. What you'll see
- A MuJoCo window: the SO-101 arm over a table with three cubes (red / green / blue).
- RViz: the front-camera image the model receives.
- The arm **reacts to the instruction** and moves.

**CPU timing (no GPU here):** the model loads (~20 s) then thinks for **~25 s** to produce a
50-step action chunk, executes it (~10 s of motion), then thinks again. So expect **motion in
bursts with ~25 s pauses**. A GPU makes it real-time.

## 3. Change the instruction live
```bash
ros2 topic pub /instruction std_msgs/String "{data: 'stack the blue cube on the red cube'}"
ros2 topic pub /instruction std_msgs/String "{data: 'move the gripper to the green cube'}"
```
Watch `/joint_command` (the model's output) and `/joint_states` (the arm following it):
```bash
ros2 topic echo /joint_command      # 6 SO-101 joint targets from SmolVLA
ros2 topic echo /joint_states       # the arm's actual joints
```

## 4. ⚠️ Important: what works and what doesn't
This uses **`smolvla_base`**, a *generalist base checkpoint*. It runs fully end-to-end — real
text + vision + state → SmolVLA → SO-101 joint targets → MuJoCo + RViz, and the arm moves from
your instructions. But it is **not fine-tuned for this scene**, so it **won't reliably grasp or
stack** — the camera view, objects, and action calibration don't match its training data.
That's expected: this demo shows a *real VLA properly plumbed through ROS 2*. To get actual
manipulation you'd fine-tune SmolVLA on demonstrations of this exact setup (a separate,
GPU-heavy effort).

## 5. Make it yours
- Edit the scene (`vla_so101_description/mjcf/tabletop_scene.xml`): move/recolor cubes, add
  more, change the camera.
- Tune rates in `vla_so101_demo/.../smolvla_node.py` (`inference_rate`) / `mujoco_driver.py`.
- Swap `checkpoint:=<a fine-tuned SO-101 SmolVLA>` on the launch to try a task-trained model.
