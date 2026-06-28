# SmolVLA SO-101 tabletop demo (the "proper" VLA)

The real **SmolVLA-450M** policy (text + camera + joint state → robot actions) driving a
**SO-101 (SO-ARM100)** arm on a tabletop in **MuJoCo**, with **RViz** showing the camera feed.
This is the heavyweight cousin of the toy `vla_demo` — it runs an actual pretrained VLA, not a
scripted policy.

## Architecture (why two interpreters)
```
 /instruction (text) ─┐
 /camera/front  ──────┼──►  smolvla_node  ──► /joint_command ──►  mujoco_driver ──► SO-101 (MuJoCo)
 /joint_states ───────┘     (venv: torch+lerobot)                 (system: mujoco)      │
                                                                   /camera/front, /joint_states
                                                                          └────► RViz (camera feed)
```
SmolVLA needs `torch` + `lerobot`, which conflict with the workspace's pinned `numpy 1.24`
(ROS Humble). So they live in an **isolated venv** (`/home/lsp/vla_venv`, `--system-site-packages`
so it still sees `rclpy`); the `smolvla_node` is launched with that interpreter, everything
else is plain ROS python. The system env (and the other demos) stay untouched.

## Setup (one-time, done on the dev box)
```bash
python3 -m venv --system-site-packages ~/vla_venv
source ~/vla_venv/bin/activate
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install "lerobot[smolvla]"
pip install "numpy<2"          # re-pin so rclpy still works in the venv
```
- **No GPU here** → CPU inference (slow; see timing in TUTORIAL). `numpy` ends at 1.26.4
  (<2 for rclpy; the rerun/opencv "want numpy>=2" warnings are harmless).
- SO-ARM100 model is vendored from **MuJoCo Menagerie** (`trs_so_arm100`) in
  `vla_so101_description/mjcf/` (its keyframe was removed — the tabletop adds free-joint cubes
  that change `nq`; the home pose is set in `mujoco_driver.py`).
- First launch downloads `lerobot/smolvla_base` + its `SmolVLM2-500M` backbone (~2 GB) to
  `~/.cache/huggingface`.

## Run
```bash
cd ~/air26-ros2-ws && source /opt/ros/humble/setup.bash
colcon build --packages-select vla_so101_description vla_so101_demo
source install/setup.bash
ros2 launch vla_so101_demo vla.launch.py instruction:='pick up the red cube'
# change the task any time:
ros2 topic pub /instruction std_msgs/String "{data: 'stack the blue cube on the red cube'}"
```

## ⚠️ What this is (and isn't)
`smolvla_base` is a **generalist base checkpoint meant to be fine-tuned**. Here it runs fully
end-to-end — real text + vision + state → SmolVLA → SO-101 joint targets → MuJoCo + RViz — and
the **arm moves in response to instructions**. But it is **not fine-tuned for this scene**, so
it will **not reliably grasp/stack**: the camera viewpoint, objects, and action calibration
don't match its training data. Outputs are clamped to the SO-101 joint limits for safety. To
get real manipulation you'd fine-tune SmolVLA on demonstrations for this exact setup (separate,
heavier effort). This demo is the honest "real VLA, properly plumbed through ROS 2" baseline.

## Files
- `mujoco_driver` — MuJoCo SO-101 tabletop; publishes `/camera/front/image_raw` + `/joint_states`,
  applies `/joint_command`. Plain ROS python.
- `smolvla_node` — loads SmolVLA (config-driven: reads the checkpoint's declared image/state
  keys), builds the observation, publishes 6 joint targets. Venv python.
- `instruction_pub` — re-publishes the task string. `vla.launch.py` — brings it all up + RViz.
