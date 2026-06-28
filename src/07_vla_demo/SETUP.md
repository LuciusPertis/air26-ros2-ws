# Project 07 — provisioning & setup (real SmolVLA SO-101 demo)

> The original no-GPU mini-VLA toy is archived under `_archive_mini_vla/` (its own setup notes
> live there). This file covers the **real SmolVLA** demo.

## 1. SmolVLA stack — isolated venv (done on the dev box 2026-06-25)
`torch` + `lerobot` are heavy and want `numpy>=2`, which breaks the workspace's pinned
`numpy 1.24` (ROS Humble / rclpy). So they go in a **dedicated venv** with
`--system-site-packages` (so it still sees `rclpy` + ROS messages); the system Python and the
other demos are untouched.
```bash
python3 -m venv --system-site-packages /home/lsp/vla_venv
source /home/lsp/vla_venv/bin/activate
pip install --upgrade pip wheel setuptools
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu  # no GPU here
pip install "lerobot[smolvla]"
pip install "numpy<2"      # re-pin for rclpy (ends at 1.26.4; rerun/opencv "want >=2" warnings are harmless)
deactivate
```
Sanity:
```bash
/home/lsp/vla_venv/bin/python -c "from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy; print('ok')"
```
- **lerobot 0.4.x** import path is `lerobot.policies.smolvla...` (not `lerobot.common...`).
- First policy load downloads `lerobot/smolvla_base` + `SmolVLM2-500M` backbone (~2 GB) to
  `~/.cache/huggingface`.

## 2. Robot model
SO-ARM100/101 is vendored from **MuJoCo Menagerie** (`trs_so_arm100`) into
`vla_so101_description/mjcf/` (+ `assets/*.stl`). The model's keyframe was removed (the
tabletop adds free-joint cubes → `nq` changes → a 6-dof keyframe is invalid); the home pose is
set in `mujoco_driver.py`.

## 3. Build & run
```bash
cd ~/air26-ros2-ws && source /opt/ros/humble/setup.bash
colcon build --packages-select vla_so101_description vla_so101_demo
source install/setup.bash
ros2 launch vla_so101_demo vla.launch.py instruction:='pick up the red cube'   # add use_rviz:=false for headless
```
One launch = MuJoCo (`mujoco_driver`) + SmolVLA (`smolvla_node`, venv) + `instruction_pub` +
RViz. The launch runs `smolvla_node` with the venv interpreter via `ExecuteProcess`
(`/home/lsp/vla_venv/bin/python`); everything else is plain ROS python.

## 4. Notes / gotchas
- **No GPU → CPU**: load ~20 s (cached; ~3 min cold), first inference ~25 s per 50-step chunk,
  then instant until the chunk drains → arm moves in bursts. Set `device:=cuda` if you ever
  have a GPU.
- SmolVLA wants `observation.state`(6) + **three** cameras (`camera1/2/3`, 3×256×256) +
  `action`(6); the driver's single front render is fed to all three slots.
- Language must be tokenised by the policy's preprocessor
  (`make_smolvla_pre_post_processors`) — `select_action` does not auto-tokenise. Base ships no
  stats → identity normalisation for state/action (VISUAL is IDENTITY).
- Benign `RTPS_TRANSPORT_SHM` warnings appear headless; comms still work.
- **It will not reliably grasp** — `smolvla_base` is a base model (not fine-tuned for this
  scene). See TUTORIAL §4.
