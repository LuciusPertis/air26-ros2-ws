# Project 07 — VLA demo: plan

Goal: run a **real Vision-Language-Action model** end-to-end through ROS 2 — text instruction +
camera + joint state → robot joint actions — on a realistic arm, so students see what a VLA
actually is (not a toy stand-in).

## Current demo (real SmolVLA SO-101)
- **Model:** SmolVLA-450M (`lerobot/smolvla_base`) — text + 3 camera images + state → 6 joint
  actions. Runs on CPU here (slow, in bursts); GPU would be real-time.
- **Robot:** SO-101 / SO-ARM100 (vendored MuJoCo Menagerie `trs_so_arm100`), 6 position joints.
- **Scene:** tabletop with three graspable/stackable cubes + a front camera (the model's eye).
- **Sim:** MuJoCo (one sim, run inside `mujoco_driver`). **RViz** shows the camera feed.

### Packages
- `vla_so101_description` — SO-ARM100/101 model + `tabletop_scene.xml` (arm + table + cubes +
  front/top cameras).
- `vla_so101_demo` — `mujoco_driver` (sim; publishes `/camera/front/image_raw` + `/joint_states`,
  applies `/joint_command`), `smolvla_node` (real SmolVLA, runs in the venv), `instruction_pub`,
  `vla.launch.py`, RViz config.

### Architecture
```
/instruction (text) ─┐
/camera/front ───────┼─► smolvla_node ─► /joint_command ─► mujoco_driver ─► SO-101 (MuJoCo)
/joint_states ───────┘   (venv: torch+lerobot)             (system: mujoco)   └─► /camera, /joint_states ─► RViz
```
SmolVLA's torch+lerobot live in an isolated venv (`/home/lsp/vla_venv`) to protect the
workspace's numpy-1.24 pin; `smolvla_node` is launched with that interpreter, everything else
is plain ROS python.

### Run
`ros2 launch vla_so101_demo vla.launch.py instruction:='pick up the red cube'` — see
`TUTORIAL.md` / `SETUP.md` / `vla_so101_demo/README.md`.

### Scope / honesty
`smolvla_base` is a **base** checkpoint: it runs end-to-end and moves the arm from real
text+vision, but is **not fine-tuned for this scene → won't reliably grasp**. Real
manipulation = fine-tune on demonstrations (deferred, GPU-heavy). This is the honest "real VLA
plumbed through ROS 2" baseline. Status: built + verified headless (arm moves from an
instruction); GUI/RViz visual run is for the display.

## Archived: the original mini-VLA (toy)
The first version — a no-GPU, no-torch scripted/pluggable "mini-VLA" → `/delta_theta` → a toy
3-DOF arm in RViz/Gazebo/MuJoCo — is preserved under `_archive_mini_vla/` (`COLCON_IGNORE`).
It's a nice "what is Δθ / swap-the-brain" lesson with zero heavy deps; move it back up to
`src/07_vla_demo/` to revive it.
