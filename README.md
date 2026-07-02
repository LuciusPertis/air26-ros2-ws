# AIR26 ROS 2 Student Workshop

A ROS 2 **Humble** workspace of self-contained robotics demos for a student workshop. Each
project under `src/` is one or more ROS 2 packages that compile and run independently, with
checkpoint-marked feature blocks students can toggle to see how the system changes.

> Working notes for contributors live in [`CLAUDE.md`](CLAUDE.md). Each project has its own
> `PLAN.md` / `TUTORIAL.md` / `SETUP.md` (and `THEORY.md` where relevant).

## Projects

| Dir | Topic | Highlights |
|-----|-------|------------|
| `01_basics` | ROS 2 fundamentals | 7 nodes in Python **and** C++ (topics/services/actions) + cross-language interop |
| `02_micro_ros` | sensors + an obstacle-avoider rover | skid-steer rover, 3 ultrasonics, behaviours escalating topic→service→action; MuJoCo + RViz; **ESP32 micro-ROS firmware** |
| `03_multi_bot_bt` | multi-robot + behaviour trees | 3 namespaced rovers patrolling under **py_trees** BTs; convoy/parallel formations; ArUco relative localization; Webots |
| `04_stretch-hr-se3` | Hello Robot Stretch (SE3) | vendored official stack + MuJoCo sim; Nav2 (SLAM/AMCL) + MoveIt2 demos |
| `05_perception` | cameras, ArUco, vision behaviours | rover + front **ESP32-CAM**; 3 sims (Webots/MuJoCo/Gazebo); light/colour/ArUco behaviours; MJPEG→ROS bridge |
| `06_llm-integration` | LLM ↔ ROS | local **Ollama** (qwen3) + native tool-calling → robot motion; drives 02 rover & 04 Stretch in MuJoCo; `backend:=mock` offline path |
| `07_vla_demo` | Vision-Language-Action | real **SmolVLA-450M** (lerobot) on a **SO-101** MuJoCo tabletop + RViz; text+camera+state → joint targets |

## Build & run

**Build the whole workspace in one shot** (rosdep + colcon, all of 01–07) — the fastest
way to confirm everything compiles on a fresh machine:

```bash
cd ~/air26-ros2-ws
./src/build_all.sh              # rosdep install + colcon build everything
source install/setup.bash
```

Or build just what you need:

```bash
source /opt/ros/humble/setup.bash
cd ~/air26-ros2-ws
colcon build --packages-select <pkg>...      # or a whole project's packages
source install/setup.bash
ros2 run <package> <node>                     # or ros2 launch <pkg> <file>.launch.py
```

📖 **[`src/INSTALL.md`](src/INSTALL.md)** is the full install guide — ROS deps (via rosdep)
**plus** every non-ROS runtime the demos need: MuJoCo, Webots, Gazebo, Ollama (project 06),
the SMOLVLA venv (project 07), and PlatformIO firmware (projects 02 & 05). Per-project detail
stays in each project's `SETUP.md`. Firmware lives under `src/<project>/firmware/` (outside
colcon, built with PlatformIO).

## Notes
- The `src/04_stretch-hr-se3/upstream/` Stretch code is **vendored** (pinned commits recorded
  in `04_stretch-hr-se3/SETUP.md`).
- Build artifacts (`build/ install/ log/`) and PlatformIO caches (`**/.pio/`) are gitignored;
  regenerate them with `colcon build` / `pio run`.

Licensed under Apache-2.0 (see `LICENSE`); vendored upstream retains its own licenses.
