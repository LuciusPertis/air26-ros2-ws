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
| `04_stretch-hr-se3` | Hello Robot Stretch (SE3) | vendored official stack + MuJoCo sim; Nav2 (SLAM/AMCL) + MoveIt2 demos |
| `05_perception` | cameras, ArUco, vision behaviours | rover + front **ESP32-CAM**; 3 sims (Webots/MuJoCo/Gazebo); light/colour/ArUco behaviours; MJPEG→ROS bridge |
| `06_llm-integration` | LLM ↔ ROS | (early) |
| `07_vla_demo` | "Δθ through ROS 2" | pluggable mini-VLA → `/delta_theta` → 3-DOF arm in RViz/Gazebo/MuJoCo |

## Build & run

```bash
source /opt/ros/humble/setup.bash
cd ~/air26-ros2-ws
colcon build --packages-select <pkg>...      # or a whole project's packages
source install/setup.bash
ros2 run <package> <node>                     # or ros2 launch <pkg> <file>.launch.py
```

Per-project setup (extra apt/pip deps, sim provisioning, firmware flashing) is in each
project's `SETUP.md`. Firmware lives under `src/<project>/firmware/` (outside colcon, built
with PlatformIO).

## Notes
- The `src/04_stretch-hr-se3/upstream/` Stretch code is **vendored** (pinned commits recorded
  in `04_stretch-hr-se3/SETUP.md`).
- Build artifacts (`build/ install/ log/`) and PlatformIO caches (`**/.pio/`) are gitignored;
  regenerate them with `colcon build` / `pio run`.

Licensed under Apache-2.0 (see `LICENSE`); vendored upstream retains its own licenses.
