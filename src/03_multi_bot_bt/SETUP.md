# Project 03 — setup

## Provisioning (done 2026-06-25)
```bash
sudo apt-get install -y ros-humble-py-trees ros-humble-py-trees-ros \
                        ros-humble-py-trees-ros-interfaces
```
- Reuses Webots R2025a + `webots_ros2` + `cv_bridge` + `vision_msgs` from project 05.
- ArUco markers use **`DICT_4X4_250`** (needs id 99 for the world anchor; 4x4_50 only goes to 49).
- Self-contained: does **not** depend on or modify project 05.

## Build & run
```bash
cd ~/air26-ros2-ws && source /opt/ros/humble/setup.bash
colcon build --packages-select multibot_interfaces multibot_description \
  multibot_perception multibot_bt multibot_sim
source install/setup.bash

ros2 launch multibot_sim patrol.launch.py formation:=convoy      # column
# or
ros2 launch multibot_sim patrol.launch.py formation:=parallel    # abreast
```
Switch one unit's style live:
```bash
ros2 service call /r2/set_formation multibot_interfaces/srv/SetFormation "{formation: parallel}"
```

## Headless verification (no display)
```bash
xvfb-run -a -s "-screen 0 1280x1024x24" ros2 launch multibot_sim patrol.launch.py formation:=convoy
```
Verified 2026-06-25: `/r1 /r2 /r3` each publish `cmd_vel`, `ultrasonic/*`, `camera/image_raw`,
`aruco/detections`, `peers`; the marker chain resolves (r2 sees id 10 + anchor 99; r3 sees id
20); TF `rN/camera_optical_frame → rN/obs_marker_*` published; all 3 `patrol_bt` run; the
leader sweeps and turns at walls.

## How the pieces map
- **driver** (`multibot_driver`) is a Webots `<extern>` plugin that **derives its namespace
  from the Webots robot name** (`r1/r2/r3`) — no launch-side namespace juggling; each unit
  publishes `/rN/...` with the exact frame_ids perception expects.
- **markers**: unit `back`/`right` panels (id `unit*10+face`) + a fixed `world_anchor` (id 99,
  a vertical landmark on the +X wall — a forward camera sees it; a true floor marker would
  need a down-tilted camera).
- **localization is Tier-1** (marker pose → named peer positions + anchor; no EKF).

## Gotchas / notes
- Same Webots multi-robot pattern as the webots_ros2 multirobot example: one `WebotsController`
  per unit (`robot_name=rN`), all sharing the sim on `--port 1234`.
- `robot_state_publisher` per unit uses `frame_prefix: rN/` so TF frames are namespaced.
- Formation tuning lives in `multibot_bt/patrol_bt.py` (`follow_distance`, `lateral_gap`,
  `gap_side`, gains). Defaults are sane; tighten on a display.
- Tier-2 (EKF / world→odom from marker orientation) is a documented hook in
  `relative_localizer.py`, intentionally deferred.
