# Project 02 — micro_ros: obstacle-avoider bot (ROS part)

> Workshop day on **Sensors, messages, TF2, URDF, RViz** + a hands-on **obstacle
> avoider**. The robot will eventually run **micro-ROS on a bare-metal ESP32**; that
> **flashing/hardware is a SEPARATE plan**. This builds the **ROS side only**: a 3D
> robot you can explore in RViz, simulated in MuJoCo (Gazebo deferred), with the
> obstacle-avoidance behaviours.

## Why the sim stands in for the ESP32
The sim exposes the *exact* topics the ESP32 will later: it publishes the 3 ultrasonic
ranges and subscribes to `/cmd_vel`. So the behaviour code is identical — later you flash
the board, run the micro-ROS Agent, and the same behaviours drive the real robot.

```
sim (MuJoCo)  --/ultrasonic/{front,left,right} (sensor_msgs/Range)-->  behaviours
              <--/cmd_vel (geometry_msgs/Twist)-----------------------
              --/odom + TF (odom->base_link)-->  RViz (robot + range cones)
```

## The behaviours — a deliberate topic -> service -> action arc
Random walk = `rand(linear vel)` + `rand(small angular vel)`. On a close front obstacle:
- **B1** (pub-sub): stop, wait on a timer, resume.
- **B2** (pub-sub): stop, turn a random direction briefly, resume.
- **B3** (service + action): ask `/check_openings` (service: which side is open *right now*),
  then run `/escape_obstacle` (action: turn to the open side, or back-up + 180 if both
  blocked) with live **feedback** and **cancel**.
Switch live with the `/set_behavior` service. Switching away from B3 mid-escape **cancels**
the action — the concrete reason actions exist (sets up the later behaviour-trees session).

## The robot (URDF)
Cuboid chassis (~0.30×0.28×0.10, almost square), **4 driven wheels** (skid-steer), **3
ultrasonics** front/left/right, and labelled **motor / MCU / battery** cuboids. Distinct
colours. One xacro for RViz/Gazebo (`use_gazebo` arg) + an MJCF for MuJoCo.

## Packages (`src/02_micro_ros/`)
- `microbot_interfaces` (ament_cmake): `SetBehavior.srv`, `CheckOpenings.srv`,
  `EscapeObstacle.action`.
- `microbot_description` (ament_python): `urdf/microbot.urdf.xacro` (+ `microbot.gazebo.xacro`
  stub for the deferred Gazebo target), `mjcf/microbot.xml`, `rviz/microbot.rviz`.
- `microbot_sim` (ament_python): `mujoco_driver.py` (drive + 3 rangefinders + odom/TF),
  `scan_to_range.py` (Gazebo helper, for later), `launch/mujoco.launch.py`.
- `microbot_behaviors` (ament_python): `behavior_manager.py` (random walk + B1/B2 + B3
  client + `/set_behavior`), `obstacle_services.py` (`/check_openings` service +
  `/escape_obstacle` action), `launch/behaviors.launch.py`.

## Status
- **DONE & verified (MuJoCo, headless):** robot drives + senses; all three behaviours;
  the full topic→service→action chain (B3 integrated: obstacle → service → action →
  feedback → resume); `/set_behavior` switching.
- **DEFERRED:** the **Gazebo (Ignition) target** — `microbot.gazebo.xacro` (DiffDrive + 3
  gpu_lidar ultrasonics), an obstacle world, the ros_gz bridge, and `scan_to_range` wiring
  + `gazebo.launch.py`. The MuJoCo+RViz path is a complete day-02 demo on its own.
- **SEPARATE (hardware plan):** micro-ROS Agent + ESP32 flashing/transport.

## Notes
- MuJoCo base is a **kinematic planar joint** (driver maps Twist→base velocity); obstacles
  are made solid by a front-range gate + an arena position clamp (so it can't phase through
  or leave). Real skid-steer friction was intentionally avoided as overkill for teaching.
- Run long-running nodes as their own processes/launch (the usual one-sim-at-a-time hygiene).
