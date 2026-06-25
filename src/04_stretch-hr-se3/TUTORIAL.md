# Tutorial 04 — Stretch (Hello Robot SE3) on ROS2 + MuJoCo

A virtual Stretch mobile manipulator, simulated in **MuJoCo**, driven by the official
Hello Robot ROS2 stack. You will explore its architecture, control every joint, then
run **Nav2** and **MoveIt2** tasks — no physical robot required.

> **One-time setup of this machine** is already done and documented in
> [`SETUP.md`](SETUP.md). If the sim won't start, read SETUP.md §7 (troubleshooting).

> **Build first** — compile the workshop packages from a clean tree (do this once,
> and again after editing a node/launch or commenting out a checkpoint). The vendored
> upstream stack (driver, Nav2, etc.) and MoveIt2 are already provisioned per
> [`SETUP.md`](SETUP.md); this builds just the authored workshop packages:
> ```bash
> cd ~/air26-ros2-ws
> source /opt/ros/humble/setup.bash
> colcon build --packages-select \
>   stretch_se3_bringup stretch_se3_control stretch_se3_nav stretch_se3_moveit2
> source install/setup.bash
> ```

> **Setup — run in every new terminal before anything else:**
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/air26-ros2-ws/install/setup.bash
> export MUJOCO_GL=egl        # offscreen GL — this machine has no GPU
> ```

The tutorial is in four parts:

| Part | Topic | Checkpoints |
|------|-------|-------------|
| **A** | Architecture — how the ROS2 system is wired | CP-A1 … CP-A4 |
| **B** | Basic control — drive the base, move every joint | CP-B1 … CP-B5 |
| **C** | Nav2 — mapping & autonomous navigation | CP-C1 … CP-C4 |
| **D** | MoveIt2 — arm motion planning | CP-D1 … CP-D4 |

> **Checkpoint idea:** every feature lives in a block you can comment out, re-run, and
> watch disappear. In this project the blocks are in **launch files** and nodes (marked
> `# === CHECKPOINT: <name> ===`), because Nav2/MoveIt2 need launch files — unlike
> `01_basics` where you used `ros2 run` directly.

---

# Part A · Architecture

Goal: understand what the Stretch ROS2 system *is* before you command it — the body,
the running nodes, the topics, the TF tree, and the sensors.

## A.0 · Bring up the simulator

```bash
ros2 launch stretch_se3_bringup sim.launch.py
```
This starts three things (see `stretch_se3_bringup/launch/sim.launch.py`):
1. `stretch_mujoco_driver` — the MuJoCo physics sim + ROS2 driver,
2. `robot_state_publisher` + `joint_state_publisher` — turn joint angles + the URDF
   into the TF tree and the visual model,
3. `rviz2` — visualization.

Wait for the line `The Mujoco Simulatior is connected.` RViz opens with the robot.

> No display, or RViz too slow? Run headless:
> ```bash
> ros2 launch stretch_se3_bringup sim.launch.py use_rviz:=false
> ```
> Everything below except the RViz steps still works.

---

## CP-A1 · The body & degrees of freedom

Stretch SE3 has these movable parts. Keep this map handy — every control command in
Part B targets one of them.

| Group | Joints / interface | Notes |
|-------|--------------------|-------|
| **Mobile base** | `/cmd_vel` (Twist), `/odom` | differential drive |
| **Lift** | `joint_lift` | vertical carriage (0–1.1 m) |
| **Arm** (telescoping) | `joint_arm_l0..l3` + aggregate `wrist_extension` | 4 nested segments extend together |
| **Head** | `joint_head_pan`, `joint_head_tilt` | aims the head camera |
| **Dex wrist** (the "SE3" wrist) | `joint_wrist_yaw`, `joint_wrist_pitch`, `joint_wrist_roll` | 3-DOF wrist |
| **Gripper** | `joint_gripper_finger_left/right` | parallel grasp |
| **Sensors** | RPLidar (`/scan_filtered`), IMUs, D435i head cam, D405 gripper cam | cameras off by default |

No command to run here — just orient yourself.

---

## CP-A2 · The ROS2 graph (nodes & topics)

In a **second terminal** (remember the setup block):

```bash
ros2 node list
```
Expected:
```
/joint_state_publisher
/robot_state_publisher
/stretch_mujoco_driver
```

```bash
ros2 topic list
```
Key topics to recognize:
```
/cmd_vel            ← you publish here to drive the base   (remapped to /stretch/cmd_vel)
/joint_states       ← current joint positions/velocities
/scan_filtered      ← 2D lidar  (NOTE: not /scan)
/odom               ← base odometry
/tf, /tf_static     ← coordinate frames
/mode               ← current driver mode (position/navigation/…)
```

See who publishes/subscribes what:
```bash
ros2 node info /stretch_mujoco_driver | head -40
rqt_graph        # GUI; needs a display
```

> **Checkpoint:** stop the launch, open `sim.launch.py`, comment out the
> `# === CHECKPOINT: robot_model_tf ===` block, rebuild-free re-launch
> (`ros2 launch ...`). Re-run `ros2 node list` — `robot_state_publisher` and
> `joint_state_publisher` are gone, and in RViz the robot model vanishes (only the
> driver's odom/base frames survive). Uncomment to restore.

---

## CP-A3 · The URDF & the TF tree

The robot's shape and joints come from a **URDF**
(`stretch_se3_bringup/urdf/stretch_se3.urdf`, 61 links). `robot_state_publisher`
combines it with live joint angles to broadcast a **TF tree** of every frame.

Look at one transform — from the world-fixed `odom` frame to the gripper:
```bash
ros2 run tf2_ros tf2_echo odom link_grasp_center
```
You should see a translation/rotation that *changes* when the robot moves (Part B).

Dump the whole tree to a PDF:
```bash
ros2 run tf2_tools view_frames
# writes frames.pdf / frames.gv in the current directory — open frames.pdf
```
Trace the chain: `odom → base_link → … → link_lift → link_arm_l0 … → link_wrist_* →
link_grasp_center`. That chain *is* the robot's kinematics.

> **Checkpoint:** with the `robot_model_tf` block commented out (CP-A2),
> `tf2_echo odom link_grasp_center` fails — without `robot_state_publisher` there is no
> link TF. Proof that the URDF + publisher are what build the tree.

---

## CP-A4 · The sensors

**Lidar** — the 2D laser the base uses for Nav2 later:
```bash
ros2 topic echo /scan_filtered --once     # frame_id: laser, range 0.2–20 m
ros2 topic hz   /scan_filtered            # publish rate
```
In RViz the red points are the lidar hits (LaserScan display, topic `/scan_filtered`).

**Joint feedback:**
```bash
ros2 topic echo /joint_states --once      # names + positions of every joint in CP-A1
```

**Odometry & IMUs:**
```bash
ros2 topic echo /odom --once              # base pose estimate
ros2 topic list | grep imu                # /imu_mobile_base, /imu_wrist
```

**Cameras (optional, slow on CPU)** — off by default. To try them:
```bash
ros2 launch stretch_se3_bringup sim.launch.py use_cameras:=true
ros2 topic list | grep -iE "image|depth|camera"
```
Expect low frame rates without a GPU — see SETUP.md §8.

---

### End of Part A

You now know the body, the nodes, the topics, the TF tree, and the sensors. Next:
**Part B — Basic control** (drive the base and move every joint).

Stop the simulator with `Ctrl-C` in the launch terminal (or `pkill -f stretch_mujoco_driver`).

---

# Part B · Basic control

Goal: command the robot. You will drive the base, then move the lift, telescoping
arm, head, dex wrist and gripper — one group at a time — and finish with a single
combined pose. The demo nodes live in the **`stretch_se3_control`** package.

## B.0 · Two ways to command Stretch

Stretch has **two** control interfaces, and knowing which is which is the whole point
of Part B:

| What | Interface | Driver mode | Used by |
|------|-----------|-------------|---------|
| The **base** (wheels) | **velocity** — a `Twist` on `/stretch/cmd_vel` | **navigation** | CP-B1 |
| Every **joint** (lift, arm, head, wrist, gripper) | **position** — a `FollowJointTrajectory` goal on `/stretch_controller/follow_joint_trajectory` | **position** (default) | CP-B2 … CP-B5 |

The bringup launch starts in `mode:=position`, so the joint demos work immediately.
The base demo switches to navigation mode itself and switches back when done. (You can
see the current mode any time with `ros2 topic echo /mode --once`.)

All of the joint demos share one helper, `stretch_trajectory.StretchController`, whose
`move({joint: target})` sends one trajectory goal and blocks until the driver reports
the joints arrived. Targets are clamped to the model's soft limits so a demo can't
fault the driver.

> **Bring up the simulator** (leave it running in its own terminal):
> ```bash
> ros2 launch stretch_se3_bringup sim.launch.py            # add use_rviz:=false to go headless
> ```
> Run each demo below in a **second** terminal (remember the three `source`/`export`
> lines). Watch the robot move in RViz (or the MuJoCo viewer).

---

## CP-B1 · Base — drive a square

```bash
ros2 run stretch_se3_control square_drive
```
The node switches to **navigation mode**, then drives four sides + four 90° turns by
publishing a `Twist` on `/stretch/cmd_vel`, and finally restores position mode. Watch
`/odom` move:
```bash
ros2 topic echo /odom --once     # pose.pose.position changes as it drives
```

> The sim runs slower than real time on CPU, so the square is **approximate**. Tune
> `FORWARD_SPEED`, `TURN_SPEED`, `SIDE_SECONDS`, `TURN_SECONDS` at the top of
> `square_drive.py`.

**Drive it yourself (teleop):** the manual equivalent of this node. Because teleop
publishes to `/cmd_vel`, you remap it to the driver's topic, and you must be in
navigation mode first:
```bash
ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/stretch/cmd_vel
```

> **Checkpoint:** open `square_drive.py`, comment out the block between
> `# === CHECKPOINT: square_drive ===` markers, re-run → the robot switches modes but
> never moves. Restore it → the square returns. Or shrink `range(4)` to drive fewer sides.

---

## CP-B2 · Lift & telescoping arm

```bash
ros2 run stretch_se3_control lift_arm
```
Raises the lift, extends the arm, retracts, lowers. The arm is one aggregate joint
`wrist_extension` (0 → 0.52 m); the driver spreads it across the four arm segments you
saw in CP-A1.

> **Checkpoint:** `lift_arm.py` has two blocks, `lift` and `arm`. Comment out `arm` →
> only the lift moves. Comment out `lift` → only the arm telescopes.

---

## CP-B3 · Head — pan & tilt

```bash
ros2 run stretch_se3_control head_scan
```
Sweeps the head left/right (`joint_head_pan`) and down/up (`joint_head_tilt`). This is
how you aim the D435i head camera at a target.

> **Checkpoint:** blocks `head_pan` and `head_tilt` — disable either to isolate the
> other axis.

---

## CP-B4 · Dex wrist + gripper

```bash
ros2 run stretch_se3_control wrist_gripper
```
Exercises the three wrist DOF that make this the "**SE3**" wrist — yaw, pitch, roll —
then opens and closes the gripper (`gripper_aperture`: negative = closed, positive =
open).

> **Checkpoint:** four blocks — `wrist_yaw`, `wrist_pitch`, `wrist_roll`, `gripper`.
> Comment out any to skip that motion.

---

## CP-B5 · Combined pose — stow → wake-up

```bash
ros2 run stretch_se3_control wake_up
```
The previous demos moved one group at a time. This node commands **eight joints in a
single trajectory point**, so the whole upper body moves to a named pose at once: first
a compact **stow**, then a **wake-up** pose (lift raised, arm out, gripper open, head
looking down at the workspace). The driver waits for every joint before the goal
completes.

> **Checkpoint:** blocks `stow` and `wake_up`. Comment out `wake_up` → the robot only
> stows. Edit the `STOW_POSE` / `WAKE_UP_POSE` dicts to design your own poses.

---

### End of Part B

You can now drive the base (velocity / navigation mode) and command every joint
(position goals / position mode), individually and as a combined pose. Next:
**Part C — Nav2** (build a map, then navigate autonomously) — where navigation mode and
that lidar from CP-A4 do the heavy lifting.

Stop the simulator with `Ctrl-C` in the launch terminal (or `pkill -f stretch_mujoco_driver`).

---

# Part C · Nav2 — mapping & autonomous navigation

Goal: give the robot a map of its world, let it figure out where it is on that map,
and have it plan and drive to goals on its own — all from the 2D lidar
(`/scan_filtered`) you met in CP-A4. The demos live in **`stretch_se3_nav`**.

This is the workshop's strongest sim demo: the lidar is *computed*, not rendered, so
Nav2 runs well even without a GPU.

## C.0 · How it fits together

Part C always uses **three terminals**:

1. **the sim**, in navigation mode (so `/cmd_vel` drives the base):
   ```bash
   ros2 launch stretch_se3_bringup sim.launch.py mode:=navigation
   ```
2. **a nav stack** — either `mapping.launch.py` (CP-C1) or `navigation.launch.py`
   (CP-C2+).
3. **a way to drive / send goals** — teleop, RViz, or a script.

Two sim-specific details the launch files handle for you:

* **The lidar topic is `/scan_filtered`** (not `/scan`), and the sim publishes
  **`base_link`** (not `base_footprint`). Our `config/nav2_params.yaml` and
  `config/mapper_params.yaml` are the upstream Stretch configs edited for exactly these
  two facts — diff them against `upstream/.../stretch_nav2/config/` to see what changed.
* **A `cmd_vel_relay` node** bridges Nav2's `/cmd_vel` output to the driver's
  `/stretch/cmd_vel`. (Upstream uses `topic_tools relay`; that package isn't installed
  here, so we ship a three-line node — open `cmd_vel_relay.py`, it's the whole nav
  stack in miniature.)

---

## CP-C1 · Build a map (SLAM)

Bring up SLAM, drive the robot around, watch the map grow in RViz:

```bash
# terminal 2
ros2 launch stretch_se3_nav mapping.launch.py
# terminal 3 — drive to explore (teleop publishes /cmd_vel, the relay forwards it)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
`slam_toolbox` fuses `/scan_filtered` into an occupancy grid on `/map` and broadcasts
the `map -> odom` transform. In RViz add a **Map** display (topic `/map`) and drive
until the walls are filled in.

Save the map into the package's `maps/` folder so the next steps find it by default:
```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/air26-ros2-ws/src/04_stretch-hr-se3/stretch_se3_nav/maps/workshop_map
```
This writes `workshop_map.yaml` + `workshop_map.pgm`.

> **Checkpoint:** in `mapping.launch.py`, comment out the `slam` block → no `/map`, no
> `map -> odom`; RViz shows only the live red lidar scan. Comment out `cmd_vel_relay` →
> teleop keypresses stop moving the robot.

---

## CP-C2 · Localize on the map (AMCL)

Now load the saved map and let **AMCL** track the robot's pose on it:

```bash
# terminal 2 (replace terminal-2 from CP-C1)
ros2 launch stretch_se3_nav navigation.launch.py
```
With no `map:=` argument it defaults to `maps/workshop_map.yaml`. This brings up the
**full Nav2 stack** — `map_server`, `amcl`, `planner_server`, `controller_server`,
`bt_navigator`, and the recovery behaviours — all auto-activated.

In RViz: click **2D Pose Estimate** and click+drag on the map where the robot actually
is. AMCL's particle cloud snaps to that pose and the laser scan lines up with the map
walls. That alignment is "localized".

> **Checkpoint:** comment out the `nav2_stack` block in `navigation.launch.py` → the
> relay and RViz still come up but nothing localizes or plans.

---

## CP-C3 · Send goals

**From RViz:** click **Nav2 Goal** and pick a free-space point. The planner draws a
path, the controller follows it, and the base drives there (Nav2 publishes `/cmd_vel`,
the relay forwards it).

**Programmatically** — the scripted equivalent, using the Nav2 Simple Commander API
(the same one you'd use on the real Stretch):
```bash
ros2 run stretch_se3_nav waypoint_demo
```
It seeds nothing — set the initial pose in RViz first (CP-C2) — then drives through the
waypoint list at the top of `waypoint_demo.py`. **Edit `WAYPOINTS`** to match your map:
pick reachable points in the free (white) costmap area.

> The sim runs slower than real time on CPU, so goals take a while; watch
> `distance_remaining` shrink in the node's log.

> **Checkpoint:** comment out the `waypoints` block in `waypoint_demo.py` → the node
> connects to an active Nav2 and exits without driving. Useful to confirm the stack is
> up before scripting motion.

---

## CP-C4 · Costmaps & recovery

Nav2 plans around obstacles using two **costmaps** — a static `global_costmap` (from
the map) and a rolling `local_costmap` (from the live lidar). In RViz add two **Map**
displays on `/global_costmap/costmap` and `/local_costmap/costmap` to see the inflation
around walls.

Try tuning one parameter and re-observing:

```bash
# widen the safety inflation around obstacles (live, no restart):
ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.9
```
Send a goal near a wall and watch the path bow further away. Then edit the same value in
`config/nav2_params.yaml` (search `inflation_radius`) to make it permanent.

If you send a goal into an obstacle or the robot gets stuck, watch the **recovery
behaviours** fire in the Nav2 logs (spin, back up, wait) — these are defined in the
`behavior_server` and `bt_navigator` sections of `nav2_params.yaml`.

> **Checkpoint:** the whole `nav2_params.yaml` is one big editable block. Change
> `inflation_radius`, `max_vel_x` (controller), or `robot_radius`, relaunch, and watch
> the planning/driving behaviour change.

---

### End of Part C

You built a map, localized on it, and drove autonomously to goals — from RViz and from
Python — entirely on the simulated lidar. Next: **Part D — MoveIt2** (plan collision-free
motions for the arm and do a pick), which brings back the joints from Part B under a
motion planner.

Stop everything with `Ctrl-C` in each launch terminal (or `pkill -f stretch_mujoco_driver`
and `pkill -f nav2`).

---

# Part D · MoveIt2 — arm motion planning

Goal: plan collision-free motions for the arm with **MoveIt2** and execute them on the
sim. Where Part B sent raw joint commands, here a motion planner figures out *how* to
get the arm from A to B. The config lives in **`stretch_se3_moveit2`**.

There is **no official MoveIt2 config** for the Stretch on Humble, so this package is a
hand-generated one (the "Setup Assistant" step done in code:
`scripts/generate_srdf.py`), adapted from Hello Robot's galactic `stretch_moveit2`
reference and reconciled against our SE3 URDF.

## D.0 · The one integration piece: the trajectory bridge

MoveIt plans in terms of the URDF's four arm segments (`joint_arm_l0..l3`), but the
MuJoCo driver only understands the *aggregate* `wrist_extension` (Part B). So this
package ships a **trajectory_bridge** node:

```
MoveIt  --/stretch_arm_controller/follow_joint_trajectory-->  bridge
bridge   --/stretch_controller/follow_joint_trajectory------>  MuJoCo driver
```

The bridge collapses `joint_arm_l0+l1+l2+l3` into one `wrist_extension = sum` and passes
the lift/wrist/head/gripper joints through. It's started automatically by the launch —
open `scripts/trajectory_bridge.py`, it's short and is the heart of Part D in sim.

> Three other sim-specific reconciliations are baked into the config (all documented in
> `scripts/generate_srdf.py` and the config files): the SRDF robot name matches the URDF
> (`stretch`); the gripper-finger URDF limits were widened from a degenerate `[0,0]`; and
> opposing gripper fingers have their self-collision disabled so the home pose is valid.

## D.1 · Bring up move_group  (CP-D1)

Run the sim in its default **position mode** (the arm executes through the trajectory
action), then move_group:

```bash
ros2 launch stretch_se3_bringup sim.launch.py
ros2 launch stretch_se3_moveit2 move_group.launch.py
```
RViz opens with the **MotionPlanning** panel and the planning group `stretch_arm`. You
should see the robot model and an interactive orange goal handle on the gripper.

> **Checkpoint:** in `move_group.launch.py`, comment out the `trajectory_bridge` block →
> MoveIt still plans, but execution fails (the driver rejects `joint_arm_lN`). Comment out
> `move_group` → nothing plans at all.

## D.2 · Interactive planning  (CP-D2)

In the RViz **MotionPlanning** panel:

1. Under the **Planning** tab, set the **Planning Group** to `stretch_arm`.
2. Drag the orange interactive marker to a new arm pose (or pick a **Goal State** named
   target like `ready` / `stow` from the dropdown).
3. Click **Plan** — a preview of the trajectory animates.
4. Click **Execute** (or **Plan & Execute**) — the real arm in the sim moves there.

Named targets (`ready`, `stow`, gripper `open`/`closed`, head poses) come from the SRDF
`group_states` — see `config/stretch_se3.srdf`.

## D.3 · Programmatic planning  (CP-D3)

The C++ equivalent, using `MoveGroupInterface` — the same API you'd use on the real
robot. It plans to the `ready` and `stow` named targets and then a custom joint goal,
executing each on the sim:

```bash
ros2 launch stretch_se3_moveit2 reach_demo.launch.py
```
(It's a launch, not `ros2 run`, because `MoveGroupInterface` needs the URDF/SRDF
parameters on its own node.) Watch the planning times and `Execute request success!` in
the log, and the arm move in RViz. Source: `src/reach_demo.cpp`.

> **Checkpoint:** `reach_demo.cpp` has `named_targets` and `joint_goal` blocks — comment
> out either to change what the arm does. Edit the joint goal to reach elsewhere.

> The sim is slower than real time, so each motion takes a few seconds; the demo pauses
> between moves to let the arm settle.

## D.4 · Navigate, then reach  (CP-D4, stretch goal)

Combine Parts C and D: drive the base to a surface with Nav2, then plan an arm motion to
reach over it. The pieces are all here — run the Part C `navigation.launch.py` (separate
terminal) to drive to a goal, then `move_group.launch.py` + `reach_demo` to reach. The
base is parked during arm planning (the MoveIt config plans the arm with a fixed base —
see the `virtual_joint` in the SRDF), which matches the real Stretch workflow: *navigate,
park, manipulate.*

> **Performance note:** move_group, RViz, and the sim together are heavy on a CPU-only
> box. Close the MuJoCo viewer and keep `use_cameras:=false`. Planning itself is fast
> (lidar-style geometry); execution is gated by the sim's real-time factor.

---

### End of Part D

You now have the whole stack: architecture (A), direct control (B), autonomous
navigation (C), and motion-planned manipulation (D) — a complete virtual Stretch on
ROS2 + MuJoCo. Every feature is a checkpoint block you can comment out, rebuild, and
watch change.

Stop everything with `Ctrl-C` in each terminal (or `pkill -f stretch_mujoco_driver`,
`pkill -f move_group`).
