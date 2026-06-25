# Tutorial 07 — A tiny VLA: how "Δθ" drives a robot through ROS2

You type a command like **"wave"**. A little **VLA** ("brain") turns it into
**delta-theta** — how much to nudge each joint — and ROS2 carries those numbers to a
robot arm that moves. The *same* arm runs in **three simulators**: RViz, Gazebo and
MuJoCo. The whole point is to *see* the action travel through ROS2.

> **VLA = Vision-Language-Action.** Here the **Language** is what you type, the **Action**
> is delta-theta, and we leave a **Vision** hook for later. Our brain is a tiny stand-in
> (no GPU needed) — but it plugs into the same interface a real model like SmolVLA would.

> **Build first** (do this once, and again after editing a node or commenting out a
> checkpoint):
> ```bash
> cd ~/air26-ros2-ws
> source /opt/ros/humble/setup.bash
> colcon build --packages-select vla_arm_description vla_demo
> source install/setup.bash
> ```

> **Setup — run in every new terminal:**
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/air26-ros2-ws/install/setup.bash
> ```

---

## The big picture

```
 you type            the brain                THE STAR TOPIC          adds the nudges up        a simulator
/instruction  ->   [ vla_brain ]   ->     /delta_theta        ->    [ theta_integrator ]  ->   moves the arm
  "wave"            mini-VLA              (Δθ per joint)              theta += Δθ  -> /joint_command
```

The arm has **3 joints**, each a different colour:
| Joint | Colour | Motion |
|-------|--------|--------|
| `joint1` (θ1) | blue | base turns left/right |
| `joint2` (θ2) | orange | shoulder up/down |
| `joint3` (θ3) | green | elbow bend/straighten |
| tip | red | the "hand" |

**Instructions you can type:** `up` `down` `left` `right` `bend` `straighten` `wave`
`circle` `home` `stop`.

---

## 1 · RViz (the simplest — no physics)

```bash
ros2 launch vla_demo rviz.launch.py
```
A second terminal — **make it move**, and **watch the action**:
```bash
ros2 topic pub /instruction std_msgs/String "{data: wave}"     # the elbow waggles
ros2 topic echo /delta_theta                                   # <-- watch the numbers!
```
Try `up`, then `home` (it drives back to zero). The numbers on `/delta_theta` ARE the
action the brain is sending — that's the lesson in one screen.

> **Checkpoint:** open `vla_demo/launch/rviz.launch.py`, comment out the `brain` block,
> rebuild, re-run → nothing publishes `/delta_theta`, the arm never moves. Put it back →
> it works again. (Same idea for the `integrator` block.)

---

## 2 · MuJoCo (real physics, lightweight)

```bash
ros2 launch vla_demo mujoco.launch.py                 # opens the MuJoCo viewer
# headless box? use:  MUJOCO_GL=egl ros2 launch vla_demo mujoco.launch.py use_viewer:=false
```
Same instructions, same `/delta_theta` — but now a **physics engine** moves the arm, so
it has weight and momentum. Type `circle` and watch the base + shoulder trace a circle.

> The exact same brain and integrator drive this — only the last box in the diagram
> changed. That's the point of ROS2: swap the robot, keep the pipeline.

---

## 3 · Gazebo (full robot simulator + ros2_control)

```bash
ros2 launch vla_demo gazebo.launch.py                 # GUI for the room
# headless check:  ros2 launch vla_demo gazebo.launch.py gz_args:='-r -s -v1 empty.sdf'
```
Here a real **ros2_control** position controller (running inside Gazebo) moves the joints.
Check it came up:
```bash
ros2 control list_controllers      # joint_state_broadcaster + position_controller -> active
ros2 topic pub /instruction std_msgs/String "{data: up}"
```
Now `/delta_theta` → `/joint_command` → `/position_controller/commands` → the arm. You can
follow the action through every hop with `ros2 topic echo`.

> **Checkpoint:** in `gazebo.launch.py`, comment out the `controllers` block → the arm
> spawns but never moves (no controller to drive it).

---

## How the "brain" works (for the curious)

`vla_demo/policies/scripted.py` is the whole VLA: it reads the instruction word and
returns a small Δθ each tick (e.g. `wave` → a sine wave on the elbow; `home` → drive every
joint toward zero). Swap it for a real learned model by implementing the same
`predict(instruction, theta, image)` method — see `policies/smolvla_adapter.py`.

> The brain uses the **commanded** joint angles (`/joint_command`) as its sense of "where
> the arm is", which keeps it stable. A real robot VLA would use the measured joint
> angles plus a controller — a nice thing to explain to older kids.

---

### What you learned
An **action** (delta-theta) is just numbers on a ROS2 **topic**. The brain produces them,
ROS2 carries them, an integrator adds them up, and *any* simulator turns them into motion.
Same pipeline, three robots.
