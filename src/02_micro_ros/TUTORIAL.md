# Tutorial 02 — micro-ROS obstacle-avoider: sensors, messages, and behaviours

Meet **microbot**: a little 4-wheeled rover with 3 ultrasonic "eyes". Today you'll explore
its **URDF** in RViz, watch its **sensor messages** and **TF** flow, then give it three
obstacle-avoidance **behaviours** — and see *why* ROS has topics, services AND actions.

> The robot will later run **micro-ROS on an ESP32** (a separate session). For now a
> **simulator** plays the robot: it publishes the same `/ultrasonic/*` ranges and listens
> to the same `/cmd_vel` — so everything you write today works unchanged on the real board.

> **Build first** (once, and again after editing a node):
> ```bash
> cd ~/air26-ros2-ws
> source /opt/ros/humble/setup.bash
> colcon build --packages-select microbot_interfaces microbot_description microbot_sim microbot_behaviors
> source install/setup.bash
> ```

> **Setup — every new terminal:**
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/air26-ros2-ws/install/setup.bash
> ```

---

## 1 · The robot (URDF + RViz + TF2)

```bash
ros2 launch microbot_sim mujoco.launch.py
```
A MuJoCo window shows the rover in an arena with walls and pillars; RViz shows the **URDF
model**, the **TF tree**, and **three coloured range cones** (the ultrasonics).

| Part | What it is |
|------|------------|
| grey box | chassis |
| 4 black cylinders | driven wheels (skid-steer) |
| 3 blue boxes | ultrasonic sensors — front / left / right |
| green / red / orange boxes | battery / MCU / motor (the "guts") |

Explore the parts and frames:
```bash
ros2 run tf2_tools view_frames          # writes a PDF of the TF tree (URDF link frames)
ros2 topic echo /ultrasonic/front       # a sensor_msgs/Range message (the "Sensors/messages" topic)
```

---

## 2 · Drive it & review pub-sub (rqt_graph)

In a second terminal, drive manually and watch the ranges change:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard      # publishes /cmd_vel
ros2 run rqt_graph rqt_graph                               # see who publishes/subscribes what
```
`rqt_graph` shows the sim publishing `/ultrasonic/*` + `/odom` and subscribing `/cmd_vel`.
Drive toward a wall and watch `/ultrasonic/front` drop. **That's the whole sensor loop.**

---

## 3 · The behaviours (run the brain)

```bash
ros2 launch microbot_behaviors behaviors.launch.py
```
The rover now **random-walks** (random speed + small random turns) and avoids obstacles.
Switch the active behaviour live:
```bash
ros2 service call /set_behavior microbot_interfaces/srv/SetBehavior "{behavior: 1}"
```

### Behaviour 1 — stop & wait   *(just topics)*
Front blocked → **stop**, wait a moment, resume the random walk. Pure pub-sub: the node
reads `/ultrasonic/front` and writes `/cmd_vel`.

### Behaviour 2 — stop & turn   *(just topics)*
Front blocked → stop, **turn a random direction** briefly, resume. Still pub-sub.

### Behaviour 3 — look, then escape   *(service + action)*
```bash
ros2 service call /set_behavior microbot_interfaces/srv/SetBehavior "{behavior: 3}"
```
Front blocked → the brain:
1. asks a **service** `/check_openings` — *"is my left or right open right now?"* (one
   question, one instant answer), then
2. sends an **action** `/escape_obstacle` — a maneuver that runs for seconds: turn to the
   open side, or **back up + spin 180°** if both sides are blocked. The action streams
   **feedback** (`backing_up` → `turning_180`) and can be **cancelled**.

Watch the action yourself:
```bash
ros2 action list
ros2 action send_goal /escape_obstacle microbot_interfaces/action/EscapeObstacle \
  "{left_open: false, right_open: false}" --feedback
```

### Why three different tools?
- a **topic** is a continuous stream (the ultrasonics, `/cmd_vel`) — B1/B2 only need this;
- a **service** is one instant question/answer (`/check_openings`);
- an **action** is a long job you can watch and cancel (`/escape_obstacle`).

Try switching from B3 to B1 *while it's backing up* — the escape **cancels** mid-move. A
topic or service couldn't do that. (That's the seed for the next session: behaviour trees.)

---

## What you learned
A robot's senses and motors are just **messages on topics**. Simple reactions are pure
pub-sub; "ask a question" wants a **service**; "do a long, watchable, cancelable task" wants
an **action**. The same behaviour code will drive the real ESP32 robot later — only the
thing publishing `/ultrasonic/*` changes.

> Stop everything with `Ctrl-C` in each terminal (or `pkill -f mujoco_driver`,
> `pkill -f behavior_manager`).
