# Tutorial 02 — micro-ROS obstacle-avoider: sensors, messages, and behaviours

Meet **microbot**: a little 4-wheeled rover with 3 ultrasonic "eyes". Today you'll explore
its **URDF** in RViz, watch its **sensor messages** and **TF** flow, then give it three
obstacle-avoidance **behaviours** — and see *why* ROS has topics, services AND actions.

> The robot will later run **micro-ROS on an ESP32** (a separate session). For now a
> **simulator** plays the robot: it publishes the same `/ultrasonic/*` readings and listens
> to the same `/cmd_vel` — so everything you write today works unchanged on the real board.

> **Note — latency branch (`10-fix-...`):** on this branch the ultrasonics ship as
> `std_msgs/UInt8` **centimetres** (not `sensor_msgs/Range`), best-effort QoS, to cut
> sim-to-real latency. The **sim was changed to match the firmware**, so `mujoco_driver` also
> publishes `UInt8` cm — sim and real still share one interface. RViz's range cones are fed by
> a tiny viz-only `range_viz_bridge` (cm → `Range` on `/ultrasonic/*/range`). See
> [`LATENCY.md`](LATENCY.md) for the full rationale and the commit diffs.

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
A MuJoCo window shows the rover in an arena with walls, two fixed **pillars**, and three
**movable boxes**; RViz shows the **URDF model**, the **TF tree**, and **three coloured
range cones** (the ultrasonics).

| Part | What it is |
|------|------------|
| grey box | chassis |
| 4 black cylinders | driven wheels (skid-steer) |
| 3 blue boxes | ultrasonic sensors — front / left / right |
| green / red / orange boxes | battery / MCU / motor (the "guts") |

### The obstacles: fixed pillars vs. movable boxes
The arena has two kinds of obstacle:
- **Fixed pillars** (teal + red) are *static* geometry — baked into the world, they never move.
- **Movable boxes** (amber + purple) are **free rigid bodies**: the rover physically
  **pushes** them, and *you* can **grab them with the mouse** to rearrange the course live —
  completely independent of ROS. In the MuJoCo window: **double-click a box to select it**,
  then **Ctrl + right-drag** to slide it, **Ctrl + left-drag** to spin it.

> **Watch the sensors respond.** As a box moves closer to (or you drag it in front of) a
> sensor, watch that sensor's **range cone in RViz shrink**, and `ros2 topic echo
> /ultrasonic/front` count down; drag it away and the cone **grows** back. The cheap
> ultrasonics are just casting rays at whatever is there — sim or real, moved by physics or
> by your cursor. This is the fastest way to *provoke* a behaviour on demand (next sections).

Explore the parts and frames:
```bash
ros2 run tf2_tools view_frames          # writes a PDF of the TF tree (URDF link frames)
ros2 topic echo /ultrasonic/front       # std_msgs/UInt8 = distance in cm (latency branch)
ros2 topic echo /ultrasonic/front/range # sensor_msgs/Range (viz-only, from range_viz_bridge)
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

> **The "both blocked → back up + 180°" trap.** The three movable boxes are placed to *set
> up this exact case* without you touching anything: two boxes flank the start position
> (one just left, one just right) with the fixed pillar dead ahead. When the random walk
> heads roughly straight out of the start, the rover drives **into the pocket between the
> two boxes** — `/check_openings` finds **left *and* right blocked** — and the fixed pillar
> blocks the front, so `/escape_obstacle` runs its full signature: **back up, then spin
> 180°** (watch the feedback go `backing_up → turning_180`). If the random heading misses
> the pocket, just **drag a box into its path** with the mouse to force the trap. Watch all
> three RViz range cones collapse at once as it enters the pocket — that's the sensor
> picture the service reads.

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
