# 02_micro_ros — Running on the REAL robot (micro-ROS + ESP32)

How to bring the **physical rover** up over micro-ROS. The ESP32 firmware **replaces the
MuJoCo sim**: over WiFi/UDP it publishes `/ultrasonic/*` (`std_msgs/UInt8`, cm) and subscribes
`/cmd_vel` — the exact same interface — so the behaviours run against it unchanged.

```
  ESP32 (firmware)  ──WiFi/UDP:8888──►  micro_ros_agent  ──DDS──►  ROS 2 graph
   /ultrasonic/* (UInt8 cm)                (~/uros_ws)             behaviours, RViz, you
   /cmd_vel  ◄───────────────────────────────────────────────────
```

The **micro-ROS Agent** is the bridge between the board's XRCE-DDS and the ROS 2 graph.
Nothing else here talks to the board directly.

---

## 0 · Prerequisites (one-time)

1. **Agent built** in `~/uros_ws` (a *separate* workspace, NOT in `air26-ros2-ws`) — see
   [`SETUP.md`](SETUP.md) § "micro-ROS Agent". Quick check:
   ```bash
   source ~/uros_ws/install/setup.bash && ros2 pkg prefix micro_ros_agent   # should print a path
   ```
2. **Firmware flashed** to the ESP32 with your WiFi + this PC's IP set in the config block.
   Pick ONE firmware:
   - `firmware/esp32_microbot` — open-loop (no encoders).
   - `firmware/esp32_microbot_enc` — closed-loop speed control (JGB37-520 encoders wired).

   ```bash
   cd firmware/esp32_microbot          # or esp32_microbot_enc
   pio run -t upload                   # flash over USB
   pio device monitor                  # watch WiFi join + agent state
   ```
3. **Same 2.4 GHz network.** The classic ESP32 is 2.4 GHz only — if the board scan reports
   `NO_SSID_AVAIL`, the SSID is 5 GHz (see the WiFi gotcha in [`SETUP.md`](SETUP.md)).
4. **`AGENT_IP` / `AGENT_PORT` in the firmware must match this PC.** Find the PC's IP with
   `hostname -I`; the port defaults to **8888** on both sides.

---

## 1 · Source everything

Every terminal that talks to the robot needs all three:
```bash
source /opt/ros/humble/setup.bash          # ROS 2 Humble
source ~/air26-ros2-ws/install/setup.bash  # this workspace (behaviours, bridge, launch)
source ~/uros_ws/install/setup.bash        # the micro-ROS Agent
```
> Tip: add these to a `~/microbot_env.sh` and `source` that.

---

## 2 · Run it

### Option A — recommended: Agent in its own terminal
Keeps the Agent's connection log visible and separate from the ROS stack.

**Terminal 1 — the Agent:**
```bash
source /opt/ros/humble/setup.bash && source ~/uros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
Power on the board; within a few seconds the Agent prints a `session established` /
`create_participant` line and `/microbot_esp32` appears.

**Terminal 2 — the ROS-side stack** (model + RViz range cones + behaviours):
```bash
source /opt/ros/humble/setup.bash && source ~/air26-ros2-ws/install/setup.bash
ros2 launch microbot_sim real.launch.py
```

### Option B — one command: launch spawns the Agent too
Needs `~/uros_ws` sourced in the same terminal (step 1).
```bash
ros2 launch microbot_sim real.launch.py agent:=true
```

### `real.launch.py` arguments
| Arg | Default | Effect |
|-----|---------|--------|
| `agent` | `false` | also spawn `micro_ros_agent udp4` here (Option B) |
| `agent_port` | `8888` | UDP port; must match the firmware's `AGENT_PORT` |
| `use_rviz` | `true` | RViz + the `range_viz_bridge` (UInt8 cm → `Range` for the cones) |
| `behaviors` | `true` | `behavior_manager` + `obstacle_services` |

It starts: `robot_state_publisher`, `joint_state_publisher`, `range_viz_bridge` (viz-only),
`rviz2` (with `microbot_real.rviz`), and the two behaviour nodes. It does **not** start
`mujoco_driver` — the board is the driver.

> **No odom, static model — by design.** The firmware publishes no odometry/TF, so this launch
> uses **no `odom`/world frame at all**: RViz's Fixed Frame is **`base_link`**
> (`microbot_real.rviz`). The only TF is the robot describing its **own links**
> (`robot_state_publisher`, straight from the URDF) — that's what places the model and the
> `us_front/left/right` frames the range cones attach to. The 4 wheel joints are `continuous`,
> so `joint_state_publisher` pins them at 0 (wheels render but don't spin). The model sits at the
> origin and doesn't move — you're watching the **live ultrasonic cones**, not the robot's
> position. That's all this view needs.

---

## 3 · Verify the board is live

```bash
ros2 node list                         # expect /microbot_esp32
ros2 topic list | grep ultrasonic      # /ultrasonic/front|left|right (+ .../range from the bridge)
```

> **QoS gotcha:** the sensor topics are **best-effort** (latency branch). `ros2 topic echo`/`hz`
> default to *reliable* and will show **nothing**. Force best-effort:
> ```bash
> ros2 topic echo /ultrasonic/front --qos-reliability best_effort   # a std_msgs/UInt8 in cm
> ros2 topic hz   /ultrasonic/front --qos-reliability best_effort   # ~10 Hz (front)
> ```
> The `/ultrasonic/*/range` topics from the bridge are reliable, so plain `echo` works on those.

**Drive test** (raw):
```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"   # nudge forward
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"                    # stop
```

**Behaviours** (same as the sim):
```bash
ros2 service call /set_behavior microbot_interfaces/srv/SetBehavior "{behavior: 3}"
ros2 service call /check_openings microbot_interfaces/srv/CheckOpenings "{threshold: 0}"  # 0=default (cm)
```

---

## 4 · Troubleshooting

| Symptom | Likely cause / fix |
|---------|--------------------|
| Board hangs after boot banner | SSID is 5 GHz — switch hotspot to **2.4 GHz** (see SETUP.md). |
| Agent never says "connected" | `AGENT_IP` in firmware ≠ this PC (`hostname -I`); or port ≠ 8888; or AP client-isolation on. |
| `/microbot_esp32` up but `echo` shows nothing | Missing `--qos-reliability best_effort` (see §3). |
| RViz range cones missing | `use_rviz:=true` (runs `range_viz_bridge`); cones read `/ultrasonic/*/range`, not the raw cm topics. |
| Motors don't move | check `/cmd_vel` is arriving (`ros2 topic echo /cmd_vel`); L298N power + shared ground; firmware `MAX_LIN`. |
| Robot much slower/faster than commanded | open-loop firmware: recalibrate `MAX_LIN`. Or flash `esp32_microbot_enc` (closed-loop). |
| Enc build: one wheel fights the command | swap that side's encoder `{A,B}` pins (`ENC_L`/`ENC_R`); check `WHEEL_DIA`/`GEAR_RATIO`/`ENC_PPR`. |

See also: [`SETUP.md`](SETUP.md) (agent build, wiring, WiFi), [`LATENCY.md`](LATENCY.md)
(why UInt8 cm + best-effort), [`TUTORIAL.md`](TUTORIAL.md) (the behaviours).
