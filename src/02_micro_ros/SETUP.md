# Project 02 — setup

## ROS part (this project) — nothing new to install
Everything the ROS-side demo needs is already present:
- ROS2 Humble + `sensor_msgs` (Range), `geometry_msgs` (Twist), `nav_msgs`, `tf2_ros`,
  `rviz2`, `rqt_graph`, `teleop_twist_keyboard`.
- MuJoCo python (3.2.6, from project 04).

Build & run:
```bash
cd ~/air26-ros2-ws
source /opt/ros/humble/setup.bash
colcon build --packages-select microbot_interfaces microbot_description microbot_sim microbot_behaviors
source install/setup.bash

# terminal 1 — the sim (robot + 3 ultrasonics)
ros2 launch microbot_sim mujoco.launch.py
# terminal 2 — the behaviours
ros2 launch microbot_behaviors behaviors.launch.py
# switch behaviour live
ros2 service call /set_behavior microbot_interfaces/srv/SetBehavior "{behavior: 3}"
```
On a GPU-less box use the offscreen viewer path:
`MUJOCO_GL=egl ros2 launch microbot_sim mujoco.launch.py use_viewer:=false use_rviz:=true`.

## Deferred — Gazebo (Ignition) target
A second sim was planned (Gazebo Sim, already installed from project 07). It is **not built
yet**: needs `urdf/microbot.gazebo.xacro` (Ignition `DiffDrive` system + 3 narrow
`gpu_lidar` ultrasonics), an obstacle world SDF, the `ros_gz` bridge config, and wiring
`scan_to_range.py` + `gazebo.launch.py`. The MuJoCo+RViz path is a complete demo on its own.
`urdf/microbot.urdf.xacro` already has the `use_gazebo` arg hook for it.

## ESP32 micro-ROS firmware — FLASHED & VERIFIED ON HARDWARE (2026-06-24)
The real-robot firmware lives in **`firmware/esp32_microbot/`** (PlatformIO +
micro_ros_platformio, Arduino framework) — outside colcon (`firmware/COLCON_IGNORE`). It
publishes the same `/ultrasonic/*` ranges and subscribes to `/cmd_vel`, so the behaviour
nodes run unchanged: the sim swaps out for `micro_ros_agent` + the board.

- **Toolchain:** PlatformIO via the OFFICIAL installer (creates `~/.platformio/penv`; bare
  `pip install platformio` does NOT and breaks the micro-ROS build). `pio` at
  `~/.platformio/penv/bin/pio`. See firmware README for the `python3.10-venv` + `dbus-x11`
  prerequisites.
- **Transport:** flash over USB serial (`pio run -t upload`), run micro-ROS over **WiFi/UDP**.
- **Reference hardware:** ESP32 DevKit + L298N (skid-steer) + 3× HC-SR04; all pins / WiFi /
  Agent IP are a CONFIG block at the top of `src/main.cpp` — edit for your build.

### Flashing workflow that worked (full detail in firmware README §4)
```bash
cd src/02_micro_ros/firmware/esp32_microbot
# 1. edit CONFIG block top of src/main.cpp: WIFI_SSID, WIFI_PASS, AGENT_IP = this PC's IP
~/.platformio/penv/bin/pio run -t upload          # compile + flash over serial
```
- **Port:** this board (WCH CH9102 bridge, `1a86:55d4`) enumerates as **`/dev/ttyACM0`**,
  NOT `/dev/ttyUSB0`. PlatformIO auto-detects it.
- **Serial monitor, HEADLESS:** `pio device monitor` needs a real TTY and fails here
  (`termios: Inappropriate ioctl`). Read the port with pyserial instead, deasserting
  DTR/RTS and pulsing a clean reset (otherwise opening the port holds the ESP32 in reset
  and you see nothing). Snippet used:
  ```python
  import serial,time
  s=serial.Serial(); s.port='/dev/ttyACM0'; s.baudrate=115200; s.timeout=1
  s.dtr=False; s.rts=False; s.open()
  s.setRTS(True); time.sleep(0.1); s.setRTS(False)   # clean run-mode reset
  while True:
      l=s.readline()
      if l: print(l.decode(errors='replace').rstrip())
  ```
  On a normal desktop with a terminal, plain `pio device monitor` works fine.

### micro-ROS Agent — BUILT NATIVELY (no Docker on this box)
`micro_ros_agent` is not in apt. Built once into a SEPARATE workspace `~/uros_ws`
(NOT in air26-ros2-ws/src) via `micro_ros_setup`:
```bash
mkdir -p ~/uros_ws/src && cd ~/uros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup src/micro_ros_setup
rosdep install --from-paths src --ignore-src -y
colcon build && source install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh && source install/setup.bash
# run it (UDP):
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### WiFi GOTCHA (cost us a few reflashes)
The classic ESP32 is **2.4 GHz only**. The hotspot `LSPrmn60x` was broadcasting on 5 GHz,
so the board's scan reported `NO_SSID_AVAIL` (status=1) and `set_microros_wifi_transports()`
**blocked forever** waiting to associate (board hangs silently right after the boot banner).
Fix: switch the hotspot to 2.4 GHz (phone: "Maximize compatibility" / 2.4 GHz band). The
firmware now does a **non-blocking WiFi join with a scan dump + status logging** (checkpoint
`serial_diag`) so this is diagnosable instead of a silent hang.

### Verified end-to-end on real hardware (2026-06-24)
Board on `/dev/ttyACM0`, hotspot `LSPrmn60x` @2.4 GHz, Agent on this PC `10.185.122.251:8888`:
- node `/microbot_esp32` online via the Agent (UDP client connected).
- publishes `/ultrasonic/{front,left,right}` at **10 Hz** (no HC-SR04 wired → range=2.0=US_MAX,
  i.e. no-echo→max, as designed).
- subscribes `/cmd_vel` (Subscription count 1); accepted a Twist, node stayed up.
- Next on real HW: wire the 3× HC-SR04 + L298N per the README pinout, then run
  `microbot_behaviors` against the board exactly like the sim.

## Verified (MuJoCo, headless)
`/cmd_vel` drives; the 3 rangefinders report correct distances; `/set_behavior`,
`/check_openings` and `/escape_obstacle` all work; integrated B3 = obstacle → service →
action (feedback) → resume. Obstacles are solid (front-gate + arena clamp).
