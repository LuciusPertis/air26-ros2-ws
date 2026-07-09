# ESP32 micro-ROS firmware — microbot (ENCODER / closed-loop variant)

> **This is the encoder variant** of `firmware/esp32_microbot`. The motors are **JGB37-520 DC
> gear motors with magnetic quadrature encoders**, and the firmware **closes the speed loop**:
> it reads the encoders, computes each side's *actual* wheel speed (wheel diameter × rev/s) and
> trims the PWM with a per-side PI controller so the driven speed matches `/cmd_vel`. So the
> distance/speed actually driven matches what was commanded, instead of the open-loop firmware's
> `MAX_LIN` guess. **It still publishes NO odometry** — the encoders are used purely on-board
> for speed regulation, so the ROS interface below is byte-for-byte identical to the base
> firmware. See `LATENCY.md` and the `closed_loop_velocity` / `encoder_isr` checkpoints in
> `src/main.cpp`.

The real robot's firmware. It exposes the **same ROS 2 interface as the MuJoCo sim**, so
the `microbot_behaviors` nodes run against it unchanged:

| Direction | Topic | Type | Hardware |
|-----------|-------|------|----------|
| publish | `/ultrasonic/front` `/ultrasonic/left` `/ultrasonic/right` | `std_msgs/UInt8` (cm) | 3× HC-SR04 |
| subscribe | `/cmd_vel` | `geometry_msgs/Twist` | L298N → 4 wheels (skid-steer) |

**Encoder wiring (JGB37-520, 6-pin: M1, GND, C2, C1, VCC, M2):** M1/M2 → L298N motor output;
encoder VCC → 3V3, GND → GND; **C1/C2 (A/B channels)** → the `ENC_L`/`ENC_R` pins in the config
block (default left `{17,18}`, right `{8,9}`). Calibrate `WHEEL_DIA`, `ENC_PPR`, `GEAR_RATIO`
for your unit. If a side counts the wrong way, swap that side's `{A,B}` pins.

Transport: **WiFi/UDP** to the micro-ROS Agent. Flash over **USB**, then it runs untethered.

> Built with **PlatformIO + micro_ros_platformio** (Arduino framework). Lives outside
> colcon (`firmware/COLCON_IGNORE`), so `colcon build` never touches it.

## 1. Edit the config (top of `src/main.cpp`)
- `WIFI_SSID` / `WIFI_PASS`
- `AGENT_IP` = the PC running the Agent; `AGENT_PORT` = 8888
- pin assignments if your wiring differs (HC-SR04 trig/echo, L298N EN/IN pins)

## 2. Reference wiring (ESP32 DevKit + L298N + 3× HC-SR04)
```
HC-SR04 front  trig=26 echo=25        L298N  ENA=13 IN1=12 IN2=14  (LEFT  wheels)
HC-SR04 left   trig=33 echo=32               ENB=27 IN3=16 IN4=17  (RIGHT wheels)
HC-SR04 right  trig=18 echo=19
```
> HC-SR04 echo is 5 V — use a divider (1k/2k) into the ESP32 echo pins (3.3 V). Power the
> L298N motors from a separate battery; share grounds with the ESP32.

## 3. Install PlatformIO (use the OFFICIAL installer, not bare `pip`)
`micro_ros_platformio` builds the micro-ROS library inside PlatformIO's `penv`, so PlatformIO
must be installed the way that creates it:
```bash
sudo apt-get install -y python3.10-venv dbus-x11   # venv (stripped by Ubuntu) + dbus-launch
python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py)"
# pio is then at ~/.platformio/penv/bin/pio  (add to PATH, or use the full path)
```
> Build-env gotchas (all hit + fixed on the dev box):
> - plain `pip install platformio` does NOT create `~/.platformio/penv` → micro-ROS build
>   fails with `cannot open .platformio/penv/bin/activate`; use the official installer above.
> - the rosidl typesupport build calls `dbus-launch` → needs `dbus-x11`.
>
> **Verified:** `pio run` compiles + links clean (Flash ~60% / 792 KB, RAM ~20% on esp32dev).

## 4. Build / flash / monitor (board on USB)
```bash
cd firmware/esp32_microbot
~/.platformio/penv/bin/pio run            # compile (no board needed)
~/.platformio/penv/bin/pio run -t upload  # flash over USB serial
~/.platformio/penv/bin/pio device monitor # serial console (115200) — needs a real terminal
```
> **Port note:** the dev board (WCH **CH9102** bridge, USB id `1a86:55d4`) enumerates as
> **`/dev/ttyACM0`**, not `/dev/ttyUSB0` — newer WCH chips use the CDC-ACM driver. PlatformIO
> auto-detects either. (Older CP210x/CH340 boards show as `/dev/ttyUSB0`.)

### Reading serial headless (no terminal / over SSH)
`pio device monitor` needs an interactive TTY; in a headless/automation shell it fails with
`termios.error: Inappropriate ioctl for device`. Read the port with pyserial instead — and
**deassert DTR/RTS + pulse a clean reset**, else opening the port holds the ESP32 in reset
and you see nothing:
```python
import serial, time
s = serial.Serial(); s.port='/dev/ttyACM0'; s.baudrate=115200; s.timeout=1
s.dtr=False; s.rts=False; s.open()
s.setRTS(True); time.sleep(0.1); s.setRTS(False)   # EN low->high, GPIO0 high = normal boot
while True:
    l = s.readline()
    if l: print(l.decode(errors='replace').rstrip())
```

### Serial diagnostics in the firmware (checkpoint `serial_diag`)
`setup()` opens `Serial` @115200 and prints a boot banner, a **2.4 GHz scan dump**, a
**non-blocking WiFi join** with `WiFi.status()` each second (0=IDLE 1=NO_SSID_AVAIL
3=CONNECTED 4=CONNECT_FAILED 6=DISCONNECTED), the obtained IP, and the Agent target;
`loop()` prints every agent-connection state change. Comment out the `serial_diag`
checkpoint blocks to run fully headless.

> **WiFi gotcha:** the classic ESP32 is **2.4 GHz only**. If the SSID is 5 GHz the scan
> reports `NO_SSID_AVAIL` and the board can't join (with the stock blocking transport it
> hangs silently after the banner). Put the AP/hotspot on 2.4 GHz. The board AND the PC
> running the Agent must be on the same network; set `AGENT_IP` to that PC's IP.

## 5. Run the micro-ROS Agent on the PC (WiFi/UDP)
> **On this dev box it's already built** in `~/uros_ws` (native, no Docker). To run:
> ```bash
> source /opt/ros/humble/setup.bash && source ~/uros_ws/install/setup.bash
> ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
> ```

`micro_ros_agent` isn't in apt; pick one:
```bash
# Docker (simplest):
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888

# or build it once via micro_ros_setup (separate ws, not air26-ros2-ws/src):
mkdir -p ~/uros_ws/src && cd ~/uros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup src/micro_ros_setup
rosdep install --from-paths src --ignore-src -y
colcon build && source install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh && source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## 6. Drive it with the SAME behaviours as the sim
```bash
# (Agent running, ESP32 powered + on WiFi)
ros2 topic list                      # see /ultrasonic/* + /cmd_vel from the board
ros2 launch microbot_behaviors behaviors.launch.py
ros2 service call /set_behavior microbot_interfaces/srv/SetBehavior "{behavior: 3}"
```
The robot now random-walks and avoids obstacles — the firmware just replaced the sim.

## Status — VERIFIED ON HARDWARE (2026-06-24)
Flashed an ESP32 (CH9102 / `/dev/ttyACM0`) over serial; Agent native in `~/uros_ws` on
`10.185.122.251:8888`; hotspot `LSPrmn60x` on 2.4 GHz. Confirmed: node `/microbot_esp32`
online via the Agent, `/ultrasonic/{front,left,right}` publishing at **10 Hz** (no HC-SR04
wired → range=2.0=US_MAX), `/cmd_vel` subscribed (count 1, accepted a Twist, node stable).
Next on real HW: wire 3× HC-SR04 + L298N per §2, then run `microbot_behaviors` unchanged.

## Notes
- The firmware uses the standard micro-ROS **reconnect state machine** (ping agent →
  create entities → spin → on link loss, stop motors + recreate), so restarting the Agent
  doesn't brick the board, and the robot stops safely if WiFi drops.
- `pio run -t upload` needs the board on a serial port + the user in the `dialout` group.
