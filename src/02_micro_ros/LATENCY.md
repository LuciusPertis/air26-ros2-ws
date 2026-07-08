# 02_micro_ros — Sim-to-Real Latency Fixes (branch `10-fix-02_micro_ros-firmware`)

> **Teaching doc.** These changes live ONLY on `10-fix-...` and are **not merged to `main`**.
> Students are walked through the **commit diffs** to see *why* the real robot felt laggy
> (~2–3 s command-to-motion) and how each change claws latency back. Every change deliberately
> makes the **real firmware diverge from the MuJoCo sim** — the sim keeps `sensor_msgs/Range`,
> reliable QoS, etc.; the real robot trades fidelity for responsiveness.

## The symptom

On the real ESP32 rover, `/cmd_vel` → wheels lagged ~2–3 s and ultrasonic updates were jerky.
Same behaviour nodes drove the MuJoCo sim instantly. So the gap is **transport + firmware**,
not the ROS logic. The fixes below attack that gap. All are in
[`firmware/esp32_microbot/src/main.cpp`](firmware/esp32_microbot/src/main.cpp).

## Changes (in impact order)

| # | Change | Before | After | Why it was slow |
|---|--------|--------|-------|-----------------|
| 1 | **Best-effort + volatile QoS** on the 3 ultrasonic pubs and `/cmd_vel` sub | `rclc_*_init_default` (RELIABLE) | `rclc_*_init_best_effort` | RELIABLE over lossy WiFi/UDP means ACKs, retransmits, and a history queue. A dropped-then-retransmitted sample arrives *late and stale*. Best-effort = fire-and-forget; freshest wins. **Biggest single win.** |
| 2 | **WiFi modem power-save off** | (default modem sleep) | `WiFi.setSleep(false)` | ESP32 default modem sleep parks the radio between beacons, adding **100–200 ms** of RX latency + jitter to every incoming `/cmd_vel`. Costs battery; worth it. One line. |
| 3 | **Round-robin ultrasonics** | 3 blocking `pulseIn` reads every 100 ms tick (up to ~36 ms stall, during which `/cmd_vel` is not serviced) | 1 read per tick, cycle `[F, L, F, R]` at a 50 ms tick | Three back-to-back blocking pings starved the executor. Now one `pulseIn` per tick. Front (obstacle-critical) polls **2× the sides**: front ~10 Hz, each side ~5 Hz. |
| 4 | **Tamed the agent ping** | `rmw_uros_ping_agent(100, 3)` | `rmw_uros_ping_agent(100, 1)` | A failed 3-try ping blocks up to **300 ms** right next to the executor spin. 1 try caps the stall at 100 ms; a real disconnect is still caught. |
| 5 | **`sensor_msgs/Range` → `std_msgs/UInt8` (cm)** | Range: header (stamp + **string** `frame_id`) + radiation_type + FoV + min/max/range ≈ 40+ bytes, ~4 of them dynamic | 1-byte centimetre value (0–200) | Every publish re-shipped a mostly-constant Range struct incl. a string. UInt8 cm is 1 byte, no header, no `micro_ros_string_utilities` dependency. Range `US_MAX_M = 2 m` → 200 cm, fits `uint8` cleanly (`cm = round(m*100)`, clamp 255). |

### Not done (and why)
- **Static IP** — rejected: the workshop router/DHCP lease can change, so a hard-coded IP would
  brick the join. Kept DHCP.
- **Fixed WiFi channel** — router-side RF tuning, not firmware; we don't control the AP.

## Change #5 ripple — the whole chain went `UInt8` cm (done)

Rather than bridge cm→Range on the real side only, we made **`std_msgs/UInt8` centimetres the
canonical ultrasonic type everywhere** and pushed `sensor_msgs/Range` out to a viz-only edge.
The behaviours only ever compare the reading to a threshold, so they never needed Range's
header/FoV/min-max — cm integers are enough. Nice side effect: **sim and real re-converge** on
one interface (both publish `UInt8`), instead of diverging.

- **Canonical type `std_msgs/UInt8` (cm), best-effort**, on `/ultrasonic/{front,left,right}`:
  - real ESP32 firmware (change #5)
  - `microbot_sim/mujoco_driver` (MuJoCo)
  - `microbot_sim/scan_to_range` (Gazebo helper)
- **Behaviours now speak cm** (`behavior_manager`, `obstacle_services`): subscribe `UInt8`,
  thresholds are integer cm, and `CheckOpenings.srv threshold` is now `uint8` cm (`0` = server
  default). `front_threshold 0.35 m → 35`, `side_threshold 0.50 m → 50`.
- **`microbot_sim/range_viz_bridge`** (NEW, viz-only, off the control loop): subscribes the 3
  `UInt8` topics, republishes `sensor_msgs/Range` (m) on `/ultrasonic/*/range` **only for RViz**.
  Launched with RViz in `mujoco.launch.py` (gated on `use_rviz`). RViz config repointed to
  `/ultrasonic/*/range`. *(No odom on the real robot — firmware has no encoders — so the real
  RViz feed is ultrasonics-only.)*

**Status: complete and verified headless** — sim publishes `UInt8` cm (e.g. `73`), the bridge
re-inflates to `Range` (`0.73 m`), `/check_openings` answers a `uint8` cm threshold, and the
behaviour nodes random-walk without errors. Firmware changes 1–5 are compile-clean (`pio run`).

## Build / verify

```bash
cd src/02_micro_ros/firmware/esp32_microbot
pio run                 # compile only (no board needed) — checks the diff builds
pio run -t upload       # flash over USB
pio device monitor      # watch WiFi join + agent state
```
On the host, run the micro-ROS agent, then `ros2 topic hz /ultrasonic/front` (expect ~10 Hz)
and `ros2 topic echo /ultrasonic/front` (expect a `std_msgs/UInt8` in cm).
