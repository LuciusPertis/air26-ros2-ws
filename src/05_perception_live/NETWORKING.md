# 05_perception_live — Networking & WiFi Routing (contributor guide)

> Audience: **contributors / instructors** setting up the room, not attendees.
> This is the part that actually decides whether the demo works. The ROS code is easy;
> getting ~20 laptops to *discover* each other over classroom WiFi is the hard part.

---

## 1. The data-flow model (read this first)

There are **two independent transports** in play. Do not confuse them.

```
                         ┌──────────────────────── LAN (one WiFi AP) ────────────────────────┐
                         │                                                                    │
  ESP32-CAM ──HTTP MJPEG─┼──►  HOST laptop                                                     │
  (http://<cam-ip>/stream)│     ├─ mjpeg_bridge      → /camera/image_raw ─┐                    │
                         │     ├─ camera_processor  → /camera/mean_*      │  DDS (ROS 2)       │
  ESP32 rover ─micro-ROS─┼──►  ├─ micro_ros_agent   ↔ /ultrasonic/*, /cmd_vel                 │
  (UDP :8888)            │     ├─ cmd_vel_odometry  → /odom, TF, /joint_states                │
                         │     ├─ aruco_detector    → /aruco/*            │                    │
                         │     └─ robot_state_publisher → /robot_description                   │
                         │                                                 ▼                    │
                         │   attendee laptop ×N:  rviz2  ◄──── DDS subscribes to all topics ───┤
                         └────────────────────────────────────────────────────────────────────┘
```

**The golden rule: exactly ONE machine (the "host") talks to the hardware.**
- Only the host runs `mjpeg_bridge` and the `micro_ros_agent`.
- The **ESP32-CAM serves MJPEG to *one* HTTP client at a time** (it is a ~4 MB-PSRAM board;
  concurrent clients + its own micro-ROS make it choke — we measured ~3.5 Hz for a single
  client, with `EV-VSYNC-OVF` spam). If 20 attendees each point `mjpeg_bridge`/a browser at
  `http://<cam-ip>/stream`, the stream dies.
- Attendees receive the camera as the **already-decoded `/camera/image_raw` ROS topic**, fanned
  out by DDS. Each attendee just runs `rviz2`. Nobody but the host touches the board.

So "many viewers" is a pure **ROS 2 multi-machine** problem: same LAN + same `ROS_DOMAIN_ID` +
working DDS discovery.

---

## 2. Minimum ROS 2 multi-machine checklist

On **every** machine (host and attendees):

```bash
export ROS_DOMAIN_ID=42          # any 0–101, but IDENTICAL everywhere. Pick one, put it on the whiteboard.
export ROS_LOCALHOST_ONLY=0      # must be 0 (default) — 1 hides you from the LAN
source /opt/ros/humble/setup.bash
source ~/air26-ros2-ws/install/setup.bash   # attendees only need the message types; see note
```

Quick sanity from an attendee laptop once the host is up:

```bash
ros2 topic list                  # should show /camera/image_raw, /odom, /tf, ...
ros2 topic hz /camera/image_raw  # should tick
ros2 topic echo /odom --once
```

If `ros2 topic list` is empty from an attendee but works on the host → **it's the network,
not ROS.** Jump to §4.

> Attendees who only run RViz still need the custom message *types* that appear on the graph.
> Everything here uses **standard** messages (`sensor_msgs`, `nav_msgs`, `vision_msgs`,
> `tf2_msgs`), so a plain `source /opt/ros/humble/setup.bash` is enough — attendees do **not**
> need to build this workspace. (`vision_msgs` ships with `ros-humble-vision-msgs`.)

---

## 3. What the WiFi AP/router MUST provide

DDS default discovery is **multicast**. Consumer/classroom WiFi frequently breaks exactly the
things DDS needs. Verify all of these on the AP:

| Requirement | Why | How it usually fails |
|---|---|---|
| **All devices on one subnet / one SSID** | DDS simple discovery is LAN-scoped, not routed | Guest SSID on a different subnet; 2.4 GHz vs 5 GHz bridged oddly |
| **AP/client isolation = OFF** | isolation blocks laptop↔laptop and laptop↔ESP32 traffic | "AP Isolation", "Client Isolation", "Guest mode" ON → discovery silently fails |
| **Multicast/IGMP allowed** | discovery uses `239.255.0.x` multicast (+ ports 7400+) | "Multicast to unicast" / IGMP snooping drops or throttles it |
| **No captive portal** | ESP32 can't click "I agree" | board associates but has no route |
| **Firewall not blocking UDP** | DDS + micro-ROS are UDP | host `ufw` blocking inbound UDP |

**Strong recommendation: bring your own router.** A cheap travel router (GL.iNet, or any
OpenWrt box) you fully control removes every "the campus network does X" unknown. Plug the host
laptop in by Ethernet if you can; put attendees + ESP32s on its WiFi.

### Host firewall
DDS uses many ephemeral UDP ports. Easiest for a closed demo LAN:
```bash
sudo ufw disable            # demo LAN only; re-enable after
# or, to keep ufw: allow the DDS/discovery + micro-ROS ports
sudo ufw allow 7400:7500/udp
sudo ufw allow 8888/udp     # micro_ros_agent
```

---

## 4. Debugging discovery (in order)

1. **Ping.** From an attendee: `ping <host-ip>`. Fails → layer-2/isolation problem, not ROS.
2. **Multicast reachability:**
   - host: `ros2 multicast receive`
   - attendee: `ros2 multicast send`
   - Host should print the datagram. Nothing → **multicast is blocked** (the #1 classroom
     failure). Go to §5 (Discovery Server or static peers).
3. **Same domain?** `echo $ROS_DOMAIN_ID` on both. Mismatch = invisible to each other.
4. **`ROS_LOCALHOST_ONLY`** unset or `0` on both.
5. **Two RMWs on the LAN?** Everyone should use the **same** RMW. Check `echo $RMW_IMPLEMENTATION`
   (empty = `rmw_fastrtps_cpp`, the Humble default). Mixing Fast DDS and Cyclone *interoperates*
   in theory but adds variables — standardize.
6. `ros2 doctor --report` for a broad sanity dump.

---

## 5. When multicast is unreliable → make discovery UNICAST

WiFi multicast is sent at the lowest basic rate and is not retransmitted, so on a busy AP
discovery is flaky even when it's "allowed." Two robust fixes; pick one and standardize the room.

### Option A — Fast DDS **Discovery Server** (recommended for a big room)
One well-known server process does discovery over **unicast**; everyone points at it. No
multicast needed at all.

On the **host** (the discovery server, e.g. host IP `10.42.0.10`):
```bash
export ROS_DISCOVERY_SERVER="10.42.0.10:11811"   # optional if you run the server standalone
fastdds discovery -i 0 -l 10.42.0.10 -p 11811 &   # the server
```
On **every** machine (host apps + attendees):
```bash
export ROS_DISCOVERY_SERVER="10.42.0.10:11811"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```
(Attendee RViz then discovers all topics via the server. If a node needs to also see
multicast peers, run it as a "super client" — for a pure-DDS-server room you don't.)

### Option B — CycloneDDS static **Peers** (no server, unicast to a fixed list)
Switch everyone to Cyclone and hand it the host IP (and any other publishers) explicitly:
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery>
  <ParticipantIndex>auto</ParticipantIndex>
  <Peers><Peer address="10.42.0.10"/></Peers>
  <MaxAutoParticipantIndex>50</MaxAutoParticipantIndex>
</Discovery></Domain></CycloneDDS>'
```
Attendees only need the host in their peer list (they only subscribe). Put this in a sourced
`env.sh` you hand out.

> Ship whichever option you choose as a **one-line `source setup_env.sh`** that attendees paste.
> The single biggest cause of a failed live demo is 20 people setting env vars 20 different ways.

---

## 6. Bandwidth: don't melt the WiFi with raw images

`/camera/image_raw` is `bgr8`. At 320×240 that's ~230 kB/frame; at 15 Hz ≈ **28 Mbps _per
subscriber_**. DDS sends a copy to each RViz over the air → 20 attendees ≈ 500+ Mbps of
multicast/unicast video the AP cannot sustain. Fixes:

1. **Publish compressed and let attendees subscribe to JPEG.** On the host:
   ```bash
   ros2 run image_transport republish raw in:=/camera/image_raw compressed out:=/camera/image_repub
   ```
   Attendees add an RViz **Image** display on `/camera/image_repub/compressed` (or use the
   `image_transport` plugin). ~10–20× less air traffic.
2. **Cap the source rate.** `mjpeg_bridge` has a `rate` param (default 15 Hz) — the ESP32-CAM
   rarely exceeds ~4 Hz anyway, so set `rate:=5`.
3. **Keep the resolution low** (QVGA/320×240) in the ESP32-CAM firmware. Don't bump it for the demo.
4. RViz **RobotModel + TF + Odometry** are tiny; only the image matters for bandwidth.

For a large room, compressed transport is effectively mandatory. Wire it into
`live.launch.py` if you standardize on it.

---

## 7. IP addressing & the micro-ROS agent (the ESP32 gotcha)

The rover firmware (`firmware/esp32_microbot`) and the camera firmware
(`firmware/esp32cam_perception`) each embed the **agent's IP** (`AGENT_IP` in `main.cpp`) and the
WiFi SSID/pass. This bites every session:

- The agent runs on the **host laptop**, so `AGENT_IP` must equal the host's **LAN IP**.
- On DHCP the host IP changes between sessions → the boards can't find the agent → **you must
  reflash**. Avoid this: give the **host a static IP / DHCP reservation** and hard-code that
  once. Our notes have seen the host jump (e.g. `10.65.205.251` → `10.185.122.251`); pin it.
- Give the **ESP32-CAM a DHCP reservation** too, so `stream_url` is stable and you don't have to
  hunt its IP each time (otherwise read it from the board's serial boot log, or the router's
  lease table).
- Ports: micro-ROS agent here is **UDP 8888** (`micro_ros_agent udp4 --port 8888`). Match the
  transport in firmware (UDP vs serial).

Recommended: a `hosts` table on the whiteboard —
```
router     10.42.0.1
host        10.42.0.10   (static)  ← AGENT_IP + discovery server
rover ESP32 10.42.0.50   (reserved)
ESP32-CAM   10.42.0.51   (reserved) ← stream_url http://10.42.0.51/stream
```

---

## 8. Recommended "known-good" classroom recipe

1. **Own router** (OpenWrt/travel), isolation OFF, one SSID, one subnet.
2. **Host laptop static IP** `10.42.0.10`; wire it by Ethernet if the router has a LAN port.
3. **DHCP reservations** for both ESP32 boards; flash `AGENT_IP=10.42.0.10` once.
4. `ROS_DOMAIN_ID=42` on the whiteboard; hand out a `setup_env.sh` (domain + Discovery Server
   `10.42.0.10:11811` + `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`).
5. Host: start `fastdds discovery`, then
   `ros2 launch perceptlive_perception live.launch.py stream_url:=http://10.42.0.51/stream agent:=true rviz:=true`.
6. Host: start compressed republish + teleop.
7. Attendees: `source setup_env.sh`, `rviz2 -d <live.rviz>` (or open the shared config), add the
   compressed Image display.
8. Verify on ONE attendee laptop before the room floods in (`ros2 topic hz /camera/image_repub/compressed`).

See `SETUP.md` for build/flash steps and `README.md` for the topic/graph summary.
