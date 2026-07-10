# ESP32-CAM perception firmware

The **real-hardware twin of the Webots camera**. It publishes the same two cheap topics the
sim's `camera_processor` does, so the `perceptbot_behaviors` light-seek (B4) and colour-seek
(B5) drive the real board unchanged — and serves the full image over WiFi for ArUco (B6).

| Path | Topic / URL | Type | Notes |
|------|-------------|------|-------|
| micro-ROS (WiFi/UDP) | `/camera/light_level` | `std_msgs/Float32` | 0..1 lit-pixel fraction, on-board |
| micro-ROS (WiFi/UDP) | `/camera/mean_color` | `std_msgs/ColorRGBA` | r,g,b 0..1, on-board |
| WiFi HTTP | `http://<board-ip>/stream` | MJPEG | full image (too big for micro-ROS) |

**Why two transports:** a scalar fits trivially in a micro-ROS UDP packet; a 320×240 image
does not. So the cheap numbers go over micro-ROS (like the microbot's ranges), and the image
goes over plain HTTP MJPEG — exactly how real ESP32-CAM projects do it. Run ArUco on a PC
that subscribes to the bridged stream; the board stays cheap.

## Build / flash / monitor
Default target = **AI-Thinker ESP32-CAM (OV2640)**; same flashing rig + gotchas as
`../esp32cam_health/` (FTDI/MB dock, `GPIO0→GND` on bare FTDI, 115200, solid 5 V). See that
README for the brltty/power/monitor notes.
```bash
cd src/05_perception/firmware/esp32cam_perception
# 1. edit CONFIG block in src/main.cpp: WIFI_SSID, WIFI_PASS, AGENT_IP (this PC's IP)
pio run -t upload
pio device monitor        # monitor_rts/dtr=0 -> opening it won't reboot the board
```

## Run end-to-end (same Agent as project 02)
```bash
# Agent on the PC (built in ~/uros_ws):
source ~/uros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
# board powered + on the same 2.4 GHz WiFi -> topics appear:
ros2 topic echo /camera/light_level        # lit-pixel fraction: ~0 normally, rises toward a bright light/window
ros2 topic echo /camera/mean_color
# full image: open http://<board-ip>/stream in a browser
```
Then run the behaviours; B4/B5 work against the board with **no code change** from the sim.

## Status — FLASHED & VERIFIED ON HARDWARE (2026-06-25)
Flashed an ESP32-CAM (CH340 MB dock, `/dev/ttyUSB0`); PSRAM 4 MB, camera OK, WiFi joined
`LSPrmn60x` (board got `10.65.205.246`), Agent on `10.65.205.251:8888`. Confirmed:
- micro-ROS: `/camera/light_level` + `/camera/mean_color` publishing via the Agent.
  (Verified value ~0.45 predates the 2026-07-10 metric change: `light_level` is now the
  **lit-pixel fraction**, ~0 in an ordinary room, rising only toward a bright light — needs a
  re-flash to match the sim; see the note below.)
- HTTP: `http://<board-ip>/stream` serves valid multipart JPEG.
- `mjpeg_bridge` → `/camera/image_raw` at 320×240 (~3.5 Hz); `aruco_detector` runs on it (show a
  printed **4x4_50 id-0** marker — `perceptbot_sim/worlds/textures/aruco_0.png` — to get B6).

**Gotchas hit:**
- **`cv2.VideoCapture` hangs** on the ESP32-CAM multipart MJPEG stream — `mjpeg_bridge` was
  rewritten to parse the stream manually (read chunks, cut JPEGs by FFD8/FFD9 markers).
- **`cam_hal: EV-VSYNC-OVF` serial spam + low (~3.5 Hz) stream fps** — the OV2640 DMA can't
  keep up with RGB565 QVGA *and* concurrent micro-ROS + MJPEG. Harmless (frames still flow);
  fine for ArUco. To speed up, capture JPEG natively for the stream (but then mean_color needs
  a separate RGB grab) or drop `PUBLISH_HZ`.
- `lsp` needed adding to the `dialout` group to flash (`/dev/ttyUSB0` was mode 660 this time).

## Notes
- Capture format is **RGB565 QVGA** (so colour can be averaged on-board); JPEG is made on
  demand for the stream via `frame2jpg`.
- **`SWAP_RGB565`** in CONFIG: if `/camera/mean_color` has red/blue swapped, flip this flag
  (RGB565 byte order varies by module).
- ArUco (B6) is not done on the ESP32 (too heavy). Bridge the MJPEG stream into ROS on the PC
  (e.g. a small `web_video_server`-style relay or an OpenCV node) and feed `aruco_detector`.
- The classic ESP32 is **2.4 GHz only** — same WiFi caveat as the microbot firmware.
