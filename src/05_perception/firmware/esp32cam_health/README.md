# ESP32-CAM health check

A standalone bring-up test to confirm an **ESP32-CAM** module actually works — camera,
PSRAM, and sensor — before building anything on it. No micro-ROS, no ROS: just flash and
read the serial console. Lives outside colcon (`../COLCON_IGNORE`).

Default target: **AI-Thinker ESP32-CAM (OV2640)**. For other modules, swap the pin-map
block in `src/main.cpp`.

## What it reports (serial, 115200)
```
========== ESP32-CAM HEALTH CHECK ==========
[esp32cam_health] chip: ESP32-D0WD rev3, 2 core(s), 240 MHz
[esp32cam_health] flash: 4096 KB
[esp32cam_health] PSRAM: FOUND (4096 KB)
[esp32cam_health] camera init: OK
[esp32cam_health] sensor: OV2640 (PID 0x26)
[esp32cam_health] 320x240  76800 bytes  mean=118 min=2 max=255  21.4 fps
```
- **PSRAM: NOT FOUND** → often a clone/fake board, or bad solder. Camera still inits but is
  limited to small frames.
- **camera init: FAILED (err 0x….)** → reseat the ribbon cable, give it a solid **5 V**
  supply (the 3.3 V pin can't power the sensor reliably), and confirm it's an AI-Thinker
  board (else fix the pin map).
- **`mean` brightness** is the live proof: **cover the lens → mean falls toward 0**, point it
  at a light → mean climbs toward 255. If `mean` reacts to light, the sensor is healthy.
- onboard **red LED (GPIO33)** blinks per frame = liveness; the **white flash LED (GPIO4)**
  pulses once at boot (checkpoint `flash_led_test`).

## Flashing (ESP32-CAM has NO USB)
Use an **FTDI / USB-TTL adapter** (or the **ESP32-CAM-MB** dock):

| FTDI (3.3 V logic) | ESP32-CAM |
|--------------------|-----------|
| 5V                 | 5V        |
| GND                | GND       |
| TX                 | U0R (RX)  |
| RX                 | U0T (TX)  |

To enter download mode on a bare FTDI: **jumper `GPIO0` → `GND`**, then reset/power-cycle.
Flash, then **remove the jumper and reset** to run. The ESP32-CAM-MB dock does this for you.

```bash
cd src/05_perception/firmware/esp32cam_health
pio run                              # compile
pio device list                      # find the adapter's port (FTDI/CH340 -> /dev/ttyUSB0)
pio run -t upload                    # flash (115200; GPIO0->GND first on bare FTDI)
pio device monitor -p /dev/ttyUSB0   # watch the health report
```
> Toolchain: PlatformIO (`espressif32` platform). Per-user core dir lives in
> `~/.platformio` — run `pio` as the same user that flashes (`pio run` auto-installs the
> platform on first use). Upload kept at **115200**; ESP32-CAM is unreliable faster.

## Troubleshooting (hit on the dev box 2026-06-24)
- **No `/dev/ttyUSB0`; pio auto-detects `/dev/ttyS0` (the onboard serial) and fails** — on
  Ubuntu 24.04 the **`brltty`** braille driver hijacks **CH340** adapters: the port appears
  then is instantly disconnected (`dmesg`: "interface 0 claimed by brltty"). Fix:
  `sudo apt-get remove brltty`, then unplug/replug the adapter. (CP2102 adapters are
  unaffected — only CH340/CH341.)
- **esptool connects + reads the chip/MAC, then dies mid-write** with `Failed to communicate
  with the flash chip` / `Packet content transfer stopped` — this is **power/brownout**, not
  wiring (RX/TX are clearly fine since it read the chip). The OV2640 + WiFi need more current
  than a 3.3 V adapter rail gives during flash erase/write. Fixes, in order: power the board's
  **`5V` pin from the adapter's 5V** (set the adapter's 3.3V/5V jumper to **5V**) or an
  external 5 V with shared GND; **retry** (often works 2nd/3rd try); short/thick jumpers, a
  rear USB port (not an unpowered hub); a **470 µF cap across 5V–GND** for a durable fix. The
  `flash_mode=dio` + `f_flash=40MHz` settings in `platformio.ini` also make the write gentler.
- **Don't flash with `sudo`** — with `brltty` gone, udev gives the port mode `666` so plain
  `pio run -t upload` works as your user; `sudo` re-roots the `.pio` cache.
- **`upload_speed`** — default 115200. If a board flashes flakily, *lower* it (57600, or 9600
  worst case) for more tolerance of marginal links; raising it (460800+) is faster but less
  reliable. A stub-level "Failed to communicate with the flash chip" is power/connection, not
  baud — changing the speed won't fix that; reseat the module + fix 5 V instead.
- **A board that repeatedly won't flash** (stub can't reach the flash) on a good dock + solid
  5 V + reseated module = likely **bad flash/solder → that board fails the health check.**
- **Monitor reboots the board on open/Ctrl+C (white LED flashes) + `␀␀␀␀` garbage** — the
  monitor toggles DTR/RTS, which drive EN+GPIO0 on the MB dock → reset; each reset prints the
  ROM bootloader at **74880 baud**, misread as nulls in a 115200 monitor. Fixed with
  `monitor_rts = 0` / `monitor_dtr = 0` in `platformio.ini`. Press the dock **RST** button to
  see the boot banner while attached. Reset-free alternatives: `screen /dev/ttyUSB0 115200`
  (exit Ctrl-A K Y) or `picocom -b 115200 /dev/ttyUSB0`.

## Notes
- `board = esp32cam` in `platformio.ini` already enables PSRAM; the `BOARD_HAS_PSRAM`
  build flag + cache-issue workaround are set explicitly too.
- This is a **checkup tool**, not the perception demo. A future `05_perception` ROS package
  (camera → image topic / web stream / detection) would build alongside this `firmware/`.
- To see an actual video stream instead of numbers, the Arduino **CameraWebServer** example
  (`esp32` board package) is the standard next step — out of scope for a pass/fail check.
