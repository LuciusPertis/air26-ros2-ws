# 05_perception — B4/B5/B6 debugging: resume notes

> ## STATUS 2026-07-10 (session 2): B4 RESOLVED + §1 remap CONFIRMED
> Branch `main`. Ran Webots headless (direct-webots path, see §4) and finished the open items:
>
> - **§1 camera remap — CONFIRMED live.** Real run: `/camera/image_raw` Publisher count 1,
>   `/camera/image_color` absent. The driver-side remap in `webots.launch.py` works.
> - **§2 B4 — FIXED.** Measured `mean_intensity` over a full spin with the new world: whole-frame
>   grayscale mean was NOT cleanly bimodal (baseline ~0.38 from the bright textured sky, panel
>   peak only ~0.52; never crossed 0.55 → still spun forever). **Decision (user-approved): change
>   the metric, not just the threshold.** `/camera/mean_intensity` is now the **fraction of
>   near-white pixels** (`gray > 200`), which is trivially firmware-computable so the ESP32-CAM
>   twin still holds. New signal: **0.00 baseline → 0.20 on-panel, cleanly bimodal.** Threshold
>   retuned `0.55 → 0.08`. Verified: B4 drives at the panel 37.8% forward from spawn (was 0%).
> - **B6 — re-confirmed** from spawn (`approach finished: reached`); the camera_processor change
>   doesn't touch the aruco path.
> - **B5 — still deferred** (whole-frame colour metric, §3). The lit-pixel-fraction idea has a
>   colour analogue (excess-green) if/when B5 is picked up.
>
> Files changed this session (uncommitted, in addition to §7):
> - `perceptbot_perception/.../camera_processor.py` — mean_intensity = lit-pixel fraction (`BRIGHT_CUT=200`)
> - `perceptbot_behaviors/.../behavior_manager.py` — `intensity_threshold` 0.55→0.08 + B4 doc line
> - `firmware/esp32cam_perception/src/main.cpp` — **firmware twin updated to match** (lit-pixel
>   fraction, integer luma). **NEEDS RE-FLASH** to the real ESP32-CAM (was last flashed 2026-06-25
>   with the old luma-mean; sim and board now differ until re-flashed).
> - Docs touched to match: `TUTORIAL.md`, `firmware/esp32cam_perception/README.md`.
>
> ## STATUS 2026-07-10 (session 2b): B4 min/max stop + topic RENAME
> Follow-up in the same session after user feedback ("B4 works, needs logic update"):
> - **B4 min/max thresholds.** `light_loop` is now 3-state: `< intensity_threshold` (0.15) spin/
>   search; `intensity_threshold..intensity_stop` drive in; `>= intensity_stop` (0.40) STOP.
>   Measured the light_level-vs-distance curve head-on (front 2.0m→0.19, 0.55m→0.38, 0.43m→0.44,
>   0.18m/at-wall→1.0). min 0.15 sits between a corner-sliver (~0.10) and a centred panel (~0.19)
>   so it stops charging at a mere corner; max 0.40 stops ~0.4-0.9m out (varies: light_level mixes
>   distance+centring) — **well clear of the wall, so no more nudging.** Verified live: acquire→
>   drive→hold-still, zero oscillation. Trade-off the user accepted: from >~2.5m the centred panel
>   is < 0.15 so B4 can spin without acquiring (bang-bang phototaxis, intensity conflates size+dist).
> - **`avoid_guard` finding:** it DOES fire (front<0.28 → spin away) but only checks the FRONT beam
>   and turns rather than halting; on its own it *causes* the nudging (turn-away then re-charge).
>   The `intensity_stop` is the real fix. NOT hardened (user chose the rename over hardening it).
> - **TOPIC RENAME `/camera/mean_intensity` → `/camera/light_level`** (user decision; the metric is
>   a lit-pixel fraction, not a mean). Renamed across **project 05 only** (code + firmware + all
>   docs + root CLAUDE.md + checkpoint marker `CHECKPOINT: light_level` + `self.light_level` +
>   `light_pub`). Params kept as `intensity_threshold`/`intensity_stop`. **Left untouched:**
>   05_perception_live (separate self-contained project, still its own `mean_intensity` + old
>   grayscale-mean metric) and 04's unrelated skimage `.mean_intensity`. **Firmware still NEEDS
>   RE-FLASH** (now also for the topic name). Verified live: `/camera/light_level` publishes,
>   behaviour reads it, B4 acquires+stops.
>
> Everything below is the original session-1 note, kept for context.

---

Session date: 2026-07-10. Branch at the time: `10-fix-02_micro_ros-firmware`
(user said "we are working on main branch" — check you're where you want to be).

Context: **B4, B5, B6 spun indefinitely.** B6 is now fixed and verified. B4 is
*partially* addressed (scene change made, **not yet measured**). B5 has a known,
separate design bug (metric doesn't discriminate colour) and is deferred.

---

## 1. Root cause of the indefinite spinning (FOUND + FIXED + VERIFIED)

In the **Webots** sim the camera frames never reached the perception nodes.

`webots_ros2_driver`'s Camera plugin **appends `/image_color`** to the device's
`topicName`. So `<topicName>/camera</topicName>` in
`perceptbot_sim/resource/perceptbot_webots.urdf` publishes **`/camera/image_color`**,
but `camera_processor` and `aruco_detector` subscribe to **`/camera/image_raw`**.

Caught live on the user's running stack:

```
/camera/image_color : Publisher count: 1   Subscription count: 0   <- webots driver
/camera/image_raw   : Publisher count: 0   Subscription count: 3   <- the perception nodes
```

Consequence: `mean_intensity` stuck at `0.0`, `mean_color` stuck at `(0,0,0)`,
`detections` stuck at `[]` → B4/B5/B6 all take their "not seen → spin to search"
branch forever.

Only Webots was affected. MuJoCo (`mujoco_driver`), Gazebo (`gz_bridge.yaml`) and the
real `mjpeg_bridge` all publish `/camera/image_raw` correctly.

### The fix (committed to the working tree, not committed to git)

`perceptbot_sim/launch/webots.launch.py` — remap on the **driver**, so all three
embodiments keep the same `/camera/image_raw` interface (this also fixes the RViz
Image display that `TUTORIAL.md` line 33 tells students to add):

```python
driver = WebotsController(
    robot_name='perceptbot',
    parameters=[{'robot_description': webots_urdf}, sim_time],
    remappings=[('/camera/image_color', '/camera/image_raw')],
    respawn=True,
)
```

### Verification actually performed

- Verified end-to-end via the **equivalent** consumer-side variant
  (`-p image_topic:=/camera/image_color` on `camera_processor` + `aruco_detector`).
  With images flowing, over 25 s per behaviour:

  | behaviour | forward | spin-only | note |
  |---|---|---|---|
  | B4 | 64.0% | 35.5% | drives at the panel |
  | B5 | 81.2% | 18.8% | drives, but see §3 — not really colour-seeking |
  | B6 | 68.4% | 28.3% | `marker_approach` logged `approach DONE: reached` |

  Also: `mean_intensity` ranged `0.161 – 0.662` (crosses the `0.55` threshold),
  25 ArUco detections seen.

- The **driver-side remap** (what's actually in the file now) was verified at the
  mechanism level only: a node publishing absolute `/camera/image_color` launched with
  `-r /camera/image_color:=/camera/image_raw` reports
  `resolved topic name: /camera/image_raw`, and `/camera/image_color` disappears.
  **A full Webots run with the remap in place was never completed** — see §4.

**TODO:** run the real launch once and confirm `/camera/image_raw` has
`Publisher count: 1` and `/camera/image_color` is absent.

---

## 2. B4 — scene changes (MADE, NOT YET MEASURED)

User's request, verbatim intent: make the white board luminous + absolute white; reduce
or remove reflectance of all other objects; **keep the world background and do NOT lower
the global light intensity**. If that makes B4 work, the reduced `mean_intensity`
scalar topic/msg format is justified. If not, fall back to **patch matching** (which
B5 probably needs too).

An earlier edit of mine wrongly removed `TexturedBackground`/`TexturedBackgroundLight`
and dimmed the `PointLight` — **that was reverted** at the user's instruction. Current
state keeps background + `PointLight { intensity 6 castShadows TRUE }` untouched.

### Why B4 was marginal even with images flowing

`mean_intensity` is a **whole-frame grayscale mean**. Measured range while spinning was
`0.161–0.662`, mean `0.479`, against `intensity_threshold = 0.55`. The frame was
dominated by things that aren't the panel:

- `TexturedBackground` — bright sky, fills the upper image because the arena walls are
  only 0.25 m tall and the camera sits at z≈0.10 m.
- `RectangleArena` defaults — `floorAppearance Parquetry{chequered}` (bright) and
  `wallAppearance BrushedAluminium{}` (**specular metal**).
- `PBRAppearance` defaults are `metalness 1, roughness 0` — i.e. a **mirror**. The
  original pillars (`baseColor 0.6 0.4 0.2`) and wheels had no overrides, so they were
  fully specular.

### Changes made to `perceptbot_sim/worlds/perceptbot.wbt`

Kept: `TexturedBackground{}`, `TexturedBackgroundLight{}`, `PointLight{intensity 6}`.

- `light_panel` → luminous absolute white:
  `baseColor 1 1 1, emissiveColor 1 1 1, emissiveIntensity 1, metalness 0, roughness 1`
- `RectangleArena` → dark matte, no image-based reflections:
  - `floorAppearance PBRAppearance{ baseColor 0.12 0.12 0.12, metalness 0, roughness 1, IBLStrength 0 }`
  - `wallAppearance  PBRAppearance{ baseColor 0.18 0.18 0.20, metalness 0, roughness 1, IBLStrength 0 }`
- pillars → `baseColor 0.22 0.15 0.08, metalness 0, roughness 1, IBLStrength 0`
- `green_box`, `aruco_marker`, chassis, wheels → added `metalness 0 roughness 1 IBLStrength 0`

Field names were checked against `/usr/local/webots/resources/nodes/*.wrl`:
`PBRAppearance` has `emissiveIntensity` and `IBLStrength`; `PointLight` has `radius`,
`attenuation`, `intensity`; `RectangleArena` exposes `floorAppearance`/`wallAppearance`
as `SFNode` (confirmed from the R2025a proto source).

Webots parses the new world fine — only two benign warnings (PointLight quadratic
attenuation advice; aruco texture 640×640 rescaled to 1024×1024).

### What still needs doing for B4

1. Launch the sim, set behaviour 4, and **sample `/camera/mean_intensity` over a full
   spin** (search_turn 0.5 rad/s → ~12.6 s per revolution). Sampler script:
   `scratchpad/sample.py` (see §5) — prints min/max/mean and how many frames clear the
   threshold.
2. Expect a **bimodal** distribution now: low baseline off-panel, clear spike on-panel.
   Then retune `intensity_threshold` (currently `0.55`) to sit between the two modes.
   Rough estimate: panel ≈14% of pixels at 2 m, so facing it ≈`0.22` vs baseline ≈`0.10`
   → a threshold around `0.15` is plausible. **Measure, don't guess.**
3. If it is *not* cleanly bimodal → go to patch matching (brightest-patch / left-vs-right
   half comparison), which also gives B4 a steering signal it currently lacks:
   `light_loop()` only ever drives straight or spins; it never steers *toward* the light.

---

## 3. B5 — separate, real bug (deferred by user: "B5 later")

`color_loop()` compares the **whole-frame mean colour** to `target_color` with
`match = 1 - euclidean_dist/sqrt(3)` and thresholds at `0.65`. This does not
discriminate green:

| scene | match | vs 0.65 |
|---|---|---|
| neutral grey `(0.33,0.33,0.33)` | 0.738 | **passes** |
| brown pillar `(0.6,0.4,0.2)` | 0.663 | **passes** |
| exact target green | 1.000 | passes |
| black `(0,0,0)` — the no-image case | 0.576 | fails |

The black row *is* the original indefinite spin. But note grey **outscores** the green
box, so raising the threshold cannot separate them — a 0.3 m cube barely moves a
whole-frame mean. Measured live: best-matching frame was `rgb=(0.340,0.306,0.278)`
(a grey wall), and 332/455 frames cleared 0.65.

Fix direction: change the metric, e.g. excess-green `g - (r+b)/2` (0 for any grey,
strongly positive for the box), or patch matching as the user suggested. This touches
the `# === CHECKPOINT: behavior_5 ===` block students read, so confirm before editing.

---

## 4. Environment gotchas that cost a lot of time (IMPORTANT)

### 4a. `TMPDIR` breaks `webots.launch.py` in this shell

Webots inherits `TMPDIR=/home/lsp/snap/code/common/tmp` (from the **VS Code snap**
environment) and creates its IPC dir at `$TMPDIR/webots/lsp/1234/ipc`. But
`ros2_supervisor.py` and `webots-controller` look in **`/tmp/webots/lsp/1234`**.
Result: both die after ~50 s with

```
Cannot open directory /tmp/webots/lsp, retrying...
Cannot connect to Webots instance, retrying...
```

This is **purely an artifact of the VS Code / Claude Code shell**. A plain user terminal
has no snap `TMPDIR`, which is why the user's own launch worked fine.

**Workaround:** `export TMPDIR=/tmp; unset WEBOTS_TMPDIR` before launching. Also
`export WEBOTS_HOME=/usr/local/webots` (else `webots-controller` prints
`Set the path to your webots installation folder in WEBOTS_HOME`).

The prepared script `scratchpad/launch_sim.sh` does all of this — **it was never run**;
that's the immediate next step.

### 4b. `pkill -f webots` kills its own shell

Any `bash -c` whose command line contains the pattern (e.g. it also mentions
`/usr/local/webots/webots`) is matched by `pgrep -f`/`pkill -f` and killed. This silently
aborted several tool calls (exit 1 / 144 with no output). Use `scratchpad/cleanup.sh`,
which skips `$$`/`$PPID` and builds the pattern from spliced strings.

### 4c. Don't `rm -rf /tmp/webots` while Webots is running
It won't recreate it and the controller can never connect.

### 4d. Killing a launch-managed node shuts the whole launch down
`kill`ing `camera_processor` took `behaviors.launch.py` down with it, leaving a stale
`behavior_manager` alive. That produced **6 publishers on `/cmd_vel`** and a ~19.5 Hz
command rate (two managers at 10 Hz), which briefly made B6 look broken. Check
`ros2 topic info /cmd_vel` — expect **3** publishers (`behavior_manager`,
`obstacle_services`, `marker_approach`) and ~10 Hz.

---

## 5. Scratch scripts (recreate if the scratchpad is gone)

Under `/tmp/claude-1000/-home-lsp-air26-ros2-ws/272ba891-b31a-42ed-b6a6-46d87d278043/scratchpad/`:

- `cleanup.sh` — safe kill of webots/perceptbot/ros2 launch + `rm -rf /tmp/webots`
- `launch_sim.sh` — `TMPDIR=/tmp` + `WEBOTS_HOME` + `ros2 launch perceptbot_sim webots.launch.py`
- `direct_webots.sh` — run webots alone on the installed world
- `attach.sh` — attach `webots-controller` + `camera_processor` + `aruco_detector` to a running webots
- `sample.py` — subscribes `/camera/mean_intensity`, `/camera/mean_color`,
  `/aruco/detections` for 30 s; prints min/max/mean + threshold pass counts
- `check.py` — cycles behaviours 4→5→6 via `/set_behavior`, counts `/cmd_vel`
  forward vs spin-only frames for 25 s each

---

## 6. Immediate next steps

1. `colcon build --packages-select perceptbot_sim` (world + launch already edited).
2. Run `scratchpad/launch_sim.sh` (or from a normal terminal:
   `ros2 launch perceptbot_sim webots.launch.py use_rviz:=true`).
3. Confirm the §1 fix: `/camera/image_raw` → `Publisher count: 1`,
   `/camera/image_color` absent.
4. `ros2 launch perceptbot_behaviors behaviors.launch.py`, then
   `ros2 service call /set_behavior perceptbot_interfaces/srv/SetBehavior "{behavior: 4}"`.
5. Run `sample.py` → is `mean_intensity` bimodal now? Retune `intensity_threshold`.
6. Decide: scalar `mean_intensity` justified, or move B4 (and B5) to patch matching.

## 7. Files changed so far (uncommitted)

- `src/05_perception/perceptbot_sim/launch/webots.launch.py` — driver remap (§1)
- `src/05_perception/perceptbot_sim/worlds/perceptbot.wbt` — luminous panel + matte,
  low-reflectance surfaces (§2)

Nothing else was modified. No `behavior_manager.py` changes were made.
No processes should be left running (`cleanup.sh` was run last).
