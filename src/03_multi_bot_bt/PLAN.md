# Project 03 — Multi-bot Behaviour-Tree patrol: plan

## Goal
Three namespaced units of the project-05 rover patrol together, coordinated by a per-unit
**Behaviour Tree** (py_trees), in **two switchable formation styles**. Self-contained
(workshop rule #1) — 03 does not modify 05.

## The robot
Project-05 perceptbot (skid-steer, 3 ultrasonics, front camera) **+ ArUco panels on its
BACK and RIGHT faces**. Marker id = `unit*10 + face` (face 0 = back, 1 = right): r1 →10/11,
r2 →20/21, r3 →30/31. A fixed **world-anchor marker (id 99)** anchors the shared frame.

## Two patrol styles, one Behaviour Tree
Per-unit tree (ticked 10 Hz):
`Selector[ safety→Avoid , patrol(InPatrolMode→[follow→Hold , recover→Search , Lead]) , RandomWalk ]`.
Three modes via the `formation` param/service: **random** (wander + obstacle avoidance — the
baseline), **convoy**, **parallel**. Safety (obstacle avoidance) is always the top priority;
`RandomWalk` is the fallback when not in a patrol formation. A tutorial builds up:
**random → patrol (no recovery) → patrol (+ recovery)** (`enable_recovery:=false|true`).
What HoldFormation does:

**1. Convoy (column) — emergent leader, fused distance.**
- Follower tracks the **back marker** of the unit ahead.
- Distance = **fused** front-ultrasonic (fast, smooth, low-latency) + ArUco range (slow,
  jittery, but *identifies* the leader and recalibrates). ArUco gates identity ("the thing
  ahead is my leader, not a wall").
- **Leadership is emergent**: see no back-marker ahead → I'm the leader. When the column
  reverses at a wall, the new front unit auto-promotes.

**2. Parallel (abreast) — velocity-match + side-US hold (sensor handoff).**
A forward camera can't see a true side neighbour, so we hand off between sensors:
`ACQUIRE` (right marker → identify neighbour) → `SLOT-IN` → `ABREAST-HOLD` (neighbour at 90°,
camera-blind → **side ultrasonic** holds the lateral gap) → re-acquire. Along-track stays
together by **matching the leader's broadcast velocity** (`/<leader>/formation/anchor`,
frame-free — no shared map needed). Leader is elected (`leader_ns`, default r1).

**Recovery (lost → search).** A unit that should be in formation but can't see its reference
runs **`SearchAndRecover`**: a vision-only 360 spin to reacquire *any* peer marker or the world
anchor, with an expanding creep between spins and a timeout → `Lead`. The moment a reference
reappears, the higher-priority follower branch resumes. **Leader-vs-lost is anchor-based**: each
unit publishes its distance to the world anchor (`anchor_range`); the nearest = "frontmost" =
leader. So a lost follower (not frontmost) searches instead of wandering off, and after a column
turnaround the new front unit's search reacquires the anchor, finds it's frontmost, and
auto-promotes. (Direction caveat: "frontmost = nearest the anchor" assumes the anchor marks the
patrol front; the bidirectional-reversal sign is a tuning point for the display.)

## Tier-1 localization (EKF deferred)
`aruco_pose_detector` solves each marker's pose (solvePnP, known intrinsics + size) → pose +
TF. `relative_localizer` turns ids into **named peer positions in the unit's base frame** (+
the world-anchor frame), published as TF + a `peers` PoseArray. Light and intrinsics-exact in
sim. **Tier-2** (a full EKF fusing odom + marker orientation into a globally-consistent
`world→odom`) is left as a documented hook — not needed for the patrol.

## Packages (`src/03_multi_bot_bt/`)
- `multibot_interfaces` — `SetFormation.srv` (convoy|parallel).
- `multibot_description` — unit xacro (+ back/right marker links) + Webots `patrol.wbt`
  (3 namespaced units r1/r2/r3 + world-anchor marker + textures).
- `multibot_perception` — `aruco_pose_detector` (pose + TF), `relative_localizer` (named
  peer poses, world anchor).
- `multibot_bt` — `patrol_bt` (py_trees: safety/follower/leader + the two formations),
  `formation_anchor` (leader velocity broadcaster).
- `multibot_sim` — self-namespacing Webots driver plugin (`multibot_driver`, derives ns from
  the robot name) + `patrol.launch.py` (world + per-unit stack, `formation:=convoy|parallel`).

## Visualising the BT (live, in-session)
`patrol_bt` wraps the tree in `py_trees_ros.trees.BehaviourTree`, so it publishes snapshot
streams: `py-trees-tree-watcher --namespace /r1/patrol_bt` shows r1's tree ticking live (active
branch highlighted). `debug_tree:=true` logs each unit's ASCII tree once a second;
`py_trees.display.render_dot_tree(root)` makes a static diagram; `rqt_py_trees` (apt) is the GUI.

## Theory mapping
FSM (random/leader/follower/lost) → BT priority; Behaviour Trees (py_trees + live watcher; Nav2
BT touch); DDS discovery + namespacing (3 units + the anchor topic). See `THEORY.md`.

## Status (2026-06-25)
Built + **verified headless**: full 3-unit Webots run (xvfb) — namespaced cmd_vel/ultrasonic/
camera/aruco/peers; marker chain resolves (r2 sees id10+anchor99, r3 sees id20); TF + all 3
py_trees BTs run; leader sweeps + turns at walls. **Recovery logic unit-tested** (mock ctx):
genuine leader drives, follower-with-ref holds, **lost follower spins (search)**, **turnaround
unit auto-leads via anchor**, safety preempts. **For the user's display:** GUI run, RViz (3
robots + anchor), formation tightness tuning, convoy↔parallel live switch, and the
reversal-direction sign for anchor leadership. Tunables in `patrol_bt.py`.
