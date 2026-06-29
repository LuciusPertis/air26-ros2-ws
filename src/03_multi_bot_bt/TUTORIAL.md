# Project 03 — Multi-bot Behaviour-Tree patrol: walkthrough

Three copies of your project-05 rover. Each runs the **same Behaviour Tree** brain; they
coordinate by **seeing each other's ArUco markers** + a little DDS chatter. We build up in
phases: **random walk → patrol (no recovery) → patrol (with recovery)**. Read `THEORY.md`
first (FSM vs BT, Nav2's BT, DDS discovery).

## 0. Build & launch
```bash
cd ~/air26-ros2-ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select multibot_interfaces multibot_description \
  multibot_perception multibot_bt multibot_sim
source install/setup.bash
ros2 launch multibot_sim patrol.launch.py            # default: convoy, recovery on
```
Webots opens with three units (r1/r2/r3) + a big ArUco **world anchor** on the far wall. Each
unit wears markers on its **back** and **right**. The `formation:`, `enable_recovery:` and
`debug_tree:` launch args drive the phases below.

## 1. See the namespaced fleet (DDS discovery)
```bash
ros2 topic list | grep -E '/r[123]/'      # three identical stacks, namespaced
ros2 node list                            # /r1/patrol_bt, /r2/aruco_pose_detector, ...
ros2 topic echo /r2/aruco/detections      # r2 sees id 10 (= r1 back) and 99 (anchor)
```

## 2. Phase 1 — random walk (the baseline)
```bash
ros2 launch multibot_sim patrol.launch.py formation:=random
```
All three just **wander independently**, each picking a new random heading every couple of
seconds and **turning away when an ultrasonic sees a wall** (the BT's top-priority `safety`
branch). No leader, no formation — the simplest behaviour, and the obstacle-avoidance you'll
keep underneath everything else.

## 3. Phase 2 — patrol WITHOUT recovery (see the problem)
```bash
ros2 launch multibot_sim patrol.launch.py formation:=convoy enable_recovery:=false
```
Now they form a **column**: the leader (whoever's at the front) sweeps the arena; each
follower holds a fixed gap to the **back marker** of the unit ahead, with distance from a
**fused** front-ultrasonic + ArUco estimate (fast US for smooth control, ArUco to confirm it's
really the leader). **Now make a follower lose its leader** (drive it off, or block the view):
with recovery off, the lost unit **wanders away** as if it were its own leader. That's the gap
— watch it happen, then turn recovery on.

## 4. Phase 3 — patrol WITH recovery (the fix)
```bash
ros2 launch multibot_sim patrol.launch.py formation:=convoy        # recovery on by default
```
Lose a follower again. This time it runs **`SearchAndRecover`**: a **360 spin** scanning for
*any* peer marker or the world anchor, creeping outward between spins, until it reacquires —
then it rejoins instantly (the BT re-checks priorities every tick). Leadership is decided from
the **world anchor** (each unit publishes `anchor_range`; nearest = leader), so a lost unit
*knows* it isn't the leader and searches — and after a column turnaround the new front unit's
search reacquires the anchor and **auto-promotes** to leader.

## 5. Parallel patrol (abreast)
```bash
ros2 launch multibot_sim patrol.launch.py formation:=parallel
```
Side-by-side now. A forward camera can't see a true side neighbour, so each follower **hands
off between sensors**: the **right marker** identifies the neighbour, then — once it's at 90°
and out of view — the **side ultrasonic** holds the lateral gap, while the unit **matches the
leader's broadcast velocity** (`/r1/formation/anchor`) to stay level.

## 6. Switch behaviour live (one unit or all)
```bash
ros2 service call /r3/set_formation multibot_interfaces/srv/SetFormation "{formation: parallel}"
ros2 service call /r2/set_formation multibot_interfaces/srv/SetFormation "{formation: random}"
```

## 7. Visualise the Behaviour Tree live
Each `patrol_bt` publishes py_trees snapshot streams, so you can watch the tree tick in real
time and see which branch is active (green = running):
```bash
# live ASCII tree for one unit (Ctrl-C to stop):
py-trees-tree-watcher --namespace /r1/patrol_bt
```
No extra tools? Just log it from the node:
```bash
ros2 launch multibot_sim patrol.launch.py debug_tree:=true   # each unit prints its tree /1s
```
Or render a static diagram of the structure:
```python
import py_trees
# (build the root as in patrol_bt.py, then:)
py_trees.display.render_dot_tree(root)   # -> patrol_root.svg/.png/.dot
```
> `rqt_py_trees` (GUI) also works if you `apt install ros-jazzy-rqt-py-trees`; the terminal
> watcher above needs nothing extra.

## 8. The Behaviour Tree
Open `multibot_bt/patrol_bt.py`. The root is
`Selector[ safety→Avoid , patrol(InPatrolMode→[follow , recover , Lead]) , RandomWalk ]`.
Safety is first (always wins); the `patrol` subtree only runs in a convoy/parallel formation,
else it falls to `RandomWalk`. Inside patrol it's `follow → recover → Lead`. That whole
leader/follower/lost/wander machine is just **priority order** — an FSM written as a BT, with
each phase added as one branch (no transition rewiring). Tunables (gains, `follow_distance`,
`lateral_gap`, `search_turn`, `search_timeout`, `walk_period`) sit at the top.

## 9. Make it yours (checkpoints)
- `markers` block in the xacro → remove a marker face.
- Swap the leader's `Lead` sweep for a **Nav2 waypoint follower** subtree (the Nav2 touch).
- Add a 4th unit: add it to `patrol.wbt` + the `UNITS` list in `patrol.launch.py` — DDS +
  namespacing handle the rest.

> Localization is **Tier-1** (markers → named relative positions + world anchor). A full EKF
> (`world→odom`) is a documented hook in `relative_localizer.py` if you want to go further.
