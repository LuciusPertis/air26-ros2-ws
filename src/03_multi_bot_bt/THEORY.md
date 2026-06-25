# Project 03 — Multi-bot Behaviour Trees: theory

Background for the multi-robot patrol day. Demo runs in **Webots** with three namespaced
units of the project-05 rover.

---

## 1. Finite State Machines (FSM)

An FSM is a set of **states** with **transitions** fired by events/conditions. Classic for
robot behaviour: `PATROL → OBSTACLE → ESCAPE → PATROL`. Each unit here has an implicit FSM:
**LEADER / FOLLOWER / AVOID**.

Strengths: simple, predictable, easy to draw. Weakness: transitions explode as states grow
(every state may need an edge to every other), and reuse is poor — logic gets tangled. That
pain is exactly what Behaviour Trees fix.

## 2. Behaviour Trees (BT) — and a touch of Nav2

A **Behaviour Tree** is a tree of nodes **ticked** top-down at a fixed rate. Leaf nodes are
**Actions** (do something, return SUCCESS/FAILURE/RUNNING) and **Conditions** (check, return
SUCCESS/FAILURE). Internal nodes compose them:

- **Sequence** — run children in order; fail fast (AND).
- **Selector / Fallback** — try children until one succeeds (OR); the basis of priorities.
- **Parallel, Decorators (inverter, retry, timeout)** — modifiers.

Why BTs beat FSMs at scale: **modularity** (subtrees are reusable), **reactivity** (every
tick re-evaluates priorities — safety always wins), and **no transition explosion** (priority
is just node order). Our per-unit tree:

```
Selector "patrol"
├── Sequence "safety"   : FrontBlocked?  → Avoid          # highest priority, always checked
├── Sequence "follower" : HaveReference? → HoldFormation
├── Sequence "recover"  : ShouldFollow?  → SearchAndRecover  # lost my ref → 360 search
└── Lead                                                  # fallback: I'm genuinely the leader
```

The leader/follower/lost **FSM is encoded as BT priority** — that's the lesson: a BT *is* a
clean way to write an FSM that stays maintainable. Adding "recovery" was just inserting one
prioritised branch — no transition rewiring (the FSM-explosion an equivalent state machine
would suffer). Reactivity matters too: every tick re-checks from the top, so the instant a
searching unit reacquires its reference the higher "follower" branch takes back over.

**Nav2 touch:** Nav2 (the ROS 2 navigation stack) is driven by a **Behaviour Tree** too —
its *BT Navigator* ticks an XML tree of nodes like `ComputePathToPose`, `FollowPath`,
`Spin`, `Wait`, with recovery fallbacks. Same idea, bigger nodes. Nav2 uses
**BehaviorTree.CPP** (+ the **Groot** visual editor); we use **py_trees** (Python) because
it's lighter to read and extend in a workshop. A natural extension is to make the leader's
patrol a Nav2 *waypoint-follower* subtree instead of our simple sweep.

## 3. DDS discovery & namespacing

ROS 2 has no master — nodes find each other over **DDS discovery**: each participant
multicasts "here I am / here's what I publish", peers match topics by name + type. Run three
robots and they *all* discover each other automatically on the same `ROS_DOMAIN_ID`.

To keep three identical robots from clashing, we give each a **namespace** (`r1/ r2/ r3/`):
relative topic/frame names get prefixed → `/r1/cmd_vel`, `/r2/camera/image_raw`,
`r3/base_link`. That's how one launch file runs N copies of the same stack without collisions.
Inter-robot coordination is then just a namespaced topic: the leader publishes
`/r1/formation/anchor` and followers subscribe to it — DDS wires it up across the "robots".

Knobs worth knowing: **`ROS_DOMAIN_ID`** (isolate whole fleets onto separate buses),
**namespaces** (isolate units within a fleet), and that discovery is multicast by default
(on real multi-machine setups you may need a **Discovery Server** or unicast peers list).

---

## How the markers tie it together
Each unit wears ArUco markers (back + right) = a **named, locatable beacon** (id = `robot*10 +
face`; 99 = a fixed world anchor). Seeing a marker gives both **identity** ("that's r1's back")
and **pose** (Tier-1 localization, `multibot_perception`). The BT consumes that + the
ultrasonics + the DDS anchor to patrol — see `PLAN.md` for the two formation styles.
