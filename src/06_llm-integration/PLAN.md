# Workshop 06 — Plan & design notes (LLM integration)

## Goal

One natural-language movement command → a local Ollama LLM with **native
tool-calling** → robot motion, in **MuJoCo (simulation only)**. Demonstrated on
two existing robots:

1. **micro_ros rover** (project 02) — base only.
2. **Stretch SE3** (project 04) — base + lift + arm + head + gripper.

The demo terminal publishes **one** instruction on `/llm/command`; the agent runs
once and the robot moves. Two standalone warm-ups (a chat REPL and a tool-API
script) precede the robot demo.

## Decisions (locked with the user)

| Question | Decision |
|----------|----------|
| Tool mechanism | **Native Ollama tool-calling** (`tools=[…]`, model returns `tool_calls`). |
| Stretch scope  | **Base + arm/lift/head/gripper**. |
| Interaction    | **One-shot** — one command per publish; node persists for re-runs. |
| Motion style   | **Hybrid** — raw `drive(linear, angular, duration)` for base; discrete primitives (`move_forward`, `set_lift`, `set_arm`, `set_head`, `set_gripper`) via an executor for the rest. |
| Reply          | The model's closing natural-language sentence is published on **`/llm/response`** (std_msgs/String) for display as the answer to each command. |
| No-action      | A shared **`respond(answer)`** tool (added centrally in `robot_base.py`) lets the model answer questions about state **without moving**; it short-circuits the agent loop and its answer becomes the reply. |
| Packaging      | **One package** `llm_integration` — the agent core/prompt/command-node are robot-agnostic; each robot is a thin adapter, so there's no duplication (the user's "keep together if clean" condition holds). |
| Model          | Default **`qwen3:1.7b`** (fast on CPU, ~2-3 s warm); `qwen3:4b` selectable via `model:=` (more accurate directions, much slower). Both pulled. |
| CPU tuning     | `think=False` (qwen3 reasoning is ruinous on CPU) + `temperature=0` (deterministic, curbs duplicate-call floods). Agent **recovers tool calls printed as JSON text** and **de-duplicates** repeated calls. |

## Architecture

```
llm_integration/
  ollama_agent.py   shared  ── chat-with-tools loop; backends: 'ollama' | 'mock'
  prompt.py         shared  ── system prompt (robot+env desc) + user msg (cmd + state JSON)
  robot_base.py     shared  ── RobotInterface(Node): /llm/command sub, MT-executor agent run
  robots/micro.py   adapter ── describe/tools/state_json/dispatch for the 02 rover
  robots/stretch.py adapter ── same for Stretch (cmd_vel + FollowJointTrajectory + mode switch)
  demos/chat_terminal.py, demos/ollama_api_demo.py   standalone, no ROS
  launch/micro.launch.py, launch/stretch.launch.py   include 02/04 sims + controller
```

### Concurrency

The agent loop blocks (model call + motion). The node runs under a
`MultiThreadedExecutor` with a `ReentrantCallbackGroup`, so while the command
callback is blocked inside a motion, other executor threads service the
action-client futures and state subscriptions. This is what lets Stretch's
`FollowJointTrajectory` calls complete without a re-entrant-spin error
(`spin_wait()` in `robot_base.py` polls `future.done()` instead of nested
spinning).

### Dependency on 02 / 04

Satisfied by the launch files: `micro.launch.py` includes
`microbot_sim/mujoco.launch.py`; `stretch.launch.py` includes
`stretch_se3_bringup/sim.launch.py`. `exec_depend`s are declared in `package.xml`.
Launching 06 brings up the corresponding sim.

## Safety

- Tool arguments are clamped: velocities/durations capped (`MAX_DURATION`), joint
  targets clamped to soft limits (mirrored from
  `stretch_se3_control/stretch_trajectory.py`).
- The model can only call the provided tools; it never emits raw motor values.
- Open-loop timed primitives (`move_forward`, `turn`) convert distance/angle to a
  duration at a fixed nominal speed — simplest mapping onto `/cmd_vel`.

## Status

- **Built** (`colcon build --packages-select llm_integration`) — clean, lint-clean.
- **Verified headless with `backend:=mock`:**
  - micro: `/llm/command` → tool call → `/cmd_vel` motion; live odom + ultrasonic
    state fed to the agent.
  - stretch: `/llm/command` → mode switch → `FollowJointTrajectory` → lift moved
    0.59 → 0.94 m on a "raise the lift to 0.95" command.
- **Real Ollama path verified** (`backend:=ollama`, `qwen3:1.7b`) on the micro
  rover:
  - movement: *"turn left 90 then drive forward half a meter"* → `turn(90)` +
    `move_forward(0.5)` executed; reply on `/llm/response`: *"You turned left 90
    degrees and then moved forward half a meter."* (~11 s cold, ~2-3 s warm).
  - question: *"how close is the wall in front of you?"* → `respond` →
    *"The wall is approximately 0.73 metres front of you."*, **no `/cmd_vel`** (~2 s).
  - Ollama installed here (v0.30.11, systemd service); `qwen3:1.7b` + `qwen3:4b`
    pulled.
- The real **Stretch** path is verified via `backend:=mock` (joint motion + mode
  switch); its `backend:=ollama` run is the same agent core as micro (not separately
  re-run here).

## Deferred / possible extensions

- Interactive REPL mode for the robot (currently one-shot per the spec; the chat
  warm-up covers conversational use).
- Perception-aware tools (reuse project 05 camera topics) — out of scope (movement
  only).
- A fine-tuned or larger local model for more reliable multi-step plans.
