# Workshop 06 — LLM Integration: talk to your robot

**Goal:** say what you want in plain English; a local LLM turns it into safe
movement commands. Simulation only (MuJoCo). You already met the robots in
projects 02 (the rover) and 04 (Stretch) — here we put a language model in front
of them.

> Prereq: Ollama installed + `qwen3:1.7b` pulled + `pip install ollama`
> (see `SETUP.md`). No GPU? Add `backend:=mock` to every launch below to run the
> plumbing without a model.

---

## Phase 0 — warm up with raw Ollama (no ROS)

Before any robot, see the LLM by itself.

```bash
ros2 run llm_integration chat_terminal
```

A plain chat REPL. Confirm the model answers. Type `exit` to quit.

Now give the model **tools** and watch it pick one:

```bash
ros2 run llm_integration ollama_api_demo "turn right 90 degrees then drive forward 2 meters"
```

You'll see something like:

```
Model chose 2 tool call(s):
  - turn(angle_deg=-90)
  - move_forward(distance=2.0)
```

That is the whole trick: we *describe* movement functions, the model *fills in the
arguments*. The robot nodes do exactly this, then execute the calls.

---

## Phase 1 — the rover (project 02)

```bash
ros2 launch llm_integration micro.launch.py
```

This brings up the **project-02 MuJoCo rover** (with its ultrasonics and odom) *and*
the LLM controller in one shot. In a second terminal, watch the model's replies:

```bash
ros2 topic echo --field data /llm/response
```

In a third, publish one instruction:

```bash
ros2 topic pub --once /llm/command std_msgs/String "data: 'drive forward a little, then turn left'"
```

Watch the controller terminal: it prints the robot **state** it sent to the model,
each **tool call** the model returned, and the result. The rover moves in the
viewer/RViz, and a natural-language summary of what it did appears on
`/llm/response`.

Try a few:

```bash
ros2 topic pub --once /llm/command std_msgs/String "data: 'back up half a meter'"
ros2 topic pub --once /llm/command std_msgs/String "data: 'spin in place to the right'"
ros2 topic pub --once /llm/command std_msgs/String "data: 'go forward until you are close to something'"
```

The last one is interesting: the model sees the front ultrasonic range in the
state JSON and can reason about it.

**Ask it a question** — the robot answers without moving. The model has a special
`respond` (no-action) tool for questions about its state:

```bash
ros2 topic pub --once /llm/command std_msgs/String "data: 'how close is the wall in front of you?'"
ros2 topic pub --once /llm/command std_msgs/String "data: 'where are you right now?'"
```

No `/cmd_vel` is sent; the answer (derived from the state JSON) appears on
`/llm/response`. This is the difference between *commanding* and *querying* the
robot in plain language.

**What the model is given** (system prompt + per-command message):
- a natural-language description of the rover and its arena (`robots/micro.py`,
  `describe()`),
- the tool schemas (`tools()`),
- the live state JSON: `{"pose":{x,y,yaw_deg}, "ultrasonic_m":{front,left,right}}`.

### Checkpoint: disable the LLM controller

Open `launch/micro.launch.py`, comment out the block between
`# === CHECKPOINT: llm-controller ===` and its `END`, rebuild, relaunch. Now only
the bare sim runs and `/llm/command` does nothing — the language layer is gone.
Restore it and it's back.

---

## Phase 2 — the manipulator (project 04 Stretch)

Same idea, a richer body: base **plus** lift, telescoping arm, head and gripper.

```bash
ros2 launch llm_integration stretch.launch.py
```

```bash
ros2 topic pub --once /llm/command std_msgs/String "data: 'raise the lift up high and look down at the table'"
ros2 topic pub --once /llm/command std_msgs/String "data: 'extend the arm halfway and open the gripper'"
ros2 topic pub --once /llm/command std_msgs/String "data: 'lower the lift and drive forward slowly'"
```

Here the model chooses among more tools — `set_lift`, `set_arm`, `set_head`,
`set_gripper` and base `drive`. Note the **two control surfaces**: the base is a
velocity command (`/stretch/cmd_vel`, needs *navigation* mode) while the joints are
position goals (`FollowJointTrajectory`, *position* mode). The controller switches
modes automatically per tool — the model never has to know.

> You will see a red `left_wheel_vel` error from the Stretch driver after each
> joint move. It is a harmless pre-existing upstream quirk (the joint still
> reaches its target); see `SETUP.md §5`.

---

## How one command flows (the whole pipeline)

```
ros2 topic pub /llm/command "raise the lift"
        │
robot_base.RobotInterface._on_command         # one-shot, in a MT executor
        │  build_system_prompt(describe())     # prompt.py
        │  build_user_message(cmd, state_json())
        ▼
ollama_agent.run_agent(... tools=tools(), dispatch=dispatch)
        │  ollama.chat(model, messages, tools)  → tool_calls   # native tool-calling
        ▼
robots/stretch.StretchInterface.dispatch('set_lift', {'height':0.9})
        │  switch_to_position_mode  →  FollowJointTrajectory
        ▼
MuJoCo: the lift rises.   model's closing sentence → /llm/response
```

For a *question* ("how high is the lift?"), the model instead calls the `respond`
tool with a natural-language answer; no motion is issued and the answer is
published on `/llm/response`. `respond` is added centrally in `robot_base.py`, so
the per-robot adapters only ever declare movement tools.

---

## Things to try / discuss

- **Prompt engineering:** edit `describe()` in `robots/micro.py` (e.g. tell it the
  arena is small) and see behaviour change. The state JSON is deliberately terse,
  like real telemetry.
- **Tool design:** the hybrid split — raw `drive(linear, angular, duration)` for
  fluid motion vs discrete `move_forward`/`set_lift` primitives — mirrors a real
  robot API. When would you expose raw velocity vs a named primitive?
- **Safety:** tools clamp to joint/velocity limits and durations, so a bad LLM
  output can't fault the driver. The model can *only* call the tools you give it.
- **Small-model reality:** the default `qwen3:1.7b` is fast on CPU (~2-3 s) but
  occasionally picks the wrong direction (e.g. "turn right") or prints a tool call
  as plain text. The agent recovers text-form calls and de-duplicates repeats, but
  some mistakes are inherent to tiny local tool-callers — that's part of the lesson.
  Try `model:=qwen3:4b` for better reasoning at the cost of much slower replies.

- **Model choice:** `ros2 launch llm_integration micro.launch.py model:=qwen3:4b`
  swaps the model with no code change; `think:=true` re-enables qwen3's reasoning
  (accurate but very slow on CPU). See `SETUP.md §2`.
