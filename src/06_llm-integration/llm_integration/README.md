# `llm_integration` — natural-language robot control via a local LLM (Workshop 06)

Turn one English sentence into robot motion. A local **Ollama** model with native
**tool-calling** receives a natural-language instruction plus the robot's current
state (as JSON), and replies with **movement tool calls** that we execute in
**MuJoCo** (simulation only).

```
  "back up half a meter then turn left"
            │
            ▼
   /llm/command (std_msgs/String)
            │
   ┌────────────────────┐   tools=[drive, move_forward, turn, set_lift, …, respond]
   │  llm_micro / _stretch │ ──────────────► Ollama (qwen3:1.7b)
   │  (this package)       │ ◄────────────── tool_calls + natural-language reply
   └────────────────────┘
            │  dispatch                          reply
            ▼                                      ▼
   /cmd_vel  ·  FollowJointTrajectory       /llm/response (std_msgs/String)
            ▼                                (shown back to the user)
       MuJoCo robot moves
```

Every command yields a natural-language **reply** on `/llm/response` describing
what the robot did. A built-in **`respond` (no-action)** tool lets the model answer
questions about its state *without moving* — e.g. "how close is the wall?" returns
an answer on `/llm/response` and issues no motion.

## Two robots, one design

| Launch | Robot (from) | Tools |
|--------|--------------|-------|
| `micro.launch.py`   | project **02** micro_ros rover | `drive(lin,ang,dur)`, `move_forward(dist)`, `turn(deg)`, `stop` |
| `stretch.launch.py` | project **04** Stretch SE3       | `drive(lin,ang,dur)`, `set_lift(h)`, `set_arm(ext)`, `set_head(pan,tilt)`, `set_gripper(open/close)`, `stop` |

The agent loop (`ollama_agent.py`), prompt assembly (`prompt.py`) and the ROS
command node (`robot_base.py`) are **robot-agnostic**; each robot is a thin adapter
in `robots/` that supplies its description, tool schemas, state and a dispatcher.
That is why both robots live in one package — there is no duplicated logic.

## Quick start

```bash
# 0. one-time: install Ollama + pull the model (see ../SETUP.md)
ollama serve &              # in its own terminal
ollama pull qwen3:1.7b
pip install ollama

# 1. warm-ups (no ROS) — confirm Ollama works, then see tool-calling in isolation
ros2 run llm_integration chat_terminal
ros2 run llm_integration ollama_api_demo "turn right 90 then go forward 1 m"

# 2. the robot demo (micro_ros rover)
ros2 launch llm_integration micro.launch.py
#    …watch the model's reply in a second terminal:
ros2 topic echo --field data /llm/response
#    …and in a third, publish ONE instruction:
ros2 topic pub --once /llm/command std_msgs/String "data: 'drive forward a bit then turn left'"
#    …or just ask a question (no motion — answered on /llm/response):
ros2 topic pub --once /llm/command std_msgs/String "data: 'how close is the wall in front of you?'"

# 3. the same on Stretch
ros2 launch llm_integration stretch.launch.py
ros2 topic pub --once /llm/command std_msgs/String "data: 'raise the lift and look down'"
```

### No GPU / no Ollama? Use the mock backend

`backend:=mock` swaps the LLM for a tiny keyword matcher so the full
command → tool → motion plumbing runs with **zero dependencies** (handy for CI and
for testing the ROS side):

```bash
ros2 launch llm_integration micro.launch.py backend:=mock
ros2 launch llm_integration stretch.launch.py backend:=mock
```

### Auto-publish one command from the launch

```bash
ros2 launch llm_integration micro.launch.py instruction:='spin in place to the left'
```

## Launch arguments

| arg | default | meaning |
|-----|---------|---------|
| `backend` | `ollama` | `ollama` (real model) or `mock` (offline keyword matcher) |
| `model` | `qwen3:1.7b` | any tool-calling Ollama model (e.g. `qwen3:4b` = slower, more accurate) |
| `ollama_host` | `` | override the Ollama server URL (default `localhost:11434`) |
| `instruction` | `` | if set, auto-publishes this command once after startup |
| `use_rviz` / `use_viewer` / `use_mujoco_viewer` | `true` | visualisation toggles |

See `../TUTORIAL.md` for the full walkthrough and `../SETUP.md` for install notes.
