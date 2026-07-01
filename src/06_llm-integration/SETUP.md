# Workshop 06 — Setup (Ollama + the `llm_integration` package)

Simulation only (MuJoCo). This project adds **no** new robot — it reuses the
project-02 rover and the project-04 Stretch and drives them with a local LLM.

## 1. Build

```bash
source /opt/ros/humble/setup.bash
cd ~/air26-ros2-ws
# 06 includes the 02 and 04 launch files, so build those too if you haven't:
colcon build --packages-select microbot_sim microbot_description \
                              stretch_se3_bringup llm_integration
source install/setup.bash
```

`llm_integration` declares `exec_depend` on `microbot_sim` and
`stretch_se3_bringup` — its launch files *include* their sim launches, which is the
project requirement that "launching 06 brings up 02/04".

## 2. Ollama (the LLM runtime) — already installed on this machine

Ollama is a standalone local server. The workshop uses **Qwen3** instruct models
(native tool-calling, CPU). The **default is `qwen3:1.7b`** — fast enough to be
interactive on CPU (~2-3 s/command warm). `qwen3:4b` is also installed: it picks
directions more reliably but is **much slower** on a CPU/low-RAM box, so it's the
opt-in choice via `model:=qwen3:4b`.

**Current machine state** (installed during project-06 setup):
- `ollama` **v0.30.11** in `/usr/local/bin/ollama`, running as a **systemd
  service** (`systemctl status ollama`) — starts on boot, listens on
  `http://localhost:11434`. You do **not** need to run `ollama serve` yourself.
- models **`qwen3:1.7b`** (default) and **`qwen3:4b`** are pulled (`ollama list`).
- the **`ollama` python client** is installed (`python3 -c "import ollama"`).

To reproduce on a fresh machine:

```bash
curl -fsSL https://ollama.com/install.sh | sh   # installs + enables the systemd service
ollama pull qwen3:1.7b                           # ~1.4 GB  (default)
ollama pull qwen3:4b                             # ~2.5 GB  (optional, more accurate/slower)
pip install ollama                               # python client used by this package
```

Handy checks:

```bash
systemctl status ollama          # is the server up?
ollama list                      # which models are present?
curl -s localhost:11434/api/tags # raw server probe
```

### Hardware note (why qwen3:1.7b is the default)

This dev box is an **AMD Ryzen APU (no usable GPU) with 14 GB RAM**, so Ollama runs
**CPU-only**. On it, measured warm latencies were ~**2-3 s** for `qwen3:1.7b` vs
**3-4 min** for `qwen3:4b` (the 4b's extra accuracy isn't worth the wait for a live
demo, and under memory pressure it swapped badly — up to 10 min). Two settings keep
it responsive (both in `ollama_agent.py`): **`think` is disabled** (qwen3's hidden
reasoning is ruinous on CPU; re-enable with `think:=true`) and **`temperature=0`**
(deterministic, and curbs small models' habit of spraying duplicate tool calls).
The agent also **recovers tool calls a small model prints as JSON text** and
**de-duplicates** repeated calls, so the robot still acts cleanly.

> **numpy pin note (shared with project 07):** the `ollama` python package is just
> a thin HTTP client (httpx + pydantic) and does **not** pull in torch/numpy, so it
> is safe to `pip install` into the system Python. If you prefer isolation you can
> install it into a venv and point ROS at that interpreter — but unlike project 07
> (SmolVLA/torch) there is no heavyweight dependency here, so a venv is optional.

### Verify Ollama before touching ROS

```bash
ros2 run llm_integration chat_terminal          # plain chat REPL
ros2 run llm_integration ollama_api_demo "turn left 45 then forward 1 m"
```

The second command prints the tool calls the model chose — exactly what the robot
nodes act on. If these work, the robot demos will too.

## 3. Run the robot demos

```bash
ros2 launch llm_integration micro.launch.py        # project-02 rover
ros2 launch llm_integration stretch.launch.py      # project-04 Stretch
```

then publish one instruction (see `TUTORIAL.md`).

## 4. No Ollama / CI: the mock backend

Every launch and node accepts `backend:=mock`, a zero-dependency keyword matcher
that emits one deterministic tool call. It exercises the entire ROS path
(command → tool → motion) without a model or GPU:

```bash
ros2 launch llm_integration micro.launch.py backend:=mock
```

This is what the headless verification uses.

## 5. Known harmless log noise (Stretch only)

When the Stretch sim executes **any** `FollowJointTrajectory` goal, its vendored
driver prints:

```
[stretch_mujoco_driver] ERROR: Error raised in execute callback:
    Please use `get_position()` for left_wheel_vel
```

This is a **pre-existing upstream bug** in `stretch_mujoco`'s trailing wheel-wait
(`wait_while_is_moving(left_wheel_vel)` calls `get_position_relative`, which the
`Actuators` enum only allows for the base) — it fires for the stock project-04
demos (`lift_arm`, etc.) too. The joint **reaches its setpoint before** the error,
so motion is correct; only the action's final status is affected. We do **not**
patch vendored upstream code for this workshop. Ignore the red line.
