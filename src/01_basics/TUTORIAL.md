# Tutorial 01 — Topics, Services, Actions

Work through each section in order. Every step tells you what to run and what to look for.

> **Build first** — compile the workshop packages from a clean tree (do this once,
> and again after editing any node or commenting out a checkpoint):
> ```bash
> cd ~/air26-ros2-ws
> source /opt/ros/jazzy/setup.bash
> colcon build --packages-select basics_py basics_cpp basics_cross
> source install/setup.bash
> ```

> **Setup** — run this once in every new terminal before anything else:
> ```bash
> source /opt/ros/jazzy/setup.bash
> source ~/air26-ros2-ws/install/setup.bash
> ```

---

## 1 · Topics

A **topic** is a named channel. Publishers push messages; subscribers receive them. Neither knows about the other.

### 1.1 Run the talker

```bash
ros2 run basics topic_talker
```

Expected output (one line per second):
```
[INFO] [talker]: Published: "Hello ROS2! count=0"
[INFO] [talker]: Published: "Hello ROS2! count=1"
```

### 1.2 Open a second terminal — run the listener

```bash
ros2 run basics topic_listener
```

Expected output:
```
[INFO] [listener]: Heard: "Hello ROS2! count=3"
```

### 1.3 Introspect from a third terminal

List all active topics:
```bash
ros2 topic list
```
```
/chatter
/parameter_events
/rosout
```

Inspect the topic type:
```bash
ros2 topic info /chatter
```

Echo messages live (Ctrl-C to stop):
```bash
ros2 topic echo /chatter
```

Check publish rate:
```bash
ros2 topic hz /chatter
```

List active nodes:
```bash
ros2 node list
```
```
/listener
/talker
```

Inspect a node's publishers and subscribers:
```bash
ros2 node info /talker
ros2 node info /listener
```

### 1.4 Experiment

Stop the **listener** (Ctrl-C). The talker keeps publishing — it doesn't care.
Restart the listener. It picks up immediately — no message is replayed, it only sees new ones.

Stop the **talker**. The listener waits silently.

> **Takeaway:** topics are fire-and-forget; there is no handshake between publisher and subscriber.

---

## 2 · Services

A **service** is a request / response call. The client blocks until the server replies.

### 2.1 Run the server

```bash
ros2 run basics service_server
```
```
[INFO] [add_server]: Service /add_two_ints ready
```

### 2.2 Run the client (second terminal)

```bash
ros2 run basics service_client
```
```
[INFO] [add_client]: Result: 3 + 5 = 8
```

The client exits after one call.

### 2.3 Call the service from the CLI (no client node needed)

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 7}"
```
```
response:
  example_interfaces.srv.AddTwoInts_Response(sum=17)
```

### 2.4 Introspect

List active services:
```bash
ros2 service list
```

Inspect type:
```bash
ros2 service type /add_two_ints
```

Show the request/response fields:
```bash
ros2 interface show example_interfaces/srv/AddTwoInts
```

### 2.5 Experiment

Try calling the client **before** the server is running. The client waits:
```
[INFO] [add_client]: Waiting for /add_two_ints service...
```
Start the server in a second terminal — the client unblocks and completes.

> **Takeaway:** services are synchronous; the client waits for exactly one response.

---

## 3 · Actions

An **action** is a long-running goal with streaming **feedback** and a final **result**. The client can cancel mid-way.

### 3.1 Run the action server

```bash
ros2 run basics action_server
```
```
[INFO] [count_server]: Action server /count_up ready
```

### 3.2 Run the action client (second terminal)

```bash
ros2 run basics action_client
```

Watch both terminals. The server prints each feedback step; the client prints received feedback:
```
# client terminal
[INFO] [count_client]: Sending goal: count to 5
[INFO] [count_client]: Goal accepted — waiting for result...
[INFO] [count_client]: Feedback: 0
[INFO] [count_client]: Feedback: 1
...
[INFO] [count_client]: Result: reached 5
```

### 3.3 Introspect

List action servers:
```bash
ros2 action list
```
```
/count_up
```

Inspect type:
```bash
ros2 action info /count_up
```

Show goal / feedback / result fields:
```bash
ros2 interface show example_interfaces/action/Fibonacci
```

Send a goal from the CLI:
```bash
ros2 action send_goal /count_up example_interfaces/action/Fibonacci "order: 3" --feedback
```

### 3.4 Experiment

Try cancelling: send a goal with a large count (`order: 20`) from the CLI, then press Ctrl-C before it finishes.

> **Takeaway:** actions are for tasks that take time — they give you feedback while running and a single result when done.

---

## 4 · Combined Node

One node that does all three at once. Use it to see how they coexist inside a single process.

### 4.1 Run it

```bash
ros2 run basics combined_node
```

### 4.2 Interact with all three interfaces

**Topics** — in a second terminal:
```bash
ros2 topic echo /chatter
```

**Services** — in a third terminal:
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 2}"
```

**Actions** — in a fourth terminal:
```bash
ros2 action send_goal /count_up example_interfaces/action/Fibonacci "order: 3" --feedback
```

### 4.3 Inspect the node

```bash
ros2 node info /combined_node
```

You will see publishers, subscribers, service servers, and action servers all listed for the same node.

### 4.4 See the full graph

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

### 4.5 Experiment — remove a feature

Open `src/01_basics/basics/basics/combined_node.py` and comment out the entire
`# === CHECKPOINT: services ===` block (both the `__init__` block and the `handle_add` method).

Rebuild and rerun:
```bash
colcon build --packages-select basics
source install/setup.bash
ros2 run basics combined_node
```

Now `ros2 service list` no longer shows `/add_two_ints`. Topics and actions still work.

Restore the block and repeat for the other checkpoints to see each feature disappear and reappear.

---

## Quick-reference cheat sheet

| Goal | Command |
|------|---------|
| List nodes | `ros2 node list` |
| Node details | `ros2 node info /<name>` |
| List topics | `ros2 topic list` |
| Echo a topic | `ros2 topic echo /<topic>` |
| Topic publish rate | `ros2 topic hz /<topic>` |
| List services | `ros2 service list` |
| Call a service | `ros2 service call /<svc> <type> "<yaml>"` |
| List actions | `ros2 action list` |
| Send an action goal | `ros2 action send_goal /<action> <type> "<yaml>" --feedback` |
| Show message/srv/action fields | `ros2 interface show <type>` |
