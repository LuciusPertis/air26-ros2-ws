# Project 07 — VLA Demo: "delta-θ through ROS2"

> Teaching module. Goal: **show kids how an action (a per-joint angle delta, Δθ)
> travels through ROS2 to move a robot**, framed as a tiny Vision-Language-Action
> (VLA) pipeline. The same simple arm runs in **RViz, Gazebo and MuJoCo**.

## The idea
A lightweight "mini-VLA" turns a typed instruction into Δθ, which is streamed over
ROS2 and integrated into joint commands that drive the arm in any of three simulators.
It's a *demonstration of the data path*, not a research model — chosen because this box
has no GPU/torch (real VLAs like SmolVLA-450M are GB-scale and embodiment-specific).

```
/instruction (String)  ->  [vla_brain]  ->  /delta_theta (Float64MultiArray)
   "wave"/"up"/"home"      mini-VLA           <-- THE STAR TOPIC kids echo
                                                   |
                                          [theta_integrator]  theta += Δθ  (clamped)
                                                   |  /joint_command (abs theta)
                         +-------------------------+--------------------------+
                         v                         v                          v
                   RViz (kinematic)         Gazebo (ros2_control)       MuJoCo (driver)
```

## The arm
3-DOF from primitive boxes (no meshes), one model for all three sims:
`joint1` base yaw (Z), `joint2` shoulder (Y), `joint3` elbow (Y); a red tip marks the hand.
Each joint a different colour so "θ1/θ2/θ3" maps to what moves.

## Packages
- `vla_arm_description` — the arm: `urdf/arm.urdf.xacro` (RViz + Gazebo, `use_gz_control`
  arg), `urdf/arm.ros2_control.xacro`, `mjcf/arm.xml` (MuJoCo), `config/controllers.yaml`,
  `rviz/arm.rviz`.
- `vla_demo` — the pipeline:
  - `policies/` — `Policy` interface + default `ScriptedPolicy` + documented `smolvla_adapter` stub.
  - `vla_brain` — `/instruction` (+commanded θ) → policy → `/delta_theta`.
  - `theta_integrator` — `/delta_theta` → clamp → `/joint_command` (+ `/joint_states` in RViz-only).
  - `mujoco_driver` — `/joint_command` → MuJoCo physics → `/joint_states`.
  - `gz_command_relay` — `/joint_command` → `/position_controller/commands`.
  - `launch/{rviz,mujoco,gazebo}.launch.py`.

## Key design choices (and why)
- **Brain reads `/joint_command` (commanded θ), not `/joint_states` (measured).** Keeps
  brain↔integrator a stable closed loop; reading laggy measured state made closed-loop
  commands like `home` overshoot (verified in Gazebo). A real VLA would read measured
  state with a controller closing the loop.
- **One Δθ topic, three sim back-ends.** `theta_integrator` is the single source of θ;
  each simulator is just a different consumer of `/joint_command`.
- **No C++**: gz_ros2_control is configured purely via xacro tags + a controllers.yaml.

## Build phases (done)
0. Provision Gazebo (Ignition Fortress) + ros2_control — see `SETUP.md`.
1. `vla_arm_description` (xacro/MJCF/controllers/rviz).
2. `vla_demo` core + RViz target.
3. MuJoCo driver + target.
4. Gazebo (gz_ros2_control) target.
5. Docs + checkpoint pass.

## Status
All three sims verified headless: an instruction streams `/delta_theta`, the integrator
produces `/joint_command`, and the arm tracks it (RViz kinematic; MuJoCo & Gazebo physics).
`up/down/left/right/bend/wave/circle/home/stop` all work.
