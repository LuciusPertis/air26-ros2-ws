"""prompt — assemble the system prompt and the per-command user message.

We keep prompt engineering deliberately small and readable so students can edit
it and watch behaviour change:

  * system message = a natural-language description of the robot and its world,
    plus a short list of rules (use only the tools, keep moves small, stop when
    done).
  * user message   = the instruction, followed by the current robot state as
    compact JSON ("not much formatting", as a real telemetry blob would be).

The tool *schemas* are passed to ollama separately (native tool-calling), so we
don't have to describe them in prose here.
"""

import json

SYSTEM_TEMPLATE = """\
You are the motion controller for a mobile robot in a physics simulator.
Your job: turn ONE natural-language instruction into a short sequence of tool
calls that move the robot. Think in terms of the robot's own body frame.

{robot_desc}

Sign conventions (follow exactly):
- forward / ahead = POSITIVE distance;  back / backward / reverse = NEGATIVE distance.
- left = POSITIVE angle;  right = NEGATIVE angle.
- up / raise = larger value;  down / lower = smaller value.

Rules:
- Use ONLY the provided tools. Never invent tool names or output raw motor values.
- Prefer the smallest sequence of calls that satisfies the instruction; emit each
  distinct motion exactly once (do not repeat the same call).
- Keep motions small and safe; distances are metres, angles are degrees unless a
  tool says otherwise.
- The current robot state is given with each instruction; use it to decide.
- If the instruction is a QUESTION or otherwise needs NO movement (for example
  "where are you?", "how close is the wall?", "are you facing the obstacle?"),
  take no motion: call the `respond` tool with a natural-language answer based on
  the state. Do not move just to acknowledge a question.
- When you finish, reply with ONE short, natural sentence describing what you did
  (or, for a question, your answer). This sentence is shown back to the user.
"""


def build_system_prompt(robot_desc):
    return SYSTEM_TEMPLATE.format(robot_desc=robot_desc.strip())


def build_user_message(instruction, state):
    """instruction + current state as compact JSON."""
    blob = json.dumps(state, separators=(',', ':'), sort_keys=True)
    return f'Instruction: {instruction}\nCurrent state (JSON): {blob}'
