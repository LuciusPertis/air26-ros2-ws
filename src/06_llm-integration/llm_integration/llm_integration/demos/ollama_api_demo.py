#!/usr/bin/env python3
"""ollama_api_demo — a standalone (no-ROS) walkthrough of Ollama tool-calling.

Run this BEFORE the robot demo to see, in isolation, exactly what project 06 does
under the hood: define a couple of "tools", hand the model an instruction, and
watch it choose a tool and fill in the arguments. No robot, no ROS — just the
request/response with a local Ollama server.

    python3 -m llm_integration.demos.ollama_api_demo "turn left then go forward 1 m"

Requires: a running Ollama server (`ollama serve`) with the model pulled
(`ollama pull qwen3:1.7b`) and `pip install ollama`.
"""

import json
import sys

try:
    import ollama
except ImportError:
    print('The "ollama" python package is missing. Run:  pip install ollama')
    sys.exit(1)

MODEL = 'qwen3:1.7b'

# Two toy movement tools, in the same OpenAI/Ollama schema the robot demo uses.
TOOLS = [
    {'type': 'function', 'function': {
        'name': 'move_forward',
        'description': 'Drive straight forward by a distance in metres.',
        'parameters': {'type': 'object', 'properties': {
            'distance': {'type': 'number', 'description': 'metres'}},
            'required': ['distance']}}},
    {'type': 'function', 'function': {
        'name': 'turn',
        'description': 'Rotate in place by an angle in degrees (+ left, - right).',
        'parameters': {'type': 'object', 'properties': {
            'angle_deg': {'type': 'number', 'description': 'degrees'}},
            'required': ['angle_deg']}}},
]

SYSTEM = ('You control a small wheeled robot. Translate the instruction into '
          'tool calls. Use only the provided tools. Positive angle = left.')


def main():
    instruction = ' '.join(sys.argv[1:]) or 'turn right 90 degrees then drive forward 2 meters'
    print(f'model       : {MODEL}')
    print(f'instruction : {instruction}\n')

    messages = [{'role': 'system', 'content': SYSTEM},
                {'role': 'user', 'content': instruction}]
    try:
        # think=False keeps qwen3 from spending minutes "thinking" on CPU before
        # the tool call (older ollama clients lack the kwarg -> fall back).
        try:
            resp = ollama.chat(model=MODEL, messages=messages, tools=TOOLS, think=False)
        except TypeError:
            resp = ollama.chat(model=MODEL, messages=messages, tools=TOOLS)
    except Exception as exc:
        print(f'Ollama call failed: {exc}\n'
              'Is the server running? (ollama serve)  Is the model pulled? '
              f'(ollama pull {MODEL})')
        sys.exit(1)

    msg = resp['message'] if isinstance(resp, dict) else resp.message
    calls = (msg.get('tool_calls') if isinstance(msg, dict)
             else getattr(msg, 'tool_calls', None)) or []

    if not calls:
        content = msg['content'] if isinstance(msg, dict) else msg.content
        print('Model returned NO tool calls. Text reply:')
        print(' ', content)
        return

    print(f'Model chose {len(calls)} tool call(s):')
    for c in calls:
        fn = c['function'] if isinstance(c, dict) else c.function
        name = fn['name'] if isinstance(fn, dict) else fn.name
        args = fn['arguments'] if isinstance(fn, dict) else fn.arguments
        if isinstance(args, str):
            args = json.loads(args)
        print(f'  - {name}({", ".join(f"{k}={v}" for k, v in dict(args).items())})')


if __name__ == '__main__':
    main()
