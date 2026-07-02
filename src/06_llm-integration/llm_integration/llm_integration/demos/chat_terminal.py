#!/usr/bin/env python3
"""chat_terminal — the simplest possible Ollama chat REPL (no ROS, no tools).

The "hello world" warm-up: type messages, get replies from the local model, with
conversation history kept so it remembers the thread. Use it first to confirm
your Ollama install works before adding tools (ollama_api_demo) or a robot.

    python3 -m llm_integration.demos.chat_terminal
    python3 -m llm_integration.demos.chat_terminal --model qwen3:4b

Ctrl-D or "exit" to quit.
"""

import argparse
import sys

try:
    import ollama
except ImportError:
    print('The "ollama" python package is missing. Run:  pip install ollama')
    sys.exit(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='qwen3:1.7b')
    ap.add_argument('--system', default='You are a concise, helpful assistant.')
    args = ap.parse_args()

    print(f'chat with {args.model} — Ctrl-D or "exit" to quit\n')
    messages = [{'role': 'system', 'content': args.system}]

    while True:
        try:
            user = input('you> ').strip()
        except EOFError:
            print()
            break
        if user in ('exit', 'quit'):
            break
        if not user:
            continue
        messages.append({'role': 'user', 'content': user})
        try:
            print('bot> ', end='', flush=True)
            reply = ''
            # think=False: fast replies on CPU (qwen3 would otherwise "think" first).
            try:
                stream = ollama.chat(model=args.model, messages=messages,
                                     stream=True, think=False)
            except TypeError:
                stream = ollama.chat(model=args.model, messages=messages, stream=True)
            for chunk in stream:
                piece = (chunk['message']['content'] if isinstance(chunk, dict)
                         else chunk.message.content)
                print(piece, end='', flush=True)
                reply += piece
            print()
            messages.append({'role': 'assistant', 'content': reply})
        except Exception as exc:
            print(f'\n[error] {exc}\nIs `ollama serve` running and '
                  f'`ollama pull {args.model}` done?')


if __name__ == '__main__':
    main()
