"""ollama_agent — the (robot-agnostic) LLM tool-calling loop.

This is the heart of project 06: take ONE natural-language instruction, hand it to
a local Ollama model together with a set of *movement tools*, and let the model
decide which tool(s) to call. We execute each call on the robot, feed the result
back, and let the model either call another tool or stop.

  natural language  ->  ollama chat(model, tools=[...])  ->  tool_calls
                                                              |
                                                       dispatch(name, args)  -> moves robot
                                                              |
                                                        result fed back -> model decides next

Two backends:
  * "ollama"  — the real thing: talks to a local Ollama server (default
                http://localhost:11434) running an instruct model with native
                tool-calling (default qwen3:1.7b). Needs `pip install ollama`.
  * "mock"    — no server required. A tiny keyword matcher that emits a single
                deterministic tool call, so the whole ROS plumbing
                (command -> tool -> motion) can be exercised offline / in CI.

Nothing here imports rclpy — it is pure Python so it can also be run from the
standalone demos in demos/.
"""

import json
import re

# === CHECKPOINT: ollama-backend ===
try:
    import ollama  # type: ignore
except ImportError:  # pragma: no cover - optional at import time
    ollama = None


def _ollama_step(model, messages, tools, host, think=False):
    """One round-trip to the Ollama server. Returns the assistant message dict.

    think=False disables "thinking" models' hidden chain-of-thought (e.g. qwen3),
    which is essential on CPU: with thinking ON, qwen3:4b spends minutes generating
    reasoning tokens before the tool call. We want a fast, direct tool call.
    """
    if ollama is None:
        raise RuntimeError(
            "The 'ollama' python package is not installed. Either "
            "`pip install ollama` (and run an Ollama server with the model "
            "pulled), or launch with backend:=mock for an offline dry-run.")
    client = ollama.Client(host=host) if host else ollama
    # temperature=0 -> deterministic, and noticeably curbs small models' habit of
    # emitting floods of duplicate/garbled tool calls.
    opts = {'temperature': 0}
    try:
        resp = client.chat(model=model, messages=messages, tools=tools,
                           think=think, options=opts)
    except TypeError:
        # Older ollama-python without the `think` kwarg.
        resp = client.chat(model=model, messages=messages, tools=tools, options=opts)
    # ollama-python returns an object in recent versions, a dict in older ones.
    msg = resp['message'] if isinstance(resp, dict) else resp.message
    return _as_message_dict(msg)


def _as_message_dict(msg):
    """Normalise an ollama message (object or dict) into a plain dict."""
    if isinstance(msg, dict):
        out = {'role': msg.get('role', 'assistant'),
               'content': msg.get('content', '') or ''}
        calls = msg.get('tool_calls') or []
    else:  # pydantic-ish object
        out = {'role': getattr(msg, 'role', 'assistant'),
               'content': getattr(msg, 'content', '') or ''}
        calls = getattr(msg, 'tool_calls', None) or []
    norm = []
    for c in calls:
        fn = c['function'] if isinstance(c, dict) else c.function
        name = fn['name'] if isinstance(fn, dict) else fn.name
        args = fn['arguments'] if isinstance(fn, dict) else fn.arguments
        if isinstance(args, str):
            try:
                args = json.loads(args)
            except json.JSONDecodeError:
                args = {}
        norm.append({'name': name, 'arguments': dict(args)})
    out['tool_calls'] = norm
    return out
# === END CHECKPOINT: ollama-backend ===


# === CHECKPOINT: mock-backend ===
def _mock_step(model, messages, tools, host, think=False):
    """A zero-dependency stand-in for an LLM.

    Looks at the latest user instruction and emits ONE plausible tool call by
    keyword. Enough to prove the command -> tool -> motion path without a server.
    On the second turn (after a tool result is in the history) it stops, so the
    agent loop terminates just like the real model would.
    """
    if any(m['role'] == 'tool' for m in messages):
        return {'role': 'assistant', 'content': '[mock] done.', 'tool_calls': []}

    text = next((m['content'] for m in reversed(messages)
                 if m['role'] == 'user'), '')
    # Only look at the instruction, not the appended state JSON (which contains
    # words like "right"/"left" from sensor keys that would fool the matcher).
    text = text.split('\nCurrent state', 1)[0].lower()
    names = {t['function']['name'] for t in tools}

    def call(name, **args):
        return {'role': 'assistant', 'content': '',
                'tool_calls': [{'name': name, 'arguments': args}]}

    num = re.search(r'(-?\d+(?:\.\d+)?)', text)
    val = float(num.group(1)) if num else None

    # A question / status query -> answer, take no physical action.
    question = ('?' in text or any(text.startswith(w) for w in (
        'what', 'where', 'how', 'is ', 'are ', 'do ', 'does ', 'can ', 'tell '))
        or 'state' in text or 'sensor' in text or 'position' in text)
    if question and 'respond' in names:
        return call('respond',
                    answer='[mock] I took no action; this looks like a question '
                           'about my state. (A real model would summarise the '
                           'state JSON here.)')

    if 'stop' in text and 'stop' in names:
        return call('stop')
    if ('lift' in text or 'raise' in text or 'lower' in text) and 'set_lift' in names:
        return call('set_lift', height=val if val is not None else 0.6)
    if ('arm' in text or 'extend' in text or 'reach' in text) and 'set_arm' in names:
        return call('set_arm', extension=val if val is not None else 0.25)
    if ('head' in text or 'look' in text) and 'set_head' in names:
        return call('set_head', pan=0.0, tilt=-0.3)
    if ('grip' in text or 'grasp' in text or 'open' in text or 'close' in text) \
            and 'set_gripper' in names:
        return call('set_gripper', state='close' if 'close' in text else 'open')
    if ('left' in text or 'right' in text or 'turn' in text or 'rotate' in text) \
            and 'turn' in names:
        ang = val if val is not None else 90.0
        return call('turn', angle_deg=-ang if 'right' in text else ang)
    if ('forward' in text or 'ahead' in text or 'back' in text) and 'move_forward' in names:
        dist = val if val is not None else 0.5
        return call('move_forward', distance=-dist if 'back' in text else dist)
    if 'drive' in names:  # generic fallback: nudge forward
        return call('drive', linear=0.15, angular=0.0, duration=2.0)
    return {'role': 'assistant', 'content': '[mock] no matching tool.', 'tool_calls': []}
# === END CHECKPOINT: mock-backend ===


def _recover_text_tool_calls(content, tool_names):
    """Recover tool calls a small model emitted as plain JSON text in `content`
    instead of via the structured tool_calls API (qwen3:1.7b does this at times).

    Returns a list of {'name','arguments'} for any JSON object that names a known
    tool; [] if the content is just a normal natural-language reply.
    """
    if not content or '{' not in content or '"name"' not in content:
        return []
    text = content.strip()
    if text.startswith('```'):  # strip ```json ... ``` fences
        text = text.strip('`')
        text = text[4:] if text.lower().startswith('json') else text

    candidates = []
    try:
        parsed = json.loads(text)
        candidates = parsed if isinstance(parsed, list) else [parsed]
    except json.JSONDecodeError:
        # Fall back to scraping brace-balanced objects that mention "name".
        for m in re.finditer(r'\{[^{}]*"name"[^{}]*\{[^{}]*\}[^{}]*\}|'
                             r'\{[^{}]*"name"[^{}]*\}', text):
            try:
                candidates.append(json.loads(m.group(0)))
            except json.JSONDecodeError:
                pass

    calls = []
    for c in candidates:
        if isinstance(c, dict) and c.get('name') in tool_names:
            args = c.get('arguments') or c.get('parameters') or {}
            if isinstance(args, str):
                try:
                    args = json.loads(args)
                except json.JSONDecodeError:
                    args = {}
            calls.append({'name': c['name'], 'arguments': dict(args)})
    return calls


def run_agent(*, model, system_prompt, user_message, tools, dispatch,
              backend='ollama', host='', think=False, max_iters=5, logger=print):
    """Run the natural-language -> tool-calls -> motion loop.

    model         : ollama model name (e.g. 'qwen3:1.7b'); ignored by the mock.
    system_prompt : robot + environment description and rules.
    user_message  : the instruction plus the current robot state (JSON).
    tools         : list of OpenAI/Ollama-style tool schemas.
    dispatch      : callable(name, args_dict) -> result dict; runs the motion.
    backend       : 'ollama' | 'mock'.
    Returns the model's final text summary.
    """
    step = _mock_step if backend == 'mock' else _ollama_step
    tool_names = {t['function']['name'] for t in tools}
    messages = [{'role': 'system', 'content': system_prompt},
                {'role': 'user', 'content': user_message}]
    last_answer = None   # most recent natural-language answer from a 'respond' call

    for i in range(max_iters):
        msg = step(model, messages, tools, host, think)
        content = msg.get('content', '') or ''
        calls = msg.get('tool_calls', [])
        if not calls:
            # Small models sometimes print the tool call as JSON text instead of
            # calling it — recover those so the robot still acts (and we don't show
            # raw JSON as the reply).
            recovered = _recover_text_tool_calls(content, tool_names)
            if recovered:
                calls, content = recovered, ''
        # Re-send the assistant turn in Ollama's own schema (tool_calls wrap the
        # name/arguments under a "function" key); our normalized {name,arguments}
        # form would fail the server's pydantic validation on the next round-trip.
        messages.append({
            'role': 'assistant',
            'content': content,
            'tool_calls': [{'function': {'name': c['name'],
                                         'arguments': c['arguments']}}
                           for c in calls],
        })
        if not calls:
            # The model's closing free text is its reply about what it did. If it
            # said nothing, fall back to the last explicit 'respond' answer.
            return content or last_answer or '(no reply)'
        # Small models sometimes emit the same call several times in one turn;
        # execute each distinct (name, args) only once so the robot doesn't repeat
        # a motion. (Order preserved.)
        seen = set()
        unique = []
        for c in calls:
            key = (c['name'], json.dumps(c['arguments'], sort_keys=True))
            if key not in seen:
                seen.add(key)
                unique.append(c)
        for c in unique:
            name, args = c['name'], c['arguments']
            logger(f'tool call: {name}({_fmt_args(args)})')
            try:
                result = dispatch(name, args)
            except Exception as exc:  # surface, don't crash the loop
                result = {'ok': False, 'error': str(exc)}
                logger(f'  tool error: {exc}')
            if isinstance(result, dict) and result.get('answer'):
                last_answer = result['answer']
            # 'respond' is a terminal answer tool — no motion, so the answer IS
            # the reply; stop here instead of spending another model turn.
            if name in ('respond', 'no_action') or (
                    isinstance(result, dict) and result.get('action') == 'no_action'):
                return last_answer or content or '(no reply)'
            messages.append({'role': 'tool', 'tool_name': name,
                             'content': json.dumps(result)})
    return last_answer or '(stopped: reached max tool iterations)'


def _fmt_args(args):
    return ', '.join(f'{k}={v}' for k, v in args.items())
