"""ScriptedPolicy — the default mini-VLA: keyword instruction -> delta-theta.

Deterministic and instant on CPU, so a live demo never stalls. It reads the
instruction string and emits a small per-joint nudge each tick; some behaviours
(wave, circle) use an internal phase clock, and `home` uses proprioception
(current theta) to drive back to zero — exactly the inputs a real VLA consumes.

Try: up / down, left / right, bend / straighten, wave, circle, home, stop.
"""

import numpy as np

from .base import Policy

STEP = 0.04   # base nudge magnitude (rad/tick) for directional commands


class ScriptedPolicy(Policy):

    def __init__(self, dt=0.1):
        self.dt = dt        # seconds per tick (matches the brain's rate)
        self.t = 0.0        # phase clock for oscillating behaviours

    def reset(self):
        self.t = 0.0

    def predict(self, instruction, theta, image=None):
        self.t += self.dt
        d = np.zeros(self.n_joints)
        instr = (instruction or '').lower().strip()
        theta = np.asarray(theta, dtype=float)

        # --- standstill ---
        if instr in ('', 'stop', 'halt', 'hold'):
            return d

        # --- oscillating behaviours (use the phase clock) ---
        if 'wave' in instr:
            d[2] = 0.18 * np.cos(2 * np.pi * 0.5 * self.t)   # waggle the elbow
            return d
        if 'circle' in instr:
            d[0] = 0.06 * np.cos(2 * np.pi * 0.25 * self.t)  # base + shoulder
            d[1] = 0.06 * np.sin(2 * np.pi * 0.25 * self.t)  # trace a circle
            return d

        # --- proprioceptive: drive every joint toward zero ---
        if 'home' in instr or 'reset' in instr:
            return np.clip(-theta, -STEP, STEP)

        # --- directional commands: nudge one joint ---
        if 'left' in instr:        d[0] = +STEP
        elif 'right' in instr:     d[0] = -STEP
        if 'up' in instr or 'raise' in instr:     d[1] = +STEP
        elif 'down' in instr or 'lower' in instr: d[1] = -STEP
        if 'bend' in instr or 'elbow' in instr:       d[2] = +STEP
        elif 'straight' in instr or 'extend' in instr: d[2] = -STEP
        return d
