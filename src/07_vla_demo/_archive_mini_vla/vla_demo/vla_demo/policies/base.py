"""The Policy interface — the contract every mini-VLA implements.

This is the "A" of VLA reduced to its essence: given a Language instruction (and
optionally Vision + proprioception), produce an Action. Here the action is a
**delta-theta**: how much to nudge each joint this tick. A real VLA (e.g. SmolVLA)
would implement exactly this interface — see scripted.py for the toy default and
smolvla_adapter.py for how a learned model would slot in.
"""

import numpy as np


class Policy:
    n_joints = 3

    def predict(self, instruction, theta, image=None):
        """Return a delta-theta (np.ndarray of length n_joints), in radians.

        instruction: str   — the language command (e.g. "wave", "raise arm").
        theta:       array — current joint angles (proprioception).
        image:       optional camera frame (the Vision hook; unused by default).
        """
        raise NotImplementedError

    def reset(self):
        """Reset any internal state (e.g. oscillation phase)."""
