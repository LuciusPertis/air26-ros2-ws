"""SmolVLA adapter — a DOCUMENTED STUB showing how a real VLA slots in.

This is intentionally NOT wired into the demo: this machine has no GPU and no
torch, and SmolVLA-450M is trained for specific arms (the LeRobot SO-100/SO-101),
so its raw outputs wouldn't be meaningful on our toy arm without finetuning. It's
here to show that "swap the brain for a real model" is a one-class change — the
rest of the pipeline (delta_theta -> integrator -> sim) is identical.

To actually use it you would:
  pip install lerobot transformers torch        # heavy; slow on CPU
  pass --policy smolvla to vla_brain (see vla_brain.py)

References:
  SmolVLA: https://huggingface.co/blog/smolvla   (arXiv:2506.01844)
"""

import numpy as np

from .base import Policy


class SmolVLAPolicy(Policy):
    def __init__(self, checkpoint='lerobot/smolvla_base'):
        raise NotImplementedError(
            'SmolVLA adapter is a stub — install lerobot/transformers/torch and '
            'implement load + predict here. See module docstring.')

    def predict(self, instruction, theta, image=None):
        # A real implementation would, each tick:
        #   1. build the observation: {image, instruction, state=theta}
        #   2. action = self.model.select_action(obs)        # learned VLA
        #   3. return delta-theta (map the model's action space to our 3 joints)
        raise NotImplementedError
