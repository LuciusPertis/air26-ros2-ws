"""Pluggable mini-VLA policies.

A *policy* is the "brain": it maps a language instruction (+ proprioception, and
optionally a camera image) to a per-joint angle delta (delta-theta) every tick.
Swap policies without touching the rest of the pipeline.
"""

from .base import Policy
from .scripted import ScriptedPolicy

__all__ = ['Policy', 'ScriptedPolicy']
