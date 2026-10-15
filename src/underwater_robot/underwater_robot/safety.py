"""
Safety helpers for actuator command handling.

Pure logic (no ROS imports) so it can be unit tested directly.

Two guarantees are provided:

1. ``clamp_commands`` enforces a hard symmetric magnitude limit on every
   thruster command, so no software bug can command an arbitrarily large
   thrust to the (simulated) ESCs.
2. ``StaleCommandGuard`` implements the "commands go to zero unless a fresh
   valid command has been received" rule.  If the upstream controller stops
   publishing (crash, network drop, dead battery in the real system) the
   command is replaced by zero within ``timeout`` seconds.
"""

from typing import Iterable, List

from .allocation import clamp, THRUSTER_ORDER


def clamp_commands(commands: Iterable[float], limit: float) -> List[float]:
    """
    Return ``commands`` clamped to ``[-limit, +limit]``.

    The output always has exactly ``len(THRUSTER_ORDER)`` elements; missing
    trailing elements are treated as zero.
    """
    out = [clamp(float(c), limit) for c in list(commands)[: len(THRUSTER_ORDER)]]
    while len(out) < len(THRUSTER_ORDER):
        out.append(0.0)
    return out


class StaleCommandGuard:
    """Track the age of the most recent command and decide staleness."""

    def __init__(self, timeout: float, now: float):
        self.timeout = float(timeout)
        self._last_valid = float(now)

    def mark_valid(self, now: float) -> None:
        """Record that a fresh, valid command was received at ``now``."""
        self._last_valid = float(now)

    def age(self, now: float) -> float:
        return float(now) - self._last_valid

    def is_stale(self, now: float) -> bool:
        return self.age(now) > self.timeout
