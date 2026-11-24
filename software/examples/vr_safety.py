"""Hardware-independent safety helpers for VR teleoperation."""

import time

STALE_AFTER_S = 0.25


def is_fresh(goal, now=None, timeout=STALE_AFTER_S):
    """True if the goal was received recently. Goals without a timestamp are never fresh."""
    stamp = (getattr(goal, "metadata", None) or {}).get("received_at")
    if stamp is None:
        return False
    return (time.monotonic() if now is None else now) - stamp <= timeout


def grip_held(goal, now=None):
    return is_fresh(goal, now) and bool(goal.metadata.get("grip_active", False))


class SlewLimiter:
    """Limits how fast each commanded joint position may change (units per second)."""

    def __init__(self, initial, rates, default_rate):
        self.value = dict(initial)
        self.rates = rates
        self.default_rate = default_rate

    def step(self, targets, dt):
        dt = min(max(dt, 0.0), 0.1)  # a stalled loop must not turn into one big step
        for joint, target in targets.items():
            limit = self.rates.get(joint, self.default_rate) * dt
            current = self.value[joint]
            self.value[joint] = current + max(-limit, min(limit, target - current))
        return dict(self.value)
