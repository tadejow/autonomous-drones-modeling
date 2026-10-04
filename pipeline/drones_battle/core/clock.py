"""Fixed-rate loop timing with jitter statistics."""

from __future__ import annotations

import time
from typing import Callable


class FixedRateClock:
    """Sleeps until the next scheduled tick.

    The next deadline is computed from the previous *scheduled* time, not from
    the moment the loop woke up, so delays do not accumulate into drift. When a
    tick overruns by more than one period the schedule is reset.
    """

    def __init__(
        self,
        hz: float,
        realtime: bool = True,
        time_fn: Callable[[], float] = time.monotonic,
        sleep_fn: Callable[[float], None] = time.sleep,
    ) -> None:
        self.period = 1.0 / hz
        self.realtime = realtime
        self._time = time_fn
        self._sleep = sleep_fn
        self._next = time_fn() + self.period
        self.lateness: list[float] = []

    def wait(self) -> None:
        if not self.realtime:
            return
        now = self._time()
        remaining = self._next - now
        if remaining > 0:
            self._sleep(remaining)
            self.lateness.append(max(self._time() - self._next, 0.0))
            self._next += self.period
        else:
            self.lateness.append(-remaining)
            self._next = now + self.period if -remaining > self.period else self._next + self.period

    def stats(self) -> dict[str, float]:
        if not self.lateness:
            return {}
        ordered = sorted(self.lateness)
        p95 = ordered[min(int(0.95 * len(ordered)), len(ordered) - 1)]
        return {
            "ticks": float(len(ordered)),
            "jitter_p95_ms": round(1000.0 * p95, 2),
            "jitter_max_ms": round(1000.0 * ordered[-1], 2),
        }
