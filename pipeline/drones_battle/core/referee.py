"""Game rules: continuous collision detection and win conditions.

Everything here is pure computation on positions, so it is unit tested without
SITL. Order of resolution inside one tick (fixed, documented in the README):

1. hits between alive attackers and alive defenders on the segment [t - dt, t],
   all applied simultaneously,
2. target check only for attackers that survived step 1 (a simultaneous hit and
   arrival therefore counts for the defenders),
3. "all attackers down",
4. time limit.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from pipeline.drones_battle.core.config import ArenaConfig
from pipeline.drones_battle.core.types import EventKind, GameEvent, MatchResult, RawState, to_vec3

_EPS = 1e-12


def closest_approach(
    a_start: np.ndarray, a_end: np.ndarray, b_start: np.ndarray, b_end: np.ndarray
) -> tuple[float, float]:
    """Minimum distance of two points moving linearly over the same time interval.

    With ``r(s) = r0 + s * delta`` for ``s`` in [0, 1], the minimum of ``|r(s)|``
    is reached at ``s* = clamp(-(r0 . delta) / |delta|^2, 0, 1)``.

    Returns:
        ``(d_min, s_star)``, where ``s_star`` is the fraction of the interval.
    """
    r0 = np.asarray(a_start, dtype=float) - np.asarray(b_start, dtype=float)
    delta = (np.asarray(a_end, dtype=float) - np.asarray(a_start, dtype=float)) - (
        np.asarray(b_end, dtype=float) - np.asarray(b_start, dtype=float)
    )
    denom = float(np.dot(delta, delta))
    s_star = 0.0 if denom < _EPS else float(np.clip(-np.dot(r0, delta) / denom, 0.0, 1.0))
    return float(np.linalg.norm(r0 + s_star * delta)), s_star


def segment_point_distance(start: np.ndarray, end: np.ndarray, point: np.ndarray) -> tuple[float, float]:
    """Distance from a fixed point to a point moving from ``start`` to ``end``."""
    point = np.asarray(point, dtype=float)
    return closest_approach(start, end, point, point)


@dataclass
class TickOutcome:
    events: list[GameEvent] = field(default_factory=list)
    result: Optional[MatchResult] = None

    @property
    def hits(self) -> list[GameEvent]:
        return [e for e in self.events if e.kind is EventKind.HIT]


class Referee:
    """Authoritative owner of the ``alive`` flags and the match result."""

    def __init__(self, config: ArenaConfig) -> None:
        self.config = config
        self.alive: dict[int, bool] = {drone_id: True for drone_id in config.all_ids}
        self.events: list[GameEvent] = []
        self.target = np.array(config.game.target_ned, dtype=float)

    def defender_disabled(self, position: np.ndarray) -> bool:
        """Anti-camping rule: a defender inside the exclusion sphere cannot score hits."""
        radius = self.config.game.defender_exclusion_radius_m
        return radius > 0 and float(np.linalg.norm(position - self.target)) < radius

    def evaluate(
        self, previous: dict[int, RawState], current: dict[int, RawState], time_s: float
    ) -> TickOutcome:
        game = self.config.game
        outcome = TickOutcome()

        # 1. Hits (continuous detection between the previous and the current tick).
        attackers = [i for i in game.attacker_ids if self.alive[i] and not current[i].stale]
        defenders = [
            i for i in game.defender_ids
            if self.alive[i] and not current[i].stale and not self.defender_disabled(current[i].pos)
        ]
        dt = game.dt
        killed: set[int] = set()
        for attacker in attackers:
            best: Optional[tuple[float, float, int]] = None
            for defender in defenders:
                d_min, s_star = closest_approach(
                    previous[attacker].pos, current[attacker].pos,
                    previous[defender].pos, current[defender].pos,
                )
                if d_min < game.kill_radius_m and (best is None or d_min < best[0]):
                    best = (d_min, s_star, defender)
            if best is None:
                continue
            _, s_star, defender = best
            where = previous[attacker].pos + s_star * (current[attacker].pos - previous[attacker].pos)
            hit_time = time_s - (1.0 - s_star) * dt
            outcome.events.append(GameEvent(EventKind.HIT, hit_time, defender, attacker, to_vec3(where)))
            killed.add(attacker)
            if game.mutual_kill:
                outcome.events.append(
                    GameEvent(EventKind.HIT, hit_time, attacker, defender, to_vec3(where), "mutual")
                )
                killed.add(defender)
        for drone_id in killed:
            self.alive[drone_id] = False

        # 2. Target reached by a surviving attacker.
        for attacker in game.attacker_ids:
            if not self.alive[attacker] or current[attacker].stale:
                continue
            d_min, s_star = segment_point_distance(previous[attacker].pos, current[attacker].pos, self.target)
            if d_min < game.target_radius_m:
                event = GameEvent(EventKind.TARGET_REACHED, time_s - (1.0 - s_star) * dt, attacker)
                outcome.events.append(event)
                return self._finish(outcome, "attackers", EventKind.TARGET_REACHED, time_s)

        # 3. Every attacker is down.
        if not any(self.alive[i] for i in game.attacker_ids):
            outcome.events.append(GameEvent(EventKind.ALL_ATTACKERS_DOWN, time_s))
            return self._finish(outcome, "defenders", EventKind.ALL_ATTACKERS_DOWN, time_s)

        # 4. Time limit.
        if time_s >= game.max_time_s:
            outcome.events.append(GameEvent(EventKind.TIMEOUT, time_s))
            return self._finish(outcome, "defenders", EventKind.TIMEOUT, time_s)

        self.events.extend(outcome.events)
        return outcome

    def forfeit(self, loser: str, time_s: float, detail: str) -> MatchResult:
        winner = "defenders" if loser == "attackers" else "attackers"
        outcome = TickOutcome([GameEvent(EventKind.FORFEIT, time_s, detail=detail)])
        result = self._finish(outcome, winner, EventKind.FORFEIT, time_s).result
        assert result is not None
        return result

    def _finish(self, outcome: TickOutcome, winner: str, reason: EventKind, time_s: float) -> TickOutcome:
        self.events.extend(outcome.events)
        outcome.result = MatchResult(
            winner=winner,  # type: ignore[arg-type]
            reason=reason,
            time=time_s,
            alive=dict(self.alive),
            events=list(self.events),
        )
        return outcome
