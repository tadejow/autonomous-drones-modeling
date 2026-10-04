"""Data types shared by the referee, backends, sandbox and visualizer."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Literal, Optional, TypedDict

import numpy as np

Vec3 = tuple[float, float, float]
TeamName = Literal["attackers", "defenders"]


class DroneState(TypedDict):
    """What student code sees about a single drone (NED metres, m/s)."""

    pos: Vec3
    vel: Vec3
    alive: bool


TeamView = dict[int, DroneState]
Commands = dict[int, Vec3]


@dataclass
class RawState:
    """Telemetry reported by a backend for one drone (NED relative to the defenders' base)."""

    pos: np.ndarray
    vel: np.ndarray
    stale: bool = False


class EventKind(str, Enum):
    HIT = "HIT"
    TARGET_REACHED = "TARGET_REACHED"
    ALL_ATTACKERS_DOWN = "ALL_ATTACKERS_DOWN"
    TIMEOUT = "TIMEOUT"
    FORFEIT = "FORFEIT"
    STRATEGY_ERROR = "STRATEGY_ERROR"


@dataclass(frozen=True)
class GameEvent:
    kind: EventKind
    time: float
    actor: Optional[int] = None
    victim: Optional[int] = None
    position: Optional[Vec3] = None
    detail: str = ""

    def as_dict(self) -> dict[str, object]:
        return {
            "kind": self.kind.value,
            "time": round(self.time, 3),
            "actor": self.actor,
            "victim": self.victim,
            "position": None if self.position is None else [round(c, 3) for c in self.position],
            "detail": self.detail,
        }

    def describe(self) -> str:
        if self.kind is EventKind.HIT:
            return f"t={self.time:5.1f}s  drone {self.actor} shot down drone {self.victim}"
        if self.kind is EventKind.TARGET_REACHED:
            return f"t={self.time:5.1f}s  drone {self.actor} reached the target"
        return f"t={self.time:5.1f}s  {self.kind.value} {self.detail}".rstrip()


@dataclass
class MatchResult:
    winner: TeamName
    reason: EventKind
    time: float
    alive: dict[int, bool]
    events: list[GameEvent] = field(default_factory=list)
    stats: dict[str, object] = field(default_factory=dict)

    def as_dict(self) -> dict[str, object]:
        return {
            "winner": self.winner,
            "reason": self.reason.value,
            "time": round(self.time, 3),
            "alive": {str(k): v for k, v in self.alive.items()},
            "events": [e.as_dict() for e in self.events],
            "stats": self.stats,
        }


def to_vec3(array: np.ndarray) -> Vec3:
    return (float(array[0]), float(array[1]), float(array[2]))
