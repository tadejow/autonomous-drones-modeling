"""Safety Limiter: every command from student code passes through here.

Order of operations for a single velocity vector (NED, m/s):

1. validation (three finite numbers, otherwise hover),
2. soft geofence walls and the defenders' exclusion sphere,
3. vertical speed limit,
4. 3D speed limit ``v <- v * min(1, v_max / |v|)`` (keeps the direction);
   ``v_max`` may differ between the teams (balance setting).

Soft wall: with ``d`` the signed distance to a wall (positive inside the arena),
the velocity component towards the wall is limited to ``gain * d`` once
``d < margin``. Outside the arena (``d < 0``) this becomes a minimum return
speed, never lower than ``fence_return_speed_mps``.
"""

from __future__ import annotations

import math
from collections import Counter
from typing import Any, Optional

import numpy as np

from pipeline.drones_battle.core.config import ArenaConfig
from pipeline.drones_battle.core.types import Vec3


def parse_command(raw: Any) -> Optional[np.ndarray]:
    """Returns a float array of shape (3,) or ``None`` when ``raw`` is not a valid vector."""
    try:
        if isinstance(raw, (str, bytes)) or len(raw) != 3:
            return None
        values = np.array([float(raw[0]), float(raw[1]), float(raw[2])])
    except (TypeError, ValueError):
        return None
    if not np.all(np.isfinite(values)):
        return None
    return values


def _limit_towards_lower(velocity: float, distance: float, margin: float, gain: float, ret: float) -> float:
    """Limits motion towards a lower bound; ``distance = position - bound``."""
    if distance >= margin:
        return velocity
    minimum = -gain * distance
    if distance < 0:
        minimum = max(minimum, ret)
    return max(velocity, minimum)


def _limit_towards_upper(velocity: float, distance: float, margin: float, gain: float, ret: float) -> float:
    """Limits motion towards an upper bound; ``distance = bound - position``."""
    return -_limit_towards_lower(-velocity, distance, margin, gain, ret)


class SafetyLimiter:
    """Applies the limits and counts interventions per drone (feedback for students)."""

    def __init__(self, config: ArenaConfig) -> None:
        self.config = config
        self.target = np.array(config.game.target_ned, dtype=float)
        self.interventions: dict[int, Counter[str]] = {i: Counter() for i in config.all_ids}

    def apply(self, drone_id: int, raw: Any, position: np.ndarray) -> Vec3:
        counter = self.interventions[drone_id]
        command = parse_command(raw)
        if command is None:
            counter["invalid"] += 1
            return (0.0, 0.0, 0.0)

        original = command.copy()
        command = self._geofence(command, position)
        if self.config.team_of(drone_id) == "defenders":
            command = self._exclusion_sphere(command, position)
        if not np.allclose(command, original):
            counter["fence"] += 1

        safety = self.config.safety
        if abs(command[2]) > safety.max_vertical_speed_mps:
            command[2] = math.copysign(safety.max_vertical_speed_mps, command[2])
            counter["vertical"] += 1

        max_speed = safety.max_speed_for(self.config.team_of(drone_id))
        speed = float(np.linalg.norm(command))
        if speed > max_speed:
            command *= max_speed / speed
            counter["speed"] += 1
        return (float(command[0]), float(command[1]), float(command[2]))

    def _geofence(self, v: np.ndarray, p: np.ndarray) -> np.ndarray:
        arena, safety = self.config.arena, self.config.safety
        margin, gain, ret = safety.fence_margin_m, safety.fence_gain, safety.fence_return_speed_mps
        v = v.copy()
        v[0] = _limit_towards_lower(v[0], p[0] - arena.north_min_m, margin, gain, ret)
        v[0] = _limit_towards_upper(v[0], arena.north_max_m - p[0], margin, gain, ret)
        v[1] = _limit_towards_lower(v[1], p[1] - arena.east_min_m, margin, gain, ret)
        v[1] = _limit_towards_upper(v[1], arena.east_max_m - p[1], margin, gain, ret)
        # Altitude is "up" = -D, so the vertical velocity "up" is -vD.
        altitude, v_up = -p[2], -v[2]
        v_up = _limit_towards_lower(v_up, altitude - arena.alt_min_m, margin, gain, ret)
        v_up = _limit_towards_upper(v_up, arena.alt_max_m - altitude, margin, gain, ret)
        v[2] = -v_up
        return v

    def _exclusion_sphere(self, v: np.ndarray, p: np.ndarray) -> np.ndarray:
        radius = self.config.game.defender_exclusion_radius_m
        if radius <= 0:
            return v
        offset = p - self.target
        distance_to_center = float(np.linalg.norm(offset))
        if distance_to_center < 1e-6:
            return v + np.array([-self.config.safety.fence_return_speed_mps, 0.0, 0.0])
        normal = offset / distance_to_center
        safety = self.config.safety
        radial = float(np.dot(v, normal))
        limited = _limit_towards_lower(
            radial, distance_to_center - radius, safety.fence_margin_m,
            safety.fence_gain, safety.fence_return_speed_mps,
        )
        return v + (limited - radial) * normal

    def summary(self) -> dict[str, dict[str, int]]:
        return {str(k): dict(v) for k, v in self.interventions.items() if v}
