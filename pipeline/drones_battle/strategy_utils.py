"""Small vector helpers that strategies may import (all NED, metres, m/s).

Nothing here talks to a drone; these are pure functions on tuples, so they can
be tested in a notebook. Reminder: in NED a positive ``vD`` means descending.
"""

from __future__ import annotations

import math
from typing import Iterable, Optional

Vec3 = tuple[float, float, float]


def add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Vec3, k: float) -> Vec3:
    return (a[0] * k, a[1] * k, a[2] * k)


def dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def norm(a: Vec3) -> float:
    return math.sqrt(dot(a, a))


def distance(a: Vec3, b: Vec3) -> float:
    return norm(sub(a, b))


def unit(a: Vec3) -> Vec3:
    length = norm(a)
    return (0.0, 0.0, 0.0) if length < 1e-9 else scale(a, 1.0 / length)


def clamp_norm(a: Vec3, max_norm: float) -> Vec3:
    length = norm(a)
    return a if length <= max_norm else scale(a, max_norm / length)


def fly_towards(position: Vec3, goal: Vec3, speed: float, arrive: bool = True) -> Vec3:
    """Velocity of magnitude ``speed`` pointing at ``goal``.

    With ``arrive=True`` the drone slows down in the last metres (stops at the goal);
    use ``arrive=False`` when chasing, otherwise the hunter brakes right before the hit.
    """
    offset = sub(goal, position)
    return scale(unit(offset), min(speed, norm(offset)) if arrive else speed)


def hold_altitude(position: Vec3, altitude_m: float, gain: float = 1.0) -> float:
    """Proportional vertical command ``vD`` keeping ``altitude_m`` above the base."""
    return gain * (position[2] + altitude_m)


def intercept_time(
    hunter_pos: Vec3, target_pos: Vec3, target_vel: Vec3, hunter_speed: float
) -> Optional[float]:
    """Earliest ``t >= 0`` with ``|target_pos + target_vel t - hunter_pos| = hunter_speed t``.

    Squaring gives ``(s^2 - |v|^2) t^2 - 2 (r . v) t - |r|^2 = 0`` with
    ``r = target_pos - hunter_pos`` (the same equation as in ``07_b_pursuit.py``).
    Returns ``None`` when the target cannot be caught.
    """
    r = sub(target_pos, hunter_pos)
    a = hunter_speed ** 2 - dot(target_vel, target_vel)
    b = -2.0 * dot(r, target_vel)
    c = -dot(r, r)
    if abs(a) < 1e-9:
        return None if abs(b) < 1e-9 else (-c / b if -c / b >= 0 else None)
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0:
        return None
    roots = [(-b + sign * math.sqrt(discriminant)) / (2.0 * a) for sign in (1.0, -1.0)]
    valid = [t for t in roots if t >= 0]
    return min(valid) if valid else None


def intercept_point(hunter_pos: Vec3, target_pos: Vec3, target_vel: Vec3, hunter_speed: float) -> Vec3:
    """Aim point for lead pursuit; falls back to the current target position."""
    t = intercept_time(hunter_pos, target_pos, target_vel, hunter_speed)
    return target_pos if t is None else add(target_pos, scale(target_vel, t))


def alive_items(team: dict[int, dict]) -> Iterable[tuple[int, dict]]:
    return ((drone_id, state) for drone_id, state in sorted(team.items()) if state["alive"])
