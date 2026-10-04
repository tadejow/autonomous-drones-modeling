"""Pincer attack: stage on a ring around the target, then strike from all sides at once.

1. Every attacker gets its own approach bearing; the bearings are spread over
   ``FAN_DEG`` degrees around the direction the attackers come from (north),
   sorted by the east coordinate so that the paths do not cross.
2. Attackers fly to staging points on a ring of radius ``STAGE_RADIUS`` and
   wait there until everybody has arrived (or ``STAGE_TIMEOUT`` passes).
3. Then all of them dive at the target simultaneously, along different
   directions. With kamikaze defenders one defender stops at most one
   attacker, so the defence needs a perfect assignment under time pressure.

While staging, attackers dodge nearby defenders (Coulomb repulsion as in module 05).
Module-level state is reset by the arena before every match.
"""

from __future__ import annotations

import math

from pipeline.drones_battle.strategy_utils import (
    Vec3, add, alive_items, clamp_norm, distance, fly_towards, norm, scale, sub,
)

SPEED = 10.0
STAGE_RADIUS = 42.0
STAGE_ALT = 16.0
FAN_DEG = 160.0
STAGE_TOLERANCE = 6.0
STAGE_TIMEOUT = 30.0
DODGE_RANGE = 18.0
DODGE_GAIN = 500.0

_bearings: dict[int, float] = {}
_strike = False


def _assign_bearings(my_team: dict) -> None:
    ids = sorted(my_team, key=lambda i: my_team[i]["pos"][1])  # west -> east
    count = len(ids)
    for index, drone_id in enumerate(ids):
        fraction = 0.5 if count == 1 else index / (count - 1)
        # Bearing measured from north, negative = west.
        _bearings[drone_id] = math.radians(-FAN_DEG / 2 + FAN_DEG * fraction)


def _staging_point(drone_id: int, target_pos: tuple) -> Vec3:
    bearing = _bearings[drone_id]
    return (
        target_pos[0] + STAGE_RADIUS * math.cos(bearing),
        target_pos[1] + STAGE_RADIUS * math.sin(bearing),
        -STAGE_ALT,
    )


def compute_commands(
    my_team: dict, enemy_team: dict, target_pos: tuple, current_time: float, game: dict | None = None
) -> dict:
    global _strike
    speed = game["max_speed"] if game else SPEED
    if not _bearings:
        _assign_bearings(my_team)

    alive = dict(alive_items(my_team))
    if not _strike:
        staged = all(
            distance(state["pos"], _staging_point(i, target_pos)) < STAGE_TOLERANCE for i, state in alive.items()
        )
        _strike = staged or current_time > STAGE_TIMEOUT

    threats = [s["pos"] for _, s in alive_items(enemy_team)]
    commands: dict[int, Vec3] = {}
    for drone_id, state in alive.items():
        position: Vec3 = state["pos"]
        if _strike:
            commands[drone_id] = fly_towards(position, target_pos, speed, arrive=False)
            continue
        velocity = fly_towards(position, _staging_point(drone_id, target_pos), speed)
        for threat in threats:
            offset = sub(position, threat)
            dist = norm(offset)
            if 1e-3 < dist < DODGE_RANGE:
                velocity = add(velocity, scale(offset, DODGE_GAIN / dist ** 3))
        commands[drone_id] = clamp_norm(velocity, speed)
    return commands
