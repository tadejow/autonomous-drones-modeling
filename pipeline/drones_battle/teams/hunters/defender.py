"""Lead-pursuit defenders with an optimal one-to-one assignment.

1. Every defender-attacker pair gets an interception time from the quadratic
   equation of ``07_b_pursuit.py`` (``strategy_utils.intercept_time``).
2. With 3 defenders and at most 3 attackers we simply try all permutations
   (3! = 6) and keep the one minimising the latest interception (minimax),
   then the total time. No SciPy needed.
3. Each defender flies at the predicted interception point, not at the
   attacker's current position.
"""

from __future__ import annotations

import itertools
import math

from pipeline.drones_battle.strategy_utils import (
    Vec3, alive_items, distance, fly_towards, intercept_point, intercept_time,
)

SPEED = 10.0
PATROL_RADIUS = 15.0


def _assignment(
    defenders: list[tuple[int, Vec3]], attackers: list[tuple[int, Vec3, Vec3]], speed: float
) -> dict[int, int]:
    best: tuple[float, float] | None = None
    best_pairs: dict[int, int] = {}
    # Pad attackers so that every defender gets someone (spare defenders double up).
    pool = attackers * math.ceil(len(defenders) / max(len(attackers), 1))
    for chosen in itertools.permutations(range(len(pool)), len(defenders)):
        times = []
        for (_, d_pos), index in zip(defenders, chosen):
            _, a_pos, a_vel = pool[index]
            t = intercept_time(d_pos, a_pos, a_vel, speed)
            times.append(1e6 if t is None else t)
        cost = (max(times), sum(times))
        if best is None or cost < best:
            best = cost
            best_pairs = {d_id: pool[index][0] for (d_id, _), index in zip(defenders, chosen)}
    return best_pairs


def compute_commands(
    my_team: dict, enemy_team: dict, target_pos: tuple, current_time: float, game: dict | None = None
) -> dict:
    """``game`` (optional) carries the rules; the arena passes it when the parameter exists."""
    speed = game["max_speed"] if game else SPEED
    defenders = [(i, s["pos"]) for i, s in alive_items(my_team)]
    attackers = [(i, s["pos"], s["vel"]) for i, s in alive_items(enemy_team)]
    commands: dict[int, Vec3] = {}
    if not attackers:
        for index, (drone_id, position) in enumerate(defenders):
            angle = 2 * math.pi * index / max(len(defenders), 1)
            spot = (PATROL_RADIUS * math.cos(angle), PATROL_RADIUS * math.sin(angle), target_pos[2])
            commands[drone_id] = fly_towards(position, spot, speed)
        return commands

    by_id = {a_id: (a_pos, a_vel) for a_id, a_pos, a_vel in attackers}
    for drone_id, attacker_id in _assignment(defenders, attackers, speed).items():
        position = dict(defenders)[drone_id]
        a_pos, a_vel = by_id[attacker_id]
        aim = intercept_point(position, a_pos, a_vel, speed)
        if distance(position, a_pos) < 6.0:
            aim = a_pos  # close range: pure pursuit is more robust to noise
        commands[drone_id] = fly_towards(position, aim, speed, arrive=False)
    return commands
