"""Guard defenders: stand between the base and "their" attacker, then strike.

Each defender guards the attacker that is currently closest to the base among
those not yet guarded. It keeps a point on the segment base -> attacker at
``GUARD_RADIUS`` from the base (outside the 8 m no-shoot sphere), and switches
to lead pursuit once the attacker comes within ``STRIKE_RANGE``.

Background: in the Target-Attacker-Defender game the defender wins if it can
reach every point of the attacker's path to the target first. The set of points
reached simultaneously by both players (equal speeds) is the perpendicular
bisector of their positions (Apollonius circle for unequal speeds); standing
on the base-attacker line keeps the defender on the winning side of it.
"""

from __future__ import annotations

from pipeline.drones_battle.strategy_utils import (
    Vec3, add, alive_items, distance, fly_towards, intercept_point, scale, sub, unit,
)

SPEED = 10.0
GUARD_RADIUS = 14.0
STRIKE_RANGE = 30.0


def compute_commands(
    my_team: dict, enemy_team: dict, target_pos: tuple, current_time: float, game: dict | None = None
) -> dict:
    """``game`` (optional) carries the rules; the arena passes it when the parameter exists."""
    speed = game["max_speed"] if game else SPEED
    defenders = [(i, s["pos"]) for i, s in alive_items(my_team)]
    attackers = sorted(
        ((i, s["pos"], s["vel"]) for i, s in alive_items(enemy_team)),
        key=lambda a: distance(a[1], target_pos),
    )
    commands: dict[int, Vec3] = {}
    free = list(defenders)
    for index in range(len(defenders)):
        if not attackers:
            break
        _, a_pos, a_vel = attackers[index % len(attackers)]
        drone_id, position = min(free, key=lambda d: distance(d[1], a_pos))
        free.remove((drone_id, position))
        if distance(position, a_pos) < STRIKE_RANGE:
            aim = intercept_point(position, a_pos, a_vel, speed)
            commands[drone_id] = fly_towards(position, aim, speed, arrive=False)
        else:
            guard = add(target_pos, scale(unit(sub(a_pos, target_pos)), GUARD_RADIUS))
            commands[drone_id] = fly_towards(position, guard, speed)
    for drone_id, position in free:
        commands[drone_id] = fly_towards(position, add(target_pos, (GUARD_RADIUS, 0.0, 0.0)), speed)
    return commands
