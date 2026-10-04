"""Evasive attackers: attraction to the target plus Coulomb repulsion from defenders.

The repulsion term ``K * r / |r|^3`` is the same one used for collision
avoidance in the Cucker-Smale scripts of module 05; here it pushes attackers
away from the defenders while they keep heading for the target.
"""

from __future__ import annotations

from pipeline.drones_battle.strategy_utils import (
    Vec3, add, alive_items, clamp_norm, distance, fly_towards, norm, scale, sub,
)

SPEED = 10.0
CRUISE_ALT = 15.0
REPULSION_GAIN = 900.0
REPULSION_RANGE = 30.0
EXCLUSION_RADIUS = 8.0  # defenders inside this sphere around the target cannot shoot


def compute_commands(
    my_team: dict, enemy_team: dict, target_pos: tuple, current_time: float, game: dict | None = None
) -> dict:
    """``game`` (optional) carries the rules; the arena passes it when the parameter exists."""
    speed = game["max_speed"] if game else SPEED
    threats = [
        s["pos"] for _, s in alive_items(enemy_team)
        if distance(s["pos"], target_pos) >= EXCLUSION_RADIUS
    ]
    commands: dict[int, Vec3] = {}
    for drone_id, state in alive_items(my_team):
        position: Vec3 = state["pos"]
        horizontal = norm((target_pos[0] - position[0], target_pos[1] - position[1], 0.0))
        goal = target_pos if horizontal < 30.0 else (target_pos[0], target_pos[1], -CRUISE_ALT)
        velocity = fly_towards(position, goal, speed)
        for threat in threats:
            offset = sub(position, threat)
            dist = norm(offset)
            if 1e-3 < dist < REPULSION_RANGE:
                velocity = add(velocity, scale(offset, REPULSION_GAIN / dist ** 3))
        commands[drone_id] = clamp_norm(velocity, speed)
    return commands
