"""Screen and sweep: a defensive line that only breaks for real threats.

* Idle defenders form a screen: a line across the attack corridor at
  ``SCREEN_NORTH`` metres north of the base, evenly spread east-west.
* A threat is an attacker whose time to the target (distance / its speed,
  at least our speed) is below ``HORIZON``, or that is closing in on one of
  our defenders (likely a rammer). Threats are ranked by time to the target.
* Each threat gets the free defender that intercepts it soonest; one defender
  per attacker (kamikaze rule), so the screen keeps covering the rest.
* Defenders are never sent into the 8 m no-shoot sphere: aim points closer to
  the base than ``MIN_AIM_RADIUS`` are pushed outwards.
"""

from __future__ import annotations

import math

from pipeline.drones_battle.strategy_utils import (
    Vec3, add, alive_items, distance, dot, fly_towards, intercept_point, intercept_time, norm, scale, sub,
    unit,
)

SPEED = 10.0
SCREEN_NORTH = 30.0
SCREEN_WIDTH = 50.0
SCREEN_ALT = 15.0
HORIZON = 14.0
RAM_RANGE = 25.0
MIN_AIM_RADIUS = 10.0


def _screen(ids: list[int], target_pos: tuple) -> dict[int, Vec3]:
    spots: dict[int, Vec3] = {}
    for index, drone_id in enumerate(ids):
        fraction = 0.5 if len(ids) == 1 else index / (len(ids) - 1)
        east = -SCREEN_WIDTH / 2 + SCREEN_WIDTH * fraction
        spots[drone_id] = (target_pos[0] + SCREEN_NORTH, target_pos[1] + east, -SCREEN_ALT)
    return spots


def _outside_base(point: Vec3, target_pos: tuple) -> Vec3:
    offset = sub(point, target_pos)
    if norm(offset) >= MIN_AIM_RADIUS:
        return point
    direction = unit(offset) if norm(offset) > 1e-6 else (1.0, 0.0, 0.0)
    return add(target_pos, scale(direction, MIN_AIM_RADIUS))


def compute_commands(
    my_team: dict, enemy_team: dict, target_pos: tuple, current_time: float, game: dict | None = None
) -> dict:
    speed = game["max_speed"] if game else SPEED
    defenders = dict(alive_items(my_team))
    spots = _screen(sorted(defenders, key=lambda i: defenders[i]["pos"][1]), target_pos)

    threats = []
    for _, state in alive_items(enemy_team):
        a_pos, a_vel = state["pos"], state["vel"]
        eta = distance(a_pos, target_pos) / max(norm(a_vel), speed * 0.5)
        ramming = any(
            distance(a_pos, d["pos"]) < RAM_RANGE and dot(a_vel, sub(d["pos"], a_pos)) > 0
            for d in defenders.values()
        )
        if eta < HORIZON or ramming:
            threats.append((eta, a_pos, a_vel))
    threats.sort(key=lambda threat: threat[0])

    commands: dict[int, Vec3] = {}
    free = set(defenders)
    for _, a_pos, a_vel in threats:
        if not free:
            break

        def time_to_catch(d: int) -> float:
            t = intercept_time(defenders[d]["pos"], a_pos, a_vel, speed)
            return math.inf if t is None else t

        chosen = min(free, key=time_to_catch)
        free.discard(chosen)
        position = defenders[chosen]["pos"]
        aim = a_pos if distance(position, a_pos) < 6.0 else intercept_point(position, a_pos, a_vel, speed)
        commands[chosen] = fly_towards(position, _outside_base(aim, target_pos), speed, arrive=False)

    for drone_id in free:
        commands[drone_id] = fly_towards(defenders[drone_id]["pos"], spots[drone_id], speed)
    return commands
