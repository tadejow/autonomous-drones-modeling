"""Zone defence with a reserve.

* Defenders hold guard posts on a ring around the base, spread over the
  northern half (where the attack comes from); the last defender is a reserve
  waiting close to the base, just outside the 8 m no-shoot sphere.
* Threat ranking: attackers sorted by their time to the target (distance /
  speed). The most urgent threat is matched first with the free guard that can
  intercept it soonest (lead pursuit, ``strategy_utils.intercept_time``).
* A guard engages only attackers inside ``ENGAGE_RADIUS``; the reserve only
  attackers inside ``RESERVE_RADIUS`` (the last line), or when no guard is free.
* Kamikaze rule: one defender stops at most one attacker, so two defenders
  are never sent after the same attacker while another one is unmatched.
"""

from __future__ import annotations

import math

from pipeline.drones_battle.strategy_utils import (
    Vec3, alive_items, distance, fly_towards, intercept_point, intercept_time,
)

SPEED = 10.0
POST_RADIUS = 22.0
POST_ALT = 14.0
POST_FAN_DEG = 150.0
RESERVE_DISTANCE = 11.0
ENGAGE_RADIUS = 70.0
RESERVE_RADIUS = 30.0


def _posts(ids: list[int], target_pos: tuple) -> dict[int, Vec3]:
    guards, reserve = ids[:-1], ids[-1]
    posts: dict[int, Vec3] = {}
    for index, drone_id in enumerate(guards):
        fraction = 0.5 if len(guards) == 1 else index / (len(guards) - 1)
        bearing = math.radians(-POST_FAN_DEG / 2 + POST_FAN_DEG * fraction)
        posts[drone_id] = (
            target_pos[0] + POST_RADIUS * math.cos(bearing),
            target_pos[1] + POST_RADIUS * math.sin(bearing),
            -POST_ALT,
        )
    posts[reserve] = (target_pos[0] + RESERVE_DISTANCE, target_pos[1], target_pos[2])
    return posts


def compute_commands(
    my_team: dict, enemy_team: dict, target_pos: tuple, current_time: float, game: dict | None = None
) -> dict:
    speed = game["max_speed"] if game else SPEED
    all_ids = sorted(my_team)
    posts = _posts(all_ids, target_pos)
    reserve = all_ids[-1]
    defenders = dict(alive_items(my_team))
    attackers = sorted(
        ((i, s["pos"], s["vel"]) for i, s in alive_items(enemy_team)),
        key=lambda a: distance(a[1], target_pos),
    )

    commands: dict[int, Vec3] = {}
    free = set(defenders)
    for _, a_pos, a_vel in attackers:
        to_target = distance(a_pos, target_pos)
        if to_target > ENGAGE_RADIUS:
            continue
        candidates = [d for d in free if d != reserve or to_target < RESERVE_RADIUS or len(free) == 1]
        if not candidates:
            continue

        def time_to_catch(d: int) -> float:
            t = intercept_time(defenders[d]["pos"], a_pos, a_vel, speed)
            return math.inf if t is None else t

        chosen = min(candidates, key=time_to_catch)
        free.discard(chosen)
        position = defenders[chosen]["pos"]
        aim = a_pos if distance(position, a_pos) < 6.0 else intercept_point(position, a_pos, a_vel, speed)
        commands[chosen] = fly_towards(position, aim, speed, arrive=False)

    for drone_id in free:
        commands[drone_id] = fly_towards(defenders[drone_id]["pos"], posts[drone_id], speed)
    return commands
