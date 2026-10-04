"""Wolfpack: rammers trade themselves for defenders, runners finish the job.

Kamikaze defenders die together with the attacker they hit, and the collision
rule is symmetric: an attacker that flies into an active defender also takes
it down. The wolfpack uses this deliberately.

* Rammers (all but ``RUNNERS`` drones) hunt the defenders with lead pursuit,
  one rammer per defender (greedy by interception time). Defenders hiding in
  the 8 m no-shoot sphere cannot be rammed, so rammers wait just outside it.
* Runners (the two outermost drones) hold back on both flanks and go for the
  target from two sides once the defence is thinned out: when no more than
  ``GO_WHEN_DEFENDERS`` active defenders are left, or after ``RUNNER_TIMEOUT``.
"""

from __future__ import annotations

import math

from pipeline.drones_battle.strategy_utils import (
    Vec3, alive_items, distance, fly_towards, intercept_point, intercept_time,
)

SPEED = 10.0
RUNNERS = 2
GO_WHEN_DEFENDERS = 1
RUNNER_TIMEOUT = 40.0
HOLD_NORTH = 95.0
HOLD_EAST = 45.0
HOLD_ALT = 18.0
EXCLUSION_RADIUS = 8.0

_runners: list[int] = []
_go = False


def compute_commands(
    my_team: dict, enemy_team: dict, target_pos: tuple, current_time: float, game: dict | None = None
) -> dict:
    global _go
    speed = game["max_speed"] if game else SPEED
    exclusion = game["defender_exclusion_radius"] if game else EXCLUSION_RADIUS
    if not _runners:
        by_east = sorted(my_team, key=lambda i: my_team[i]["pos"][1])
        _runners.extend([by_east[0], by_east[-1]][:RUNNERS])

    attackers = dict(alive_items(my_team))
    defenders = [
        (i, s["pos"], s["vel"]) for i, s in alive_items(enemy_team)
        if distance(s["pos"], target_pos) > exclusion
    ]
    rammers = [i for i in attackers if i not in _runners]
    if not _go:
        _go = len(defenders) <= GO_WHEN_DEFENDERS or current_time > RUNNER_TIMEOUT or not rammers

    commands: dict[int, Vec3] = {}
    for index, drone_id in enumerate(i for i in _runners if i in attackers):
        position = attackers[drone_id]["pos"]
        if _go:
            commands[drone_id] = fly_towards(position, target_pos, speed, arrive=False)
        else:
            side = -1.0 if index == 0 else 1.0
            commands[drone_id] = fly_towards(position, (HOLD_NORTH, side * HOLD_EAST, -HOLD_ALT), speed)

    free_targets = list(defenders)
    for drone_id in sorted(rammers, key=lambda i: attackers[i]["pos"][0]):
        position = attackers[drone_id]["pos"]
        if not free_targets:
            commands[drone_id] = fly_towards(position, target_pos, speed, arrive=False)
            continue

        def time_to_ram(target: tuple) -> float:
            t = intercept_time(position, target[1], target[2], speed)
            return math.inf if t is None else t

        prey = min(free_targets, key=time_to_ram)
        free_targets.remove(prey)
        aim = intercept_point(position, prey[1], prey[2], speed)
        commands[drone_id] = fly_towards(position, aim, speed, arrive=False)
    return commands
