"""Decoy attack: two flankers draw the defenders aside, the runner goes in late.

* Flankers (the two outer drones) fly wide arcs through waypoints east and
  west of the base and then dive at the target.
* The runner (middle drone) hovers for ``RUNNER_DELAY`` seconds and then flies
  straight at the target, hoping the defenders are busy with the flankers.

Module-level state is allowed: the arena keeps the strategy process alive for
the whole match. Here we remember which waypoint each flanker already passed.
"""

from __future__ import annotations

from pipeline.drones_battle.strategy_utils import Vec3, alive_items, distance, fly_towards

SPEED = 10.0
CRUISE_ALT = 15.0
RUNNER_DELAY = 9.0
FLANK_EAST = 45.0
FLANK_NORTH = 55.0

_passed_waypoint: set[int] = set()


def compute_commands(
    my_team: dict, enemy_team: dict, target_pos: tuple, current_time: float, game: dict | None = None
) -> dict:
    """``game`` (optional) carries the rules; the arena passes it when the parameter exists."""
    speed = game["max_speed"] if game else SPEED
    ids = sorted(my_team)
    runner = ids[len(ids) // 2]
    commands: dict[int, Vec3] = {}
    for drone_id, state in alive_items(my_team):
        position: Vec3 = state["pos"]
        if drone_id == runner:
            if current_time < RUNNER_DELAY:
                commands[drone_id] = (0.0, 0.0, 0.0)
                continue
            goal = target_pos
        else:
            side = 1.0 if ids.index(drone_id) > ids.index(runner) else -1.0
            waypoint = (FLANK_NORTH, side * FLANK_EAST, -CRUISE_ALT)
            if distance(position, waypoint) < 8.0:
                _passed_waypoint.add(drone_id)
            goal = target_pos if drone_id in _passed_waypoint else waypoint
        commands[drone_id] = fly_towards(position, goal, speed, arrive=False)
    return commands
