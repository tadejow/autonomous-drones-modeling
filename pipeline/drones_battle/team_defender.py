"""Student template: DEFENDERS strategy.

Rules in short (details in README.md):
* you control drones 4, 5, 6; they are KAMIKAZE drones: when one comes within
  2 m of an attacker, both are destroyed (one defender = at most one attacker),
* you win when all attackers are down or after 120 s,
* ``target_pos`` is your own base; a defender inside the 8 m sphere around it
  cannot shoot (anti-camping rule) and is pushed out by the arena,
* commands are capped at 10 m/s (3 m/s vertically).

Coordinates: NED in metres relative to your base (N = north, E = east,
D = DOWN). A positive vD means DESCENDING.

The function is called 10 times per second. It must be fast (< 30 ms) and must
not talk to the drones directly; the arena does that for you.
"""

import math

CHASE_SPEED = 10.0
GUARD_DISTANCE = 15.0


def compute_commands(my_team: dict, enemy_team: dict, target_pos: tuple, current_time: float) -> dict:
    """Returns ``{drone_id: (v_north, v_east, v_down)}`` in m/s for the drones of my team.

    Args:
        my_team: ``{drone_id: {"pos": (N, E, D), "vel": (VN, VE, VD), "alive": bool}}``.
        enemy_team: the same format for the other team.
        target_pos: ``(N, E, D)`` of the base you defend.
        current_time: seconds since the start of the battle.
    """
    attackers = [s["pos"] for s in enemy_team.values() if s["alive"]]
    commands = {}
    for drone_id, state in my_team.items():
        if not state["alive"]:
            continue
        n, e, d = state["pos"]
        if attackers:
            # Pure pursuit of the nearest living attacker.
            goal = min(attackers, key=lambda p: (p[0] - n) ** 2 + (p[1] - e) ** 2 + (p[2] - d) ** 2)
        else:
            goal = (target_pos[0] + GUARD_DISTANCE, target_pos[1], target_pos[2])
        dn, de, dd = goal[0] - n, goal[1] - e, goal[2] - d
        dist = math.sqrt(dn * dn + de * de + dd * dd)
        if dist < 1e-6:
            commands[drone_id] = (0.0, 0.0, 0.0)
            continue
        speed = CHASE_SPEED if attackers else min(CHASE_SPEED, dist)  # never brake while chasing
        commands[drone_id] = (speed * dn / dist, speed * de / dist, speed * dd / dist)
    return commands
