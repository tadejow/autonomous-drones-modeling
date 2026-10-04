"""Student template: ATTACKERS strategy.

Rules in short (details in README.md):
* you control the drones listed in ``my_team`` (1-3 in 3 vs 3, 1-5 in 5 vs 5;
  never hard-code the ids); the goal is to bring any of them within 5 m of
  ``target_pos`` (10 m above the defenders' base),
* an attacker that comes within 2 m of an active defender is destroyed; the
  defenders are kamikaze drones, so that defender is destroyed as well,
* the match lasts 120 s; commands are capped at 10 m/s (3 m/s vertically).

Coordinates: NED in metres relative to the defenders' base (N = north,
E = east, D = DOWN). A positive vD means DESCENDING.

The function is called 10 times per second. It must be fast (< 30 ms) and must
not talk to the drones directly; the arena does that for you.
"""

import math

CRUISE_SPEED = 10.0
CRUISE_ALT = 15.0


def compute_commands(my_team: dict, enemy_team: dict, target_pos: tuple, current_time: float) -> dict:
    """Returns ``{drone_id: (v_north, v_east, v_down)}`` in m/s for the drones of my team.

    Args:
        my_team: ``{drone_id: {"pos": (N, E, D), "vel": (VN, VE, VD), "alive": bool}}``.
        enemy_team: the same format for the other team.
        target_pos: ``(N, E, D)`` of the target (the defenders' base).
        current_time: seconds since the start of the battle.
    """
    commands = {}
    for drone_id, state in my_team.items():
        if not state["alive"]:
            continue
        n, e, d = state["pos"]
        dn, de, dd = target_pos[0] - n, target_pos[1] - e, target_pos[2] - d
        horizontal = math.hypot(dn, de)
        # Fly level at CRUISE_ALT, then descend to the target height in the last 30 m.
        goal_d = target_pos[2] if horizontal < 30.0 else -CRUISE_ALT
        speed = min(CRUISE_SPEED, math.sqrt(dn * dn + de * de + dd * dd))
        v_n = speed * dn / horizontal if horizontal > 1e-6 else 0.0
        v_e = speed * de / horizontal if horizontal > 1e-6 else 0.0
        v_d = 1.0 * (goal_d - d)  # proportional altitude control (positive = down)
        commands[drone_id] = (v_n, v_e, v_d)
    return commands
