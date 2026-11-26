"""Start positions of all drones, shared by the backends and ``start_arena.sh``.

Run as a script to print one line per drone for the shell launcher::

    python3 -m pipeline.drones_battle.core.layout [--config arena_config.toml]
    # mode udp|tcp
    # events <port of the map's battle events>
    # instance sysid lat lon alt_amsl heading orchestrator_port map_port team
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass

from pipeline.drones_battle.core.config import ArenaConfig, load_config
from pipeline.drones_battle.core.types import Vec3
from pipeline.math.geodesy import ned_to_gps


@dataclass(frozen=True)
class DroneSlot:
    drone_id: int
    instance: int
    team: str
    start_ned: Vec3
    heading_deg: float


def drone_slots(config: ArenaConfig) -> list[DroneSlot]:
    """Attackers stand in a west-east row at the north end, defenders at the base (south)."""
    arena, game = config.arena, config.game
    slots: list[DroneSlot] = []
    for team, ids, north, heading in (
        ("attackers", game.attacker_ids, arena.attacker_start_north_m, 180.0),
        ("defenders", game.defender_ids, 0.0, 0.0),
    ):
        middle = (len(ids) - 1) / 2.0
        for index, drone_id in enumerate(ids):
            east = (index - middle) * arena.start_spacing_m
            slots.append(DroneSlot(drone_id, drone_id - 1, team, (north, east, 0.0), heading))
    return sorted(slots, key=lambda s: s.instance)


def connection_string(config: ArenaConfig, slot: DroneSlot) -> str:
    sitl = config.sitl
    if sitl.connection_mode == "tcp":
        return f"tcp:127.0.0.1:{sitl.tcp_base_port + sitl.port_step * slot.instance}"
    return f"udp:127.0.0.1:{sitl.udp_base_port + sitl.port_step * slot.instance}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=None)
    config = load_config(parser.parse_args().config)
    sitl = config.sitl
    print(f"mode {sitl.connection_mode}")
    print(f"events {sitl.map_events_port}")
    for slot in drone_slots(config):
        lat, lon, alt = ned_to_gps(config.arena.origin, *slot.start_ned)
        if sitl.connection_mode == "tcp":
            # Orchestrator on SERIAL0 (5760 + 10 i), map on SERIAL1 (5762 + 10 i).
            orchestrator_port = sitl.tcp_base_port + sitl.port_step * slot.instance
            map_port = orchestrator_port + 2
        else:
            orchestrator_port = sitl.udp_base_port + sitl.port_step * slot.instance
            map_port = sitl.map_udp_base_port + sitl.port_step * slot.instance
        print(
            f"{slot.instance} {slot.drone_id} {lat:.7f} {lon:.7f} {alt:.1f} "
            f"{slot.heading_deg:.0f} {orchestrator_port} {map_port} {slot.team}"
        )


if __name__ == "__main__":
    main()
