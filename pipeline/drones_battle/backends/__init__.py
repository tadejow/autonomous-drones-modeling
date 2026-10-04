"""Physics backends. ``sitl`` imports DroneKit lazily, so the rest works without it."""

from __future__ import annotations

from pipeline.drones_battle.backends.base import PhysicsBackend
from pipeline.drones_battle.core.config import ArenaConfig


def make_backend(name: str, config: ArenaConfig, seed: int | None = None, realtime: bool = False) -> PhysicsBackend:
    if name == "sitl":
        from pipeline.drones_battle.backends.sitl import SitlBackend

        return SitlBackend(config)
    if name == "kinematic":
        from pipeline.drones_battle.backends.kinematic import KinematicBackend

        return KinematicBackend(config, seed=seed, realtime=realtime)
    raise ValueError(f"Unknown backend {name!r} (use 'sitl' or 'kinematic')")
