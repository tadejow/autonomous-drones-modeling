"""Interface between the orchestrator and the physics (ArduPilot SITL or a simple model)."""

from __future__ import annotations

from typing import Protocol

from pipeline.drones_battle.core.config import KillMode
from pipeline.drones_battle.core.types import RawState, Vec3


class PhysicsBackend(Protocol):
    realtime: bool
    """True when time flows on its own (SITL); False when ``step`` advances it."""

    def connect(self) -> None:
        """Opens connections and prepares every drone."""

    def takeoff_all(self, altitude_m: float) -> None:
        """Arms all drones and blocks until every one reaches ``altitude_m``."""

    def read_states(self) -> dict[int, RawState]:
        """Latest NED position and velocity of every drone (relative to the defenders' base)."""

    def send_velocity(self, drone_id: int, velocity_ned: Vec3) -> None:
        """Velocity command in the NED frame (m/s)."""

    def kill(self, drone_id: int, mode: KillMode) -> None:
        """Takes a shot-down drone out of the fight (once)."""

    def step(self, dt: float) -> None:
        """Advances simulated time (no-op for SITL)."""

    def now(self) -> float:
        """Seconds since the start of the battle phase."""

    def start_clock(self) -> None:
        """Marks t = 0 of the battle."""

    def shutdown(self) -> None:
        """Lands or returns all drones and closes connections."""
