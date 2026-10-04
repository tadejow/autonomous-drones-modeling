"""SITL-free physics: a point mass whose velocity follows the command with lag.

Model (per drone)::

    dv/dt = clip_a((v_cmd - v) / tau),   |dv/dt| <= a_max
    dp/dt = v

The linear part is integrated exactly, ``v(t+dt) = v_cmd + (v - v_cmd) e^(-dt/tau)``,
and the resulting change of velocity is clipped to ``a_max * dt``. ``tau`` and
``a_max`` should be calibrated against a step response recorded in SITL.

A match of 120 s at 10 Hz runs in a fraction of a second, which makes the
backend suitable for unit tests, tournaments and balance tuning.
"""

from __future__ import annotations

import math

import numpy as np

from pipeline.drones_battle.core.config import ArenaConfig, KillMode
from pipeline.drones_battle.core.layout import drone_slots
from pipeline.drones_battle.core.types import RawState, Vec3

GRAVITY_MPS2 = 9.81


class KinematicBackend:
    simulated_time = True

    def __init__(self, config: ArenaConfig, seed: int | None = None, realtime: bool = False) -> None:
        self.config = config
        self.realtime = realtime
        params = config.kinematic
        self.tau = params.tau_s
        self.max_accel = params.max_accel_mps2
        self.noise_std = params.position_noise_std_m
        self.rng = np.random.default_rng(params.seed if seed is None else seed)
        self.pos: dict[int, np.ndarray] = {}
        self.vel: dict[int, np.ndarray] = {}
        self.cmd: dict[int, np.ndarray] = {}
        self.mode: dict[int, str] = {}
        self.time = 0.0

    def connect(self) -> None:
        for slot in drone_slots(self.config):
            self.pos[slot.drone_id] = np.array(slot.start_ned, dtype=float)
            self.vel[slot.drone_id] = np.zeros(3)
            self.cmd[slot.drone_id] = np.zeros(3)
            self.mode[slot.drone_id] = "guided"

    def perturb_start(self, max_offset_m: float) -> None:
        """Random horizontal offset of every start position (used by tournaments)."""
        for drone_id in self.pos:
            self.pos[drone_id][:2] += self.rng.uniform(-max_offset_m, max_offset_m, size=2)

    def takeoff_all(self, altitude_m: float) -> None:
        for drone_id in self.pos:
            self.pos[drone_id][2] = -altitude_m
            self.vel[drone_id][:] = 0.0

    def read_states(self) -> dict[int, RawState]:
        states: dict[int, RawState] = {}
        for drone_id, position in self.pos.items():
            noisy = position.copy()
            if self.noise_std > 0:
                noisy += self.rng.normal(0.0, self.noise_std, size=3)
            states[drone_id] = RawState(pos=noisy, vel=self.vel[drone_id].copy())
        return states

    def send_velocity(self, drone_id: int, velocity_ned: Vec3) -> None:
        if self.mode[drone_id] == "guided":
            self.cmd[drone_id] = np.array(velocity_ned, dtype=float)

    def kill(self, drone_id: int, mode: KillMode) -> None:
        self.mode[drone_id] = mode
        self.cmd[drone_id] = np.zeros(3)

    def start_clock(self) -> None:
        self.time = 0.0

    def now(self) -> float:
        return self.time

    def step(self, dt: float) -> None:
        land_speed = self.config.kinematic.land_speed_mps
        for drone_id in self.pos:
            position, velocity = self.pos[drone_id], self.vel[drone_id]
            mode = self.mode[drone_id]
            if mode == "freefall":
                velocity[:2] *= math.exp(-dt / max(self.tau, 1e-3))
                velocity[2] += GRAVITY_MPS2 * dt
            else:
                command = self.cmd[drone_id] if mode == "guided" else np.array([0.0, 0.0, land_speed])
                desired = command + (velocity - command) * math.exp(-dt / max(self.tau, 1e-3))
                change = desired - velocity
                limit = self.max_accel * dt
                norm = float(np.linalg.norm(change))
                if norm > limit:
                    change *= limit / norm
                velocity += change
            position += velocity * dt
            if position[2] > 0.0:  # ground (NED: down is positive)
                position[2] = 0.0
                velocity[:] = 0.0
        self.time += dt

    def end_match(self) -> None:
        pass

    def shutdown(self) -> None:
        for drone_id in self.pos:
            self.kill(drone_id, "land")
