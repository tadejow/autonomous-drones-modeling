"""ArduPilot SITL backend built on DroneKit.

Key decisions (see the implementation plan, chapter 6.2):

* the six vehicles are connected and armed in parallel threads,
* positions come from our own ``GLOBAL_POSITION_INT`` listener, so latitude,
  longitude, altitude and velocity always belong to the same message,
* telemetry older than ``stale_after_s`` marks the drone as stale (the referee
  then ignores it in that tick),
* velocity commands are sent every tick, also when unchanged, so that GUIDED
  mode never times out (``GUID_TIMEOUT``).
"""

from __future__ import annotations

import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import Any, Callable, Optional

import numpy as np

from pipeline.drones_battle.core import compat  # noqa: F401  (must precede dronekit)
from pipeline.drones_battle.core.config import ArenaConfig, KillMode
from pipeline.drones_battle.core.layout import DroneSlot, connection_string, drone_slots
from pipeline.drones_battle.core.types import RawState, Vec3
from pipeline.math.geodesy import gps_to_ned, ned_to_gps

from dronekit import LocationGlobalRelative, Vehicle, VehicleMode, connect  # noqa: E402
from pymavlink import mavutil  # noqa: E402

VELOCITY_ONLY_MASK = 0b0000111111000111
FORCE_DISARM_MAGIC = 21196
GLOBAL_POSITION_INT_ID = 33
SLOT_TOLERANCE_M = 3.0


@dataclass
class _Telemetry:
    pos: np.ndarray
    vel: np.ndarray
    received_at: float


class SitlBackend:
    realtime = True
    simulated_time = False

    def __init__(self, config: ArenaConfig) -> None:
        self.config = config
        self.slots: dict[int, DroneSlot] = {s.drone_id: s for s in drone_slots(config)}
        self.vehicles: dict[int, Vehicle] = {}
        self._telemetry: dict[int, _Telemetry] = {}
        self._lock = threading.Lock()
        self._killed: set[int] = set()
        self._t0 = time.monotonic()

    # ------------------------------------------------------------------ setup
    def connect(self) -> None:
        """Connects once; on later calls (next match of a tournament) brings the drones home first."""
        if self.vehicles:
            self._killed.clear()
            self._return_to_slots()
            return
        with ThreadPoolExecutor(max_workers=len(self.slots)) as pool:
            futures = {i: pool.submit(self._connect_one, slot) for i, slot in self.slots.items()}
            for drone_id, future in futures.items():
                self.vehicles[drone_id] = future.result()
        print(f"Connected to {len(self.vehicles)} vehicles.")

    def _connect_one(self, slot: DroneSlot) -> Vehicle:
        address = connection_string(self.config, slot)
        print(f"  drone {slot.drone_id} ({slot.team}) -> {address}")
        vehicle = connect(address, wait_ready=True, timeout=self.config.sitl.connect_timeout_s)
        # BATT_FS_*, WPNAV_SPEED etc. come from arena.parm at SITL start; setting them here again
        # timed out with six vehicles connecting at once ("timeout setting parameter WPNAV_SPEED").
        if self.config.sitl.wind_speed_mps > 0:
            vehicle.parameters["SIM_WIND_SPD"] = self.config.sitl.wind_speed_mps
            vehicle.parameters["SIM_WIND_DIR"] = self.config.sitl.wind_direction_deg
        self._request_position_rate(vehicle)
        origin = self.config.arena.origin
        drone_id = slot.drone_id

        @vehicle.on_message("GLOBAL_POSITION_INT")
        def _on_position(_vehicle: Any, _name: str, message: Any) -> None:
            pos = gps_to_ned(origin, message.lat * 1e-7, message.lon * 1e-7, message.alt * 1e-3)
            vel = (message.vx * 0.01, message.vy * 0.01, message.vz * 0.01)
            with self._lock:
                self._telemetry[drone_id] = _Telemetry(np.array(pos), np.array(vel), time.monotonic())

        return vehicle

    def _request_position_rate(self, vehicle: Vehicle) -> None:
        interval_us = 1e6 / self.config.sitl.position_rate_hz
        message = vehicle.message_factory.command_long_encode(
            0, 0, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            GLOBAL_POSITION_INT_ID, interval_us, 0, 0, 0, 0, 0,
        )
        vehicle.send_mavlink(message)

    def takeoff_all(self, altitude_m: float) -> None:
        with ThreadPoolExecutor(max_workers=len(self.vehicles)) as pool:
            for future in [pool.submit(self._arm_and_takeoff, i, altitude_m) for i in self.vehicles]:
                future.result()
        deadline = time.monotonic() + self.config.sitl.takeoff_timeout_s
        while True:
            states = self.read_states()
            low = [i for i, s in states.items() if -s.pos[2] < 0.95 * altitude_m]
            if not low:
                break
            if time.monotonic() > deadline:
                raise TimeoutError(f"Drones {low} did not reach {altitude_m} m")
            altitudes = ", ".join(f"{i}:{-s.pos[2]:.1f}" for i, s in sorted(states.items()))
            print(f"  climbing... {altitudes}")
            time.sleep(1.0)
        print("All drones airborne.")

    def _arm_and_takeoff(self, drone_id: int, altitude_m: float) -> None:
        vehicle = self.vehicles[drone_id]
        deadline = time.monotonic() + self.config.sitl.takeoff_timeout_s
        while not vehicle.is_armable:
            if time.monotonic() > deadline:
                raise TimeoutError(f"Drone {drone_id} is not armable (GPS/EKF not ready)")
            time.sleep(0.5)
        vehicle.mode = VehicleMode("GUIDED")
        while vehicle.mode.name != "GUIDED":
            time.sleep(0.2)
        vehicle.armed = True
        while not vehicle.armed:
            if time.monotonic() > deadline:
                raise TimeoutError(f"Drone {drone_id} refused to arm")
            time.sleep(0.2)
        vehicle.simple_takeoff(altitude_m)

    # ------------------------------------------------------------------ battle
    def read_states(self) -> dict[int, RawState]:
        now = time.monotonic()
        stale_after = self.config.sitl.stale_after_s
        states: dict[int, RawState] = {}
        with self._lock:
            for drone_id, slot in self.slots.items():
                telemetry: Optional[_Telemetry] = self._telemetry.get(drone_id)
                if telemetry is None:
                    states[drone_id] = RawState(np.array(slot.start_ned, dtype=float), np.zeros(3), stale=True)
                    continue
                states[drone_id] = RawState(
                    telemetry.pos.copy(), telemetry.vel.copy(), stale=now - telemetry.received_at > stale_after
                )
        return states

    def send_velocity(self, drone_id: int, velocity_ned: Vec3) -> None:
        if drone_id in self._killed:
            return
        vehicle = self.vehicles[drone_id]
        message = vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, VELOCITY_ONLY_MASK,
            0, 0, 0, velocity_ned[0], velocity_ned[1], velocity_ned[2], 0, 0, 0, 0, 0,
        )
        vehicle.send_mavlink(message)

    def kill(self, drone_id: int, mode: KillMode) -> None:
        if drone_id in self._killed:
            return
        self._killed.add(drone_id)
        vehicle = self.vehicles[drone_id]
        if mode == "freefall":
            message = vehicle.message_factory.command_long_encode(
                0, 0, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, FORCE_DISARM_MAGIC, 0, 0, 0, 0, 0
            )
            vehicle.send_mavlink(message)
        else:
            vehicle.mode = VehicleMode("LAND")

    def step(self, dt: float) -> None:
        pass

    def start_clock(self) -> None:
        self._t0 = time.monotonic()

    def now(self) -> float:
        return time.monotonic() - self._t0

    def end_match(self) -> None:
        """Between tournament matches: surviving drones hover until the next ``connect``."""
        for drone_id in self.vehicles:
            if drone_id not in self._killed:
                try:
                    self.send_velocity(drone_id, (0.0, 0.0, 0.0))
                except Exception as exc:  # noqa: BLE001
                    print(f"  drone {drone_id}: {exc}")

    # ------------------------------------------------------------------ reset between matches
    def _return_to_slots(self) -> None:
        """Flies every drone back to its start slot and lands it there (parallel, with a timeout).

        Shot-down drones are on the ground somewhere in the arena: they are armed
        again, take off and fly back. RTL is not used on purpose: arming sets the
        home position, so a drone re-armed where it crashed would "return" there.
        """
        away = []
        states = self.read_states()
        for drone_id, vehicle in self.vehicles.items():
            slot = np.array(self.slots[drone_id].start_ned, dtype=float)
            distance = float(np.linalg.norm(states[drone_id].pos[:2] - slot[:2]))
            if vehicle.armed or distance > SLOT_TOLERANCE_M:
                away.append(drone_id)
        if not away:
            return
        print(f"Returning drones {away} to their start positions...")
        with ThreadPoolExecutor(max_workers=len(away)) as pool:
            for future in [pool.submit(self._fly_to_slot_and_land, i) for i in away]:
                future.result()
        print("All drones are back at their start positions.")

    def _fly_to_slot_and_land(self, drone_id: int) -> None:
        vehicle = self.vehicles[drone_id]
        deadline = time.monotonic() + self.config.sitl.reset_timeout_s
        slot = self.slots[drone_id]
        transit_alt = self.config.arena.takeoff_alt_m

        def wait(condition: Callable[[], bool], what: str) -> None:
            while not condition():
                if time.monotonic() > deadline:
                    raise TimeoutError(f"Drone {drone_id}: {what} timed out")
                time.sleep(0.5)

        if not vehicle.armed:
            wait(lambda: vehicle.is_armable, "waiting until armable")
            vehicle.mode = VehicleMode("GUIDED")
            wait(lambda: vehicle.mode.name == "GUIDED", "switching to GUIDED")
            vehicle.armed = True
            wait(lambda: vehicle.armed, "arming")
            vehicle.simple_takeoff(transit_alt)
            wait(lambda: -self.read_states()[drone_id].pos[2] > 0.8 * transit_alt, "take-off")
        else:
            vehicle.mode = VehicleMode("GUIDED")
            wait(lambda: vehicle.mode.name == "GUIDED", "switching to GUIDED")
        lat, lon, _ = ned_to_gps(self.config.arena.origin, slot.start_ned[0], slot.start_ned[1], 0.0)
        vehicle.simple_goto(LocationGlobalRelative(lat, lon, transit_alt))
        target = np.array(slot.start_ned[:2], dtype=float)
        wait(lambda: float(np.linalg.norm(self.read_states()[drone_id].pos[:2] - target)) < 1.0, "flying home")
        vehicle.mode = VehicleMode("LAND")
        wait(lambda: not vehicle.armed, "landing")

    def shutdown(self) -> None:
        end_mode = self.config.sitl.end_mode
        for drone_id, vehicle in self.vehicles.items():
            try:
                if drone_id not in self._killed:
                    vehicle.mode = VehicleMode(end_mode)
                vehicle.close()
            except Exception as exc:  # noqa: BLE001 - keep closing the remaining vehicles
                print(f"  drone {drone_id}: {exc}")
