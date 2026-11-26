"""SITL backend telemetry handling, with DroneKit and pymavlink replaced by stand-ins."""

import importlib
import sys
import time
import types
from types import SimpleNamespace

import pytest

from pipeline.drones_battle.core.config import ArenaConfig
from pipeline.math.geodesy import ned_to_gps


@pytest.fixture()
def sitl(monkeypatch: pytest.MonkeyPatch):
    dronekit = types.ModuleType("dronekit")
    dronekit.Vehicle = object
    dronekit.VehicleMode = lambda name: name
    dronekit.LocationGlobalRelative = lambda *args: args
    dronekit.connect = lambda *args, **kwargs: None
    pymavlink = types.ModuleType("pymavlink")
    pymavlink.mavutil = SimpleNamespace(mavlink=SimpleNamespace())
    monkeypatch.setitem(sys.modules, "dronekit", dronekit)
    monkeypatch.setitem(sys.modules, "pymavlink", pymavlink)
    sys.modules.pop("pipeline.drones_battle.backends.sitl", None)
    module = importlib.import_module("pipeline.drones_battle.backends.sitl")
    yield module
    sys.modules.pop("pipeline.drones_battle.backends.sitl", None)


def _message(config: ArenaConfig, north: float, east: float, down: float, v_north: float, boot_ms: int):
    lat, lon, alt = ned_to_gps(config.arena.origin, north, east, down)
    return SimpleNamespace(lat=round(lat * 1e7), lon=round(lon * 1e7), alt=round(alt * 1000),
                           vx=round(v_north * 100), vy=0, vz=0, time_boot_ms=boot_ms)


def test_positions_are_brought_to_the_same_instant(sitl) -> None:
    config = ArenaConfig()
    backend = sitl.SitlBackend(config)
    # Attacker 1 reported 0.2 s ago flying south at 10 m/s, defender 4 reported just now flying north.
    backend._store_position(1, _message(config, 100.0, 0.0, -15.0, -10.0, 5000), received=10.0)
    backend._store_position(4, _message(config, 50.0, 0.0, -15.0, 10.0, 7000), received=10.2)
    states = backend.read_states(now=10.2)
    assert states[1].pos[0] == pytest.approx(98.0, abs=0.05)  # moved on by 10 m/s * 0.2 s
    assert states[4].pos[0] == pytest.approx(50.0, abs=0.05)
    assert states[1].pos[2] == pytest.approx(-15.0, abs=0.01)
    assert not states[1].stale and not states[4].stale
    # Old data: extrapolation is capped and the drone is marked stale (ignored by the referee).
    late = backend.read_states(now=11.5)
    assert late[1].pos[0] == pytest.approx(100.0 - 10.0 * sitl.MAX_EXTRAPOLATION_S, abs=0.05)
    assert late[1].stale
    assert late[2].stale  # never reported: start position, stale


def test_duplicate_messages_are_ignored_and_counted(sitl) -> None:
    config = ArenaConfig()
    backend = sitl.SitlBackend(config)
    backend._t0 = time.monotonic() - 2.0
    first = _message(config, 100.0, 0.0, -15.0, 0.0, 5000)
    backend._store_position(1, first, received=10.0)
    backend._store_position(1, first, received=10.01)  # the same message through a second --out
    backend._store_position(1, _message(config, 200.0, 0.0, -15.0, 0.0, 4900), received=10.02)  # older
    assert backend.read_states(now=10.02)[1].pos[0] == pytest.approx(100.0, abs=0.05)
    stats = backend.diagnostics()
    assert stats["duplicate_messages"] == 2
    assert stats["position_rate_max_hz"] == pytest.approx(0.5, abs=0.05)  # 1 message in 2 s
    assert stats["position_rate_min_hz"] == 0.0  # the other drones sent nothing


def test_telemetry_delay_is_measured_against_the_autopilot_clock(sitl) -> None:
    config = ArenaConfig()
    backend = sitl.SitlBackend(config)
    backend._store_position(1, _message(config, 0.0, 0.0, -15.0, 0.0, 1000), received=100.0)
    backend._store_position(1, _message(config, 0.0, 0.0, -15.0, 0.0, 2000), received=101.0)
    backend._store_position(1, _message(config, 0.0, 0.0, -15.0, 0.0, 3000), received=102.8)  # 0.8 s late
    assert backend.diagnostics()["telemetry_lag_max_s"] == pytest.approx(0.8, abs=1e-6)
