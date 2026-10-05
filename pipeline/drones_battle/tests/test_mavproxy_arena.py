"""The MAVProxy map module, tested against a minimal stand-in for MAVProxy."""

import importlib
import sys
import types
from types import SimpleNamespace

import pytest


class FakeSlipMap:
    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def icon(self, filename: str) -> str:
        return filename

    def add_object(self, obj: object) -> None:
        self.calls.append(("add", obj.key, obj.img))

    def set_position(self, key, latlon, rotation=0, label=None, colour=None) -> None:
        self.calls.append(("pos", key, latlon, label))

    def set_zoom(self, width: float) -> None:
        self.calls.append(("zoom", width))

    def set_center(self, lat: float, lon: float) -> None:
        self.calls.append(("center", lat, lon))


@pytest.fixture()
def arena_module(monkeypatch: pytest.MonkeyPatch):
    class MPModule:
        def __init__(self, mpstate, name, description=None, **_kwargs):
            self.mpstate = mpstate
            self.commands = {}

        def add_command(self, name, callback, description, completions=None):
            self.commands[name] = callback

    class SlipIcon:
        def __init__(self, key, latlon, img, **_kwargs):
            self.key, self.img = key, img

    fake = {
        "MAVProxy": types.ModuleType("MAVProxy"),
        "MAVProxy.modules": types.ModuleType("MAVProxy.modules"),
        "MAVProxy.modules.lib": types.ModuleType("MAVProxy.modules.lib"),
        "MAVProxy.modules.lib.mp_module": types.ModuleType("MAVProxy.modules.lib.mp_module"),
        "MAVProxy.modules.mavproxy_map": types.ModuleType("MAVProxy.modules.mavproxy_map"),
        "MAVProxy.modules.mavproxy_map.mp_slipmap": types.ModuleType("MAVProxy.modules.mavproxy_map.mp_slipmap"),
    }
    fake["MAVProxy.modules.lib.mp_module"].MPModule = MPModule
    fake["MAVProxy.modules.lib"].mp_module = fake["MAVProxy.modules.lib.mp_module"]
    fake["MAVProxy.modules.mavproxy_map.mp_slipmap"].SlipIcon = SlipIcon
    fake["MAVProxy.modules.mavproxy_map.mp_slipmap"].SlipTrail = lambda: None
    fake["MAVProxy.modules.mavproxy_map"].mp_slipmap = fake["MAVProxy.modules.mavproxy_map.mp_slipmap"]
    for name, module in fake.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setenv("ARENA_MAP_TEAMS", "1:red,2:red,4:blue,5:blue")
    sys.modules.pop("pipeline.drones_battle.mavproxy_arena", None)
    return importlib.import_module("pipeline.drones_battle.mavproxy_arena")


def _position(sysid: int, lat: float, lon: float):
    return SimpleNamespace(
        get_type=lambda: "GLOBAL_POSITION_INT", get_srcSystem=lambda: sysid,
        lat=int(lat * 1e7), lon=int(lon * 1e7), hdg=9000,
    )


def test_team_colours_and_view_fit(arena_module, monkeypatch: pytest.MonkeyPatch) -> None:
    slipmap = FakeSlipMap()
    module = arena_module.init(SimpleNamespace(map=slipmap))
    clock = [1000.0]
    monkeypatch.setattr(arena_module.time, "time", lambda: clock[0])

    for sysid, lat in ((1, -35.36155), (2, -35.36155), (4, -35.36335), (5, -35.36335)):
        module.mavlink_packet(_position(sysid, lat, 149.165 + 0.0001 * sysid))
    icons = {call[1]: call[2] for call in slipmap.calls if call[0] == "add"}
    assert icons == {"ArenaDrone1": "redcopter.png", "ArenaDrone2": "redcopter.png",
                     "ArenaDrone4": "bluecopter.png", "ArenaDrone5": "bluecopter.png"}
    assert ("pos", "ArenaDrone4", pytest.approx((-35.36335, 149.1654)), "4") in slipmap.calls

    module.idle_task()  # all four seen: the fit is scheduled, not done yet
    assert not [c for c in slipmap.calls if c[0] in ("zoom", "center")]
    for delay in arena_module.FIT_DELAYS_S:
        clock[0] = 1000.0 + delay + 0.01
        module.idle_task()
    fits = [c for c in slipmap.calls if c[0] in ("zoom", "center")]
    assert [c[0] for c in fits] == ["zoom", "center"] * len(arena_module.FIT_DELAYS_S)
    _, lat, lon = fits[1]
    assert lat == pytest.approx(-35.36245) and lon == pytest.approx(149.1653)
    assert fits[0][1] > 200 * 1.5  # 200 m between the teams fits into the view

    module.commands["arena"](["center"])
    assert slipmap.calls[-1][0] == "center"


def test_without_map_or_teams(arena_module, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ARENA_MAP_TEAMS", "")
    module = arena_module.init(SimpleNamespace(map=None))
    module.mavlink_packet(_position(3, -35.3615, 149.165))  # no map yet: nothing breaks
    module.fit_view()
    assert module.positions == {3: pytest.approx((-35.3615, 149.165))}
    assert arena_module.parse_teams("1:red 6:blue") == {1: "red", 6: "blue"}
