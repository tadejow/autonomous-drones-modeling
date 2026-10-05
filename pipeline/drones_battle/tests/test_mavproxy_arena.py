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

    def set_follow(self, enable: int) -> None:
        self.calls.append(("follow", enable))


@pytest.fixture()
def arena_module(monkeypatch: pytest.MonkeyPatch):
    class MPModule:
        def __init__(self, mpstate, name, description=None, multi_vehicle=False, **_kwargs):
            self.mpstate = mpstate
            self.multi_vehicle = multi_vehicle
            self.commands = {}

        def module(self, name):
            return self.mpstate.modules.get(name)

        def add_command(self, name, callback, description, completions=None):
            self.commands[name] = callback

    class MPSlipMap:
        def _wait_ready(self, timeout):
            return timeout  # the test only checks which limit is used

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
    fake["MAVProxy.modules.mavproxy_map.mp_slipmap"].MPSlipMap = MPSlipMap
    fake["MAVProxy.modules.mavproxy_map.mp_slipmap"].SlipTrail = lambda: None
    fake["MAVProxy.modules.mavproxy_map"].mp_slipmap = fake["MAVProxy.modules.mavproxy_map.mp_slipmap"]
    for name, module in fake.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setenv("ARENA_MAP_TEAMS", "1:red,2:red,4:blue,5:blue")
    sys.modules.pop("pipeline.drones_battle.mavproxy_arena", None)
    return importlib.import_module("pipeline.drones_battle.mavproxy_arena")


class FakeSettings:
    def __init__(self) -> None:
        self.values = {"showahrspos": 1, "showgpspos": 1}

    def set(self, name: str, value: int) -> None:
        self.values[name] = value


def _mpstate(slipmap):
    """MAVProxy state with the map module (public, with its settings) when ``slipmap`` is given."""
    modules = {} if slipmap is None else {"map": SimpleNamespace(map_settings=FakeSettings())}
    return SimpleNamespace(map=slipmap, modules=modules)


def test_module_opens_the_map_with_a_longer_limit(arena_module) -> None:
    mpstate = SimpleNamespace(map=None, modules={})

    def load_module(name):
        assert name == "map"
        mpstate.map = FakeSlipMap()
        mpstate.modules["map"] = SimpleNamespace(map_settings=FakeSettings())

    mpstate.load_module = load_module
    module = arena_module.init(mpstate)
    slip_class = sys.modules["MAVProxy.modules.mavproxy_map.mp_slipmap"].MPSlipMap
    assert slip_class()._wait_ready(5.0) == arena_module.MAP_START_TIMEOUT_S
    assert mpstate.map.calls == [("follow", 0)]
    assert module.default_icons_hidden is True


def test_map_failure_keeps_the_module_alive(arena_module) -> None:
    def load_module(_name):
        raise RuntimeError("map not ready")

    mpstate = SimpleNamespace(map=None, modules={}, load_module=load_module)
    module = arena_module.init(mpstate)
    module.mavlink_packet(_position(1, -35.3615, 149.165))
    assert module.positions


def _position(sysid: int, lat: float, lon: float):
    return SimpleNamespace(
        get_type=lambda: "GLOBAL_POSITION_INT", get_srcSystem=lambda: sysid,
        lat=int(lat * 1e7), lon=int(lon * 1e7), hdg=9000,
    )


def test_team_colours_and_view_fit(arena_module, monkeypatch: pytest.MonkeyPatch) -> None:
    slipmap = FakeSlipMap()
    mpstate = _mpstate(slipmap)
    module = arena_module.init(mpstate)
    # MAVProxy passes other vehicles' packets only to multi-vehicle modules.
    assert module.multi_vehicle is True
    assert mpstate.modules["map"].map_settings.values == {"showahrspos": 0, "showgpspos": 0}
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
    assert fits[0][1] > 200 * 4 / 3  # 200 m between the teams fits into a 4:3 window

    wide, tall = (arena_module.fit([(0.0, 0.0), (0.0018, 0.0)], aspect) for aspect in (2.0, 0.5))
    assert wide[1] > tall[1]  # a wide window needs a larger ground width to show the same north-south span

    module.commands["arena"](["center"])
    assert slipmap.calls[-1][0] == "center"


def test_without_map_or_teams(arena_module, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ARENA_MAP_TEAMS", "")
    module = arena_module.init(_mpstate(None))
    assert module.default_icons_hidden is False  # no map module: the standard icons are left alone
    module.mavlink_packet(_position(3, -35.3615, 149.165))  # no map yet: nothing breaks
    module.fit_view()
    assert module.positions == {3: pytest.approx((-35.3615, 149.165))}
    assert arena_module.parse_teams("1:red 6:blue") == {1: "red", 6: "blue"}
