"""The MAVProxy map module, tested against a minimal stand-in for MAVProxy."""

import importlib
import json
import socket
import sys
import time
import types
from types import SimpleNamespace

import pytest


class FakeSlipMap:
    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def icon(self, filename: str) -> str:
        return filename

    def add_object(self, obj: object) -> None:
        self.calls.append(("add", obj.key, obj.img, getattr(obj, "label", None)))

    def remove_object(self, key: str) -> None:
        self.calls.append(("remove", key))

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
        def __init__(self, key, latlon, img, label=None, **_kwargs):
            self.key, self.img, self.label = key, img, label

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
    monkeypatch.setenv("ARENA_MAP_EVENTS_PORT", str(_free_udp_port()))
    sys.modules.pop("pipeline.drones_battle.mavproxy_arena", None)
    return importlib.import_module("pipeline.drones_battle.mavproxy_arena")


def _free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


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


def _deliver(module) -> None:
    """Lets the datagrams arrive, then runs MAVProxy's idle hook."""
    time.sleep(0.1)
    module.idle_task()


def test_kills_hide_drones_and_show_one_explosion_per_collision(arena_module, monkeypatch) -> None:
    from pipeline.drones_battle.core.map_events import MapEvents

    slipmap = FakeSlipMap()
    module = arena_module.init(_mpstate(slipmap))
    for sysid, lat in ((1, -35.36155), (4, -35.36335)):
        module.mavlink_packet(_position(sysid, lat, 149.165))
    events = MapEvents(module.events.getsockname()[1])

    # Kamikaze collision: both drones die at (almost) the same point -> one explosion "1+4".
    events.kill(1, 4, -35.3625, 149.165)
    events.kill(4, 1, -35.3625, 149.1650001)
    _deliver(module)
    assert ("remove", "ArenaDrone1") in slipmap.calls and ("remove", "ArenaDrone4") in slipmap.calls
    explosions = [c for c in slipmap.calls if c[0] == "add" and c[1].startswith("ArenaExplosion")]
    assert {c[1] for c in explosions} == {"ArenaExplosion1"}
    assert explosions[-1][3] == "1+4"
    assert explosions[-1][2].shape == (64, 64, 3)

    # A destroyed drone is no longer drawn even though it keeps sending positions (LAND).
    slipmap.calls.clear()
    module.mavlink_packet(_position(1, -35.3625, 149.165))
    assert not [c for c in slipmap.calls if c[1] == "ArenaDrone1"]

    # After the flash the explosion shrinks to a dim marker (same key, smaller image).
    created = module.explosions[0].created
    with monkeypatch.context() as later:
        later.setattr(arena_module.time, "time", lambda: created + arena_module.EXPLOSION_FLASH_S + 0.1)
        module.idle_task()
    assert slipmap.calls[-1][1] == "ArenaExplosion1" and slipmap.calls[-1][2].shape[0] < 64

    # A new battle: explosions disappear and every drone is drawn again.
    events.start()
    _deliver(module)
    assert ("remove", "ArenaExplosion1") in slipmap.calls
    module.mavlink_packet(_position(1, -35.36155, 149.165))
    assert slipmap.calls[-2][:2] == ("add", "ArenaDrone1")
    events.close()


def test_separate_hits_get_separate_explosions(arena_module) -> None:
    slipmap = FakeSlipMap()
    module = arena_module.init(_mpstate(slipmap))
    module.handle_event({"event": "kill", "sysid": 1, "lat": -35.3625, "lon": 149.165})
    module.handle_event({"event": "kill", "sysid": 2, "lat": -35.3620, "lon": 149.165})  # ~55 m away
    module.handle_event({"event": "kill", "lat": 1.0})  # malformed: ignored
    keys = {c[1] for c in slipmap.calls if c[0] == "add"}
    assert keys == {"ArenaExplosion1", "ArenaExplosion2"}


def test_explosion_icon_is_transparent_around_a_bright_core(arena_module) -> None:
    icon = arena_module.explosion_icon(64)
    assert icon.dtype.name == "uint8" and icon.shape == (64, 64, 3)
    assert icon[0, 0].tolist() == [0, 0, 0]  # black corners: MAVProxy adds icons onto the map
    assert icon[32, 32].min() > 200  # white-hot centre
    assert arena_module.explosion_icon(64, 0.5).max() < icon.max()


def test_map_events_datagrams() -> None:
    from pipeline.drones_battle.core.map_events import MapEvents

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(2.0)
        events = MapEvents(receiver.getsockname()[1])
        events.start()
        events.kill(3, 6, -35.36, 149.16)
        messages = [json.loads(receiver.recv(4096)) for _ in range(2)]
        events.close()
    assert messages == [{"event": "start"}, {"event": "kill", "sysid": 3, "by": 6, "lat": -35.36, "lon": 149.16}]
    MapEvents(9).kill(1, None, 0.0, 0.0)  # nobody listening: no exception
