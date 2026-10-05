"""MAVProxy module for the arena map: team colours and a view that fits all drones.

``start_arena.sh`` loads it into the combined map MAVProxy::

    mavproxy.py --master=... --cmd="module load pipeline.drones_battle.mavproxy_arena"

* MAVProxy gives the map window only 5 s to start and otherwise fails with
  "map not ready", leaving an empty window that never shows a drone. On the
  lab machines that happened even with the simulators idle. This module
  therefore loads the map itself, allowing ``MAP_START_TIMEOUT_S``. (No extra
  ``module load map``: the map module may run several times, which opened a
  second window, "Map2".)

* The standard map draws every vehicle as the same red icon. Once loaded,
  this module hides those (map settings ``showahrspos``/``showgpspos``) and
  draws its own: attackers red, defenders blue (as in the 3D views), labelled
  with the SysID. If the module cannot load, the standard icons stay.
* It is a multi-vehicle module: MAVProxy passes other modules only the
  packets of its current target vehicle.
* The view is fitted to the drones once all of them report a position and
  again a few times during the next ``FIT_DELAYS_S[-1]`` seconds: the map
  keeps its top-left corner when ``start_arena.sh`` resizes the window, which
  would otherwise shift the view. ``ARENA_MAP_ASPECT`` (window width / height,
  from ``start_arena.sh``) makes the north-south extent fit as well.
* ``arena center`` in the map console fits the view again at any time.

Team colours come from ``ARENA_MAP_TEAMS`` (``"1:red,2:red,4:blue"``), set by
``start_arena.sh`` from the arena layout. The module runs inside MAVProxy's own
Python, so it imports nothing from the arena except the standard library.
"""

import math
import os
import time

from MAVProxy.modules.lib import mp_module

METRES_PER_DEGREE = 111320.0
FIT_DELAYS_S = (2.0, 5.0, 8.0, 12.0, 16.0, 20.0)
MAP_START_TIMEOUT_S = 60.0


def allow_slow_map_start():
    """Raises MAVProxy's hard-coded 5 s limit for the map window to ``MAP_START_TIMEOUT_S``."""
    from MAVProxy.modules.mavproxy_map import mp_slipmap

    original = mp_slipmap.MPSlipMap._wait_ready
    if getattr(original, "arena_patched", False):
        return

    def wait_ready(self, timeout):
        return original(self, max(timeout, MAP_START_TIMEOUT_S))

    wait_ready.arena_patched = True
    mp_slipmap.MPSlipMap._wait_ready = wait_ready


def parse_teams(spec):
    """``"1:red,2:blue"`` -> ``{1: "red", 2: "blue"}``."""
    colours = {}
    for item in spec.replace(" ", ",").split(","):
        if ":" in item:
            sysid, colour = item.split(":", 1)
            colours[int(sysid)] = colour.strip()
    return colours


def fit(positions, aspect=4.0 / 3.0):
    """Centre ``(lat, lon)`` and ground width (m) of a view showing all positions.

    The map zoom is a ground *width*; ``aspect`` (window width / height) converts
    the north-south extent into the width needed to show it.
    """
    lats = [p[0] for p in positions]
    lons = [p[1] for p in positions]
    centre = ((min(lats) + max(lats)) / 2.0, (min(lons) + max(lons)) / 2.0)
    north_south = (max(lats) - min(lats)) * METRES_PER_DEGREE
    east_west = (max(lons) - min(lons)) * METRES_PER_DEGREE * math.cos(math.radians(centre[0]))
    needed = max(east_west, north_south * aspect)
    return centre, max(150.0, 1.4 * needed + 80.0)


class ArenaMapModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ArenaMapModule, self).__init__(mpstate, "arena", "drone battle arena map", multi_vehicle=True)
        self.colours = parse_teams(os.environ.get("ARENA_MAP_TEAMS", ""))
        try:
            self.aspect = float(os.environ.get("ARENA_MAP_ASPECT", "") or 4.0 / 3.0)
        except ValueError:
            self.aspect = 4.0 / 3.0
        self.positions = {}
        self.icons = set()
        self.all_seen_at = None
        self.pending_fits = list(FIT_DELAYS_S)
        self.default_icons_hidden = False
        self.add_command("arena", self.cmd_arena, "arena map helpers", ["center"])
        if self.colours:
            print("arena: team colours for SysIDs %s" % sorted(self.colours))
        else:
            print("arena: ARENA_MAP_TEAMS is empty, every drone will be red")
        self.load_map()
        self.hide_default_icons()

    def load_map(self):
        if self.module("map") is not None:
            return
        try:
            allow_slow_map_start()
            print("arena: opening the map (up to %.0f s)..." % MAP_START_TIMEOUT_S)
            self.mpstate.load_module("map")
        except Exception as exc:  # the arena must keep running without a map
            print("arena: could not open the map: %s" % exc)
            return
        slipmap = self._slipmap()
        if slipmap is not None:
            slipmap.set_follow(0)  # keep the whole arena in view instead of one drone

    def hide_default_icons(self):
        """Turns off the standard (identical) vehicle icons of the map module."""
        if self.default_icons_hidden:
            return
        map_module = self.module("map")
        settings = getattr(map_module, "map_settings", None)
        if settings is None:
            return
        settings.set("showahrspos", 0)
        settings.set("showgpspos", 0)
        self.default_icons_hidden = True

    def _slipmap(self):
        return getattr(self.mpstate, "map", None)

    def cmd_arena(self, args):
        if args and args[0] == "center":
            self.fit_view()
        else:
            print("usage: arena center")

    def mavlink_packet(self, m):
        if m.get_type() != "GLOBAL_POSITION_INT":
            return
        lat, lon = m.lat * 1.0e-7, m.lon * 1.0e-7
        if abs(lat) < 1.0e-3 and abs(lon) < 1.0e-3:
            return
        sysid = m.get_srcSystem()
        heading = m.hdg * 0.01 if m.hdg != 65535 else 0.0
        self.positions[sysid] = (lat, lon)
        slipmap = self._slipmap()
        if slipmap is None:
            return
        key = "ArenaDrone%u" % sysid
        if key not in self.icons:
            from MAVProxy.modules.mavproxy_map import mp_slipmap

            icon = slipmap.icon(self.colours.get(sysid, "red") + "copter.png")
            slipmap.add_object(mp_slipmap.SlipIcon(key, (lat, lon), icon, layer=3, trail=mp_slipmap.SlipTrail()))
            self.icons.add(key)
        slipmap.set_position(key, (lat, lon), rotation=heading, label=str(sysid), colour=(255, 255, 255))

    def idle_task(self):
        self.hide_default_icons()
        expected = set(self.colours) or set(self.positions)
        if self.all_seen_at is None and expected and expected <= set(self.positions):
            self.all_seen_at = time.time()
        if self.all_seen_at is not None and self.pending_fits:
            if time.time() - self.all_seen_at >= self.pending_fits[0]:
                self.pending_fits.pop(0)
                self.fit_view()

    def fit_view(self):
        slipmap = self._slipmap()
        if slipmap is None or not self.positions:
            return
        (lat, lon), ground_width = fit(list(self.positions.values()), self.aspect)
        # Zoom first: changing the zoom moves the view around its top-left corner.
        slipmap.set_zoom(ground_width)
        slipmap.set_center(lat, lon)
        print("arena: view fitted to %u drones (%.0f m wide)" % (len(self.positions), ground_width))


def init(mpstate):
    """Entry point called by MAVProxy's ``module load``."""
    return ArenaMapModule(mpstate)
