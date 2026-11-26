"""MAVProxy module for the arena map: team colours, explosions and a view that fits all drones.

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
* Battle events come from the orchestrator as small JSON datagrams on
  ``127.0.0.1:ARENA_MAP_EVENTS_PORT``: ``{"event": "start"}`` shows every drone
  again and clears old explosions; ``{"event": "kill", "sysid": 2, "by": 5,
  "lat": ..., "lon": ...}`` hides the destroyed drone and draws an explosion
  where it was hit. A kamikaze collision kills two drones at one point, which
  becomes a single explosion labelled with both SysIDs. After
  ``EXPLOSION_FLASH_S`` the explosion shrinks to a dim marker for the rest of
  the match.

Team colours come from ``ARENA_MAP_TEAMS`` (``"1:red,2:red,4:blue"``), set by
``start_arena.sh`` from the arena layout. The module runs inside MAVProxy's own
Python, so it imports nothing from the arena; only the standard library and
numpy (a MAVProxy dependency).
"""

import json
import math
import os
import socket
import time

import numpy as np

from MAVProxy.modules.lib import mp_module

METRES_PER_DEGREE = 111320.0
FIT_DELAYS_S = (2.0, 5.0, 8.0, 12.0, 16.0, 20.0)
MAP_START_TIMEOUT_S = 60.0
DEFAULT_EVENTS_PORT = 14800
EXPLOSION_FLASH_S = 3.0
EXPLOSION_MERGE_M = 6.0
EXPLOSION_MERGE_S = 2.0


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


def distance_m(a, b):
    """Approximate distance between two ``(lat, lon)`` points a few hundred metres apart."""
    north = (a[0] - b[0]) * METRES_PER_DEGREE
    east = (a[1] - b[1]) * METRES_PER_DEGREE * math.cos(math.radians(a[0]))
    return math.hypot(north, east)


def explosion_icon(size=64, brightness=1.0):
    """BGR image of a jagged fireball on black.

    MAVProxy adds icons onto the map image (``cv2.add``), so black is
    transparent and the flame colours glow over the satellite picture.
    """
    y, x = np.mgrid[0:size, 0:size].astype(float)
    centre = (size - 1) / 2.0
    radius = np.hypot(x - centre, y - centre) / (size / 2.0)
    theta = np.arctan2(y - centre, x - centre)
    # Two spike frequencies give an uneven, hand-drawn rim.
    rim = 0.58 + 0.30 * np.abs(np.cos(4.5 * (theta + math.pi))) ** 2.5 + 0.12 * np.abs(np.sin(7 * theta))
    ratio = radius / rim
    img = np.zeros((size, size, 3), np.uint8)
    for limit, colour in ((1.0, (0, 40, 230)), (0.74, (0, 130, 255)), (0.5, (60, 220, 255)), (0.24, (215, 250, 255))):
        img[ratio < limit] = [int(channel * brightness) for channel in colour]
    return img


class Explosion(object):
    def __init__(self, key, latlon, victims, created):
        self.key = key
        self.latlon = latlon
        self.victims = list(victims)
        self.created = created
        self.faded = False

    @property
    def label(self):
        return "+".join(str(v) for v in self.victims)


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
        self.dead = set()
        self.explosions = []
        self.explosion_count = 0
        self.all_seen_at = None
        self.pending_fits = list(FIT_DELAYS_S)
        self.default_icons_hidden = False
        self.events = self._open_events_socket()
        self.add_command("arena", self.cmd_arena, "arena map helpers", ["center"])
        if self.colours:
            print("arena: team colours for SysIDs %s" % sorted(self.colours))
        else:
            print("arena: ARENA_MAP_TEAMS is empty, every drone will be red")
        self.load_map()
        self.hide_default_icons()

    # ------------------------------------------------------------------ set-up
    def _open_events_socket(self):
        try:
            port = int(os.environ.get("ARENA_MAP_EVENTS_PORT", "") or DEFAULT_EVENTS_PORT)
        except ValueError:
            port = DEFAULT_EVENTS_PORT
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("127.0.0.1", port))
            sock.setblocking(False)
        except OSError as exc:
            print("arena: no battle events (port %u: %s); explosions will not be shown" % (port, exc))
            return None
        print("arena: listening for battle events on port %u" % port)
        return sock

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

    def unload(self):
        if self.events is not None:
            self.events.close()

    # ------------------------------------------------------------------ commands and packets
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
        if slipmap is None or sysid in self.dead:
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
        self.read_events()
        self.fade_explosions()
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

    # ------------------------------------------------------------------ battle events
    def read_events(self):
        while self.events is not None:
            try:
                data = self.events.recv(4096)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                return
            try:
                event = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, ValueError):
                continue
            self.handle_event(event)

    def handle_event(self, event):
        kind = event.get("event")
        if kind == "start":
            self.start_match()
        elif kind == "kill":
            try:
                sysid = int(event["sysid"])
                latlon = (float(event["lat"]), float(event["lon"]))
            except (KeyError, TypeError, ValueError):
                return
            self.kill(sysid, latlon)

    def start_match(self):
        """New battle: every drone is shown again and old explosions disappear."""
        slipmap = self._slipmap()
        if slipmap is not None:
            for explosion in self.explosions:
                slipmap.remove_object(explosion.key)
        self.explosions = []
        self.dead.clear()

    def kill(self, sysid, latlon):
        """Hides a destroyed drone and shows an explosion where it was hit."""
        self.dead.add(sysid)
        slipmap = self._slipmap()
        if slipmap is None:
            return
        key = "ArenaDrone%u" % sysid
        if key in self.icons:
            slipmap.remove_object(key)
            self.icons.discard(key)
        now = time.time()
        for explosion in self.explosions:
            # Kamikaze: attacker and defender die at the same point, show one explosion.
            if now - explosion.created < EXPLOSION_MERGE_S and distance_m(explosion.latlon, latlon) < EXPLOSION_MERGE_M:
                explosion.victims.append(sysid)
                self._draw_explosion(explosion)
                return
        self.explosion_count += 1
        explosion = Explosion("ArenaExplosion%u" % self.explosion_count, latlon, [sysid], now)
        self.explosions.append(explosion)
        self._draw_explosion(explosion)
        print("arena: drone %u destroyed" % sysid)

    def fade_explosions(self):
        now = time.time()
        for explosion in self.explosions:
            if not explosion.faded and now - explosion.created >= EXPLOSION_FLASH_S:
                explosion.faded = True
                self._draw_explosion(explosion)

    def _draw_explosion(self, explosion):
        slipmap = self._slipmap()
        if slipmap is None:
            return
        from MAVProxy.modules.mavproxy_map import mp_slipmap

        icon = explosion_icon(36, 0.6) if explosion.faded else explosion_icon(64, 1.0)
        # Adding an object with an existing key replaces it (bigger flash -> small marker).
        slipmap.add_object(mp_slipmap.SlipIcon(explosion.key, explosion.latlon, icon, layer=3,
                                               label=explosion.label, colour=(255, 230, 120)))


def init(mpstate):
    """Entry point called by MAVProxy's ``module load``."""
    return ArenaMapModule(mpstate)
