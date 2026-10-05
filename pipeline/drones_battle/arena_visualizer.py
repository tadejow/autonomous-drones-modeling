"""3D views of the battle (windows 2 and 3 of the specification).

* One large figure with two 3D subplots: the defenders' view (camera south of
  the base, looking north) and the attackers' view (looking south).
* Optional third subplot: a 2D top-down map (fallback when the MAVProxy map
  does not work, e.g. on WSL2 without wxPython).
* Plotting frame is ENU (x = East, y = North, z = Up). It is right-handed and
  matches the MAVProxy map; plotting (N, E, Up) would mirror the scene.
* Artists are created once and only updated, which is much faster than
  ``ax.clear()`` and keeps the camera still.
* Blitting: the static scene (panes, grid, ticks, target spheres) is drawn
  once and cached as a bitmap; each frame restores it and draws only the
  drones, trails and texts. A full redraw of two 3D axes takes 0.2-1 s,
  which made the view lag behind the MAVProxy map. For the same reason the
  window loop never calls ``plt.pause``: on Tk every ``plt.pause`` calls
  ``show()``, which re-renders the whole figure and raises the window.

``VisualizerProcess`` runs the visualizer in its own process, so drawing can
never slow down the 10 Hz control loop of the orchestrator.
"""

from __future__ import annotations

import multiprocessing as mp
import queue
import shutil
import subprocess
import sys
import time
from collections import deque
from typing import Any, Optional

import matplotlib

if matplotlib.get_backend().lower() != "agg":
    try:
        matplotlib.use("TkAgg")
    except (ImportError, RuntimeError):  # headless machines fall back to the default backend
        pass

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from pipeline.drones_battle.core.config import ArenaConfig, load_config  # noqa: E402
from pipeline.math.geodesy import ned_to_enu  # noqa: E402

TEAM_COLORS = {"attackers": "red", "defenders": "blue"}
TITLE_BAR_PX = 32
DELAY_SMOOTHING = 0.2

matplotlib.rcParams["figure.raise_window"] = False  # do not pull the window to the front on every update
LAYOUT_RETRIES_S = (0.0, 1.0, 3.0)
DEAD_COLOR = "gray"


def _sphere(center_enu: tuple[float, float, float], radius: float, steps: int = 14) -> tuple[np.ndarray, ...]:
    u, v = np.meshgrid(np.linspace(0, 2 * np.pi, steps), np.linspace(0, np.pi, steps // 2))
    return (
        center_enu[0] + radius * np.cos(u) * np.sin(v),
        center_enu[1] + radius * np.sin(u) * np.sin(v),
        center_enu[2] + radius * np.cos(v),
    )


def _x11_work_area() -> Optional[tuple[int, int, int, int]]:
    """Usable screen area without panels ``(x, y, width, height)`` from the X11 window manager."""
    try:
        output = subprocess.run(
            ["xprop", "-root", "_NET_WORKAREA"], capture_output=True, text=True, timeout=2, check=True
        ).stdout
        values = [int(v) for v in output.split("=", 1)[1].replace(",", " ").split()[:4]]
        if len(values) == 4 and values[2] > 0 and values[3] > 0:
            return values[0], values[1], values[2], values[3]
    except (OSError, subprocess.SubprocessError, IndexError, ValueError):
        pass
    return None


def _work_area(window: Any) -> tuple[int, int, int, int]:
    """Usable screen area (without panels) as ``(x, y, width, height)``."""
    area = _x11_work_area()
    if area is not None:
        return area
    # Fallback (e.g. Windows): whole screen minus a typical taskbar.
    return 0, 0, int(window.winfo_screenwidth()), int(window.winfo_screenheight()) - 48


def _right_half(area: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
    """``(x, y, width, height)`` of the right half of a work area, minus a title bar."""
    x, y, width, height = area
    half = width // 2
    return x + half, y, width - half, max(height - TITLE_BAR_PX, 200)


def _wmctrl_place(title: str, x: int, y: int, width: int, height: int) -> None:
    """Asks the X11 window manager directly (wmctrl), as start_arena.sh does for the map.

    Some window managers ignore Tk's geometry request for a window they have
    maximized; wmctrl first removes the maximized state, then moves and resizes.
    """
    if not sys.platform.startswith("linux") or shutil.which("wmctrl") is None or not title:
        return
    for args in (["-b", "remove,maximized_vert,maximized_horz"], ["-e", f"0,{x},{y},{width},{height}"]):
        try:
            subprocess.run(["wmctrl", "-F", "-r", title, *args], timeout=2, check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except (OSError, subprocess.SubprocessError):
            return


def _smooth(previous: Optional[float], value: float) -> float:
    """Exponential moving average used for the on-screen timings."""
    return value if previous is None else (1 - DELAY_SMOOTHING) * previous + DELAY_SMOOTHING * value


class ArenaVisualizer:
    def __init__(
        self,
        config: Optional[ArenaConfig] = None,
        trail_length: Optional[int] = None,
        topdown: Optional[bool] = None,
        title: str = "Drone Battle Arena",
        window_layout: Optional[str] = None,
    ) -> None:
        self.config = config or load_config()
        viz = self.config.visualization
        self.trail_length = trail_length or viz.trail_length
        self.topdown = viz.topdown if topdown is None else topdown
        self.pause_s = viz.pause_s
        self.target_enu = ned_to_enu(self.config.game.target_ned)
        self.history: dict[int, deque[tuple[float, float, float]]] = {}

        plt.ion()
        columns = 3 if self.topdown else 2
        self._pending_layout: Optional[str] = window_layout or viz.window_layout
        figsize: tuple[float, float] = (8.0 * columns, 8.0)
        area = _x11_work_area() if self._pending_layout == "right_half" else None
        if area is not None:
            # Open the window at its final size: a window larger than the screen gets maximized
            # by some window managers (xfwm4), which then ignore any later geometry request.
            _, _, width, height = _right_half(area)
            dpi = float(matplotlib.rcParams["figure.dpi"])
            figsize = (width / dpi, (height - 40) / dpi)
        self.fig = plt.figure(figsize=figsize)
        self.fig.subplots_adjust(left=0.0, right=1.0, bottom=0.14, top=0.84, wspace=0.0)
        try:
            self.fig.canvas.manager.set_window_title(title)
        except AttributeError:
            pass
        self.ax_def = self.fig.add_subplot(1, columns, 1, projection="3d")
        self.ax_att = self.fig.add_subplot(1, columns, 2, projection="3d")
        self.ax_top = self.fig.add_subplot(1, columns, 3) if self.topdown else None
        self._setup_3d(self.ax_def, "Defenders' view (looking north)", azim=-90.0)
        self._setup_3d(self.ax_att, "Attackers' view (looking south)", azim=90.0)
        if self.ax_top is not None:
            self._setup_top(self.ax_top)

        # Animated artists are left out of normal redraws and drawn by present() (blitting).
        self._blit = bool(getattr(self.fig.canvas, "supports_blit", False))
        self._background: Any = None
        self.hud_text = self.fig.suptitle("", fontsize=14, fontweight="bold", animated=self._blit)
        self.events_text = self.fig.text(0.01, 0.005, "", fontsize=9, family="monospace", va="bottom",
                                         animated=self._blit)
        self.stats_text = self.fig.text(0.99, 0.005, "", fontsize=8, color="#777777", ha="right", va="bottom",
                                        animated=self._blit)
        self.trails: dict[tuple[int, int], Any] = {}
        self.markers: dict[tuple[int, int], Any] = {}
        self.hit_lines = [self._new_line(ax, marker="x", color="black", markersize=14, mew=3, linestyle="")
                          for ax in self._axes()]
        self._delay: Optional[float] = None
        self._max_delay = 0.0
        self._frame_times: deque[float] = deque(maxlen=20)
        self._draw_ms: Optional[float] = None
        self._screen_ms: Optional[float] = None
        self.full_redraws = 0
        self.fig.canvas.mpl_connect("draw_event", self._on_draw)
        self._layout_retries = list(LAYOUT_RETRIES_S)
        self._mapped_at: Optional[float] = None

    # ------------------------------------------------------------------ window
    def apply_window_layout(self) -> None:
        """Puts the Tk window on the right half of the work area (or maximizes it).

        ``start_arena.sh`` places the MAVProxy map on the left half, so together
        the two windows fill the screen. Called after every GUI pause; it acts
        only once the window is shown (a geometry set earlier is overridden when
        matplotlib maps the window) and repeats at ``LAYOUT_RETRIES_S`` after
        that, because window managers may still move a window just after mapping it.
        """
        layout = self._pending_layout
        window = getattr(self.fig.canvas.manager, "window", None)
        if layout is None or window is None or not hasattr(window, "wm_geometry"):
            self._pending_layout = None
            return
        if not window.winfo_ismapped():
            return
        now = time.monotonic()
        if self._mapped_at is None:
            self._mapped_at = now
        if now - self._mapped_at < self._layout_retries[0]:
            return
        self._layout_retries.pop(0)
        if not self._layout_retries:
            self._pending_layout = None
        if layout == "maximized":
            try:
                window.state("zoomed")  # Windows
            except Exception:  # noqa: BLE001 - X11 window managers use the -zoomed attribute
                try:
                    window.attributes("-zoomed", True)
                except Exception:  # noqa: BLE001
                    pass
        elif layout == "right_half":
            x, y, width, height = _right_half(_work_area(window))
            if sys.platform.startswith("linux"):
                try:
                    window.attributes("-zoomed", False)  # undo a maximize by the window manager
                except Exception:  # noqa: BLE001
                    pass
            window.wm_geometry(f"{width}x{height}+{x}+{y}")
            window.update()
            _wmctrl_place(window.wm_title(), x, y, width, height)
        window.update()

    # ------------------------------------------------------------------ setup
    def _limits(self) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
        arena = self.config.arena
        return (arena.east_min_m, arena.east_max_m), (arena.north_min_m, arena.north_max_m), (0.0, arena.alt_max_m)

    def _setup_3d(self, ax: Any, title: str, azim: float) -> None:
        (x0, x1), (y0, y1), (z0, z1) = self._limits()
        ax.set_title(title)
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        ax.set_zlim(z0, z1)
        try:  # altitude exaggerated 2.5x, zoomed in (matplotlib >= 3.6 knows ``zoom``)
            ax.set_box_aspect((x1 - x0, y1 - y0, 2.5 * (z1 - z0)), zoom=1.4)
        except TypeError:
            ax.set_box_aspect((x1 - x0, y1 - y0, 2.5 * (z1 - z0)))
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.set_zlabel("Up (m)")
        ax.view_init(elev=20.0, azim=azim)
        try:
            ax.set_proj_type("persp", focal_length=0.5)
        except TypeError:  # matplotlib < 3.6
            ax.set_proj_type("persp")
        self._static_scene(ax)

    def _static_scene(self, ax: Any) -> None:
        tx, ty, tz = self.target_enu
        ax.plot([tx], [ty], [tz], marker="*", color="gold", markersize=22, markeredgecolor="black", linestyle="")
        ax.plot([tx, tx], [ty, ty], [0.0, tz], color="goldenrod", linewidth=1, linestyle=":")
        ax.plot_wireframe(*_sphere(self.target_enu, self.config.game.target_radius_m), color="gold", alpha=0.35,
                          linewidth=0.6)
        exclusion = self.config.game.defender_exclusion_radius_m
        if exclusion > 0:
            ax.plot_wireframe(*_sphere(self.target_enu, exclusion), color="blue", alpha=0.08, linewidth=0.5)

    def _setup_top(self, ax: Any) -> None:
        (x0, x1), (y0, y1), _ = self._limits()
        ax.set_title("Top-down map")
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        ax.set_aspect("equal")
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.grid(True, alpha=0.3)
        tx, ty, _ = self.target_enu
        ax.plot([tx], [ty], marker="*", color="gold", markersize=18, markeredgecolor="black")
        ax.add_patch(plt.Circle((tx, ty), self.config.game.target_radius_m, color="gold", alpha=0.3))
        exclusion = self.config.game.defender_exclusion_radius_m
        if exclusion > 0:
            ax.add_patch(plt.Circle((tx, ty), exclusion, color="blue", fill=False, alpha=0.3, linestyle="--"))

    def _axes(self) -> list[Any]:
        return [ax for ax in (self.ax_def, self.ax_att, self.ax_top) if ax is not None]

    def _new_line(self, ax: Any, **style: Any) -> Any:
        """An empty (animated, when blitting) line in a 2D or 3D axes."""
        empty: list[list[float]] = [[], []] if ax is self.ax_top else [[], [], []]
        (line,) = ax.plot(*empty, animated=self._blit, **style)
        return line

    def _ensure_artists(self, drone_id: int) -> None:
        if (0, drone_id) in self.markers:
            return
        for index, ax in enumerate(self._axes()):
            self.trails[(index, drone_id)] = self._new_line(ax, linestyle="--", linewidth=1.0)
            self.markers[(index, drone_id)] = self._new_line(ax, marker="o", markersize=8, linestyle="")

    # ------------------------------------------------------------------ drawing
    def _team(self, drone_id: int, state: dict[str, Any]) -> str:
        team = state.get("team")
        return team if team else self.config.team_of(int(drone_id))

    def render(self, drones_state: dict[Any, dict[str, Any]], hud: Optional[dict[str, Any]] = None) -> None:
        """Updates all artists; does not process GUI events (see :meth:`update_frame`)."""
        for raw_id, state in drones_state.items():
            drone_id = int(raw_id)
            alive = bool(state.get("alive", state.get("is_alive", True)))
            east, north, up = ned_to_enu(tuple(state["pos"]))
            trail = self.history.setdefault(drone_id, deque(maxlen=self.trail_length))
            trail.append((east, north, up))
            xs, ys, zs = (list(c) for c in zip(*trail))
            color = TEAM_COLORS[self._team(drone_id, state)] if alive else DEAD_COLOR
            self._ensure_artists(drone_id)
            for index, ax in enumerate(self._axes()):
                line, marker = self.trails[(index, drone_id)], self.markers[(index, drone_id)]
                if ax is self.ax_top:
                    line.set_data(xs, ys)
                    marker.set_data([east], [north])
                else:
                    line.set_data_3d(xs, ys, zs)
                    marker.set_data_3d([east], [north], [up])
                line.set_color(color)
                marker.set_color(color)
                marker.set_marker("o" if alive else "X")
        if hud:
            self._render_hud(hud)

    def _render_hud(self, hud: dict[str, Any]) -> None:
        alive = hud.get("alive", {})
        text = (
            f"t = {hud.get('time', 0.0):5.1f} / {hud.get('max_time', 0.0):.0f} s    "
            f"Attackers alive: {alive.get('attackers', '?')}    Defenders alive: {alive.get('defenders', '?')}"
        )
        if hud.get("banner"):
            text = f"{hud['banner']}\n{text}"
        if hud.get("title"):
            text = f"{hud['title']}\n{text}"
        self.hud_text.set_text(text)
        self.events_text.set_text("\n".join(hud.get("events", [])[-5:]))
        hits = [ned_to_enu(tuple(position)) for position in hud.get("hits", [])]
        xs, ys, zs = ([p[i] for p in hits] for i in range(3))
        for ax, line in zip(self._axes(), self.hit_lines):
            if ax is self.ax_top:
                line.set_data(xs, ys)
            else:
                line.set_data_3d(xs, ys, zs)

    def update_frame(self, drones_state: dict[Any, dict[str, Any]], hud: Optional[dict[str, Any]] = None) -> None:
        """Draws one frame and lets the GUI process its events.

        The specification asked for ``plt.pause(0.05)`` here; with Tk that redraws
        the whole figure every time (see the module docstring), so the frame is
        blitted and the GUI events are flushed instead.
        """
        started = time.perf_counter()
        self.render(drones_state, hud)
        self.present()
        drawn = time.perf_counter()
        self.fig.canvas.flush_events()  # the GUI copies the image to the screen here
        shown = time.perf_counter()
        self._draw_ms = _smooth(self._draw_ms, 1000.0 * (drawn - started))
        self._screen_ms = _smooth(self._screen_ms, 1000.0 * (shown - drawn))
        self.apply_window_layout()

    def _draw_dynamic(self) -> None:
        """Draws the animated artists over whatever is in the canvas buffer."""
        axes = self._axes()
        for (index, _), line in self.trails.items():
            axes[index].draw_artist(line)
        for (index, _), marker in self.markers.items():
            axes[index].draw_artist(marker)
        for ax, line in zip(axes, self.hit_lines):
            ax.draw_artist(line)
        for text in (self.hud_text, self.events_text, self.stats_text):
            self.fig.draw_artist(text)

    def _on_draw(self, event: Any) -> None:
        """After every full redraw (first show, resize, mouse rotation) cache the static scene."""
        if not self._blit:
            return
        if self.fig.canvas.is_saving():
            # When saving (GIF export, toolbar "save") axes include their animated artists,
            # but the figure leaves out its animated texts: draw them into the saved image.
            for text in (self.hud_text, self.events_text, self.stats_text):
                text.draw(event.renderer)
            return
        self.full_redraws += 1
        self._background = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        self._draw_dynamic()

    def present(self) -> None:
        """Shows the current state: static background from the cache plus the moving artists."""
        canvas = self.fig.canvas
        if not self._blit:
            canvas.draw_idle()
        elif self._background is None:
            canvas.draw()  # full redraw; _on_draw caches the background and draws the drones
        else:
            canvas.restore_region(self._background)
            self._draw_dynamic()
            canvas.blit(self.fig.bbox)

    def idle(self, seconds: float) -> None:
        """Keeps the window responsive for ``seconds`` without redrawing anything."""
        self.fig.canvas.start_event_loop(seconds)
        self.apply_window_layout()

    def record_delay(self, sent_at: float) -> None:
        """Shows how long frames take from the arena to the screen (bottom-right corner)."""
        now = time.time()
        delay = max(now - sent_at, 0.0)
        self._max_delay = max(self._max_delay, delay)
        self._delay = _smooth(self._delay, delay)
        self._frame_times.append(now)
        self.stats_text.set_text(self._stats_line())

    def _fps(self) -> float:
        if len(self._frame_times) < 2:
            return 0.0
        return (len(self._frame_times) - 1) / max(self._frame_times[-1] - self._frame_times[0], 1e-6)

    def _stats_line(self) -> str:
        """Delay and where the time goes: drawing (matplotlib) vs. copying to the screen (GUI, X server)."""
        line = f"view delay {self._delay or 0.0:.2f} s | {self._fps():.0f} frames/s"
        if self._draw_ms is not None and self._screen_ms is not None:
            line += f" | draw {self._draw_ms:.0f} ms, screen {self._screen_ms:.0f} ms"
        return line + f" | full redraws {self.full_redraws}"

    def delay_summary(self) -> str:
        if self._delay is None:
            return "3D view: no frames"
        return f"3D view: {self._stats_line()} (max delay {self._max_delay:.2f} s)"

    def is_open(self) -> bool:
        return plt.fignum_exists(self.fig.number)

    def close(self) -> None:
        plt.ioff()
        plt.close(self.fig)



# ---------------------------------------------------------------------- process wrapper
def _drain(channel: Any) -> None:
    while True:
        try:
            channel.get_nowait()
        except queue.Empty:
            return


def _visualizer_main(
    config: ArenaConfig, frames: Any, control: Any, ready: Any, closed: Any, window_layout: str, title: str
) -> None:
    """Child process: one window, reused for every match until ``stop`` arrives.

    ``frames`` (size 1) carries only the newest state; ``control`` carries
    ``reset`` (new match), ``final`` (result frame) and ``stop``, which must
    never be dropped, hence the separate queue.
    """
    visualizer: Optional[ArenaVisualizer] = None
    frozen = False

    def open_window(window_title: str) -> ArenaVisualizer:
        if visualizer is not None and visualizer.is_open():
            visualizer.close()
        fresh = ArenaVisualizer(config, window_layout=window_layout, title=window_title)
        fresh.present()
        fresh.idle(0.05)
        closed.clear()
        ready.set()
        return fresh

    visualizer = open_window(title)
    while True:
        try:
            command = control.get_nowait()
        except queue.Empty:
            command = None
        if command is not None:
            if command[0] == "stop":
                break
            if command[0] == "reset":
                _drain(frames)
                visualizer = open_window(command[1])
                frozen = False
            elif command[0] == "final":
                _drain(frames)
                frozen = True
                if visualizer.is_open():
                    visualizer.update_frame(command[1], command[2])
                    print(visualizer.delay_summary(), flush=True)
            continue
        if not visualizer.is_open():
            closed.set()
            time.sleep(0.05)
            continue
        try:
            _, drones, hud = frames.get(timeout=0.005)
        except queue.Empty:
            visualizer.idle(0.02)
            continue
        if not frozen:
            visualizer.update_frame(drones, hud)
            if "sent_at" in hud:
                visualizer.record_delay(hud["sent_at"])
    if visualizer is not None and visualizer.is_open():
        visualizer.close()


class VisualizerProcess:
    """Runs :class:`ArenaVisualizer` in a separate process fed with the newest frame only.

    One process (and one window) can show many matches in a row: call
    :meth:`new_match` before each of them, which is what tournaments do.
    """

    def __init__(self, config: ArenaConfig, window_layout: str = "none", title: str = "Drone Battle Arena") -> None:
        context = mp.get_context("spawn")
        self._frames = context.Queue(maxsize=1)
        self._control = context.Queue()
        self._ready = context.Event()
        self._closed = context.Event()
        self._process = context.Process(
            target=_visualizer_main,
            args=(config, self._frames, self._control, self._ready, self._closed, window_layout, title),
            name="arena-visualizer",
            daemon=True,
        )
        self._process.start()

    def wait_ready(self, timeout: float = 30.0) -> bool:
        """Blocks until the window is on screen, so the first seconds of a match are not lost."""
        return self._ready.wait(timeout)

    def new_match(self, title: str, timeout: float = 30.0) -> bool:
        """Clears the window (trails, hits) for the next match and waits until it is shown."""
        self._ready.clear()
        self._control.put(("reset", title))
        return self.wait_ready(timeout)

    def publish(self, drones: dict[Any, Any], hud: dict[str, Any]) -> None:
        item = ("frame", drones, {**hud, "sent_at": time.time()})
        try:
            self._frames.put_nowait(item)
        except queue.Full:
            try:
                self._frames.get_nowait()  # drop the stale frame
            except queue.Empty:
                pass
            try:
                self._frames.put_nowait(item)
            except queue.Full:
                pass

    def show_final(self, drones: dict[Any, Any], hud: dict[str, Any]) -> None:
        """Last frame with the result banner; later frames are ignored until :meth:`new_match`."""
        self._control.put(("final", drones, hud))

    def wait_closed(self, cancel: Optional[Any] = None) -> None:
        """Blocks until the user closes the window, the process ends or ``cancel`` is set."""
        while not self._closed.wait(0.2):
            if not self._process.is_alive() or (cancel is not None and cancel.is_set()):
                return

    def is_alive(self) -> bool:
        return self._process.is_alive()

    def close(self) -> None:
        if self._process.is_alive():
            self._control.put(("stop",))
            self._process.join(timeout=3.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=2.0)
