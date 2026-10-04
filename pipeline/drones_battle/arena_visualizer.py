"""3D views of the battle (windows 2 and 3 of the specification).

* One large figure with two 3D subplots: the defenders' view (camera south of
  the base, looking north) and the attackers' view (looking south).
* Optional third subplot: a 2D top-down map (fallback when the MAVProxy map
  does not work, e.g. on WSL2 without wxPython).
* Plotting frame is ENU (x = East, y = North, z = Up). It is right-handed and
  matches the MAVProxy map; plotting (N, E, Up) would mirror the scene.
* Artists are created once and only updated, which is much faster than
  ``ax.clear()`` and keeps the camera still.

``VisualizerProcess`` runs the visualizer in its own process, so drawing can
never slow down the 10 Hz control loop of the orchestrator.
"""

from __future__ import annotations

import multiprocessing as mp
import queue
import subprocess
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
DEAD_COLOR = "gray"


def _sphere(center_enu: tuple[float, float, float], radius: float, steps: int = 14) -> tuple[np.ndarray, ...]:
    u, v = np.meshgrid(np.linspace(0, 2 * np.pi, steps), np.linspace(0, np.pi, steps // 2))
    return (
        center_enu[0] + radius * np.cos(u) * np.sin(v),
        center_enu[1] + radius * np.sin(u) * np.sin(v),
        center_enu[2] + radius * np.cos(v),
    )


def _work_area(window: Any) -> tuple[int, int, int, int]:
    """Usable screen area (without panels) as ``(x, y, width, height)``."""
    try:
        output = subprocess.run(
            ["xprop", "-root", "_NET_WORKAREA"], capture_output=True, text=True, timeout=2, check=True
        ).stdout
        values = [int(v) for v in output.split("=", 1)[1].replace(",", " ").split()[:4]]
        if len(values) == 4 and values[2] > 0 and values[3] > 0:
            return values[0], values[1], values[2], values[3]
    except (OSError, subprocess.SubprocessError, IndexError, ValueError):
        pass
    # Fallback (e.g. Windows): whole screen minus a typical taskbar.
    return 0, 0, int(window.winfo_screenwidth()), int(window.winfo_screenheight()) - 48


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
        self.fig = plt.figure(figsize=(8 * columns, 8))
        self.fig.subplots_adjust(left=0.0, right=1.0, bottom=0.14, top=0.9, wspace=0.0)
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

        self.hud_text = self.fig.suptitle("", fontsize=14, fontweight="bold")
        self.events_text = self.fig.text(0.01, 0.005, "", fontsize=9, family="monospace", va="bottom")
        self.trails: dict[tuple[int, int], Any] = {}
        self.markers: dict[tuple[int, int], Any] = {}
        self.hit_artists: list[Any] = []
        self._pending_layout: Optional[str] = window_layout or viz.window_layout

    # ------------------------------------------------------------------ window
    def apply_window_layout(self) -> None:
        """Puts the Tk window on the right half of the work area (or maximizes it).

        ``start_arena.sh`` places the MAVProxy map on the left half, so together
        the two windows fill the screen. Runs once, after the window is shown:
        a geometry set earlier is overridden when matplotlib maps the window.
        """
        layout = self._pending_layout
        window = getattr(self.fig.canvas.manager, "window", None)
        if layout is None or window is None or not hasattr(window, "wm_geometry"):
            self._pending_layout = None
            return
        if not window.winfo_ismapped():
            return
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
            x, y, width, height = _work_area(window)
            half = width // 2
            # Height minus a title bar; the WM adds decorations outside the geometry.
            window.wm_geometry(f"{width - half}x{max(height - 32, 200)}+{x + half}+{y}")
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

    def _ensure_artists(self, drone_id: int) -> None:
        if (0, drone_id) in self.markers:
            return
        for index, ax in enumerate(self._axes()):
            if ax is self.ax_top:
                (trail,) = ax.plot([], [], linestyle="--", linewidth=1.0)
                (marker,) = ax.plot([], [], marker="o", markersize=8, linestyle="")
            else:
                (trail,) = ax.plot([], [], [], linestyle="--", linewidth=1.0)
                (marker,) = ax.plot([], [], [], marker="o", markersize=8, linestyle="")
            self.trails[(index, drone_id)] = trail
            self.markers[(index, drone_id)] = marker

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
        self.hud_text.set_text(text)
        self.events_text.set_text("\n".join(hud.get("events", [])[-5:]))
        for artist in self.hit_artists:
            artist.remove()
        self.hit_artists.clear()
        for position in hud.get("hits", []):
            east, north, up = ned_to_enu(tuple(position))
            for ax in self._axes():
                if ax is self.ax_top:
                    (artist,) = ax.plot([east], [north], marker="x", color="black", markersize=14, mew=3)
                else:
                    (artist,) = ax.plot([east], [north], [up], marker="x", color="black", markersize=14, mew=3)
                self.hit_artists.append(artist)

    def update_frame(self, drones_state: dict[Any, dict[str, Any]], hud: Optional[dict[str, Any]] = None) -> None:
        """Draws one frame and lets the GUI breathe (``plt.pause``), as in the specification."""
        self.render(drones_state, hud)
        plt.pause(self.pause_s)
        self.apply_window_layout()

    def is_open(self) -> bool:
        return plt.fignum_exists(self.fig.number)

    def close(self) -> None:
        plt.ioff()
        plt.close(self.fig)


# ---------------------------------------------------------------------- process wrapper
def _visualizer_main(config: ArenaConfig, frames: Any, keep_open: bool, window_layout: str) -> None:
    visualizer = ArenaVisualizer(config, window_layout=window_layout)
    while visualizer.is_open():
        try:
            item = frames.get(timeout=0.005)
        except queue.Empty:
            plt.pause(visualizer.pause_s)
            visualizer.apply_window_layout()
            continue
        if item is None:
            break
        kind, drones, hud = item
        visualizer.update_frame(drones, hud)
        if kind == "final":
            if keep_open and visualizer.is_open():
                plt.ioff()
                plt.show()
            break
    visualizer.close()


class VisualizerProcess:
    """Runs :class:`ArenaVisualizer` in a separate process fed with the newest frame only."""

    def __init__(self, config: ArenaConfig, keep_open: bool = True, window_layout: str = "none") -> None:
        context = mp.get_context("spawn")
        self._frames = context.Queue(maxsize=1)
        self._process = context.Process(
            target=_visualizer_main,
            args=(config, self._frames, keep_open, window_layout),
            name="arena-visualizer",
            daemon=True,
        )
        self._process.start()

    def publish(self, drones: dict[Any, Any], hud: dict[str, Any]) -> None:
        item = ("frame", drones, hud)
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

    def finish(self, drones: dict[Any, Any], hud: dict[str, Any], wait: bool = True) -> None:
        try:
            self._frames.get_nowait()
        except queue.Empty:
            pass
        self._frames.put(("final", drones, hud))
        if wait:
            print("Close the plot window to exit.")
            self._process.join()

    def close(self) -> None:
        if self._process.is_alive():
            self._process.terminate()
        self._process.join(timeout=2.0)
