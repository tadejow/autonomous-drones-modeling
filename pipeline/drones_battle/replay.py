"""Replays a recorded match (no SITL needed) and optionally exports a GIF/MP4.

Examples::

    python -m pipeline.drones_battle.replay pipeline/drones_battle/matches/2026-10-12_101500.jsonl
    python -m pipeline.drones_battle.replay match.jsonl --save battle.gif --fps 10

Controls in the window: the slider at the bottom moves through time, the space
bar pauses and resumes.
"""

from __future__ import annotations

import argparse
from typing import Any, Optional

from pipeline.drones_battle.arena_visualizer import ArenaVisualizer, plt
from pipeline.drones_battle.core.config import ArenaConfig
from pipeline.drones_battle.core.recorder import read_recording
from pipeline.drones_battle.core.types import EventKind, GameEvent


def _config_from_header(header: dict[str, Any]) -> ArenaConfig:
    raw = header.get("config")
    return ArenaConfig().with_overrides(**raw) if raw else ArenaConfig()


def _event(record: dict[str, Any]) -> GameEvent:
    position = record.get("position")
    return GameEvent(
        EventKind(record["kind"]), record["time"], record.get("actor"), record.get("victim"),
        None if position is None else tuple(position), record.get("detail", ""),
    )


def build_huds(frames: list[dict[str, Any]], config: ArenaConfig, result: Optional[dict[str, Any]]) -> list[dict]:
    """Cumulative HUD (events so far, hit markers, alive counts) for every frame."""
    huds: list[dict] = []
    lines: list[str] = []
    hits: list[Any] = []
    for index, frame in enumerate(frames):
        for record in frame.get("events", []):
            event = _event(record)
            lines.append(event.describe())
            if event.kind is EventKind.HIT and event.position is not None:
                hits.append(event.position)
        drones = frame["drones"]
        banner = None
        if result and index == len(frames) - 1:
            banner = f"{result['winner'].upper()} WIN ({result['reason']}, t = {result['time']:.1f} s)"
        huds.append({
            "time": frame["t"],
            "max_time": config.game.max_time_s,
            "alive": {
                team: sum(1 for d in drones.values() if d["team"] == team and d["alive"])
                for team in ("attackers", "defenders")
            },
            "events": list(lines[-5:]),
            "hits": list(hits),
            "banner": banner,
        })
    return huds


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Replay a recorded drone battle.")
    parser.add_argument("recording")
    parser.add_argument("--save", default=None, help="output .gif or .mp4 (needs ffmpeg for mp4)")
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--step", type=int, default=1, help="use every N-th frame")
    parser.add_argument("--topdown", action="store_true")
    args = parser.parse_args(argv)

    header, frames, result = read_recording(args.recording)
    if not frames:
        raise SystemExit("The recording has no frames.")
    frames = frames[:: max(args.step, 1)]
    config = _config_from_header(header)
    huds = build_huds(frames, config, result)
    title = f"Replay: {header.get('attacker', '?')} vs {header.get('defender', '?')}"
    visualizer = ArenaVisualizer(
        config, topdown=args.topdown or None, title=title, window_layout=None if args.save else "maximized"
    )

    def draw(index: int) -> None:
        visualizer.history.clear()
        start = max(0, index - visualizer.trail_length + 1)
        for past in frames[start:index]:
            visualizer.render(past["drones"])
        visualizer.render(frames[index]["drones"], huds[index])

    if args.save:
        from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter

        def advance(index: int) -> list[Any]:
            visualizer.render(frames[index]["drones"], huds[index])
            return []

        animation = FuncAnimation(visualizer.fig, advance, frames=len(frames), blit=False)
        writer = PillowWriter(fps=args.fps) if args.save.endswith(".gif") else FFMpegWriter(fps=args.fps)
        animation.save(args.save, writer=writer)
        print(f"Saved {args.save}")
        return

    from matplotlib.widgets import Slider

    slider_ax = visualizer.fig.add_axes((0.2, 0.02, 0.6, 0.025))
    slider = Slider(slider_ax, "frame", 0, len(frames) - 1, valinit=0, valstep=1)
    state = {"index": 0, "playing": True, "internal": False}

    def on_slider(value: float) -> None:
        if state["internal"]:
            return
        state["index"] = int(value)
        draw(state["index"])

    def on_key(event: Any) -> None:
        if event.key == " ":
            state["playing"] = not state["playing"]

    slider.on_changed(on_slider)
    visualizer.fig.canvas.mpl_connect("key_press_event", on_key)
    while visualizer.is_open():
        if state["playing"] and state["index"] < len(frames) - 1:
            state["index"] += 1
            visualizer.render(frames[state["index"]]["drones"], huds[state["index"]])
            state["internal"] = True
            slider.set_val(state["index"])
            state["internal"] = False
        plt.pause(1.0 / args.fps)
        visualizer.apply_window_layout()


if __name__ == "__main__":
    main()
