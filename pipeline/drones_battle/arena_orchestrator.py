"""Main engine of the drone battle: one match between two strategies.

Examples (run from the repository root)::

    # SITL: first ./pipeline/drones_battle/start_arena.sh, then
    python -m pipeline.drones_battle.arena_orchestrator --backend sitl

    # Without SITL (kinematic model), with or without plots
    python -m pipeline.drones_battle.arena_orchestrator --backend kinematic
    python -m pipeline.drones_battle.arena_orchestrator --backend kinematic --fast --no-viz \
        --attacker pipeline.drones_battle.teams.hunters.attacker \
        --defender pipeline.drones_battle.teams.tricksters.defender

Tick (10 Hz): read telemetry -> referee (hits, target, time) -> kill shot-down
drones -> ask both strategies in parallel -> Safety Limiter -> send velocity
commands -> record -> publish frame to the visualizer process.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Optional

from pipeline.drones_battle.backends import make_backend
from pipeline.drones_battle.backends.base import PhysicsBackend
from pipeline.drones_battle.core.clock import FixedRateClock
from pipeline.drones_battle.core.config import ArenaConfig, load_config
from pipeline.drones_battle.core.recorder import JsonlRecorder, NullRecorder, Recorder
from pipeline.drones_battle.core.referee import Referee
from pipeline.drones_battle.core.safety import SafetyLimiter
from pipeline.drones_battle.core.sandbox import StrategyCall, TeamController, make_controller
from pipeline.drones_battle.core.types import (
    Commands, DroneState, EventKind, MatchResult, RawState, TeamView, to_vec3,
)

DEFAULT_ATTACKER = "pipeline.drones_battle.team_attacker"
DEFAULT_DEFENDER = "pipeline.drones_battle.team_defender"


class MatchCancelled(Exception):
    """Raised when the ``cancel`` event is set during a match (GUI "Stop" button)."""


def game_info(config: ArenaConfig, team: str, time_s: float) -> dict[str, Any]:
    """The optional ``game`` argument passed to strategies that accept it."""
    game, arena = config.game, config.arena
    return {
        "dt": game.dt,
        "time_left": game.max_time_s - time_s,
        "kill_radius": game.kill_radius_m,
        "target_radius": game.target_radius_m,
        "defender_exclusion_radius": game.defender_exclusion_radius_m,
        "max_speed": config.safety.max_speed_for(team),
        "enemy_max_speed": config.safety.max_speed_for("defenders" if team == "attackers" else "attackers"),
        "bounds": {
            "north": (arena.north_min_m, arena.north_max_m),
            "east": (arena.east_min_m, arena.east_max_m),
            "alt": (arena.alt_min_m, arena.alt_max_m),
        },
    }


class Match:
    """Runs one battle on a prepared backend. The class owns no GUI and no network code."""

    def __init__(
        self,
        config: ArenaConfig,
        backend: PhysicsBackend,
        attackers: TeamController,
        defenders: TeamController,
        recorder: Optional[Recorder] = None,
        visualizer: Optional[Any] = None,
        realtime: Optional[bool] = None,
        verbose: bool = True,
        title: str = "",
        speedup: float = 1.0,
        cancel: Optional[threading.Event] = None,
        on_event: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.config = config
        self.backend = backend
        self.attackers = attackers
        self.defenders = defenders
        self.recorder = recorder or NullRecorder()
        self.visualizer = visualizer
        self.realtime = backend.realtime if realtime is None else realtime
        self.verbose = verbose
        self.title = title
        # Only simulated time can run faster than the wall clock (SITL runs at its own pace).
        self.speedup = speedup if getattr(backend, "simulated_time", False) else 1.0
        self.cancel = cancel
        self.on_event = on_event
        self.referee = Referee(config)
        self.limiter = SafetyLimiter(config)
        self.hits: list[tuple[float, float, float]] = []
        self.event_lines: list[str] = []

    # ------------------------------------------------------------------ views
    def _view(self, ids: tuple[int, ...], states: dict[int, RawState]) -> TeamView:
        return {
            i: DroneState(pos=to_vec3(states[i].pos), vel=to_vec3(states[i].vel), alive=self.referee.alive[i])
            for i in ids
        }

    def _game_info(self, time_s: float, team: str) -> dict[str, Any]:
        return game_info(self.config, team, time_s)

    def _frame(self, time_s: float, states: dict[int, RawState], commands: dict[int, Any]) -> dict[str, Any]:
        drones = {
            i: {
                "pos": to_vec3(s.pos),
                "vel": to_vec3(s.vel),
                "alive": self.referee.alive[i],
                "team": self.config.team_of(i),
                "stale": s.stale,
                "cmd": commands.get(i),
            }
            for i, s in states.items()
        }
        return {"t": time_s, "drones": drones}

    def _hud(self, time_s: float, banner: Optional[str] = None) -> dict[str, Any]:
        game = self.config.game
        return {
            "time": time_s,
            "max_time": game.max_time_s,
            "alive": {
                "attackers": sum(self.referee.alive[i] for i in game.attacker_ids),
                "defenders": sum(self.referee.alive[i] for i in game.defender_ids),
            },
            "events": self.event_lines[-5:],
            "hits": self.hits,
            "banner": banner,
            "title": self.title,
        }

    def _log(self, message: str) -> None:
        if self.verbose:
            print(message)
        if self.on_event is not None:
            self.on_event(message)

    # ------------------------------------------------------------------ main loop
    def run(self) -> MatchResult:
        game = self.config.game
        target = game.target_ned
        clock = FixedRateClock(game.tick_hz * self.speedup, realtime=self.realtime)
        self.backend.start_clock()
        previous = self.backend.read_states()
        result: Optional[MatchResult] = None
        last_frame: dict[str, Any] = {}

        while result is None:
            if self.cancel is not None and self.cancel.is_set():
                raise MatchCancelled()
            time_s = self.backend.now()
            states = self.backend.read_states()

            outcome = self.referee.evaluate(previous, states, time_s)
            for event in outcome.events:
                self.event_lines.append(event.describe())
                self._log(event.describe())
                if event.kind is EventKind.HIT and event.victim is not None:
                    self.backend.kill(event.victim, game.kill_mode)
                    if event.position is not None:
                        self.hits.append(event.position)
            result = outcome.result
            if result is None:
                for controller in (self.attackers, self.defenders):
                    if controller.forfeited:
                        result = self.referee.forfeit(controller.team, time_s, f"{controller.team} strategy failed")

            applied: dict[int, Any] = {}
            if result is None:
                att_view = self._view(game.attacker_ids, states)
                def_view = self._view(game.defender_ids, states)
                # Both teams get the same snapshot before either answers.
                self.attackers.submit(StrategyCall(att_view, def_view, target, time_s,
                                                           self._game_info(time_s, "attackers")))
                self.defenders.submit(StrategyCall(def_view, att_view, target, time_s,
                                                           self._game_info(time_s, "defenders")))
                deadline = time.monotonic() + self.config.sandbox.tick_budget_s
                commands: Commands = {}
                commands.update(self.attackers.collect(deadline))
                commands.update(self.defenders.collect(deadline))
                for drone_id in self.config.all_ids:
                    if not self.referee.alive[drone_id]:
                        continue
                    raw = commands.get(drone_id, (0.0, 0.0, 0.0))
                    velocity = self.limiter.apply(drone_id, raw, states[drone_id].pos)
                    self.backend.send_velocity(drone_id, velocity)
                    applied[drone_id] = velocity

            last_frame = self._frame(time_s, states, applied)
            self.recorder.write_frame({**last_frame, "events": [e.as_dict() for e in outcome.events]})
            if self.visualizer is not None and result is None:
                self.visualizer.publish(last_frame["drones"], self._hud(time_s))

            previous = states
            if result is None:
                self.backend.step(game.dt)
                clock.wait()

        assert result is not None
        result.stats = {
            "safety_interventions": self.limiter.summary(),
            "attackers": self.attackers.stats(),
            "defenders": self.defenders.stats(),
            "loop": clock.stats(),
        }
        self.recorder.write_result(result.as_dict())
        banner = f"{result.winner.upper()} WIN ({result.reason.value}, t = {result.time:.1f} s)"
        self._log(banner)
        if self.visualizer is not None and last_frame:
            self.visualizer.show_final(last_frame["drones"], self._hud(result.time, banner))
        return result


def window_layout_for(config: ArenaConfig, backend_name: str) -> str:
    """Resolves ``window_layout = "auto"``: next to the MAVProxy map for SITL, maximized otherwise."""
    layout = config.visualization.window_layout
    if layout == "auto":
        return "right_half" if backend_name == "sitl" else "maximized"
    return layout


def run_match(
    config: ArenaConfig,
    attacker_ref: str = DEFAULT_ATTACKER,
    defender_ref: str = DEFAULT_DEFENDER,
    backend_name: str = "kinematic",
    fast: bool = False,
    visualize: bool = True,
    record_path: Optional[str | Path] = None,
    seed: Optional[int] = None,
    start_jitter_m: float = 0.0,
    verbose: bool = True,
    *,
    visualizer: Optional[Any] = None,
    final_hold_s: Optional[float] = None,
    speedup: float = 1.0,
    backend: Optional[PhysicsBackend] = None,
    cancel: Optional[threading.Event] = None,
    on_event: Optional[Callable[[str], None]] = None,
    title: Optional[str] = None,
) -> MatchResult:
    """Builds every component, runs the match and always cleans up (``finally``).

    Extra keyword arguments used by the GUI and tournaments:

    * ``visualizer``: an existing :class:`VisualizerProcess` reused for many
      matches (``visualize`` is then ignored and the window stays open);
    * ``final_hold_s``: how long the result stays on screen; ``None`` waits
      until the user closes the window (only for a visualizer created here);
    * ``speedup``: kinematic real-time playback faster than real time;
    * ``backend``: an already connected backend reused between matches (SITL
      tournaments); it is not shut down at the end, only ``end_match`` is called;
    * ``cancel``: setting the event stops the match with :class:`MatchCancelled`;
    * ``on_event``: receives every log line (hits, result).
    """
    sandbox = config.sandbox
    owns_backend = backend is None
    if backend is None:
        backend = make_backend(backend_name, config, seed=seed, realtime=not fast)
    attackers = defenders = None
    owns_visualizer = visualizer is None and visualize
    recorder: Recorder = JsonlRecorder(record_path) if record_path else NullRecorder()
    title = title or f"{_short(attacker_ref)} (attack) vs {_short(defender_ref)} (defence)"
    try:
        def controller(team: str, ref: str, ids: tuple[int, ...]) -> TeamController:
            return make_controller(team, ref, ids, sandbox.isolation, sandbox.max_late_s, sandbox.max_restarts)

        if sandbox.isolation == "process":
            # Both strategy processes start at the same time (each needs ~1 s to import).
            with ThreadPoolExecutor(max_workers=2) as pool:
                attacker_future = pool.submit(controller, "attackers", attacker_ref, config.game.attacker_ids)
                defender_future = pool.submit(controller, "defenders", defender_ref, config.game.defender_ids)
                defenders = defender_future.result() if defender_future.exception() is None else None
                attackers = attacker_future.result()
                if defenders is None:
                    defender_future.result()  # re-raises the defenders' error
        else:
            attackers = controller("attackers", attacker_ref, config.game.attacker_ids)
            defenders = controller("defenders", defender_ref, config.game.defender_ids)
        if verbose:
            print(f"Backend: {backend_name}. Connecting...")
        backend.connect()
        if start_jitter_m > 0 and hasattr(backend, "perturb_start"):
            backend.perturb_start(start_jitter_m)
        backend.takeoff_all(config.arena.takeoff_alt_m)
        recorder.write_header({
            "created": datetime.now().isoformat(timespec="seconds"),
            "backend": backend_name,
            "attacker": attacker_ref,
            "defender": defender_ref,
            "title": title,
            "seed": seed,
            "config": config.as_dict(),
        })
        if owns_visualizer:
            from pipeline.drones_battle.arena_visualizer import VisualizerProcess

            visualizer = VisualizerProcess(config, window_layout=window_layout_for(config, backend_name), title=title)
            visualizer.wait_ready()
        elif visualizer is not None:
            visualizer.new_match(title)
        if not getattr(backend, "simulated_time", False):  # SITL: give the pilots a moment
            for remaining in range(int(config.game.countdown_s), 0, -1):
                if cancel is not None and cancel.is_set():
                    raise MatchCancelled()
                if verbose:
                    print(f"Battle starts in {remaining}...")
                time.sleep(1.0)
        if verbose:
            print("FIGHT!")
        match = Match(config, backend, attackers, defenders, recorder, visualizer, verbose=verbose, title=title,
                      speedup=speedup, cancel=cancel, on_event=on_event)
        result = match.run()
        if visualizer is not None:
            if final_hold_s is None and owns_visualizer:
                if verbose:
                    print("Close the plot window to exit.")
                visualizer.wait_closed(cancel)
            elif final_hold_s:
                _sleep_unless_cancelled(final_hold_s, cancel)
        return result
    finally:
        for controller in (attackers, defenders):
            if controller is not None:
                controller.close()
        try:
            if owns_backend:
                backend.shutdown()
            else:
                backend.end_match()
        finally:
            recorder.close()
            if owns_visualizer and visualizer is not None:
                visualizer.close()


def _short(ref: str) -> str:
    """Readable team name from a module path or a file path (``.../300538/attacker.py`` -> ``300538``)."""
    if ref.endswith(".py") or "/" in ref or "\\" in ref:
        path = Path(ref)
        return path.parent.name if path.stem in ("attacker", "defender") else path.stem
    parts = ref.split(".")
    if len(parts) >= 2 and parts[-1] in ("attacker", "defender"):
        return parts[-2]
    return parts[-1]


def _sleep_unless_cancelled(seconds: float, cancel: Optional[threading.Event]) -> None:
    if cancel is None:
        time.sleep(seconds)
    else:
        cancel.wait(seconds)


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Drone battle 3 vs 3: one match.")
    parser.add_argument("--attacker", default=DEFAULT_ATTACKER, help="module path or .py file")
    parser.add_argument("--defender", default=DEFAULT_DEFENDER, help="module path or .py file")
    parser.add_argument("--backend", choices=("sitl", "kinematic"), default="sitl")
    parser.add_argument("--config", default=None, help="TOML file (default: arena_config.toml)")
    parser.add_argument("--fast", action="store_true", help="kinematic only: no real-time pacing")
    parser.add_argument("--no-viz", action="store_true", help="do not open the 3D views")
    parser.add_argument("--inline", action="store_true", help="run strategies in-process (debugging)")
    parser.add_argument("--topdown", action="store_true", help="add a 2D top-down map to the 3D window")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--jitter", type=float, default=0.0, help="kinematic only: random start offset (m)")
    parser.add_argument("--record", default=None, help="JSONL file (default: matches/<timestamp>.jsonl)")
    parser.add_argument("--no-record", action="store_true")
    args = parser.parse_args(argv)

    config = load_config(args.config)
    if args.inline:
        config = config.with_overrides(sandbox={"isolation": "inline"})
    if args.topdown:
        config = config.with_overrides(visualization={"topdown": True})
    record = None
    if not args.no_record:
        stamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        record = args.record or Path(__file__).resolve().parent / "matches" / f"{stamp}.jsonl"

    result = run_match(
        config, args.attacker, args.defender, args.backend, fast=args.fast, visualize=not args.no_viz,
        record_path=record, seed=args.seed, start_jitter_m=args.jitter,
    )
    print(json.dumps(result.as_dict()["stats"], indent=2))
    if record:
        print(f"Recording: {record}")


if __name__ == "__main__":
    main()
