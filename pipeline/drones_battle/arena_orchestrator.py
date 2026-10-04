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
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

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
    ) -> None:
        self.config = config
        self.backend = backend
        self.attackers = attackers
        self.defenders = defenders
        self.recorder = recorder or NullRecorder()
        self.visualizer = visualizer
        self.realtime = backend.realtime if realtime is None else realtime
        self.verbose = verbose
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
        game, arena = self.config.game, self.config.arena
        return {
            "dt": game.dt,
            "time_left": game.max_time_s - time_s,
            "kill_radius": game.kill_radius_m,
            "target_radius": game.target_radius_m,
            "defender_exclusion_radius": game.defender_exclusion_radius_m,
            "max_speed": self.config.safety.max_speed_for(team),
            "enemy_max_speed": self.config.safety.max_speed_for(
                "defenders" if team == "attackers" else "attackers"
            ),
            "bounds": {
                "north": (arena.north_min_m, arena.north_max_m),
                "east": (arena.east_min_m, arena.east_max_m),
                "alt": (arena.alt_min_m, arena.alt_max_m),
            },
        }

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
        }

    def _log(self, message: str) -> None:
        if self.verbose:
            print(message)

    # ------------------------------------------------------------------ main loop
    def run(self) -> MatchResult:
        game = self.config.game
        target = game.target_ned
        clock = FixedRateClock(game.tick_hz, realtime=self.realtime)
        self.backend.start_clock()
        previous = self.backend.read_states()
        result: Optional[MatchResult] = None
        last_frame: dict[str, Any] = {}

        while result is None:
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
            self.visualizer.finish(last_frame["drones"], self._hud(result.time, banner))
        return result


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
) -> MatchResult:
    """Builds every component, runs the match and always cleans up (``finally``)."""
    sandbox = config.sandbox
    backend = make_backend(backend_name, config, seed=seed, realtime=not fast)
    attackers = defenders = None
    visualizer = None
    recorder: Recorder = JsonlRecorder(record_path) if record_path else NullRecorder()
    try:
        attackers = make_controller(
            "attackers", attacker_ref, config.game.attacker_ids, sandbox.isolation, sandbox.max_late_s,
            sandbox.max_restarts,
        )
        defenders = make_controller(
            "defenders", defender_ref, config.game.defender_ids, sandbox.isolation, sandbox.max_late_s,
            sandbox.max_restarts,
        )
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
            "seed": seed,
            "config": config.as_dict(),
        })
        if visualize:
            from pipeline.drones_battle.arena_visualizer import VisualizerProcess

            layout = config.visualization.window_layout
            if layout == "auto":
                layout = "right_half" if backend_name == "sitl" else "maximized"
            visualizer = VisualizerProcess(config, window_layout=layout)
        if backend.realtime:
            for remaining in range(int(config.game.countdown_s), 0, -1):
                if verbose:
                    print(f"Battle starts in {remaining}...")
                time.sleep(1.0)
        if verbose:
            print("FIGHT!")
        match = Match(config, backend, attackers, defenders, recorder, visualizer, verbose=verbose)
        return match.run()
    finally:
        for controller in (attackers, defenders):
            if controller is not None:
                controller.close()
        try:
            backend.shutdown()
        finally:
            recorder.close()
            if visualizer is not None:
                visualizer.close()


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Drone battle 3 vs 3: one match.")
    parser.add_argument("--attacker", default=DEFAULT_ATTACKER, help="module path or .py file")
    parser.add_argument("--defender", default=DEFAULT_DEFENDER, help="module path or .py file")
    parser.add_argument("--backend", choices=("sitl", "kinematic"), default="sitl")
    parser.add_argument("--config", default=None, help="TOML file (default: arena_config.toml)")
    parser.add_argument("--fast", action="store_true", help="kinematic only: no real-time pacing")
    parser.add_argument("--no-viz", action="store_true", help="do not open the 3D views")
    parser.add_argument("--inline", action="store_true", help="run strategies in-process (debugging)")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--jitter", type=float, default=0.0, help="kinematic only: random start offset (m)")
    parser.add_argument("--record", default=None, help="JSONL file (default: matches/<timestamp>.jsonl)")
    parser.add_argument("--no-record", action="store_true")
    args = parser.parse_args(argv)

    config = load_config(args.config)
    if args.inline:
        config = config.with_overrides(sandbox={"isolation": "inline"})
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
