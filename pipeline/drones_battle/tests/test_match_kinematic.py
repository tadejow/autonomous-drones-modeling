import json
import time
from pathlib import Path

import numpy as np
import pytest

from pipeline.drones_battle.arena_orchestrator import run_match
from pipeline.drones_battle.backends.kinematic import KinematicBackend
from pipeline.drones_battle.core.config import ArenaConfig, load_config
from pipeline.drones_battle.core.recorder import read_recording
from pipeline.drones_battle.tournament import parse_overrides, run_tournament

STRATEGIES = Path(__file__).resolve().parent / "strategies"
INLINE = ArenaConfig().with_overrides(sandbox={"isolation": "inline"})


def test_default_toml_matches_dataclass_defaults() -> None:
    assert load_config() == ArenaConfig()


def test_full_match_is_fast_and_deterministic(tmp_path: Path) -> None:
    started = time.perf_counter()
    first = run_match(INLINE, fast=True, visualize=False, seed=3, start_jitter_m=3.0, verbose=False,
                      record_path=tmp_path / "match.jsonl")
    assert time.perf_counter() - started < 5.0
    second = run_match(INLINE, fast=True, visualize=False, seed=3, start_jitter_m=3.0, verbose=False)
    assert (first.winner, first.reason, first.time) == (second.winner, second.reason, second.time)

    header, frames, result = read_recording(tmp_path / "match.jsonl")
    assert header["attacker"].endswith("team_attacker")
    assert len(frames) == int(round(first.time * INLINE.game.tick_hz)) + 1
    assert result is not None and result["winner"] == first.winner
    json.dumps(result)


def test_crashing_attackers_lose_on_time() -> None:
    result = run_match(INLINE, attacker_ref=str(STRATEGIES / "crashing.py"), fast=True, visualize=False,
                       verbose=False)
    assert result.winner == "defenders"
    assert result.stats["attackers"]["exceptions"] > 0


def test_hanging_strategy_does_not_freeze_the_arena() -> None:
    config = ArenaConfig().with_overrides(
        sandbox={"isolation": "process", "max_late_s": 0.5, "max_restarts": 1},
        game={"max_time_s": 30.0},
    )
    started = time.perf_counter()
    result = run_match(config, attacker_ref=str(STRATEGIES / "hanging.py"), fast=True, visualize=False,
                       verbose=False)
    assert time.perf_counter() - started < 45.0
    assert result.winner == "defenders"


def test_kinematic_velocity_response() -> None:
    config = ArenaConfig()
    backend = KinematicBackend(config)
    backend.connect()
    backend.takeoff_all(15.0)
    for _ in range(50):  # 5 s of a 10 m/s step command
        backend.send_velocity(1, (-10.0, 0.0, 0.0))
        backend.step(0.1)
    assert backend.read_states()[1].vel[0] == pytest.approx(-10.0, abs=0.05)
    backend.kill(1, "freefall")
    for _ in range(30):
        backend.step(0.1)
    assert backend.read_states()[1].pos[2] == pytest.approx(0.0)  # on the ground


def test_tournament_runs_all_pairings() -> None:
    standings, pairings = run_tournament(INLINE, ["baseline", "hunters"], rounds=1, start_jitter_m=2.0,
                                         base_seed=0)
    assert len(pairings) == 4
    assert sum(s.points for s in standings) == 3 * 4


def test_parse_overrides() -> None:
    assert parse_overrides(["game.kill_radius_m=1.5", "game.kill_mode=freefall"]) == {
        "game": {"kill_radius_m": 1.5, "kill_mode": "freefall"}
    }


def test_strategy_views_hide_nothing_but_copy_everything() -> None:
    # Mutating the dicts inside a strategy must not change the referee state.
    path = STRATEGIES / "mutating.py"
    path.write_text(
        "def compute_commands(my_team, enemy_team, target_pos, current_time):\n"
        "    for s in enemy_team.values():\n"
        "        s['alive'] = False\n"
        "    return {}\n",
        encoding="utf-8",
    )
    try:
        result = run_match(INLINE.with_overrides(game={"max_time_s": 5.0}), defender_ref=str(path),
                           fast=True, visualize=False, verbose=False)
        assert any(result.alive[i] for i in INLINE.game.attacker_ids)
    finally:
        path.unlink()


def test_layout_matches_specification() -> None:
    from pipeline.drones_battle.core.layout import drone_slots

    slots = {s.drone_id: s for s in drone_slots(ArenaConfig())}
    arena = ArenaConfig().arena
    assert [slots[i].start_ned[0] for i in (1, 2, 3)] == [arena.attacker_start_north_m] * 3
    assert np.diff([slots[i].start_ned[1] for i in (4, 5, 6)]).tolist() == [arena.start_spacing_m] * 2
    assert slots[1].instance == 0 and slots[6].instance == 5


CONFIG_5V5 = Path(__file__).resolve().parent.parent / "arena_config_5v5.toml"


@pytest.mark.parametrize("config_path", [None, CONFIG_5V5], ids=["3v3", "5v5"])
def test_every_example_team_plays_without_errors(config_path) -> None:
    from pipeline.drones_battle.tournament import discover_teams

    config = load_config(config_path).with_overrides(sandbox={"isolation": "inline"})
    teams = discover_teams()
    assert {"baseline", "hunters", "tricksters", "pincer", "wolfpack"} <= set(teams)
    for team in teams:
        for attacker, defender in ((team, "baseline"), ("baseline", team)):
            result = run_match(
                config, f"pipeline.drones_battle.teams.{attacker}.attacker",
                f"pipeline.drones_battle.teams.{defender}.defender",
                fast=True, visualize=False, seed=1, start_jitter_m=3.0, verbose=False,
            )
            for side in ("attackers", "defenders"):
                assert result.stats[side]["exceptions"] == 0, (team, side)


def test_5v5_layout_and_ports() -> None:
    from pipeline.drones_battle.core.layout import connection_string, drone_slots

    config = load_config(CONFIG_5V5)
    slots = drone_slots(config)
    assert len(slots) == 10
    assert [s.team for s in slots].count("defenders") == 5
    assert connection_string(config, slots[-1]) == "udp:127.0.0.1:14640"


def test_every_hit_is_reported_to_the_map_backend() -> None:
    from pipeline.drones_battle.core.types import EventKind

    class ReportingBackend(KinematicBackend):
        def __init__(self, config: ArenaConfig) -> None:
            super().__init__(config, seed=1)
            self.reports: list[tuple] = []

        def report_hit(self, victim, by, position) -> None:
            self.reports.append((victim, by, position))

    backend = ReportingBackend(INLINE)
    result = run_match(INLINE, defender_ref="pipeline.drones_battle.teams.hunters.defender", fast=True,
                       visualize=False, verbose=False, backend=backend)
    hits = [e for e in result.events if e.kind is EventKind.HIT]
    assert hits, "the hunters defence should shoot something down"
    assert [(v, b) for v, b, _ in backend.reports] == [(e.victim, e.actor) for e in hits]
    assert all(position is not None for _, _, position in backend.reports)
