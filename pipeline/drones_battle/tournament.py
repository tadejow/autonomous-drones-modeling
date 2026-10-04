"""Round-robin tournament on the kinematic backend (no SITL needed).

Every team is a package with ``attacker.py`` and ``defender.py`` (see ``teams/``).
Every ordered pair plays: team X attacks, team Y defends, ``--rounds`` times
with random start offsets (seeded, hence reproducible).

Scoring per match: the winning side gets 3 points. Tie-breakers: total attack
time of won attacks (lower is better), then total survival time of lost
defences (higher is better).

Examples::

    python -m pipeline.drones_battle.tournament
    python -m pipeline.drones_battle.tournament --teams baseline hunters --rounds 20
    # Balance check of the rules (attacker win rate per pairing):
    python -m pipeline.drones_battle.tournament --set safety.defender_max_speed_mps=8 --set game.kill_radius_m=1.5
"""

from __future__ import annotations

import argparse
import ast
import csv
import itertools
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

from pipeline.drones_battle.arena_orchestrator import run_match
from pipeline.drones_battle.core.config import ArenaConfig, load_config

TEAMS_PACKAGE = "pipeline.drones_battle.teams"
TEAMS_DIR = Path(__file__).resolve().parent / "teams"


@dataclass
class Standing:
    team: str
    points: int = 0
    attacks_won: int = 0
    attacks_played: int = 0
    defences_won: int = 0
    defences_played: int = 0
    attack_time: float = 0.0
    defence_time: float = 0.0

    def key(self) -> tuple[float, float, float]:
        return (-self.points, self.attack_time, -self.defence_time)


@dataclass
class PairingStats:
    attacker_wins: int = 0
    matches: int = 0
    times: list[float] = field(default_factory=list)


def discover_teams() -> list[str]:
    return sorted(
        p.name for p in TEAMS_DIR.iterdir()
        if p.is_dir() and (p / "attacker.py").exists() and (p / "defender.py").exists()
    )


def parse_overrides(items: list[str]) -> dict[str, dict[str, Any]]:
    """``["game.kill_radius_m=1.5"]`` -> ``{"game": {"kill_radius_m": 1.5}}``."""
    overrides: dict[str, dict[str, Any]] = defaultdict(dict)
    for item in items:
        key, _, raw = item.partition("=")
        section, _, name = key.strip().partition(".")
        try:
            value: Any = ast.literal_eval(raw.strip())
        except (ValueError, SyntaxError):
            value = raw.strip()
        overrides[section][name] = value
    return dict(overrides)


def run_tournament(
    config: ArenaConfig, teams: list[str], rounds: int, start_jitter_m: float, base_seed: int,
    verbose: bool = False,
) -> tuple[list[Standing], dict[tuple[str, str], PairingStats]]:
    standings = {team: Standing(team) for team in teams}
    pairings: dict[tuple[str, str], PairingStats] = defaultdict(PairingStats)
    for attacker, defender in itertools.product(teams, repeat=2):
        for round_index in range(rounds):
            seed = base_seed + round_index
            result = run_match(
                config,
                f"{TEAMS_PACKAGE}.{attacker}.attacker",
                f"{TEAMS_PACKAGE}.{defender}.defender",
                backend_name="kinematic", fast=True, visualize=False, seed=seed,
                start_jitter_m=start_jitter_m, verbose=False,
            )
            stats = pairings[(attacker, defender)]
            stats.matches += 1
            stats.times.append(result.time)
            att, dfn = standings[attacker], standings[defender]
            att.attacks_played += 1
            dfn.defences_played += 1
            if result.winner == "attackers":
                stats.attacker_wins += 1
                att.points += 3
                att.attacks_won += 1
                att.attack_time += result.time
                dfn.defence_time += result.time
            else:
                dfn.points += 3
                dfn.defences_won += 1
            if verbose:
                print(f"{attacker:>12} -> {defender:<12} seed {seed:3d}: {result.winner} "
                      f"({result.reason.value}, {result.time:.1f} s)")
    return sorted(standings.values(), key=Standing.key), dict(pairings)


def format_report(standings: list[Standing], pairings: dict[tuple[str, str], PairingStats]) -> str:
    lines = [
        "| # | Team | Points | Attacks won | Defences won |",
        "|---|------|-------:|------------:|-------------:|",
    ]
    for place, s in enumerate(standings, start=1):
        lines.append(
            f"| {place} | {s.team} | {s.points} | {s.attacks_won}/{s.attacks_played} | "
            f"{s.defences_won}/{s.defences_played} |"
        )
    teams = [s.team for s in standings]
    lines += ["", "Attacker win rate (rows attack, columns defend):", "",
              "| attacker \\ defender | " + " | ".join(teams) + " |",
              "|---|" + "---:|" * len(teams)]
    for attacker in teams:
        cells = []
        for defender in teams:
            stats = pairings.get((attacker, defender))
            cells.append("-" if stats is None else f"{100.0 * stats.attacker_wins / stats.matches:.0f}%")
        lines.append(f"| {attacker} | " + " | ".join(cells) + " |")
    total = sum(p.matches for p in pairings.values())
    wins = sum(p.attacker_wins for p in pairings.values())
    lines += ["", f"Overall attacker win rate: {100.0 * wins / max(total, 1):.1f}% of {total} matches"]
    return "\n".join(lines)


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Round-robin drone battle tournament (kinematic backend).")
    parser.add_argument("--teams", nargs="*", default=None, help="team packages in teams/ (default: all)")
    parser.add_argument("--rounds", type=int, default=5)
    parser.add_argument("--jitter", type=float, default=3.0, help="random start offset (m)")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--config", default=None)
    parser.add_argument("--set", action="append", default=[], metavar="SECTION.KEY=VALUE")
    parser.add_argument("--process", action="store_true", help="isolate strategies in processes (slower)")
    parser.add_argument("--csv", default=None, help="write standings to a CSV file")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args(argv)

    overrides = parse_overrides(args.set)
    overrides.setdefault("sandbox", {})["isolation"] = "process" if args.process else "inline"
    config = load_config(args.config).with_overrides(**overrides)
    teams = args.teams or discover_teams()
    standings, pairings = run_tournament(config, teams, args.rounds, args.jitter, args.seed, args.verbose)
    print(format_report(standings, pairings))
    if args.csv:
        with open(args.csv, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["place", "team", "points", "attacks_won", "attacks_played", "defences_won",
                             "defences_played"])
            for place, s in enumerate(standings, start=1):
                writer.writerow([place, s.team, s.points, s.attacks_won, s.attacks_played, s.defences_won,
                                 s.defences_played])


if __name__ == "__main__":
    main()
