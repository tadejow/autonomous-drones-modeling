"""Post-match analysis of a recorded battle: near misses, altitudes, telemetry, loop timing.

Answers "why did the drones not collide?" from a recording alone (no SITL needed)::

    python -m pipeline.drones_battle.replay latest --analyze

Unlike the referee, the near misses here include ticks in which a drone was stale
(the referee ignores those), so a pass that only failed because of late telemetry
shows up as a small distance marked "stale".
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

import numpy as np

from pipeline.drones_battle.core.config import ArenaConfig
from pipeline.drones_battle.core.referee import closest_approach


@dataclass
class NearMiss:
    attacker: str
    defender: str
    distance: float
    time: float
    horizontal: float
    vertical: float
    relative_speed: float
    stale: list[str]


def _config(header: dict[str, Any]) -> ArenaConfig:
    raw = header.get("config")
    return ArenaConfig().with_overrides(**raw) if raw else ArenaConfig()


def _team(frames: list[dict[str, Any]], team: str) -> list[str]:
    return sorted((i for i, d in frames[0]["drones"].items() if d["team"] == team), key=int)


def near_misses(frames: list[dict[str, Any]]) -> list[NearMiss]:
    """The closest approach of every attacker to any defender (continuous, between ticks), closest first."""
    attackers, defenders = _team(frames, "attackers"), _team(frames, "defenders")
    best: dict[str, NearMiss] = {}
    for previous, current in zip(frames, frames[1:]):
        dt = current["t"] - previous["t"]
        before, after = previous["drones"], current["drones"]
        for a in attackers:
            if not before[a]["alive"]:
                continue
            for d in defenders:
                if not before[d]["alive"]:
                    continue
                a0, a1 = np.asarray(before[a]["pos"], float), np.asarray(after[a]["pos"], float)
                d0, d1 = np.asarray(before[d]["pos"], float), np.asarray(after[d]["pos"], float)
                distance, s_star = closest_approach(a0, a1, d0, d1)
                if a in best and distance >= best[a].distance:
                    continue
                gap = (a0 - d0) + s_star * ((a1 - a0) - (d1 - d0))
                speed = float(np.linalg.norm((a1 - a0) - (d1 - d0))) / dt if dt > 0 else 0.0
                stale = [i for i in (a, d) if after[i].get("stale")]
                best[a] = NearMiss(a, d, distance, current["t"] - (1.0 - s_star) * dt,
                                   float(np.hypot(gap[0], gap[1])), float(abs(gap[2])), speed, stale)
    return sorted(best.values(), key=lambda m: m.distance)


def _telemetry_line(frames: list[dict[str, Any]], ids: list[str]) -> str:
    """How often a moving drone's position changed between ticks (unchanged = no new SITL message)."""
    fresh, stale = [], []
    for drone_id in ids:
        ticks = [
            (p["drones"][drone_id], c["drones"][drone_id]) for p, c in zip(frames, frames[1:])
            if p["drones"][drone_id]["alive"] and float(np.linalg.norm(p["drones"][drone_id]["vel"])) > 1.0
        ]
        if ticks:
            fresh.append(100.0 * sum(c["pos"] != p["pos"] for p, c in ticks) / len(ticks))
            stale.append(100.0 * sum(bool(c.get("stale")) for _, c in ticks) / len(ticks))
    return (
        f"Telemetry: new position in {_range(fresh, ' %')} of the ticks per flying drone; "
        f"stale (ignored by the referee) in {_range(stale, ' %')}"
    )


def _range(values: list[float], unit: str) -> str:
    return f"{min(values):.1f}-{max(values):.1f}{unit}" if values else "-"


def analyze_recording(
    header: dict[str, Any], frames: list[dict[str, Any]], result: Optional[dict[str, Any]]
) -> list[str]:
    """Human-readable report (one string per line)."""
    if len(frames) < 2:
        return ["The recording has fewer than two frames."]
    config = _config(header)
    game = config.game
    teams = {team: _team(frames, team) for team in ("attackers", "defenders")}
    lines = [
        f"Match: {header.get('attacker', '?')} vs {header.get('defender', '?')} "
        f"(backend {header.get('backend', '?')}, recorded {header.get('created', '?')})"
    ]
    if result:
        lines.append(f"Result: {result['winner'].upper()} win ({result['reason']}) at t = {result['time']:.1f} s")

    times = np.array([f["t"] for f in frames])
    intervals = np.diff(times)
    lines.append(
        f"Arena loop: {len(frames)} ticks, every {1000 * intervals.mean():.0f} ms on average "
        f"(longest {1000 * intervals.max():.0f} ms; expected {1000 * game.dt:.0f} ms)"
    )

    if header.get("backend") == "sitl":
        lines.append(_telemetry_line(frames, teams["attackers"] + teams["defenders"]))

    for team, ids in teams.items():
        altitudes = [-f["drones"][i]["pos"][2] for f in frames for i in ids if f["drones"][i]["alive"]]
        speeds = [float(np.linalg.norm(f["drones"][i]["vel"][:2])) for f in frames for i in ids
                  if f["drones"][i]["alive"]]
        errors = [
            float(np.linalg.norm(np.subtract(p["drones"][i]["cmd"], c["drones"][i]["vel"])))
            for p, c in zip(frames, frames[1:]) for i in ids
            if p["drones"][i]["alive"] and c["drones"][i]["alive"] and p["drones"][i].get("cmd") is not None
        ]
        tracking = f", command - velocity {np.mean(errors):.1f} m/s on average" if errors else ""
        lines.append(
            f"{team.capitalize()}: altitude {_range(altitudes, ' m')}, horizontal speed "
            f"{np.mean(speeds) if speeds else 0:.1f} m/s on average (max {max(speeds, default=0):.1f}){tracking}"
        )

    misses = near_misses(frames)
    hits = {
        (str(e["victim"]), str(e["actor"])) for e in (result or {}).get("events", [])
        if e["kind"] == "HIT" and not e.get("detail")
    }
    if misses:
        lines.append(f"Closest approaches (a hit needs < {game.kill_radius_m:.1f} m):")
        for miss in misses:
            note = f", stale: {', '.join(miss.stale)}" if miss.stale else ""
            note += " -> HIT" if (miss.attacker, miss.defender) in hits else ""
            lines.append(
                f"  {miss.attacker}-{miss.defender}: {miss.distance:.1f} m at t = {miss.time:.1f} s "
                f"(horizontal {miss.horizontal:.1f} m, vertical {miss.vertical:.1f} m, "
                f"relative speed {miss.relative_speed:.1f} m/s{note})"
            )

    stats = (result or {}).get("stats", {})
    late = [f"{team} {stats[team].get('late_ticks', 0)}" for team in ("attackers", "defenders") if team in stats]
    if late:
        lines.append("Strategy answers later than the tick budget: " + ", ".join(late))
    telemetry = stats.get("telemetry")
    if telemetry:
        lines.append(
            f"SITL telemetry: {telemetry.get('position_rate_min_hz', 0):.1f}-"
            f"{telemetry.get('position_rate_max_hz', 0):.1f} positions/s per drone, max delay "
            f"{telemetry.get('telemetry_lag_max_s', 0):.2f} s, duplicates {telemetry.get('duplicate_messages', 0)}"
        )
    machine = stats.get("machine")
    if machine:
        lines.append(f"Machine: load {machine['load_1min']:.1f} on {machine['cpus']:.0f} CPUs")
    return lines
