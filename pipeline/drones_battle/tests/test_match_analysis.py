"""Post-match analysis of recordings (near misses, telemetry, timing) and ``replay --analyze``."""

import os
from pathlib import Path

import pytest

from pipeline.drones_battle import replay
from pipeline.drones_battle.core.match_analysis import analyze_recording, near_misses
from pipeline.drones_battle.core.recorder import JsonlRecorder


def _frames(defender_alt: float, stale_at: float | None = None, hold_every: int = 0) -> list[dict]:
    """Attacker 1 flies south at 10 m/s, defender 4 north at 10 m/s on the same line; they meet at t = 5 s."""
    frames = []
    for k in range(101):
        t = round(0.1 * k, 1)
        # hold_every = n: the attacker's position only changes every n-th tick (sparse SITL telemetry)
        held_t = round(0.1 * (k - k % hold_every), 1) if hold_every else t
        frames.append({"t": t, "drones": {
            "1": {"pos": [100.0 - 10.0 * held_t, 0.0, -15.0], "vel": [-10.0, 0.0, 0.0], "alive": True,
                  "team": "attackers", "stale": stale_at is not None and abs(t - stale_at) < 0.05,
                  "cmd": [-10.0, 0.0, 0.0]},
            "4": {"pos": [10.0 * t, 0.0, -defender_alt], "vel": [10.0, 0.0, 0.0], "alive": True,
                  "team": "defenders", "stale": False, "cmd": [10.0, 0.0, 0.0]},
        }})
    return frames


def test_near_miss_is_split_into_horizontal_and_vertical() -> None:
    (miss,) = near_misses(_frames(defender_alt=12.0))
    assert (miss.attacker, miss.defender) == ("1", "4")
    assert miss.distance == pytest.approx(3.0, abs=1e-6)
    assert miss.vertical == pytest.approx(3.0, abs=1e-6)
    assert miss.horizontal == pytest.approx(0.0, abs=1e-6)
    assert miss.time == pytest.approx(5.0, abs=1e-6)
    assert miss.relative_speed == pytest.approx(20.0, abs=1e-6)
    assert miss.stale == []


def test_report_marks_stale_drones_hits_and_sparse_telemetry() -> None:
    header = {"attacker": "a", "defender": "d", "backend": "sitl"}
    result = {"winner": "defenders", "reason": "ALL_ATTACKERS_DOWN", "time": 5.0,
              "events": [{"kind": "HIT", "actor": 4, "victim": 1, "detail": ""}],
              "stats": {"attackers": {"late_ticks": 0}, "defenders": {"late_ticks": 7}}}
    report = "\n".join(analyze_recording(header, _frames(15.0, stale_at=5.0, hold_every=2), result))
    assert "1-4: 0.0 m at t = 5.0 s" in report
    assert "stale: 1 -> HIT" in report
    assert "new position in 50.0-100.0 % of the ticks per flying drone" in report
    assert "defenders 7" in report
    assert "every 100 ms on average" in report


def test_replay_analyze_uses_the_newest_recording(tmp_path: Path, monkeypatch, capsys) -> None:
    for name, mtime in (("old.jsonl", 1_000_000), ("new.jsonl", 2_000_000)):
        recorder = JsonlRecorder(tmp_path / name)
        recorder.write_header({"attacker": name, "defender": "d", "backend": "kinematic"})
        for frame in _frames(12.0)[:60]:
            recorder.write_frame(frame)
        recorder.close()
        os.utime(tmp_path / name, (mtime, mtime))
    monkeypatch.setattr(replay, "MATCHES_DIR", tmp_path)
    replay.main(["latest", "--analyze"])
    output = capsys.readouterr().out
    assert "Match: new.jsonl vs d" in output
    assert "1-4: 3.0 m" in output
