import shutil
import threading
import time
import zipfile
from pathlib import Path

import pytest

from pipeline.drones_battle.arena_orchestrator import MatchCancelled, run_match
from pipeline.drones_battle.core.config import ArenaConfig
from pipeline.drones_battle.core.sandbox import InlineController, StrategyCall
from pipeline.drones_battle.core.submissions import (
    check_submission, discover_submissions, example_teams, extract_zips,
)
from pipeline.drones_battle.tournament import run_tournament

PACKAGE = Path(__file__).resolve().parent.parent
TEMPLATE = PACKAGE / "student_teams" / "_szablon"


def _team(root: Path, name: str, attacker: str | None = None, defender: str | None = None) -> Path:
    folder = root / name
    shutil.copytree(TEMPLATE, folder)
    if attacker is not None:
        (folder / "attacker.py").write_text(attacker, encoding="utf-8")
    if defender is not None:
        (folder / "defender.py").write_text(defender, encoding="utf-8")
    return folder


def _entry(root: Path, name: str):
    return next(e for e in discover_submissions(root) if e.key == name)


def test_template_is_valid_and_hidden(tmp_path: Path) -> None:
    shutil.copytree(TEMPLATE, tmp_path / "_szablon")
    _team(tmp_path, "300538")
    entries = discover_submissions(tmp_path)
    assert [e.key for e in entries] == ["300538"]
    assert entries[0].name == "Nazwa drużyny"
    check = check_submission(entries[0])
    assert check.ok, check.errors


def test_missing_file_and_syntax_error(tmp_path: Path) -> None:
    folder = _team(tmp_path, "300539", defender="def compute_commands(a, b, c, d)\n    return {}\n")
    check = check_submission(_entry(tmp_path, "300539"))
    assert not check.ok and "SyntaxError" in check.errors[0]
    assert "importlib" not in check.errors[0]  # only the student's own code is shown
    (folder / "defender.py").unlink()
    check = check_submission(_entry(tmp_path, "300539"))
    assert not check.ok and "defender.py" in check.errors[0]


def test_runtime_error_bad_output_and_foreign_ids(tmp_path: Path) -> None:
    _team(tmp_path, "300540", attacker=(
        "def compute_commands(my_team, enemy_team, target_pos, current_time):\n"
        "    if current_time > 10:\n"
        "        return {i: 1 / 0 for i in my_team}\n"
        "    return {99: (1, 2, 3), **{i: (1.0, 0.0, 0.0) for i in my_team}}\n"
    ), defender="def compute_commands(my_team, enemy_team, target_pos, current_time):\n    return [1, 2, 3]\n")
    check = check_submission(_entry(tmp_path, "300540"))
    assert not check.ok
    assert any("ZeroDivisionError" in e and "attacker.py" in e for e in check.errors)
    assert any("zwraca list" in e for e in check.errors)
    assert any("cudzych" in w for w in check.warnings)


def test_hanging_import_times_out(tmp_path: Path) -> None:
    _team(tmp_path, "300541", attacker="while True:\n    pass\n")
    started = time.monotonic()
    check = check_submission(_entry(tmp_path, "300541"), timeout_s=3.0)
    assert not check.ok and "przekroczyło" in check.errors[0]
    assert time.monotonic() - started < 15.0


def test_helper_modules_with_the_same_name_do_not_mix(tmp_path: Path) -> None:
    for name, value in (("300542", 1.0), ("300543", 2.0)):
        folder = _team(tmp_path, name, attacker=(
            "import helpers\n\n"
            "def compute_commands(my_team, enemy_team, target_pos, current_time):\n"
            "    return {i: (helpers.SPEED, 0.0, 0.0) for i in my_team}\n"
        ))
        (folder / "helpers.py").write_text(f"SPEED = {value}\n", encoding="utf-8")
    call = StrategyCall({1: {"pos": (0, 0, 0), "vel": (0, 0, 0), "alive": True}}, {}, (0.0, 0.0, -10.0), 0.0)
    speeds = []
    for name in ("300542", "300543"):
        controller = InlineController("attackers", str(tmp_path / name / "attacker.py"), (1,))
        controller.submit(call)
        speeds.append(controller.collect(time.monotonic() + 1.0)[1][0])
    assert speeds == [1.0, 2.0]


def test_zip_layouts_and_path_traversal(tmp_path: Path) -> None:
    files = {"attacker.py": TEMPLATE / "attacker.py", "defender.py": TEMPLATE / "defender.py"}
    with zipfile.ZipFile(tmp_path / "300544.zip", "w") as bundle:  # files at the top level
        for name, source in files.items():
            bundle.write(source, name)
    with zipfile.ZipFile(tmp_path / "300545.zip", "w") as bundle:  # one folder with another name
        for name, source in files.items():
            bundle.write(source, f"moje_rozwiazanie/{name}")
    with zipfile.ZipFile(tmp_path / "300546.zip", "w") as bundle:
        bundle.writestr("../evil.py", "x = 1\n")
    messages = extract_zips(tmp_path)
    assert (tmp_path / "300544" / "attacker.py").exists()
    assert (tmp_path / "300545" / "defender.py").exists()
    assert not (tmp_path / "evil.py").exists()
    assert any("300546" in m and "nie udało" in m for m in messages)
    assert extract_zips(tmp_path) == [m for m in messages if "300546" in m]  # already unpacked are skipped


def test_tournament_with_student_folders(tmp_path: Path) -> None:
    _team(tmp_path, "300547")
    _team(tmp_path, "300548")
    teams = discover_submissions(tmp_path)
    played = []
    config = ArenaConfig().with_overrides(sandbox={"isolation": "inline"})
    standings, pairings = run_tournament(
        config, teams, rounds=1, start_jitter_m=2.0, base_seed=0, self_play=False,
        on_match=lambda record, i, n: played.append((record.attacker, record.defender, i, n)),
        record_dir=tmp_path / "records",
    )
    assert [(a, d) for a, d, _, _ in played] == [("300547", "300548"), ("300548", "300547")]
    assert sum(s.points for s in standings) == 6
    assert len(list((tmp_path / "records").glob("*.jsonl"))) == 2


def test_example_teams_are_listed() -> None:
    assert {e.name for e in example_teams()} >= {"baseline", "hunters", "pincer", "wolfpack", "tricksters"}


def test_cancel_stops_a_running_match() -> None:
    cancel = threading.Event()
    cancel.set()
    with pytest.raises(MatchCancelled):
        run_match(ArenaConfig().with_overrides(sandbox={"isolation": "inline"}), fast=True, visualize=False,
                  verbose=False, cancel=cancel)


def test_visualizer_process_reused_between_matches(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("MPLBACKEND", "Agg")
    from pipeline.drones_battle.arena_visualizer import VisualizerProcess

    config = ArenaConfig().with_overrides(sandbox={"isolation": "inline"})
    visualizer = VisualizerProcess(config, title="test")
    try:
        assert visualizer.wait_ready(60.0)
        for seed in (1, 2):
            result = run_match(config, fast=True, visualize=False, verbose=False, seed=seed, visualizer=visualizer,
                               final_hold_s=0.0, title=f"match {seed}")
            assert result.winner in ("attackers", "defenders")
        assert visualizer.is_alive()
    finally:
        visualizer.close()
    assert not visualizer.is_alive()
