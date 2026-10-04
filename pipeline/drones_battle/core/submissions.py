"""Student team folders: discovery, ZIP unpacking and validation.

Every team is a folder named after the student's index number inside
``pipeline/drones_battle/student_teams/`` (the format is described in
``student_teams/README.md``)::

    student_teams/
    ├── 300538/
    │   ├── attacker.py      # compute_commands(...) when attacking   (required)
    │   ├── defender.py      # compute_commands(...) when defending   (required)
    │   ├── team.toml        # name = "...", authors = ["..."]        (optional)
    │   └── helpers.py       # any helper modules                     (optional)
    └── 300539.zip           # unpacked automatically to 300539/

Validation runs the strategies in a separate process with a timeout, on
sample 3 vs 3 and 5 vs 5 situations, so a broken or hanging submission is
reported before the tournament instead of during it.

Command line (also for students, to check their folder before submitting)::

    python -m pipeline.drones_battle.core.submissions                 # all folders
    python -m pipeline.drones_battle.core.submissions path/to/300538  # one folder
"""

from __future__ import annotations

import argparse
import multiprocessing as mp
import random
import statistics
import sys
import time
import traceback
import zipfile
from dataclasses import dataclass, field
from pathlib import Path, PurePosixPath
from typing import Any, Optional

PACKAGE_DIR = Path(__file__).resolve().parent.parent
DEFAULT_SUBMISSIONS_DIR = PACKAGE_DIR / "student_teams"
EXAMPLE_TEAMS_DIR = PACKAGE_DIR / "teams"
EXAMPLE_TEAMS_PACKAGE = "pipeline.drones_battle.teams"
CONFIG_5V5 = PACKAGE_DIR / "arena_config_5v5.toml"
REQUIRED_FILES = ("attacker.py", "defender.py")
FORBIDDEN_IMPORTS = ("dronekit", "pymavlink", "socket", "subprocess")
CHECK_TIMEOUT_S = 30.0


@dataclass(frozen=True)
class TeamEntry:
    """One team that can play: two strategy references (module path or ``.py`` file)."""

    key: str
    name: str
    attacker: str
    defender: str
    folder: Optional[str] = None
    kind: str = "student"
    authors: tuple[str, ...] = ()

    @property
    def label(self) -> str:
        if self.kind == "example":
            return f"[przykład] {self.name}"
        return self.key if self.name == self.key else f"{self.key} – {self.name}"


@dataclass
class SubmissionCheck:
    ok: bool
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    mean_ms: float = 0.0
    max_ms: float = 0.0

    def summary(self) -> str:
        if not self.ok:
            return "BŁĄD: " + (self.errors[0].splitlines()[0] if self.errors else "nieznany")
        if self.warnings:
            return f"OK ({len(self.warnings)} ostrz.), {self.mean_ms:.1f} ms"
        return f"OK, {self.mean_ms:.1f} ms"


# ---------------------------------------------------------------------- discovery
def example_teams() -> list[TeamEntry]:
    entries = []
    for folder in sorted(EXAMPLE_TEAMS_DIR.iterdir()):
        if folder.is_dir() and all((folder / f).exists() for f in REQUIRED_FILES):
            package = f"{EXAMPLE_TEAMS_PACKAGE}.{folder.name}"
            entries.append(TeamEntry(
                key=f"example:{folder.name}", name=folder.name, attacker=f"{package}.attacker",
                defender=f"{package}.defender", folder=str(folder), kind="example",
            ))
    return entries


def read_team_info(folder: Path) -> tuple[Optional[str], tuple[str, ...]]:
    """``(name, authors)`` from the optional ``team.toml``."""
    info_file = folder / "team.toml"
    if not info_file.exists():
        return None, ()
    try:
        try:
            import tomllib
        except ModuleNotFoundError:
            import tomli as tomllib  # type: ignore[no-redef]
        with info_file.open("rb") as handle:
            data = tomllib.load(handle)
    except Exception:  # noqa: BLE001 - a broken team.toml must not hide the team
        return None, ()
    name = data.get("name")
    authors = data.get("authors", ())
    if isinstance(authors, str):
        authors = (authors,)
    return (str(name) if name else None), tuple(str(a) for a in authors)


def discover_submissions(root: Path | str = DEFAULT_SUBMISSIONS_DIR) -> list[TeamEntry]:
    """Every sub-folder of ``root`` except those starting with ``_`` or ``.`` (e.g. ``_szablon``)."""
    root = Path(root)
    if not root.is_dir():
        return []
    entries = []
    for folder in sorted(root.iterdir(), key=lambda p: p.name):
        if not folder.is_dir() or folder.name.startswith(("_", ".")) or folder.name == "__pycache__":
            continue
        name, authors = read_team_info(folder)
        entries.append(TeamEntry(
            key=folder.name, name=name or folder.name, attacker=str(folder / "attacker.py"),
            defender=str(folder / "defender.py"), folder=str(folder), kind="student", authors=authors,
        ))
    return entries


def extract_zips(root: Path | str = DEFAULT_SUBMISSIONS_DIR) -> list[str]:
    """Unpacks every ``<index>.zip`` in ``root`` that has no folder yet. Returns messages.

    Both layouts are accepted: files at the top of the archive, or one folder
    containing them (whatever its name, the result is ``root/<zip name>/``).
    Paths escaping the target folder are rejected.
    """
    root = Path(root)
    messages = []
    for archive in sorted(root.glob("*.zip")):
        target = root / archive.stem
        if target.exists():
            continue
        try:
            with zipfile.ZipFile(archive) as bundle:
                names = [n for n in bundle.namelist() if not n.endswith("/") and "__MACOSX" not in n]
                parts = [PurePosixPath(n).parts for n in names]
                if any(p.is_absolute() or ".." in p.parts for p in map(PurePosixPath, names)):
                    raise ValueError("archiwum zawiera niebezpieczne ścieżki")
                strip = 1 if parts and len({p[0] for p in parts}) == 1 and all(len(p) > 1 for p in parts) else 0
                for name, part in zip(names, parts):
                    destination = target.joinpath(*part[strip:])
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    destination.write_bytes(bundle.read(name))
            messages.append(f"rozpakowano {archive.name} -> {target.name}/")
        except (zipfile.BadZipFile, ValueError, OSError) as exc:
            messages.append(f"nie udało się rozpakować {archive.name}: {exc}")
    return messages


# ---------------------------------------------------------------------- validation
def _sample_situations(config: Any, seed: int) -> list[tuple[dict, float]]:
    """Start positions after take-off and a random mid-battle state with some drones down."""
    from pipeline.drones_battle.core.layout import drone_slots

    rng = random.Random(seed)
    slots = drone_slots(config)
    start = {s.drone_id: {"pos": (s.start_ned[0], s.start_ned[1], -15.0), "vel": (0.0, 0.0, 0.0), "alive": True}
             for s in slots}
    middle = {}
    for s in slots:
        middle[s.drone_id] = {
            "pos": (rng.uniform(0, 140), rng.uniform(-40, 40), -rng.uniform(8, 20)),
            "vel": (rng.uniform(-8, 8), rng.uniform(-8, 8), rng.uniform(-1, 1)),
            "alive": rng.random() > 0.3,
        }
    return [(start, 0.0), (middle, 37.5)]


def _student_traceback(folder: Optional[str]) -> str:
    """The current exception limited to the student's own files (no arena internals)."""
    kind, error, trace = sys.exc_info()
    frames = traceback.extract_tb(trace)
    if folder:
        own = [f for f in frames if Path(f.filename).resolve().is_relative_to(Path(folder).resolve())]
        # A SyntaxError has no frame of its own: the message already shows file, line and caret.
        frames = traceback.StackSummary.from_list(own) if own or isinstance(error, SyntaxError) else frames[-1:]
    lines = traceback.format_list(frames) + traceback.format_exception_only(kind, error)
    return "".join(lines).rstrip()


def _check_worker(entry: TeamEntry, connection: Any) -> None:
    """Child process: load both strategies and call them on sample situations."""
    from pipeline.drones_battle.arena_orchestrator import game_info
    from pipeline.drones_battle.core.config import ArenaConfig, load_config
    from pipeline.drones_battle.core.safety import parse_command
    from pipeline.drones_battle.core.sandbox import StrategyCall, call_strategy, load_strategy

    errors: list[str] = []
    warnings: list[str] = []
    timings: list[float] = []
    configs = [("3v3", ArenaConfig())]
    if CONFIG_5V5.exists():
        configs.append(("5v5", load_config(CONFIG_5V5)))

    for role, ref in (("attacker", entry.attacker), ("defender", entry.defender)):
        if entry.kind == "student":
            source_path = Path(ref)
            if not source_path.exists():
                errors.append(f"brak pliku {source_path.name}")
                continue
            source = source_path.read_text(encoding="utf-8", errors="replace")
            for banned in FORBIDDEN_IMPORTS:
                if f"import {banned}" in source or f"from {banned}" in source:
                    warnings.append(f"{source_path.name}: importuje '{banned}' (niedozwolone w strategii)")
        try:
            function, accepts_game = load_strategy(ref)
        except Exception:  # noqa: BLE001
            errors.append(f"{role}.py nie daje się zaimportować:\n{_student_traceback(entry.folder)}")
            continue
        team = "attackers" if role == "attacker" else "defenders"
        for label, config in configs:
            game = config.game
            mine, theirs = (game.attacker_ids, game.defender_ids) if team == "attackers" else \
                (game.defender_ids, game.attacker_ids)
            for situation, time_s in _sample_situations(config, seed=len(mine)):
                my_team = {i: dict(situation[i]) for i in mine}
                enemy = {i: dict(situation[i]) for i in theirs}
                call = StrategyCall(my_team, enemy, game.target_ned, time_s, game_info(config, team, time_s))
                started = time.perf_counter()
                try:
                    result = call_strategy(function, accepts_game, call)
                except Exception:  # noqa: BLE001
                    errors.append(f"{role}.py ({label}, t={time_s}) rzuca wyjątek:\n"
                                  f"{_student_traceback(entry.folder)}")
                    break
                timings.append(1000.0 * (time.perf_counter() - started))
                if not isinstance(result, dict):
                    errors.append(f"{role}.py ({label}) zwraca {type(result).__name__}, oczekiwano dict")
                    break
                foreign = sorted(set(result) - set(mine), key=str)
                if foreign:
                    warnings.append(f"{role}.py ({label}) zwraca komendy dla cudzych dronów {foreign} "
                                    "(zostaną zignorowane)")
                for drone_id, vector in result.items():
                    if drone_id in mine and parse_command(vector) is None:
                        errors.append(f"{role}.py ({label}) zła komenda dla drona {drone_id}: {vector!r} "
                                      "(oczekiwano 3 liczb)")
                        break
                missing = [i for i in mine if my_team[i]["alive"] and i not in result]
                if missing and time_s == 0.0:
                    warnings.append(f"{role}.py ({label}) nie steruje dronami {missing} (będą wisieć w miejscu)")

    mean_ms = statistics.mean(timings) if timings else 0.0
    max_ms = max(timings) if timings else 0.0
    if max_ms > 1000.0:
        errors.append(f"jedno wywołanie trwało {max_ms:.0f} ms (limit areny: 30 ms na krok)")
    elif max_ms > 30.0:
        warnings.append(f"najwolniejsze wywołanie {max_ms:.0f} ms > 30 ms: arena użyje poprzednich komend")
    connection.send(SubmissionCheck(not errors, errors, _unique(warnings), mean_ms, max_ms))


def _unique(items: list[str]) -> list[str]:
    seen: dict[str, None] = {}
    for item in items:
        seen.setdefault(item, None)
    return list(seen)


def check_submission(entry: TeamEntry, timeout_s: float = CHECK_TIMEOUT_S) -> SubmissionCheck:
    """Validates a team in a separate process; a hanging import or loop ends as an error."""
    if entry.kind == "student" and entry.folder:
        missing = [f for f in REQUIRED_FILES if not (Path(entry.folder) / f).exists()]
        if missing:
            present = sorted(p.name for p in Path(entry.folder).iterdir())
            return SubmissionCheck(False, [f"brak wymaganych plików: {', '.join(missing)} "
                                          f"(w katalogu jest: {', '.join(present) or 'nic'})"])
    context = mp.get_context("spawn")
    parent, child = context.Pipe(duplex=False)
    process = context.Process(target=_check_worker, args=(entry, child), daemon=True)
    process.start()
    child.close()
    try:
        if parent.poll(timeout_s):
            return parent.recv()
        return SubmissionCheck(False, [f"sprawdzanie przekroczyło {timeout_s:.0f} s (nieskończona pętla?)"])
    except (EOFError, OSError):
        return SubmissionCheck(False, ["proces sprawdzający zakończył się błędem (np. sys.exit w strategii)"])
    finally:
        if process.is_alive():
            process.terminate()
        process.join(timeout=2.0)


# ---------------------------------------------------------------------- CLI
def _print_report(entry: TeamEntry, check: SubmissionCheck) -> None:
    print(f"{'OK ' if check.ok else 'ERR'} {entry.label:30s} {check.summary() if check.ok else 'BŁĄD'}")
    for error in check.errors:
        print("      błąd: " + error.replace("\n", "\n            "))
    for warning in check.warnings:
        print("      uwaga: " + warning)


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Check student team folders for the drone battle.")
    parser.add_argument("folders", nargs="*", help="team folders (default: all in student_teams/)")
    parser.add_argument("--root", default=str(DEFAULT_SUBMISSIONS_DIR))
    args = parser.parse_args(argv)
    if args.folders:
        entries = []
        for folder in map(Path, args.folders):
            entries += [e for e in discover_submissions(folder.resolve().parent) if e.key == folder.resolve().name]
    else:
        for message in extract_zips(args.root):
            print(message)
        entries = discover_submissions(args.root)
    if not entries:
        print("Nie znaleziono katalogów drużyn.")
        return
    failed = 0
    for entry in entries:
        check = check_submission(entry)
        failed += not check.ok
        _print_report(entry, check)
    print(f"\n{len(entries) - failed}/{len(entries)} drużyn gotowych do gry.")
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()


__all__ = [
    "TeamEntry", "SubmissionCheck", "example_teams", "discover_submissions", "extract_zips", "check_submission",
    "read_team_info", "DEFAULT_SUBMISSIONS_DIR",
]
