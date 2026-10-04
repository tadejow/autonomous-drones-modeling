"""Running student strategies without letting them break the arena.

Two isolation modes share one interface (``submit`` then ``collect``):

* ``ProcessController``: one persistent child process per team. The module is
  imported once, so module-level state survives between ticks. A tick that does
  not answer within the budget reuses the last valid commands; a strategy that
  stays silent for ``max_late_s`` is restarted, and after ``max_restarts``
  restarts the team forfeits.
* ``InlineController``: calls the function in-process inside ``try/except``.
  Use it for debugging with breakpoints (``--inline``).

This protects against crashes, infinite loops and slow code, not against
malicious code (a strategy can still call ``os.system``).
"""

from __future__ import annotations

import importlib
import importlib.util
import inspect
import multiprocessing as mp
import sys
import time
import traceback
from dataclasses import dataclass, field
from multiprocessing.connection import Connection
from pathlib import Path
from types import ModuleType
from typing import Any, Callable, Optional, Protocol

from pipeline.drones_battle.core.types import Commands, TeamView, Vec3

StrategyFn = Callable[..., Any]


@dataclass(frozen=True)
class StrategyCall:
    """Arguments of one call of ``compute_commands``."""

    my_team: TeamView
    enemy_team: TeamView
    target_pos: Vec3
    current_time: float
    game: dict[str, Any] = field(default_factory=dict)


def _load_file(path: Path) -> ModuleType:
    """Executes a strategy file as a fresh module.

    The file's folder is on ``sys.path`` while the file runs, so a student team
    may split its code into helper modules (``import helpers`` at the top of
    ``attacker.py``). Helper modules imported from that folder are removed from
    ``sys.modules`` afterwards: two teams can both have a ``helpers.py`` without
    one silently getting the other's code.
    """
    folder = path.parent
    name = f"_strategy_{path.stem}_{abs(hash(str(path)))}"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load strategy file {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module  # dataclasses and pickling look the module up by name
    before = set(sys.modules)
    sys.path.insert(0, str(folder))
    try:
        spec.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(name, None)
        raise
    finally:
        try:
            sys.path.remove(str(folder))
        except ValueError:
            pass
        for loaded in set(sys.modules) - before:
            file = getattr(sys.modules.get(loaded), "__file__", None)
            if file and Path(file).resolve().is_relative_to(folder):
                del sys.modules[loaded]
    return module


def load_module(ref: str) -> ModuleType:
    """Imports ``ref`` given either as a dotted module path or a path to a ``.py`` file."""
    if ref.endswith(".py") or Path(ref).exists():
        return _load_file(Path(ref).resolve())
    if ref in sys.modules:
        # Fresh module state for every match (strategies may keep state in globals).
        return importlib.reload(sys.modules[ref])
    return importlib.import_module(ref)


def load_strategy(ref: str) -> tuple[StrategyFn, bool]:
    """Returns ``(compute_commands, accepts_game_info)``."""
    module = load_module(ref)
    function = getattr(module, "compute_commands", None)
    if not callable(function):
        raise AttributeError(f"{ref} does not define compute_commands(...)")
    parameters = inspect.signature(function).parameters
    return function, "game" in parameters


def call_strategy(function: StrategyFn, accepts_game: bool, call: StrategyCall) -> Any:
    args = (call.my_team, call.enemy_team, call.target_pos, call.current_time)
    if accepts_game:
        return function(*args, game=call.game)
    return function(*args)


def clean_output(raw: Any, my_ids: set[int]) -> tuple[Commands, list[str]]:
    """Keeps only entries for the caller's own drones; vectors are validated later."""
    warnings: list[str] = []
    if not isinstance(raw, dict):
        return {}, [f"compute_commands returned {type(raw).__name__}, expected dict"]
    commands: Commands = {}
    for key, value in raw.items():
        if key in my_ids:
            commands[key] = value
        else:
            warnings.append(f"ignored command for drone {key!r} (not in your team)")
    return commands, warnings


class TeamController(Protocol):
    team: str
    errors: list[str]
    forfeited: bool

    def submit(self, call: StrategyCall) -> None: ...

    def collect(self, deadline: float) -> Commands: ...

    def close(self) -> None: ...

    def stats(self) -> dict[str, Any]: ...


class InlineController:
    """In-process execution; a crash costs one tick of hovering."""

    def __init__(self, team: str, ref: str, my_ids: tuple[int, ...], load: bool = True) -> None:
        self.team = team
        self.ref = ref
        self.my_ids = set(my_ids)
        if load:
            self.function, self.accepts_game = load_strategy(ref)
        self.errors: list[str] = []
        self.forfeited = False
        self._pending: Optional[StrategyCall] = None
        self._last: Commands = {}
        self._calls = 0
        self._exceptions = 0
        self._compute_time = 0.0

    def submit(self, call: StrategyCall) -> None:
        self._pending = call

    def collect(self, deadline: float) -> Commands:
        call, self._pending = self._pending, None
        if call is None:
            return self._last
        started = time.perf_counter()
        try:
            raw = call_strategy(self.function, self.accepts_game, call)
        except Exception:  # noqa: BLE001 - a student error must never stop the arena
            self._exceptions += 1
            self._log(traceback.format_exc())
            return {}
        finally:
            self._compute_time += time.perf_counter() - started
            self._calls += 1
        commands, warnings = clean_output(raw, self.my_ids)
        for warning in warnings:
            self._log(warning)
        self._last = commands
        return commands

    def close(self) -> None:
        pass

    def stats(self) -> dict[str, Any]:
        return {
            "calls": self._calls,
            "exceptions": self._exceptions,
            "late_ticks": 0,
            "restarts": 0,
            "mean_compute_ms": 1000.0 * self._compute_time / max(self._calls, 1),
        }

    def _log(self, message: str) -> None:
        if len(self.errors) < 200:
            self.errors.append(message)
        if len(self.errors) <= 3:
            print(f"[{self.team}] {message.strip().splitlines()[-1]}")


def _worker(ref: str, connection: Connection) -> None:
    """Child process loop: receive ``(seq, call)``, answer ``(seq, status, payload, seconds)``."""
    try:
        function, accepts_game = load_strategy(ref)
    except Exception:  # noqa: BLE001
        connection.send((-1, "load_error", traceback.format_exc(), 0.0))
        return
    connection.send((-1, "ready", None, 0.0))
    while True:
        try:
            message = connection.recv()
        except (EOFError, OSError):
            return
        if message is None:
            return
        seq, call = message
        started = time.perf_counter()
        try:
            result = call_strategy(function, accepts_game, call)
            connection.send((seq, "ok", result, time.perf_counter() - started))
        except Exception:  # noqa: BLE001
            connection.send((seq, "error", traceback.format_exc(), time.perf_counter() - started))


class ProcessController(InlineController):
    """One persistent child process per team with a per-tick time budget."""

    def __init__(
        self, team: str, ref: str, my_ids: tuple[int, ...], max_late_s: float = 3.0, max_restarts: int = 3
    ) -> None:
        # Student code is imported only in the child; its import errors come back with a traceback.
        super().__init__(team, ref, my_ids, load=False)
        self.max_late_s = max_late_s
        self.max_restarts = max_restarts
        self._context = mp.get_context("spawn")
        self._process: Optional[Any] = None
        self._connection: Optional[Connection] = None
        self._seq = 0
        self._busy_seq: Optional[int] = None
        self._busy_since = 0.0
        self._late_ticks = 0
        self._restarts = 0
        self._start()

    def _start(self) -> None:
        parent, child = self._context.Pipe()
        self._process = self._context.Process(
            target=_worker, args=(self.ref, child), name=f"strategy-{self.team}", daemon=True
        )
        self._process.start()
        child.close()
        self._connection = parent
        if not parent.poll(30.0):
            raise RuntimeError(f"[{self.team}] strategy process did not start")
        _, status, payload, _ = parent.recv()
        if status != "ready":
            raise RuntimeError(f"[{self.team}] cannot load strategy:\n{payload}")
        self._busy_seq = None

    def _kill(self) -> None:
        if self._process is not None and self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=2.0)
        if self._connection is not None:
            self._connection.close()

    def submit(self, call: StrategyCall) -> None:
        if self.forfeited or self._connection is None:
            return
        if self._busy_seq is not None:
            if time.monotonic() - self._busy_since > self.max_late_s:
                self._restart()
            return
        self._seq += 1
        self._busy_seq = self._seq
        self._busy_since = time.monotonic()
        try:
            self._connection.send((self._seq, call))
        except (BrokenPipeError, OSError):
            self._restart()

    def collect(self, deadline: float) -> Commands:
        connection = self._connection
        if self.forfeited or connection is None or self._busy_seq is None:
            if self._busy_seq is not None:
                self._late_ticks += 1
            return self._last
        try:
            while True:
                remaining = deadline - time.monotonic()
                if not connection.poll(max(remaining, 0.0)):
                    self._late_ticks += 1
                    return self._last
                seq, status, payload, seconds = connection.recv()
                if seq == self._busy_seq:
                    break
        except (EOFError, OSError):
            self._log("strategy process died")
            self._restart()
            return self._last
        self._busy_seq = None
        self._calls += 1
        self._compute_time += seconds
        if status != "ok":
            self._exceptions += 1
            self._log(str(payload))
            return self._last
        commands, warnings = clean_output(payload, self.my_ids)
        for warning in warnings:
            self._log(warning)
        self._last = commands
        return commands

    def _restart(self) -> None:
        self._kill()
        self._restarts += 1
        self._log(f"strategy restarted ({self._restarts}/{self.max_restarts})")
        if self._restarts > self.max_restarts:
            self.forfeited = True
            self._connection = None
            return
        self._last = {}
        self._start()

    def close(self) -> None:
        if self._connection is not None:
            try:
                self._connection.send(None)
            except (BrokenPipeError, OSError):
                pass
        if self._process is not None:
            self._process.join(timeout=1.0)
        self._kill()

    def stats(self) -> dict[str, Any]:
        data = super().stats()
        data.update(late_ticks=self._late_ticks, restarts=self._restarts)
        return data


def make_controller(
    team: str, ref: str, my_ids: tuple[int, ...], isolation: str, max_late_s: float, max_restarts: int
) -> TeamController:
    if isolation == "inline":
        return InlineController(team, ref, my_ids)
    return ProcessController(team, ref, my_ids, max_late_s, max_restarts)
