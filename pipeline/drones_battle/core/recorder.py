"""Match recording in JSON Lines: one header line, then one line per tick."""

from __future__ import annotations

import json
import subprocess
from pathlib import Path
from typing import Any, Iterator, Optional, Protocol


def _git_revision() -> Optional[str]:
    try:
        output = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, timeout=2, check=False
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return output.stdout.strip() or None


def _round(value: Any) -> Any:
    if isinstance(value, float):
        return round(value, 3)
    if isinstance(value, (list, tuple)):
        return [_round(v) for v in value]
    if isinstance(value, dict):
        return {str(k): _round(v) for k, v in value.items()}
    return value


class Recorder(Protocol):
    def write_header(self, header: dict[str, Any]) -> None: ...

    def write_frame(self, frame: dict[str, Any]) -> None: ...

    def write_result(self, result: dict[str, Any]) -> None: ...

    def close(self) -> None: ...


class NullRecorder:
    def write_header(self, header: dict[str, Any]) -> None:
        pass

    def write_frame(self, frame: dict[str, Any]) -> None:
        pass

    def write_result(self, result: dict[str, Any]) -> None:
        pass

    def close(self) -> None:
        pass


class JsonlRecorder:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.path.open("w", encoding="utf-8")

    def _write(self, record: dict[str, Any]) -> None:
        self._handle.write(json.dumps(_round(record), separators=(",", ":")) + "\n")

    def write_header(self, header: dict[str, Any]) -> None:
        self._write({"type": "header", "git": _git_revision(), **header})

    def write_frame(self, frame: dict[str, Any]) -> None:
        self._write({"type": "frame", **frame})

    def write_result(self, result: dict[str, Any]) -> None:
        self._write({"type": "result", **result})
        self._handle.flush()

    def close(self) -> None:
        self._handle.close()


def read_recording(path: str | Path) -> tuple[dict[str, Any], list[dict[str, Any]], Optional[dict[str, Any]]]:
    """Returns ``(header, frames, result)`` of a recorded match."""
    header: dict[str, Any] = {}
    frames: list[dict[str, Any]] = []
    result: Optional[dict[str, Any]] = None
    for record in _iter_lines(Path(path)):
        kind = record.pop("type", None)
        if kind == "header":
            header = record
        elif kind == "frame":
            frames.append(record)
        elif kind == "result":
            result = record
    return header, frames, result


def _iter_lines(path: Path) -> Iterator[dict[str, Any]]:
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                yield json.loads(line)
